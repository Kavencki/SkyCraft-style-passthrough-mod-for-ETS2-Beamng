#define WIN32_LEAN_AND_MEAN
#include "truckpuppet.h"

#include <windows.h>
#include <tlhelp32.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <deque>
#include <iterator>

#include "../protocol/mapping.h"
#include "codescan.h"
#include "ets2_code.h"

using namespace es;

// How ETS2 1.61.1.1s moves the player truck (found with tools/re, 2026-10-11):
//   The truck object keeps an array of its body states ([truck+0x218], count [truck+0x220]:
//   chassis, cabin). Every physics step (+0x86d6d0 loop) ETS2 integrates each state S:
//     S+0x00 physics actor P, S+0x0c step dt, S+0x10 placement (float x,y,z, int16 sx,sz,
//     sector 512 m) + S+0x20 quaternion (w,x,y,z), S+0x30 linear velocity, S+0x3c angular
//     velocity, S+0x48 the previous placement (same 0x20 bytes)
//   then P->setPlacement(S+0x10) (vtable +0x110 = +0x163bd40) pushes it into PhysX.
//   Writing these states moves the truck through ETS2's own machinery (PhysX bodies are only
//   its output: writing them was overwritten every step).
//   The truck object is found from the telemetry thunk: +0x5a79f6 `call [rax+0x28]`, rcx = truck
//   when [rax+0x28] == +0x5a7eec (the placement channel getter).
// ETS2 1.61.1.9s moved that code (+0x170, the loop +0xa30, the actor vtable 0x2449d48): it is
// found by its bytes now (codescan.h), and the actor class from the truck itself.
namespace truckpuppet {
namespace {

// The code it relies on: ets2_code.h (its bytes, and where it was in the versions seen).
using namespace ets2code;
constexpr int kSlot = 3;              // DR3 (camhook uses DR2, the research probe DR0/DR1)
constexpr double kSector = 512.0;
constexpr int kMaxStates = 8;
constexpr double kMaxStep = 3.0;      // m per frame the truck may move towards its goal (catch-up)
constexpr double kGiveUp = 2000.0;    // m: further than this from its goal, stop for good
constexpr double kCarOriginAboveRoad = 0.5;   // m: BeamNG car origin over the road surface
constexpr double kPlausible = 15.0;   // m between the chassis state and the telemetry
constexpr size_t kTrail = 2000;       // car track samples, every 0.5 m (~1 km)

Settings g_set;
uintptr_t g_base = 0;
uintptr_t g_thunk = 0, g_getter = 0;   // found in this game version
int32_t g_placementSlot = 0;           // setPlacement's offset in the actor vtable (0x110)
uintptr_t g_actorVt = 0;               // the body states' actor class, learned from the player truck
codescan::Range g_text, g_rdata;
bool g_checked = false, g_codeOk = false;
uintptr_t g_pc = 0;
std::atomic<uintptr_t> g_found{0};
PVOID g_veh = nullptr;
uintptr_t g_truck = 0;
bool g_active = false;
bool g_far = false;                   // the car is too far from the truck: waiting for it
uint64_t g_lastUs = 0, g_lastFail = 0;
EtsPlacement g_lastGoal{};
bool g_haveLast = false;
std::deque<EtsPlacement> g_trail;
constexpr uint64_t kHoldMs = 60000;   // gaps up to this long: the truck is held, not let go

bool SafeCopy(void* dst, const void* src, size_t n) {
    __try { memcpy(dst, src, n); return true; }
    __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}
template <class T> bool Read(uintptr_t a, T& v) { return SafeCopy(&v, reinterpret_cast<const void*>(a), sizeof(T)); }

double Dist2(const Vec3d& a, const Vec3d& b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
}

LONG CALLBACK Veh(EXCEPTION_POINTERS* e) {
    if (e->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT* c = e->ContextRecord;
    if (!(c->Dr6 & (1ull << kSlot)) || !g_pc || c->Rip != g_pc) return EXCEPTION_CONTINUE_SEARCH;
    c->Dr6 = 0;
    c->EFlags |= 0x10000;  // RF
    uint64_t getter = 0;
    if (Read(c->Rax + 0x28, getter) && getter == g_getter) {
        g_found.store(c->Rcx);
        c->Dr7 &= ~(3ull << (kSlot * 2));
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

uintptr_t g_req = 0;
DWORD WINAPI SetAllThreads(LPVOID) {
    const uintptr_t addr = g_req;
    const DWORD self = GetCurrentThreadId(), pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return 0;
    THREADENTRY32 te{};
    te.dwSize = sizeof(te);
    for (BOOL ok = Thread32First(snap, &te); ok; ok = Thread32Next(snap, &te)) {
        if (te.th32OwnerProcessID != pid || te.th32ThreadID == self) continue;
        HANDLE th = OpenThread(THREAD_GET_CONTEXT | THREAD_SET_CONTEXT | THREAD_SUSPEND_RESUME, FALSE, te.th32ThreadID);
        if (!th) continue;
        if (SuspendThread(th) != DWORD(-1)) {
            CONTEXT c{};
            c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
            if (GetThreadContext(th, &c)) {
                c.Dr3 = addr;
                const uint64_t mask = (3ull << (kSlot * 2)) | (0xFull << (16 + kSlot * 4));
                c.Dr7 &= ~mask;
                if (addr) c.Dr7 |= 1ull << (kSlot * 2);
                SetThreadContext(th, &c);
            }
            ResumeThread(th);
        }
        CloseHandle(th);
    }
    CloseHandle(snap);
    return 0;
}
void SetAll(uintptr_t addr) {
    g_req = addr;
    HANDLE h = CreateThread(nullptr, 0, SetAllThreads, nullptr, 0, nullptr);
    if (h) { WaitForSingleObject(h, 5000); CloseHandle(h); }
}
void Disarm() { if (g_pc) { SetAll(0); g_pc = 0; } }

// ---- quaternions (w, x, y, z); ETS2: Y up, yaw about +Y, pitch about +X, roll about +Z ----
struct Q { double w, x, y, z; };
Q FromPlacement(const EtsPlacement& o) {
    const double hy = o.heading * kPi, hp = o.pitch * kPi, hr = o.roll * kPi;
    const double cy = std::cos(hy), sy = std::sin(hy), cp = std::cos(hp), sp = std::sin(hp),
                 cr = std::cos(hr), sr = std::sin(hr);
    const double aw = cy * cp, ax = cy * sp, ay = sy * cp, az = -sy * sp;
    return {aw * cr - az * sr, ax * cr + ay * sr, ay * cr - ax * sr, az * cr + aw * sr};
}
Q Conj(const Q& q) { return {q.w, -q.x, -q.y, -q.z}; }
Q Mul(const Q& a, const Q& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Vec3d Rotate(const Q& q, const Vec3d& v) {
    const Vec3d u{q.x, q.y, q.z};
    const Vec3d t{2 * (u.y * v.z - u.z * v.y), 2 * (u.z * v.x - u.x * v.z), 2 * (u.x * v.y - u.y * v.x)};
    return {v.x + q.w * t.x + (u.y * t.z - u.z * t.y), v.y + q.w * t.y + (u.z * t.x - u.x * t.z),
            v.z + q.w * t.z + (u.x * t.y - u.y * t.x)};
}

// One body state of the truck (see the header comment).
struct State {
    uintptr_t s = 0;
    uintptr_t vt = 0;   // its physics actor's vtable
    Vec3d pos{};   // world
    Q q{};
};

// A physics actor's class: the one learned from the player truck, or (before that) a vtable in the
// game's .rdata whose setPlacement slot holds a function starting like setPlacement.
bool ActorClassOk(uintptr_t vt) {
    if (g_actorVt) return vt == g_actorVt;
    uintptr_t fn = 0;
    return g_rdata.Has(vt) && Read(vt + g_placementSlot, fn) && g_text.Has(fn) && codescan::Matches(fn, kSetPlacementSig);
}
// The poses last written (held through short gaps, see Update).
Vec3d g_heldPos[kMaxStates];
Q g_heldQ[kMaxStates];
int g_heldN = 0;
uint64_t g_lastWriteMs = 0;
#pragma pack(push, 1)
struct RawPlacement { float x, y, z; int16_t sx, sz; float qw, qx, qy, qz; };
#pragma pack(pop)
static_assert(sizeof(RawPlacement) == 0x20, "placement is 0x20 bytes");

bool ReadStates(uintptr_t truck, State* out, int& n) {
    uintptr_t arr = 0;
    uint32_t count = 0;
    if (!Read(truck + 0x218, arr) || !arr || !Read(truck + 0x220, count) || count == 0 || count > kMaxStates) return false;
    n = 0;
    for (uint32_t i = 0; i < count; ++i) {
        uintptr_t s = 0, actor = 0, vt = 0;
        if (!Read(arr + 8 * i, s) || !s || !Read(s, actor) || !actor || !Read(actor, vt) || !ActorClassOk(vt) ||
            (n > 0 && vt != out[0].vt))
            return false;
        out[n].vt = vt;
        RawPlacement p;
        if (!Read(s + 0x10, p)) return false;
        const float f[7] = {p.x, p.y, p.z, p.qw, p.qx, p.qy, p.qz};
        for (float v : f) if (!std::isfinite(v)) return false;
        out[n].s = s;
        out[n].pos = {p.sx * kSector + p.x, p.y, p.sz * kSector + p.z};
        out[n].q = {p.qw, p.qx, p.qy, p.qz};
        ++n;
    }
    return n > 0;
}

// The chassis state is where the telemetry says the truck is, turned the same way.
bool Plausible(const State& st, const EtsTruckState& truck) {
    if (Dist2(st.pos, truck.placement.pos) > kPlausible * kPlausible) return false;
    const Q t = FromPlacement(truck.placement);
    const double dot = t.w * st.q.w + t.x * st.q.x + t.y * st.q.y + t.z * st.q.z;
    return std::fabs(dot) > 0.9;
}

bool WriteState(uintptr_t s, const Vec3d& pos, const Q& q, const float lin[3], const float ang[3]) {
    RawPlacement p;
    const double sx = std::floor(pos.x / kSector), sz = std::floor(pos.z / kSector);
    p.sx = int16_t(sx);
    p.sz = int16_t(sz);
    p.x = float(pos.x - sx * kSector);
    p.y = float(pos.y);
    p.z = float(pos.z - sz * kSector);
    p.qw = float(q.w); p.qx = float(q.x); p.qy = float(q.y); p.qz = float(q.z);
    uint8_t* b = reinterpret_cast<uint8_t*>(s);
    return SafeCopy(b + 0x10, &p, sizeof(p)) && SafeCopy(b + 0x48, &p, sizeof(p)) &&
           SafeCopy(b + 0x30, lin, 12) && SafeCopy(b + 0x3c, ang, 12);
}

const char* Stop(const char* why) {
    g_set.enabled = false;
    g_truck = 0;
    g_active = false;
    Disarm();
    return why;
}

}  // namespace

void Configure(const Settings& s) { g_set = s; }
bool Active() { return g_active; }
uintptr_t TruckObject() { return g_truck; }

const char* Update(Shared*, bool want, const EtsTruckState& truck, const EtsTarget* target) {
    static char msg[256];
    if (!g_checked) {
        g_checked = true;
        g_base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
        g_text = codescan::Section(".text");
        g_rdata = codescan::Section(".rdata");
        const char* missing = nullptr;
        // The body state loop, with the states array before it and the setPlacement call after it.
        const uintptr_t loop = codescan::Locate(kLoopSig, kLoopKnown, Count(kLoopKnown));
        const uintptr_t call = loop ? codescan::FindUnique(kCallSig, loop, loop + 0x180) : 0;
        if (!loop) missing = "body state loop";
        else if (!codescan::FindUnique(kStatesSig, loop - 0x180, loop)) missing = "body states array";
        else if (!call || !Read(call + 15, g_placementSlot) || g_placementSlot <= 0 || g_placementSlot > 0x1000)
            missing = "setPlacement call";
        const uintptr_t thunkSig = codescan::Locate(kThunkSig, kThunkKnown, Count(kThunkKnown));
        g_thunk = thunkSig ? thunkSig + kThunkCall : 0;
        g_getter = codescan::Locate(kGetterSig, kGetterKnown, Count(kGetterKnown));
        if (!missing && !g_thunk) missing = "telemetry thunk";
        if (!missing && !g_getter) missing = "placement getter";
        g_codeOk = missing == nullptr;
        if (!g_codeOk) {
            snprintf(msg, sizeof(msg), "truck puppet unavailable: this ETS2 version changed the truck code (%s not found; plugin update needed)", missing);
            return msg;
        }
        snprintf(msg, sizeof(msg), "truck puppet: game code at +0x%llx (states), +0x%llx (telemetry), actor slot +0x%x",
                 (unsigned long long)(loop - g_base), (unsigned long long)(g_thunk - g_base), unsigned(g_placementSlot));
        return msg;
    }
    if (!g_codeOk || !g_set.enabled) return nullptr;
    want = want && target && truck.hasPlacement && !truck.paused;
    // A short gap (the bridge or BeamNG busy, loading roads): the truck is held where it was.
    // Let go, ETS2 puts it back where its own simulation had it - kilometres behind the car.
    if (!want && g_active && g_truck && g_heldN > 0 && GetTickCount64() - g_lastWriteMs < kHoldMs) {
        State st[kMaxStates];
        int n = 0;
        if (ReadStates(g_truck, st, n) && n == g_heldN) {
            const float zero[3] = {0, 0, 0};
            for (int i = 0; i < n; ++i) WriteState(st[i].s, g_heldPos[i], g_heldQ[i], zero, zero);
            g_haveLast = false;
            return nullptr;
        }
    }
    if (!want) {
        Disarm();
        g_haveLast = false;
        g_trail.clear();
        if (g_active) { g_active = false; return "truck puppet off: the truck stays where it is"; }
        return nullptr;
    }

    // Find the truck object (once, or again when it stops making sense).
    if (!g_truck) {
        if (const uintptr_t f = g_found.exchange(0)) {
            Disarm();
            State st[kMaxStates];
            int n = 0;
            if (ReadStates(f, st, n) && Plausible(st[0], truck)) {
                g_truck = f;
                g_actorVt = st[0].vt;   // the player truck's: from now on only this class
            } else {
                const uint64_t now = GetTickCount64();
                if (now - g_lastFail > 10000) {
                    g_lastFail = now;
                    return "truck puppet: found an object that is not the player truck, retrying";
                }
            }
        }
        if (!g_truck) {
            if (!g_pc) {
                if (!g_veh) g_veh = AddVectoredExceptionHandler(1, Veh);
                g_pc = g_thunk;
                SetAll(g_pc);
            }
            return nullptr;
        }
    }

    State st[kMaxStates];
    int n = 0;
    if (!ReadStates(g_truck, st, n)) { g_truck = 0; g_active = false; return "truck puppet: lost the truck, searching again"; }
    if (!Plausible(st[0], truck)) return Stop("truck puppet stopped: the truck state no longer matches the telemetry (safety stop)");

    // Where the truck goes: on the car's own track, `back` metres behind it, on the road and on
    // its wheels - a normally driving truck (in the air or underground ETS2 rescues it as stuck).
    {
        const EtsPlacement& cp = target->placement;
        if (!g_trail.empty() && Dist2(g_trail.back().pos, cp.pos) > 30.0 * 30.0) g_trail.clear();   // car was moved
        if (g_trail.empty() || Dist2(g_trail.back().pos, cp.pos) > 0.5 * 0.5) {
            g_trail.push_back(cp);
            if (g_trail.size() > kTrail) g_trail.pop_front();
        }
    }
    const EtsPlacement* behind = nullptr;
    for (auto it = g_trail.rbegin(); it != g_trail.rend(); ++it)
        if (Dist2(it->pos, target->placement.pos) >= g_set.back * g_set.back) { behind = &*it; break; }
    if (!behind) { g_haveLast = false; return nullptr; }   // not far enough yet / reversing: leave it
    EtsPlacement goal = *behind;


    // The goal's own motion: the truck gets the car's velocity, so ETS2 moves it smoothly.
    float lin[3] = {0, 0, 0}, ang[3] = {0, 0, 0};
    const uint64_t nowUs = NowUs();
    double catchUpDt = 0;
    if (g_lastUs && nowUs > g_lastUs) {
        const double fdt = (nowUs - g_lastUs) * 1e-6;
        if (fdt > 0.001 && fdt < 0.25) catchUpDt = fdt;
    }
    if (g_haveLast && nowUs > g_lastUs) {
        const double dt = (nowUs - g_lastUs) * 1e-6;
        if (dt > 0.001 && dt < 0.25) {
            const double vx = (goal.pos.x - g_lastGoal.pos.x) / dt, vy = (goal.pos.y - g_lastGoal.pos.y) / dt,
                         vz = (goal.pos.z - g_lastGoal.pos.z) / dt;
            const double wy = WrapTurnsHalf(goal.heading - g_lastGoal.heading) * kTau / dt;
            if (vx * vx + vy * vy + vz * vz < 100.0 * 100.0) { lin[0] = float(vx); lin[1] = float(vy); lin[2] = float(vz); }
            if (std::fabs(wy) < 3) ang[1] = float(wy);
        }
    }
    g_lastGoal = goal;
    g_lastUs = nowUs;
    g_haveLast = true;

    // New chassis pose: the goal, keeping the chassis' offset from the truck's origin (telemetry).
    // Fully ours: at the car (or `back` m behind it on its track), `height` m above the road,
    // upright, facing the car's way, moving with the car. It touches nothing, so ETS2's physics
    // cannot throw it around (sideways pushes against the road flipped it and damaged it).
    const double roadY = goal.pos.y - kCarOriginAboveRoad;
    const double chassisAboveOrigin = std::clamp(st[0].pos.y - truck.placement.pos.y, -1.0, 3.0);
    EtsPlacement flat = goal;
    flat.pitch = 0;
    flat.roll = 0;
    const Q nq = FromPlacement(flat);
    const Q rot = Mul(nq, Conj(st[0].q));   // turns every state from its current to the new pose
    const double dyaw = WrapTurnsHalf(goal.heading - truck.placement.heading) * kTau;
    const Q yawOnly{std::cos(dyaw / 2), 0.0, std::sin(dyaw / 2), 0.0};
    const Vec3d off0 = Rotate(yawOnly, Vec3d{st[0].pos.x - truck.placement.pos.x, 0.0, st[0].pos.z - truck.placement.pos.z});
    const Vec3d chassisGoal{goal.pos.x + off0.x, roadY + g_set.height + chassisAboveOrigin, goal.pos.z + off0.z};
    // Far from its goal (car reset, start): never jump sideways - ETS2 hung after big jumps.
    // Catch up at most kMaxStep per frame on the ground plane; height snaps.
    Vec3d chassisNew = chassisGoal;
    const double gx = chassisGoal.x - st[0].pos.x, gz = chassisGoal.z - st[0].pos.z;
    const double gap = std::sqrt(gx * gx + gz * gz);
    if (gap > kGiveUp) {
        // The car is far from the truck: a new anchor (bridge restarted) or a reset, and the car is
        // put back near the truck next. Never jump the truck there - wait, it follows again once
        // the car is near. (This used to stop the puppet for the whole ETS2 session.)
        g_haveLast = false;
        if (!g_far) {
            g_far = true;
            snprintf(msg, sizeof(msg), "truck puppet waiting: the car is %.0f m from the truck (moved or reset) - it follows again when the car is near", gap);
            return msg;
        }
        return nullptr;
    }
    if (g_far) {
        g_far = false;
        snprintf(msg, sizeof(msg), "truck puppet: the car is near again (%.0f m), following", gap);
        return msg;   // (writes from the next frame)
    }
    if (gap > kMaxStep) {
        const double f = kMaxStep / gap;
        chassisNew.x = st[0].pos.x + gx * f;
        chassisNew.z = st[0].pos.z + gz * f;
        if (catchUpDt > 0) { lin[0] = float(gx * f / catchUpDt); lin[2] = float(gz * f / catchUpDt); }
    }
    lin[1] = 0;
    ang[0] = ang[2] = 0;
    for (int i = 0; i < n; ++i) {
        const Vec3d rel = Rotate(rot, Vec3d{st[i].pos.x - st[0].pos.x, st[i].pos.y - st[0].pos.y, st[i].pos.z - st[0].pos.z});
        const Vec3d p{chassisNew.x + rel.x, chassisNew.y + rel.y, chassisNew.z + rel.z};
        if (!WriteState(st[i].s, p, Mul(rot, st[i].q), lin, ang)) return Stop("truck puppet stopped: write failed (safety stop)");
        g_heldPos[i] = p;
        g_heldQ[i] = Mul(rot, st[i].q);
    }
    g_heldN = n;
    g_lastWriteMs = GetTickCount64();
    if (!g_active) {
        g_active = true;
        snprintf(msg, sizeof(msg), "truck puppet on: %d bodies, the truck rides %.1f m above the road, %.0f m behind the car", n, g_set.height, g_set.back);
        return msg;
    }
    return nullptr;
}

void Shutdown() {
    Disarm();
    if (g_veh) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
}

}  // namespace truckpuppet
