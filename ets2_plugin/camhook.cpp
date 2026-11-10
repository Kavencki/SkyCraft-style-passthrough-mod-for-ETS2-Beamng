#define WIN32_LEAN_AND_MEAN
#include "camhook.h"

#include <windows.h>
#include <tlhelp32.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../protocol/mapping.h"
#include "codescan.h"
#include "ets2_code.h"

#pragma comment(lib, "user32.lib")

using namespace es;

namespace camhook {
namespace {

// The camera's placement read (ets2_code.h: its bytes and where it was in the versions seen).
constexpr int       kSlot = 2;                                    // DR2 (DR0/DR1 belong to the dev probe)
constexpr double    kSector = 512.0;

uintptr_t g_pc = 0;           // armed instruction address, 0 = disarmed
uintptr_t g_hook = 0;         // the placement read in this game version
bool      g_checked = false, g_codeOk = false;
Shared*   g_sh = nullptr;
PVOID     g_veh = nullptr;
std::atomic<uint32_t> g_applied{0};
int    g_fovMode = 1;      // 0 off, 1 vertical (as BeamNG), 2 horizontal (converted with g_aspect)
double g_aspect = 0;       // ETS2 picture width / height, refreshed every frame

bool SafeCopy(void* dst, const void* src, size_t n) {
    __try { memcpy(dst, src, n); return true; }
    __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

void Quat(const EtsPlacement& o, float q[4]) {
    // ETS2: Y up, -Z forward. yaw about +Y (CCW from above), pitch about +X (+ = nose up),
    // roll about +Z. Stored w, x, y, z.
    const double hy = o.heading * kPi, hp = o.pitch * kPi, hr = o.roll * kPi;  // half angles
    const double cy = std::cos(hy), sy = std::sin(hy), cp = std::cos(hp), sp = std::sin(hp),
                 cr = std::cos(hr), sr = std::sin(hr);
    const double aw = cy * cp, ax = cy * sp, ay = sy * cp, az = -sy * sp;  // yaw * pitch
    q[0] = float(aw * cr - az * sr);
    q[1] = float(ax * cr + ay * sr);
    q[2] = float(ay * cr - ax * sr);
    q[3] = float(az * cr + aw * sr);
}

// ---- smoothing: BeamNG poses arrive at BeamNG's frame rate, ETS2 renders at its own.
// Keep recent samples and render a little in the past, interpolating between them.
struct Sample { EtsPlacement p; uint64_t t; };
constexpr int kRing = 16;
Sample   g_ring[kRing];
int      g_count = 0, g_head = 0;           // g_head = index of the newest
uint64_t g_lastSampleUs = 0;
double   g_intervalUs = 16000;              // smoothed spacing between samples

double LerpTurns(double a, double b, double t) {
    double d = WrapTurnsHalf(b - a);        // shortest way around
    return a + d * t;
}

EtsPlacement Lerp(const EtsPlacement& a, const EtsPlacement& b, double t) {
    EtsPlacement r;
    r.pos = {a.pos.x + (b.pos.x - a.pos.x) * t, a.pos.y + (b.pos.y - a.pos.y) * t, a.pos.z + (b.pos.z - a.pos.z) * t};
    r.heading = WrapTurns01(LerpTurns(a.heading, b.heading, t));
    r.pitch = a.pitch + (b.pitch - a.pitch) * t;
    r.roll = LerpTurns(a.roll, b.roll, t);
    return r;
}

void PushSample(const EtsCamTarget& cam) {
    if (cam.sampleUs == 0 || cam.sampleUs <= g_lastSampleUs) return;
    if (g_lastSampleUs) {
        const double dt = double(cam.sampleUs - g_lastSampleUs);
        if (dt < 200000) g_intervalUs += (dt - g_intervalUs) * 0.1;
        else g_count = 0;                   // long gap (pause, teleport): start fresh
    }
    g_lastSampleUs = cam.sampleUs;
    g_head = (g_head + 1) % kRing;
    g_ring[g_head] = {cam.placement, cam.sampleUs};
    if (g_count < kRing) ++g_count;
}

// Pose to show now: interpolated ~1.5 sample intervals in the past.
EtsPlacement Smoothed(const EtsCamTarget& latest) {
    if (g_count < 2) return latest.placement;
    double delay = g_intervalUs * 1.5 + 2000;
    delay = delay < 8000 ? 8000 : delay > 60000 ? 60000 : delay;
    const uint64_t now = NowUs();
    const uint64_t t = now > uint64_t(delay) ? now - uint64_t(delay) : 0;
    // Walk from newest to oldest to find the pair around t.
    for (int k = 0; k < g_count - 1; ++k) {
        const Sample& b = g_ring[(g_head - k + kRing) % kRing];
        const Sample& a = g_ring[(g_head - k - 1 + kRing) % kRing];
        if (t >= a.t) {
            if (t >= b.t) return b.p;       // newer than everything we have: hold the newest
            return Lerp(a.p, b.p, double(t - a.t) / double(b.t - a.t));
        }
    }
    return g_ring[(g_head - (g_count - 1) + kRing) % kRing].p;  // older than our history
}

LONG CALLBACK Veh(EXCEPTION_POINTERS* e) {
    if (e->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT* c = e->ContextRecord;
    if (!(c->Dr6 & (1ull << kSlot)) || !g_pc || c->Rip != g_pc) return EXCEPTION_CONTINUE_SEARCH;
    c->Dr6 = 0;
    c->EFlags |= 0x10000;  // RF: run the instruction without re-triggering

    EtsCamTarget cam{};
    if (!g_sh || !g_sh->cam.read(cam) || !cam.valid) return EXCEPTION_CONTINUE_EXECUTION;
    PushSample(cam);
    const EtsPlacement pose = Smoothed(cam);

    uint8_t* obj = reinterpret_cast<uint8_t*>(c->Rbx);
    int16_t s[2];
    if (!SafeCopy(s, obj + 0x4c, 4)) return EXCEPTION_CONTINUE_EXECUTION;
    const double x = pose.pos.x, y = pose.pos.y, z = pose.pos.z;
    // Keep the game's sector while the offset stays small; re-sector on long moves.
    if (std::fabs(x - s[0] * kSector) > 384.0 || std::fabs(z - s[1] * kSector) > 384.0) {
        s[0] = int16_t(std::floor(x / kSector));
        s[1] = int16_t(std::floor(z / kSector));
    }
    uint8_t block[32];
    const float local[3] = {float(x - s[0] * kSector), float(y), float(z - s[1] * kSector)};
    memcpy(block, local, 12);
    memcpy(block + 12, s, 4);
    float q[4];
    Quat(pose, q);
    memcpy(block + 16, q, 16);
    if (SafeCopy(obj + 0x40, block, sizeof(block))) g_applied.fetch_add(1);
    // Field of view (+0x20) from the physics game's camera, so overlaid cabins line up.
    if (g_fovMode != 0 && cam.fovDeg > 10 && cam.fovDeg < 150) {
        float fov = cam.fovDeg;  // BeamNG: vertical degrees
        if (g_fovMode == 2 && g_aspect > 0) {
            const double v = cam.fovDeg * kPi / 180.0;
            fov = float(2.0 * std::atan(std::tan(v / 2) * g_aspect) * 180.0 / kPi);
        }
        SafeCopy(obj + 0x20, &fov, sizeof(fov));
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

struct Req { uintptr_t addr; };
Req g_req{};

DWORD WINAPI SetAllThreads(LPVOID) {
    const uintptr_t addr = g_req.addr;
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
                c.Dr2 = addr;
                const uint64_t mask = (3ull << (kSlot * 2)) | (0xFull << (16 + kSlot * 4));
                c.Dr7 &= ~mask;
                if (addr) c.Dr7 |= 1ull << (kSlot * 2);  // local enable; RW=00 execute, LEN=00
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
    g_req.addr = addr;
    HANDLE h = CreateThread(nullptr, 0, SetAllThreads, nullptr, 0, nullptr);
    if (h) { WaitForSingleObject(h, 5000); CloseHandle(h); }
}

uintptr_t HookAddress() { return g_hook; }

}  // namespace

void Configure(int fovMode) { g_fovMode = fovMode; }

const char* Update(Shared* sh, bool want) {
    g_sh = sh;
    // ETS2's picture aspect (for the horizontal FOV mode).
    if (HWND fg = GetForegroundWindow()) {
        DWORD pid = 0;
        GetWindowThreadProcessId(fg, &pid);
        RECT r{};
        if (pid == GetCurrentProcessId() && GetClientRect(fg, &r) && r.bottom > 0)
            g_aspect = double(r.right) / double(r.bottom);
    }
    if (!g_checked) {
        g_checked = true;
        g_hook = ets2code::FindCamera();
        g_codeOk = g_hook != 0;
        if (!g_codeOk) return "camera driver unavailable: this ETS2 version changed the camera code (plugin update needed)";
        static char found[96];
        snprintf(found, sizeof(found), "camera driver: game code at +0x%llx",
                 (unsigned long long)(g_hook - reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr))));
        return found;
    }
    if (!g_codeOk) return nullptr;
    if (want && !g_pc) {
        if (!g_veh) g_veh = AddVectoredExceptionHandler(1, Veh);
        g_pc = HookAddress();
        SetAll(g_pc);
        return "camera driver on: ETS2 renders from the physics car";
    }
    if (!want && g_pc) {
        SetAll(0);
        g_pc = 0;
        return "camera driver off: ETS2 camera is back to normal";
    }
    return nullptr;
}

uint32_t Applied() { return g_applied.load(); }

void Shutdown() {
    if (g_pc) { SetAll(0); g_pc = 0; }
    if (g_veh) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
}

}  // namespace camhook
