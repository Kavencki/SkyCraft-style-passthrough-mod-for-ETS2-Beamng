#define WIN32_LEAN_AND_MEAN
#include "probe.h"
#include "../protocol/mapping.h"

#include <windows.h>
#include <tlhelp32.h>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

using namespace es;

namespace probe {
namespace {

std::vector<Candidate> g_cands;
std::atomic<bool> g_scanRunning{false};
std::vector<Candidate> g_scanOut;
uint64_t g_scanMs = 0, g_scanBytes = 0;

uint32_t g_lastSeq = 0;
uint32_t g_activeSeq = 0;   // command waiting for completion (scan / poke)
uint32_t g_activeOp = DEV_NONE;

// Poke state
int       g_pokeIdx = -1;
int       g_pokeFramesLeft = 0;
double    g_pokeOrig[3], g_pokeDelta[3];
EtsPlacement g_pokeBefore{};

// Snapshot/diff state
constexpr size_t kWin = 512;
std::vector<Candidate> g_rotHits;
std::vector<uint8_t> g_snap;        // kWin bytes per rotation hit
std::vector<uint8_t> g_snapOk;
EtsPlacement g_snapTruck{};

// Autopoke state
size_t g_apIdx = 0;
int    g_apFrame = 0, g_apFrames = 4;
bool   g_apOk = false;
double g_apOrig[3];
EtsPlacement g_apBefore{};
std::vector<Candidate> g_apHits;

// Remembered values (parallel to g_cands) for KEEP filtering.
struct Val3 { double v[3]; bool ok; };
std::vector<Val3> g_mem;

// Puppet state (relative: value = v0 + (target - t0), shifted on origin re-base)
int g_puppetIdx = -1;
bool g_puppetCam = false;
bool g_puppetStarted = false;
double g_puppetV0[3], g_puppetLast[3];
Vec3d g_puppetT0{};
uint32_t g_puppetWrites = 0, g_puppetRebases = 0;

// ---- hardware write-watchpoint (DR0) ----
struct WatchHit {
    uint64_t rip;        // instruction after the write (data breakpoints trap after executing)
    uint64_t regs[16];   // rax rcx rdx rbx rsp rbp rsi rdi r8..r15
    uint32_t count;
    uint32_t tid;
    uint64_t stack[64];  // [rsp .. rsp+0x200] at the first hit (return addresses)
};
constexpr int kMaxHits = 8;
WatchHit g_hits[kMaxHits];
std::atomic<int> g_nHits{0};
std::atomic<long> g_hitLock{0};
std::atomic<uint32_t> g_totalHits{0};
PVOID g_veh = nullptr;
uintptr_t g_watchAddr = 0;
uint64_t g_watchUntil = 0;
std::atomic<bool> g_watchThreadDone{true};
bool g_watchExec = false;  // DR0 as an execute breakpoint (DEV_EXEC) instead of a write watch
int g_filterReg = -1;      // DEV_EXEC: record only hits where this register == g_filterVal
uint64_t g_filterVal = 0;

bool SafeCopy(void* dst, const void* src, size_t n);

LONG CALLBACK WatchVeh(EXCEPTION_POINTERS* e) {
    if (e->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT* c = e->ContextRecord;
    if (!(c->Dr6 & 1)) return EXCEPTION_CONTINUE_SEARCH;
    c->Dr6 = 0;
    c->EFlags |= 0x10000;  // RF: an execute breakpoint must not re-trigger on the same instruction
    if (g_filterReg >= 0) {
        const uint64_t fr[16] = {c->Rax, c->Rcx, c->Rdx, c->Rbx, c->Rsp, c->Rbp, c->Rsi, c->Rdi,
                                 c->R8,  c->R9,  c->R10, c->R11, c->R12, c->R13, c->R14, c->R15};
        if (fr[g_filterReg] != g_filterVal) return EXCEPTION_CONTINUE_EXECUTION;
    }
    g_totalHits.fetch_add(1);
    long expected = 0;
    if (g_hitLock.compare_exchange_strong(expected, 1)) {
        int n = g_nHits.load();
        int i = 0;
        while (i < n && g_hits[i].rip != c->Rip) ++i;
        if (i < n) {
            ++g_hits[i].count;
        } else if (n < kMaxHits) {
            WatchHit& h = g_hits[n];
            h.rip = c->Rip;
            const uint64_t r[16] = {c->Rax, c->Rcx, c->Rdx, c->Rbx, c->Rsp, c->Rbp, c->Rsi, c->Rdi,
                                    c->R8,  c->R9,  c->R10, c->R11, c->R12, c->R13, c->R14, c->R15};
            memcpy(h.regs, r, sizeof(r));
            h.count = 1;
            h.tid = GetCurrentThreadId();
            if (!SafeCopy(h.stack, reinterpret_cast<const void*>(c->Rsp), sizeof(h.stack))) memset(h.stack, 0, sizeof(h.stack));
            g_nHits.store(n + 1);
        }
        g_hitLock.store(0);
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

bool SafeCopy(void* dst, const void* src, size_t n);

// ---- camera hook: execute breakpoint (DR1) ----
uintptr_t g_hookPc = 0;          // instruction address
uintptr_t g_hookDst = 0;         // unused (kept for the report)
volatile uintptr_t g_hookRdi = 0; // learned camera object
double    g_hookRef[3] = {};     // camera world position at arm time
bool      g_hookRotate = false;  // rotation layout not confirmed yet: position only
uint32_t  g_hookSrcOff = 0, g_hookDstOff = 0;
std::atomic<uint32_t> g_hookFires{0}, g_hookApplied{0};
Shared* g_hookShared = nullptr;
PVOID g_hookVeh = nullptr;

LONG CALLBACK HookVeh(EXCEPTION_POINTERS* e) {
    if (e->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT* c = e->ContextRecord;
    if (!(c->Dr6 & 2) || c->Rip != g_hookPc) return EXCEPTION_CONTINUE_SEARCH;
    c->Dr6 = 0;
    c->EFlags |= 0x10000;  // RF: execute the instruction without re-triggering
    g_hookFires.fetch_add(1);
    uint8_t* src = reinterpret_cast<uint8_t*>(c->Rsi + g_hookSrcOff);
    // Learn the camera object: the first copy whose source decodes to the reference position.
    if (!g_hookRdi) {
        uint8_t b[16];
        if (SafeCopy(b, src, 16)) {
            float f[3];
            int16_t s[2];
            memcpy(f, b, 12);
            memcpy(s, b + 12, 4);
            const double wx = s[0] * kSectorSize + f[0], wz = s[1] * kSectorSize + f[2];
            if (std::fabs(wx - g_hookRef[0]) < 1.0 && std::fabs(f[1] - g_hookRef[1]) < 1.0 &&
                std::fabs(wz - g_hookRef[2]) < 1.0)
                g_hookRdi = c->Rdi;
        }
    }
    if (g_hookShared && g_hookRdi && c->Rdi == g_hookRdi) {
        EtsCamTarget cam{};
        if (g_hookShared->cam.read(cam) && cam.valid) {
            // Source block: float x,y,z then int16 sectorX, sectorZ. Keep the sector, write local.
            int16_t s[2];
            if (SafeCopy(s, src + 12, 4)) {
                const float f[3] = {float(cam.placement.pos.x - s[0] * kSectorSize), float(cam.placement.pos.y),
                                    float(cam.placement.pos.z - s[1] * kSectorSize)};
                if (SafeCopy(src, f, sizeof(f))) g_hookApplied.fetch_add(1);
                // Rotation quaternion (w,x,y,z) follows the position block: yaw(+Y)*pitch(+X)*roll(+Z).
                const EtsPlacement& o = cam.placement;
                const double hy = o.heading * kPi, hp = o.pitch * kPi, hr = o.roll * kPi;
                const double cy = std::cos(hy), sy = std::sin(hy), cp = std::cos(hp), sp = std::sin(hp),
                             cr = std::cos(hr), sr = std::sin(hr);
                const double aw = cy * cp, ax = cy * sp, ay = sy * cp, az = -sy * sp;
                const float q[4] = {float(aw * cr - az * sr), float(ax * cr + ay * sr), float(ay * cr - ax * sr),
                                    float(az * cr + aw * sr)};
                if (g_hookRotate) SafeCopy(src + 16, q, sizeof(q));
            }
        }
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

struct DrRequest { int slot; uintptr_t addr; uint32_t rw; uint32_t len; };
DrRequest g_drReq{};

// Sets (addr != 0) or clears debug register `slot` on every thread except this helper.
DWORD WINAPI SetDrAllThreads(LPVOID) {
    const DrRequest r = g_drReq;
    const DWORD self = GetCurrentThreadId(), pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap != INVALID_HANDLE_VALUE) {
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
                    (&c.Dr0)[r.slot] = r.addr;
                    const int sh = 16 + r.slot * 4;
                    const uint64_t mask = (3ull << (r.slot * 2)) | (0xFull << sh);
                    c.Dr7 &= ~mask;
                    if (r.addr) c.Dr7 |= (1ull << (r.slot * 2)) | (uint64_t(r.rw) << sh) | (uint64_t(r.len) << (sh + 2));
                    c.Dr6 = 0;
                    SetThreadContext(th, &c);
                }
                ResumeThread(th);
            }
            CloseHandle(th);
        }
        CloseHandle(snap);
    }
    return 0;
}

void SetDrSync(int slot, uintptr_t addr, uint32_t rw, uint32_t len) {
    g_drReq = {slot, addr, rw, len};
    HANDLE h = CreateThread(nullptr, 0, SetDrAllThreads, nullptr, 0, nullptr);
    if (h) { WaitForSingleObject(h, 5000); CloseHandle(h); }
}

// Sets (addr != 0) or clears DR0 on every thread except the calling helper thread.
DWORD WINAPI SetWatchAllThreads(LPVOID param) {
    const uintptr_t addr = reinterpret_cast<uintptr_t>(param);
    const DWORD self = GetCurrentThreadId(), pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap != INVALID_HANDLE_VALUE) {
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
                    c.Dr0 = addr;
                    // L0 enable; write watch: RW0 = 01, LEN0 = 11 (4 bytes); execute: RW0 = 00, LEN0 = 00
                    c.Dr7 = addr ? ((c.Dr7 & ~0xF0003ull) | 1ull | (g_watchExec ? 0ull : ((1ull << 16) | (3ull << 18))))
                                 : (c.Dr7 & ~0xF0003ull);
                    c.Dr6 = 0;
                    SetThreadContext(th, &c);
                }
                ResumeThread(th);
            }
            CloseHandle(th);
        }
        CloseHandle(snap);
    }
    g_watchThreadDone.store(true);
    return 0;
}

void StartWatchThread(uintptr_t addr) {
    g_watchThreadDone.store(false);
    HANDLE h = CreateThread(nullptr, 0, SetWatchAllThreads, reinterpret_cast<LPVOID>(addr), 0, nullptr);
    if (h) CloseHandle(h); else g_watchThreadDone.store(true);
}

void Result(Shared* sh, uint32_t seq, int status, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(sh->devResult.text, sizeof(sh->devResult.text), fmt, ap);
    va_end(ap);
    sh->devResult.status = status;
    std::atomic_thread_fence(std::memory_order_release);
    sh->devResult.seq = seq;
}

void Append(char* buf, size_t cap, const char* fmt, ...) {
    size_t n = strlen(buf);
    if (n + 1 >= cap) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf + n, cap - n, fmt, ap);
    va_end(ap);
}

bool SafeCopy(void* dst, const void* src, size_t n) {
    __try { memcpy(dst, src, n); return true; }
    __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

bool Writable(uintptr_t a, size_t n) {
    MEMORY_BASIC_INFORMATION mbi;
    if (!VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi))) return false;
    if (mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS))) return false;
    const DWORD rw = PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    if (!(mbi.Protect & rw)) return false;
    return a + n <= reinterpret_cast<uintptr_t>(mbi.BaseAddress) + mbi.RegionSize;
}

// Scans a region for (x,y,z) as 3 consecutive doubles or floats.
double g_scanTol = 0;

void ScanRegion(const uint8_t* base, size_t size, const double p[3], std::vector<Candidate>& out) {
    __try {
        const double epsD = g_scanTol > 0 ? g_scanTol : 0.01;
        const float  epsF = g_scanTol > 0 ? float(g_scanTol) : 0.05f;
        const float fx = float(p[0]), fy = float(p[1]), fz = float(p[2]);
        for (size_t o = 0; o + 24 <= size; o += 4) {
            if ((o & 7) == 0) {
                const double* d = reinterpret_cast<const double*>(base + o);
                if (std::fabs(d[0] - p[0]) < epsD && std::fabs(d[1] - p[1]) < epsD &&
                    std::fabs(d[2] - p[2]) < epsD)
                    out.push_back({reinterpret_cast<uintptr_t>(base + o), true});
            }
            const float* f = reinterpret_cast<const float*>(base + o);
            if (std::fabs(f[0] - fx) < epsF && std::fabs(f[1] - fy) < epsF && std::fabs(f[2] - fz) < epsF)
                out.push_back({reinterpret_cast<uintptr_t>(base + o), false});
            if (out.size() > 100000) return;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

// Rotation scan. Candidate.isDouble is reused as a kind tag via g_rotKinds.
double g_rotC = 0, g_rotS = 0, g_rotHc = 0, g_rotHs = 0;
bool g_scanRot = false;
std::vector<int> g_rotKinds;  // 3 = mat stride 3, 4 = mat stride 4, 9 = quat

inline bool Near(float v, double t, double tol) { return std::fabs(v - t) < tol; }

void ScanRegionRot(const uint8_t* base, size_t size, std::vector<Candidate>& out, std::vector<int>& kinds) {
    __try {
        const double tol = 0.03;
        const double ac = std::fabs(g_rotC), as = std::fabs(g_rotS);
        const double ahc = std::fabs(g_rotHc), ahs = std::fabs(g_rotHs);
        for (size_t o = 0; o + 48 <= size; o += 4) {
            const float* f = reinterpret_cast<const float*>(base + o);
            // Quaternion (x,y,z,w) yaw-only: x,z ~ 0, |y| = sin(h/2), |w| = cos(h/2).
            if (std::fabs(f[0]) < tol && std::fabs(f[2]) < tol && Near(std::fabs(f[1]), ahs, tol) &&
                Near(std::fabs(f[3]), ahc, tol) && std::fabs(f[1]) > 0.05) {
                out.push_back({reinterpret_cast<uintptr_t>(base + o), false});
                kinds.push_back(9);
            }
            // 3x3 rotation rows (a,~0,b) (~0,~1,~0) (d,~0,e), stride 3 or 4.
            if (!Near(std::fabs(f[0]), ac, tol) || std::fabs(f[1]) > tol || !Near(std::fabs(f[2]), as, tol)) continue;
            for (int k = 3; k <= 4; ++k) {
                const float* r1 = f + k;
                const float* r2 = f + 2 * k;
                if (std::fabs(r1[0]) < tol && Near(r1[1], 1.0, tol) && std::fabs(r1[2]) < tol &&
                    Near(r2[0], -f[2], tol) && std::fabs(r2[1]) < tol && Near(r2[2], f[0], tol)) {
                    out.push_back({reinterpret_cast<uintptr_t>(base + o), false});
                    kinds.push_back(k);
                }
            }
            if (out.size() > 4000) return;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

// Local-frame snapshot: every triple with y near the truck's height and
// plausible local x/z. DEV_DIFF later keeps the ones that moved like the truck.
bool g_scanLocal = false;
double g_localY = 0;
constexpr size_t kLocalMax = 2000000;

void ScanRegionLocal(const uint8_t* base, size_t size, std::vector<Candidate>& out) {
    __try {
        const float ylo = float(g_localY - 4), yhi = float(g_localY + 4);
        for (size_t o = 0; o + 24 <= size && out.size() < kLocalMax; o += 4) {
            const float* f = reinterpret_cast<const float*>(base + o);
            if (f[1] > ylo && f[1] < yhi && std::fabs(f[0]) < 20000.f && std::fabs(f[2]) < 20000.f &&
                (std::fabs(f[0]) > 0.5f || std::fabs(f[2]) > 0.5f))
                out.push_back({reinterpret_cast<uintptr_t>(base + o), false});
            if ((o & 7) == 0) {
                const double* d = reinterpret_cast<const double*>(base + o);
                if (d[1] > ylo && d[1] < yhi && std::fabs(d[0]) < 1e6 && std::fabs(d[2]) < 1e6 &&
                    (std::fabs(d[0]) > 0.5 || std::fabs(d[2]) > 0.5))
                    out.push_back({reinterpret_cast<uintptr_t>(base + o), true});
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

// Sector scan: float x,y,z + int16 sx,sz encoding a world point near the centre.
bool g_scanSector = false;

void ScanRegionSector(const uint8_t* base, size_t size, const double c[3], double tol, std::vector<Candidate>& out) {
    __try {
        for (size_t o = 0; o + 16 <= size; o += 4) {
            const float* f = reinterpret_cast<const float*>(base + o);
            if (std::fabs(f[1] - c[1]) >= tol) continue;  // cheapest test first: height
            if (!(std::fabs(f[0]) < 4096.f) || !(std::fabs(f[2]) < 4096.f)) continue;
            const int16_t* s = reinterpret_cast<const int16_t*>(base + o + 12);
            const double wx = s[0] * kSectorSize + f[0], wz = s[1] * kSectorSize + f[2];
            if (std::fabs(wx - c[0]) < tol && std::fabs(wz - c[2]) < tol) {
                Candidate cand{reinterpret_cast<uintptr_t>(base + o), false};
                cand.isSector = true;
                out.push_back(cand);
                if (out.size() > 10000) return;
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

void ScanThread(double x, double y, double z, uintptr_t skipLo, uintptr_t skipHi) {
    if (g_scanSector) {
        const double c[3] = {x, y, z};
        std::vector<Candidate> out;
        uint64_t t0 = GetTickCount64(), bytes = 0;
        uintptr_t a = 0x10000;
        MEMORY_BASIC_INFORMATION mbi;
        while (a < 0x7FFFFFFF0000ull && VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi))) {
            uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress), next = base + mbi.RegionSize;
            const DWORD rw = PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
            if (mbi.State == MEM_COMMIT && (mbi.Protect & rw) && !(mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) &&
                mbi.Type != MEM_MAPPED && !(base < skipHi && next > skipLo)) {
                ScanRegionSector(reinterpret_cast<const uint8_t*>(base), mbi.RegionSize, c,
                                 g_scanTol > 0 ? g_scanTol : 1.0, out);
                bytes += mbi.RegionSize;
            }
            a = next;
        }
        g_scanOut = std::move(out);
        g_scanMs = GetTickCount64() - t0;
        g_scanBytes = bytes;
        g_scanRunning.store(false, std::memory_order_release);
        return;
    }
    if (g_scanLocal) {
        std::vector<Candidate> out;
        out.reserve(kLocalMax);
        uint64_t t0 = GetTickCount64(), bytes = 0;
        uintptr_t a = 0x10000;
        MEMORY_BASIC_INFORMATION mbi;
        while (a < 0x7FFFFFFF0000ull && VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi))) {
            uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress), next = base + mbi.RegionSize;
            const DWORD rw = PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
            if (mbi.State == MEM_COMMIT && (mbi.Protect & rw) && !(mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) &&
                mbi.Type == MEM_PRIVATE && !(base < skipHi && next > skipLo)) {
                ScanRegionLocal(reinterpret_cast<const uint8_t*>(base), mbi.RegionSize, out);
                bytes += mbi.RegionSize;
            }
            a = next;
        }
        g_scanOut = std::move(out);
        g_scanMs = GetTickCount64() - t0;
        g_scanBytes = bytes;
        g_scanRunning.store(false, std::memory_order_release);
        return;
    }
    if (g_scanRot) {
        std::vector<Candidate> out;
        std::vector<int> kinds;
        uint64_t t0 = GetTickCount64(), bytes = 0;
        uintptr_t a = 0x10000;
        MEMORY_BASIC_INFORMATION mbi;
        while (a < 0x7FFFFFFF0000ull && VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi))) {
            uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress), next = base + mbi.RegionSize;
            const DWORD rw = PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
            if (mbi.State == MEM_COMMIT && (mbi.Protect & rw) && !(mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) &&
                mbi.Type != MEM_MAPPED && !(base < skipHi && next > skipLo)) {
                ScanRegionRot(reinterpret_cast<const uint8_t*>(base), mbi.RegionSize, out, kinds);
                bytes += mbi.RegionSize;
            }
            a = next;
        }
        g_rotKinds = std::move(kinds);
        g_scanOut = std::move(out);
        g_scanMs = GetTickCount64() - t0;
        g_scanBytes = bytes;
        g_scanRunning.store(false, std::memory_order_release);
        return;
    }
    const double p[3] = {x, y, z};
    std::vector<Candidate> out;
    uint64_t t0 = GetTickCount64(), bytes = 0;
    uintptr_t a = 0x10000;
    MEMORY_BASIC_INFORMATION mbi;
    while (a < 0x7FFFFFFF0000ull && VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi))) {
        uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress);
        uintptr_t next = base + mbi.RegionSize;
        const DWORD rw = PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
        bool ok = mbi.State == MEM_COMMIT && (mbi.Protect & rw) && !(mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) &&
                  mbi.Type != MEM_MAPPED && !(base < skipHi && next > skipLo);
        if (ok) {
            ScanRegion(reinterpret_cast<const uint8_t*>(base), mbi.RegionSize, p, out);
            bytes += mbi.RegionSize;
        }
        a = next;
    }
    g_scanOut = std::move(out);
    g_scanMs = GetTickCount64() - t0;
    g_scanBytes = bytes;
    g_scanRunning.store(false, std::memory_order_release);
}

void ListCandidates(char* buf, size_t cap, size_t maxN) {
    for (size_t i = 0; i < g_cands.size() && i < maxN; ++i) {
        double x, y, z;
        bool ok = ReadPos(g_cands[i], x, y, z);
        Append(buf, cap, "  [%zu] %p %s %s (%.3f, %.3f, %.3f)\n", i, (void*)g_cands[i].addr,
               g_cands[i].isSector ? "sec" : g_cands[i].isDouble ? "dbl" : "flt", ok ? "" : "GONE", x, y, z);
    }
}

void OurModuleRange(uintptr_t& lo, uintptr_t& hi) {
    HMODULE m = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCSTR>(&OurModuleRange), &m);
    lo = reinterpret_cast<uintptr_t>(m);
    MEMORY_BASIC_INFORMATION mbi;
    hi = lo;
    // Walk the image's regions.
    uintptr_t a = lo;
    while (VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi)) && mbi.AllocationBase == m) {
        a = reinterpret_cast<uintptr_t>(mbi.BaseAddress) + mbi.RegionSize;
        hi = a;
    }
}

}  // namespace

bool ReadPos(const Candidate& c, double& x, double& y, double& z) {
    x = y = z = 0;
    if (c.isSector) {
        uint8_t b[16];
        if (!SafeCopy(b, reinterpret_cast<void*>(c.addr), sizeof(b))) return false;
        float f[3];
        int16_t s[2];
        memcpy(f, b, 12);
        memcpy(s, b + 12, 4);
        x = s[0] * kSectorSize + f[0];
        y = f[1];
        z = s[1] * kSectorSize + f[2];
        return true;
    }
    if (c.isDouble) {
        double d[3];
        if (!SafeCopy(d, reinterpret_cast<void*>(c.addr), sizeof(d))) return false;
        x = d[0]; y = d[1]; z = d[2];
    } else {
        float f[3];
        if (!SafeCopy(f, reinterpret_cast<void*>(c.addr), sizeof(f))) return false;
        x = f[0]; y = f[1]; z = f[2];
    }
    return true;
}

bool WritePos(const Candidate& c, double x, double y, double z) {
    if (c.isSector) {
        // Keep the game's sector and write the local offset relative to it.
        if (!Writable(c.addr, 16)) return false;
        int16_t s[2];
        if (!SafeCopy(s, reinterpret_cast<void*>(c.addr + 12), 4)) return false;
        float f[3] = {float(x - s[0] * kSectorSize), float(y), float(z - s[1] * kSectorSize)};
        return SafeCopy(reinterpret_cast<void*>(c.addr), f, 12);
    }
    const size_t n = c.isDouble ? 24 : 12;
    if (!Writable(c.addr, n)) return false;
    if (c.isDouble) {
        double d[3] = {x, y, z};
        return SafeCopy(reinterpret_cast<void*>(c.addr), d, n);
    }
    float f[3] = {float(x), float(y), float(z)};
    return SafeCopy(reinterpret_cast<void*>(c.addr), f, n);
}

void Tick(Shared* sh, const EtsTruckState& truck, const EtsTarget* target) {
    // ---- continuing work ----
    if (g_activeOp == DEV_SCAN && !g_scanRunning.load(std::memory_order_acquire)) {
        g_cands = std::move(g_scanOut);
        char* t = sh->devResult.text;
        t[0] = 0;
        Append(t, sizeof(sh->devResult.text), "scan: %zu candidates in %llu KB, %llu ms\n", g_cands.size(),
               g_scanBytes >> 10, g_scanMs);
        if (g_scanRot) {
            // Show what follows each rotation: translation usually sits right after it.
            for (size_t i = 0; i < g_cands.size() && i < 30; ++i) {
                int k = g_rotKinds[i];
                size_t span = k == 9 ? 16 : size_t(k) * 12;
                float after[4] = {};
                double dafter[3] = {};
                SafeCopy(after, reinterpret_cast<void*>(g_cands[i].addr + span), sizeof(after));
                SafeCopy(dafter, reinterpret_cast<void*>((g_cands[i].addr + span + 7) & ~uintptr_t(7)), sizeof(dafter));
                Append(t, sizeof(sh->devResult.text), "  [%zu] %p %s | next f (%.3f, %.3f, %.3f, %.3f) d (%.3f, %.3f, %.3f)\n",
                       i, (void*)g_cands[i].addr, k == 9 ? "quat" : k == 3 ? "mat3x3" : "mat3x4", after[0], after[1],
                       after[2], after[3], dafter[0], dafter[1], dafter[2]);
            }
            g_rotHits = g_cands;
            g_cands.clear();  // rotation hits aren't position candidates
        } else {
            ListCandidates(t, sizeof(sh->devResult.text), 40);
        }
        sh->devResult.status = 0;
        std::atomic_thread_fence(std::memory_order_release);
        sh->devResult.seq = g_activeSeq;
        g_activeOp = DEV_NONE;
    }
    if (g_activeOp == DEV_POKE && g_pokeIdx >= 0) {
        if (g_pokeFramesLeft > 0) {
            --g_pokeFramesLeft;
            WritePos(g_cands[g_pokeIdx], g_pokeOrig[0] + g_pokeDelta[0], g_pokeOrig[1] + g_pokeDelta[1],
                     g_pokeOrig[2] + g_pokeDelta[2]);
        } else {
            const auto& b = g_pokeBefore.pos;
            const auto& a = truck.placement.pos;
            double moved = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
            double cx, cy, cz;
            ReadPos(g_cands[g_pokeIdx], cx, cy, cz);
            Result(sh, g_activeSeq, 0,
                   "poke [%d]: telemetry before (%.3f, %.3f, %.3f) after (%.3f, %.3f, %.3f) moved %.3f m%s\n"
                   "  candidate now (%.3f, %.3f, %.3f)\n",
                   g_pokeIdx, b.x, b.y, b.z, a.x, a.y, a.z, moved, moved > 0.5 ? "  <== DRIVES THE TRUCK" : "",
                   cx, cy, cz);
            g_activeOp = DEV_NONE;
            g_pokeIdx = -1;
        }
    }

    if (g_activeOp == DEV_AUTOPOKE) {
        // Phase per candidate: frames [0, N) write delta, frame N check + restore.
        auto& c = g_cands[g_apIdx];
        if (g_apFrame == 0) {
            g_apOk = ReadPos(c, g_apOrig[0], g_apOrig[1], g_apOrig[2]);
            g_apBefore = truck.placement;
        }
        if (g_apOk && g_apFrame < g_apFrames) {
            WritePos(c, g_apOrig[0] + g_pokeDelta[0], g_apOrig[1] + g_pokeDelta[1], g_apOrig[2] + g_pokeDelta[2]);
            ++g_apFrame;
        } else {
            const auto& b = g_apBefore.pos;
            const auto& a = truck.placement.pos;
            double moved = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
            if (g_apOk && moved > 0.3) {
                Append(sh->devResult.text, sizeof(sh->devResult.text),
                       "  HIT [%zu] %p %s orig (%.3f, %.3f, %.3f) -> telemetry moved %.3f m\n", g_apIdx,
                       (void*)c.addr, c.isDouble ? "dbl" : "flt", g_apOrig[0], g_apOrig[1], g_apOrig[2], moved);
                g_apHits.push_back(c);
            } else if (g_apOk) {
                WritePos(c, g_apOrig[0], g_apOrig[1], g_apOrig[2]);  // not it: put it back
            }
            g_apFrame = 0;
            if (++g_apIdx >= g_cands.size()) {
                Append(sh->devResult.text, sizeof(sh->devResult.text), "autopoke done: %zu hits of %zu\n",
                       g_apHits.size(), g_cands.size());
                if (!g_apHits.empty()) g_cands = g_apHits;  // keep only hits
                sh->devResult.status = 0;
                std::atomic_thread_fence(std::memory_order_release);
                sh->devResult.seq = g_activeSeq;
                g_activeOp = DEV_NONE;
            }
        }
    }

    if (g_activeOp == DEV_WATCH && g_watchThreadDone.load()) {
        if (g_watchAddr && GetTickCount64() >= g_watchUntil) {
            g_watchAddr = 0;
            StartWatchThread(0);  // clear DR0 everywhere
        } else if (!g_watchAddr) {
            if (g_veh) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
            char* t = sh->devResult.text;
            t[0] = 0;
            const size_t cap = sizeof(sh->devResult.text);
            Append(t, cap, "watch: %u writes, %d distinct sites\n", g_totalHits.load(), g_nHits.load());
            static const char* rn[16] = {"rax", "rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi",
                                         "r8",  "r9",  "r10", "r11", "r12", "r13", "r14", "r15"};
            for (int i = 0; i < g_nHits.load(); ++i) {
                const WatchHit& h = g_hits[i];
                HMODULE mod = nullptr;
                char name[MAX_PATH] = "?";
                if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                       reinterpret_cast<LPCSTR>(h.rip), &mod)) {
                    GetModuleFileNameA(mod, name, MAX_PATH);
                }
                const char* base = strrchr(name, '\\');
                Append(t, cap, "site %d: %s+0x%llx x%u tid %u\n  code[-24..+8]:", i, base ? base + 1 : name,
                       (unsigned long long)(h.rip - reinterpret_cast<uintptr_t>(mod)), h.count, h.tid);
                uint8_t code[32];
                if (SafeCopy(code, reinterpret_cast<void*>(h.rip - 24), sizeof(code)))
                    for (int k = 0; k < 32; ++k) Append(t, cap, "%s%02x", k == 24 ? " |" : " ", code[k]);
                Append(t, cap, "\n  ");
                for (int r = 0; r < 16; ++r) Append(t, cap, "%s=%llx%s", rn[r], (unsigned long long)h.regs[r], r % 4 == 3 ? "\n  " : " ");
                // Stack slots pointing into the game's code: likely return addresses (the callers).
                const uintptr_t mb = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
                Append(t, cap, "stack code ptrs:");
                for (int k = 0; k < 64; ++k)
                    if (h.stack[k] > mb + 0x1000 && h.stack[k] < mb + 0x4000000)
                        Append(t, cap, " [rsp+%x]=+%llx", k * 8, (unsigned long long)(h.stack[k] - mb));
                Append(t, cap, "\n");
            }
            sh->devResult.status = 0;
            std::atomic_thread_fence(std::memory_order_release);
            sh->devResult.seq = g_activeSeq;
            g_activeOp = DEV_NONE;
        }
    }

    // ---- puppet ----
    if (g_puppetIdx >= 0 && g_puppetIdx < int(g_cands.size())) {
        EtsCamTarget cam{};
        bool valid = false;
        Vec3d p{};
        if (g_puppetCam) {
            valid = sh->cam.read(cam) && cam.valid;
            p = cam.placement.pos;
        } else if (target && target->valid) {
            valid = true;
            p = target->placement.pos;
        }
        const Candidate& c = g_cands[g_puppetIdx];
        double cur[3];
        if (valid && ReadPos(c, cur[0], cur[1], cur[2])) {
            if (!g_puppetStarted) {
                memcpy(g_puppetV0, cur, sizeof(cur));
                g_puppetT0 = p;
                g_puppetStarted = true;
            } else {
                // Big jump not caused by us: the game re-based its local origin.
                const double jx = cur[0] - g_puppetLast[0], jy = cur[1] - g_puppetLast[1], jz = cur[2] - g_puppetLast[2];
                if (std::sqrt(jx * jx + jy * jy + jz * jz) > 50.0) {
                    g_puppetV0[0] += jx; g_puppetV0[1] += jy; g_puppetV0[2] += jz;
                    ++g_puppetRebases;
                }
            }
            double w[3] = {g_puppetV0[0] + (p.x - g_puppetT0.x), g_puppetV0[1] + (p.y - g_puppetT0.y),
                           g_puppetV0[2] + (p.z - g_puppetT0.z)};
            if (WritePos(c, w[0], w[1], w[2])) {
                memcpy(g_puppetLast, w, sizeof(w));
                ++g_puppetWrites;
            }
            // Camera object: quaternion (w,x,y,z) follows the position block (+16).
            if (g_puppetCam && c.isSector) {
                const EtsPlacement& o = cam.placement;
                const double hy = o.heading * kPi, hp = o.pitch * kPi, hr = o.roll * kPi;  // half angles
                // q = yaw(+Y) * pitch(+X) * roll(+Z)
                const double cy = std::cos(hy), sy = std::sin(hy), cp = std::cos(hp), sp = std::sin(hp),
                             cr = std::cos(hr), sr = std::sin(hr);
                // yaw*pitch = (cy*cp, cy*sp, sy*cp, -sy*sp)
                const double aw = cy * cp, ax = cy * sp, ay = sy * cp, az = -sy * sp;
                // (yaw*pitch) * roll(+Z): roll = (cr, 0, 0, sr)
                const float q[4] = {float(aw * cr - az * sr), float(ax * cr + ay * sr),
                                    float(ay * cr - ax * sr), float(az * cr + aw * sr)};
                if (Writable(c.addr + 16, 16)) SafeCopy(reinterpret_cast<void*>(c.addr + 16), q, sizeof(q));
            }
        }
    }

    // ---- new command ----
    const uint32_t seq = sh->devSeq.load(std::memory_order_acquire);
    static bool first = true;
    if (first) { first = false; g_lastSeq = seq; return; }  // fresh load: don't replay the last command
    if (seq == g_lastSeq || g_activeOp != DEV_NONE) return;
    g_lastSeq = seq;
    DevCmd cmd;
    memcpy(&cmd, &sh->devCmd, sizeof(cmd));
    sh->devResult.status = 1;
    sh->devResult.nbytes = 0;

    switch (cmd.op) {
    case DEV_SCAN: {
        if (!truck.hasPlacement) { Result(sh, seq, -1, "no truck placement yet (load into a drive)\n"); break; }
        uintptr_t lo, hi;
        OurModuleRange(lo, hi);
        g_scanTol = cmd.d[0];
        g_scanRot = false;
        g_scanLocal = false;
        g_scanSector = false;
        g_scanRunning.store(true);
        g_activeOp = DEV_SCAN;
        g_activeSeq = seq;
        std::thread(ScanThread, truck.placement.pos.x, truck.placement.pos.y, truck.placement.pos.z, lo, hi).detach();
        break;
    }
    case DEV_FILTER: {
        std::vector<Candidate> keep;
        const auto& p = truck.placement.pos;
        for (auto& c : g_cands) {
            double x, y, z;
            if (!ReadPos(c, x, y, z)) continue;
            double eps = c.isDouble ? 0.01 : 0.05;
            if (std::fabs(x - p.x) < eps && std::fabs(y - p.y) < eps && std::fabs(z - p.z) < eps) keep.push_back(c);
        }
        size_t before = g_cands.size();
        g_cands = std::move(keep);
        char* t = sh->devResult.text;
        snprintf(t, sizeof(sh->devResult.text), "filter vs (%.3f, %.3f, %.3f): %zu -> %zu\n", p.x, p.y, p.z, before,
                 g_cands.size());
        ListCandidates(t, sizeof(sh->devResult.text), 40);
        Result(sh, seq, 0, "%s", std::string(t).c_str());
        break;
    }
    case DEV_LIST: {
        char* t = sh->devResult.text;
        snprintf(t, sizeof(sh->devResult.text),
                 "%zu candidates, puppet %d (%u writes), cam hook %s (%u fires, %u applied, obj %p), truck (%.3f, %.3f, %.3f) hdg %.4f\n",
                 g_cands.size(), g_puppetIdx, g_puppetWrites, g_hookPc ? "on" : "off", g_hookFires.load(),
                 g_hookApplied.load(), (void*)g_hookRdi, truck.placement.pos.x, truck.placement.pos.y, truck.placement.pos.z,
                 truck.placement.heading);
        ListCandidates(t, sizeof(sh->devResult.text), 60);
        Result(sh, seq, 0, "%s", std::string(t).c_str());
        break;
    }
    case DEV_POKE: {
        size_t i = size_t(cmd.arg0);
        if (i >= g_cands.size()) { Result(sh, seq, -1, "bad index\n"); break; }
        if (!ReadPos(g_cands[i], g_pokeOrig[0], g_pokeOrig[1], g_pokeOrig[2])) { Result(sh, seq, -1, "unreadable\n"); break; }
        g_pokeIdx = int(i);
        g_pokeDelta[0] = cmd.d[0]; g_pokeDelta[1] = cmd.d[1]; g_pokeDelta[2] = cmd.d[2];
        g_pokeFramesLeft = cmd.arg1 ? int(cmd.arg1) : 30;
        g_pokeBefore = truck.placement;
        g_activeOp = DEV_POKE;
        g_activeSeq = seq;
        break;
    }
    case DEV_SCANROT: {
        if (!truck.hasPlacement) { Result(sh, seq, -1, "no truck placement yet\n"); break; }
        const double h = truck.placement.heading * 6.283185307179586;
        g_rotC = std::cos(h); g_rotS = std::sin(h);
        g_rotHc = std::cos(h / 2); g_rotHs = std::sin(h / 2);
        uintptr_t lo, hi;
        OurModuleRange(lo, hi);
        g_scanRot = true;
        g_scanLocal = false;
        g_scanSector = false;
        g_scanRunning.store(true);
        g_activeOp = DEV_SCAN;
        g_activeSeq = seq;
        std::thread(ScanThread, 0.0, 0.0, 0.0, lo, hi).detach();
        break;
    }
    case DEV_SNAP: {
        g_snap.assign(g_rotHits.size() * kWin, 0);
        g_snapOk.assign(g_rotHits.size(), 0);
        size_t ok = 0;
        for (size_t i = 0; i < g_rotHits.size(); ++i) {
            g_snapOk[i] = SafeCopy(&g_snap[i * kWin], reinterpret_cast<void*>(g_rotHits[i].addr - kWin / 2), kWin);
            ok += g_snapOk[i];
        }
        g_snapTruck = truck.placement;
        Result(sh, seq, 0, "snapshot of %zu/%zu windows at truck (%.3f, %.3f, %.3f). Now move the truck, then diff.\n",
               ok, g_rotHits.size(), truck.placement.pos.x, truck.placement.pos.y, truck.placement.pos.z);
        break;
    }
    case DEV_DIFF: {
        const double dx = truck.placement.pos.x - g_snapTruck.pos.x;
        const double dy = truck.placement.pos.y - g_snapTruck.pos.y;
        const double dz = truck.placement.pos.z - g_snapTruck.pos.z;
        char* t = sh->devResult.text;
        snprintf(t, sizeof(sh->devResult.text), "truck moved (%.3f, %.3f, %.3f)\n", dx, dy, dz);
        if (std::sqrt(dx * dx + dz * dz) < 2.0) { Result(sh, seq, -1, "%s  move the truck at least 2 m first\n", std::string(t).c_str()); break; }
        std::vector<Candidate> found;
        std::vector<uint8_t> now(kWin);
        // Internal state may lead/lag telemetry by a frame or two at speed.
        const double tol = 0.05 + 0.01 * std::sqrt(dx * dx + dy * dy + dz * dz) + (cmd.d[0] > 0 ? cmd.d[0] : 0);
        Append(t, sizeof(sh->devResult.text), "tolerance %.2f m\n", tol);
        for (size_t i = 0; i < g_rotHits.size(); ++i) {
            if (!g_snapOk[i]) continue;
            const uintptr_t base = g_rotHits[i].addr - kWin / 2;
            if (!SafeCopy(now.data(), reinterpret_cast<void*>(base), kWin)) continue;
            const uint8_t* old = &g_snap[i * kWin];
            for (size_t o = 0; o + 24 <= kWin; o += 4) {
                float fo[3], fn[3];
                memcpy(fo, old + o, 12); memcpy(fn, now.data() + o, 12);
                if (std::fabs((fn[0] - fo[0]) - dx) < tol && std::fabs((fn[1] - fo[1]) - dy) < tol &&
                    std::fabs((fn[2] - fo[2]) - dz) < tol)
                    found.push_back({base + o, false});
                if ((o & 7) == 0 && ((base + o) & 7) == 0) {
                    double d0[3], d1[3];
                    memcpy(d0, old + o, 24); memcpy(d1, now.data() + o, 24);
                    if (std::fabs((d1[0] - d0[0]) - dx) < tol && std::fabs((d1[1] - d0[1]) - dy) < tol &&
                        std::fabs((d1[2] - d0[2]) - dz) < tol)
                        found.push_back({base + o, true});
                }
            }
        }
        // De-duplicate (windows overlap).
        std::vector<Candidate> uniq;
        for (auto& c : found) {
            bool dup = false;
            for (auto& u : uniq) dup |= u.addr == c.addr && u.isDouble == c.isDouble;
            if (!dup) uniq.push_back(c);
        }
        g_cands = uniq;
        Append(t, sizeof(sh->devResult.text), "%zu position candidates move with the truck:\n", g_cands.size());
        ListCandidates(t, sizeof(sh->devResult.text), 40);
        Result(sh, seq, 0, "%s", std::string(t).c_str());
        break;
    }
    case DEV_AUTOPOKE: {
        if (g_cands.empty()) { Result(sh, seq, -1, "no candidates\n"); break; }
        g_pokeDelta[0] = cmd.d[0]; g_pokeDelta[1] = cmd.d[1]; g_pokeDelta[2] = cmd.d[2];
        g_apFrames = cmd.arg1 ? int(cmd.arg1) : 4;
        g_apIdx = 0; g_apFrame = 0; g_apHits.clear();
        snprintf(sh->devResult.text, sizeof(sh->devResult.text), "autopoke %zu candidates, delta (%.2f, %.2f, %.2f), %d frames each\n",
                 g_cands.size(), cmd.d[0], cmd.d[1], cmd.d[2], g_apFrames);
        g_activeOp = DEV_AUTOPOKE;
        g_activeSeq = seq;
        break;
    }
    case DEV_DUMP: {
        size_t n = size_t(cmd.arg1 > 1024 ? 1024 : cmd.arg1);
        bool ok = SafeCopy(sh->devResult.bytes, reinterpret_cast<void*>(cmd.arg0), n);
        sh->devResult.nbytes = ok ? uint32_t(n) : 0;
        Result(sh, seq, ok ? 0 : -1, ok ? "dump ok\n" : "unreadable\n");
        break;
    }
    case DEV_PUPPET: {
        if (cmd.arg0 >= g_cands.size()) {
            g_puppetIdx = -1;
            Result(sh, seq, 0, "puppet off\n");
        } else {
            g_puppetIdx = int(cmd.arg0);
            g_puppetCam = cmd.arg1 != 0;
            g_puppetStarted = false;
            g_puppetWrites = 0;
            g_puppetRebases = 0;
            Result(sh, seq, 0, "puppet on candidate [%d] %p following the %s target\n", g_puppetIdx,
                   (void*)g_cands[g_puppetIdx].addr, g_puppetCam ? "camera" : "vehicle");
        }
        break;
    }
    case DEV_REMEMBER: {
        g_mem.resize(g_cands.size());
        for (size_t i = 0; i < g_cands.size(); ++i)
            g_mem[i].ok = ReadPos(g_cands[i], g_mem[i].v[0], g_mem[i].v[1], g_mem[i].v[2]);
        Result(sh, seq, 0, "remembered %zu values\n", g_cands.size());
        break;
    }
    case DEV_KEEP: {
        if (g_mem.size() != g_cands.size()) { Result(sh, seq, -1, "run 'remember' first\n"); break; }
        const bool keepChanged = cmd.arg0 != 0;
        const double thr = cmd.d[0] > 0 ? cmd.d[0] : 0.5;
        std::vector<Candidate> keep;
        std::vector<Val3> keepMem;
        for (size_t i = 0; i < g_cands.size(); ++i) {
            double v[3];
            if (!g_mem[i].ok || !ReadPos(g_cands[i], v[0], v[1], v[2])) continue;
            const double dx = v[0] - g_mem[i].v[0], dy = v[1] - g_mem[i].v[1], dz = v[2] - g_mem[i].v[2];
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (!std::isfinite(d)) continue;
            if (keepChanged ? d > thr : d < thr) {
                keep.push_back(g_cands[i]);
                keepMem.push_back({{v[0], v[1], v[2]}, true});  // re-remember for the next round
            }
        }
        const size_t before = g_cands.size();
        g_cands = std::move(keep);
        g_mem = std::move(keepMem);
        char* t = sh->devResult.text;
        snprintf(t, sizeof(sh->devResult.text), "keep %s (%.2f m): %zu -> %zu\n", keepChanged ? "changed" : "unchanged",
                 thr, before, g_cands.size());
        ListCandidates(t, sizeof(sh->devResult.text), 40);
        Result(sh, seq, 0, "%s", std::string(t).c_str());
        break;
    }
    case DEV_SCANSECTOR: {
        double c[3];
        if (cmd.arg0 < g_cands.size()) {
            if (!ReadPos(g_cands[size_t(cmd.arg0)], c[0], c[1], c[2])) { Result(sh, seq, -1, "centre unreadable\n"); break; }
        } else if (truck.hasPlacement) {
            c[0] = truck.placement.pos.x; c[1] = truck.placement.pos.y; c[2] = truck.placement.pos.z;
        } else {
            Result(sh, seq, -1, "no centre\n");
            break;
        }
        uintptr_t lo, hi;
        OurModuleRange(lo, hi);
        g_scanTol = cmd.d[0];
        g_scanSector = true; g_scanLocal = false; g_scanRot = false;
        g_scanRunning.store(true);
        g_activeOp = DEV_SCAN;
        g_activeSeq = seq;
        std::thread(ScanThread, c[0], c[1], c[2], lo, hi).detach();
        break;
    }
    case DEV_HOOKCAM: {
        if (g_hookPc) {  // always disarm the old hook first
            SetDrSync(1, 0, 0, 0);
            g_hookPc = 0;
        }
        if (cmd.arg0 >= g_cands.size()) {
            Result(sh, seq, 0, "camera hook off (%u fires, %u applied)\n", g_hookFires.load(), g_hookApplied.load());
            break;
        }
        HMODULE game = GetModuleHandleA(nullptr);
        MEMORY_BASIC_INFORMATION mbi{};
        const uintptr_t pc = reinterpret_cast<uintptr_t>(game) + uintptr_t(cmd.arg1);
        // Only instructions inside eurotrucks2.exe's image; only small register offsets.
        if (!VirtualQuery(reinterpret_cast<void*>(pc), &mbi, sizeof(mbi)) || mbi.AllocationBase != game ||
            !(mbi.Protect & (PAGE_EXECUTE_READ | PAGE_EXECUTE | PAGE_EXECUTE_READWRITE)) || cmd.d[0] < 0 ||
            cmd.d[0] > 0x200 || cmd.d[1] < 0 || cmd.d[1] > 0x200) {
            Result(sh, seq, -1, "refused: not code in eurotrucks2.exe or offsets out of range\n");
            break;
        }
        g_hookShared = sh;
        g_hookSrcOff = uint32_t(cmd.d[0]);
        g_hookDstOff = uint32_t(cmd.d[1]);
        g_hookDst = g_cands[size_t(cmd.arg0)].addr;
        if (!ReadPos(g_cands[size_t(cmd.arg0)], g_hookRef[0], g_hookRef[1], g_hookRef[2])) {
            Result(sh, seq, -1, "reference candidate unreadable\n");
            break;
        }
        g_hookRdi = 0;
        g_hookRotate = false;  // TODO: enable once the rotation offset in the source block is confirmed
        g_hookFires.store(0);
        g_hookApplied.store(0);
        if (!g_hookVeh) g_hookVeh = AddVectoredExceptionHandler(1, HookVeh);
        g_hookPc = pc;
        SetDrSync(1, pc, 0 /*execute*/, 0);
        Result(sh, seq, 0, "camera hook armed at eurotrucks2.exe+0x%llx: src [rsi+0x%x], camera at (%.2f, %.2f, %.2f)\n",
               (unsigned long long)cmd.arg1, g_hookSrcOff, g_hookRef[0], g_hookRef[1], g_hookRef[2]);
        break;
    }
    case DEV_EXEC:
    case DEV_WATCH: {
        uintptr_t addr = 0;
        if (cmd.op == DEV_EXEC) {
            addr = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr)) + uintptr_t(cmd.arg1);
        } else if (cmd.arg0 == ~0ull) {
            addr = uintptr_t(cmd.arg1);   // watchaddr: any address
        } else {
            size_t i = size_t(cmd.arg0);
            if (i >= g_cands.size()) { Result(sh, seq, -1, "bad index\n"); break; }
            addr = g_cands[i].addr;
        }
        if (cmd.op == DEV_WATCH && !Writable(addr, 8)) { Result(sh, seq, -1, "address not writable memory\n"); break; }
        g_watchExec = cmd.op == DEV_EXEC;
        g_filterReg = (cmd.op == DEV_EXEC && cmd.d[1] >= 1 && cmd.d[1] <= 16) ? int(cmd.d[1]) - 1 : -1;
        g_filterVal = uint64_t(cmd.d[2]);
        g_nHits.store(0);
        g_totalHits.store(0);
        g_veh = AddVectoredExceptionHandler(1, WatchVeh);
        g_watchAddr = addr;
        g_watchUntil = GetTickCount64() + uint64_t(cmd.d[0] > 0 ? cmd.d[0] : 500);
        StartWatchThread(g_watchAddr);
        g_activeOp = DEV_WATCH;
        g_activeSeq = seq;
        break;
    }
    case DEV_WRITE: {
        const uintptr_t a = uintptr_t(cmd.arg0);
        const size_t n = size_t(cmd.arg1);
        if (n == 0 || n > sizeof(cmd.data) || !Writable(a, n)) { Result(sh, seq, -1, "write refused (not writable memory)\n"); break; }
        memcpy(reinterpret_cast<void*>(a), cmd.data, n);
        Result(sh, seq, 0, "wrote %zu bytes at %llx\n", n, (unsigned long long)a);
        break;
    }
    case DEV_SCANLOCAL: {
        if (!truck.hasPlacement) { Result(sh, seq, -1, "no truck placement yet\n"); break; }
        uintptr_t lo, hi;
        OurModuleRange(lo, hi);
        g_localY = truck.placement.pos.y;
        g_scanLocal = true;
        g_scanSector = false;
        g_scanRot = false;
        g_scanRunning.store(true);
        g_activeOp = DEV_SCAN;
        g_activeSeq = seq;
        std::thread(ScanThread, 0.0, 0.0, 0.0, lo, hi).detach();
        break;
    }
    default:
        Result(sh, seq, -1, "unknown op %u\n", cmd.op);
    }
}

bool Busy() { return g_scanRunning.load(); }

void Shutdown() {
    // Clear any armed watchpoint on all threads (including this one) before the handler goes away.
    if (g_veh || g_watchAddr) {
        HANDLE h = CreateThread(nullptr, 0, SetWatchAllThreads, nullptr, 0, nullptr);
        if (h) { WaitForSingleObject(h, 5000); CloseHandle(h); }
        if (g_veh) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
        g_watchAddr = 0;
    }
    if (g_hookPc) {
        SetDrSync(1, 0, 0, 0);
        g_hookPc = 0;
    }
    if (g_hookVeh) { RemoveVectoredExceptionHandler(g_hookVeh); g_hookVeh = nullptr; }
    while (g_scanRunning.load()) Sleep(10);
}

}  // namespace probe
