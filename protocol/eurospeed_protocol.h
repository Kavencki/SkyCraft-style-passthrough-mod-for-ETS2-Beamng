// EuroSpeed shared-memory protocol. Both halves include this file:
//   lfs_bridge.exe   (LFS side: OutSim/InSim -> here)
//   eurospeed.dll    (ETS2 SCS SDK plugin, runs inside eurotrucks2.exe)
// Fixed-size, little-endian, x64 only. Bump kVersion on any layout change.
#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>

namespace es {

constexpr uint32_t kMagic   = 0x44505345;  // "ESPD"
constexpr uint32_t kVersion = 6;
constexpr wchar_t  kShmName[] = L"Local\\EuroSpeed_v1";

// Peer is considered gone if its heartbeat is older than this.
constexpr uint64_t kHeartbeatTimeoutMs = 2000;

struct Vec3d { double x, y, z; };

// LFS car, in LFS world frame: X east, Y north, Z up. Metres, radians,
// angles anticlockwise (OutSim convention). Written by the bridge.
struct LfsCarState {
    uint32_t lfsTimeMs;     // OutSim Time
    uint32_t packets;       // OutSim packets received so far
    Vec3d    pos;
    float    heading, pitch, roll;
    float    vel[3];
    float    angVel[3];
    float    throttle, brake, steer, clutch, handbrake;
    int32_t  gear;          // OutSim: 0=R 1=N 2=first
    float    engineRadS;
    float    speedMs;
};

// ETS2 world placement, SDK convention: X east, Y up, Z south, metres;
// heading/pitch/roll in turns (heading 0 = north, CCW; pitch + = nose up).
struct EtsPlacement {
    Vec3d  pos;
    double heading, pitch, roll;
};

// Bridge -> plugin: where the ETS2 vehicle should be this frame.
struct EtsTarget {
    EtsPlacement placement;
    uint32_t     valid;      // 0 until an anchor exists
    uint32_t     lfsPackets; // which LFS sample produced it
};

// BeamNG player camera + vehicle (BeamNG world: X east, Y north, Z up).
struct BngState {
    uint32_t seq;
    uint32_t timeMs;
    Vec3d    camPos;
    float    camFwd[3], camUp[3];
    float    fovDeg;
    uint32_t _pad;
    Vec3d    vehPos;
    float    vehFwd[3], vehUp[3];
    float    speedMs;
    uint32_t _pad2;
};

// Bridge -> plugin: where the ETS2 camera should be this frame.
struct EtsCamTarget {
    EtsPlacement placement;  // ETS2 world, turns
    float        fovDeg;     // BeamNG vertical FOV
    uint32_t     valid;
    uint64_t     sampleUs;   // when the bridge received this pose (QueryPerformanceCounter, microseconds);
                             // the plugin interpolates between samples for smooth motion
};

// Plugin -> bridge: the driver's raw inputs as ETS2 reads them (wheel/keyboard/pad
// bound in ETS2), forwarded to the physics car. SDK conventions.
struct EtsInput {
    float    steering;   // <-1;1>, counterclockwise (+ = left)
    float    throttle;   // <0;1>
    float    brake;      // <0;1>
    float    clutch;     // <0;1>
    uint32_t parkingBrake;
    int32_t  gear;       // truck.engine.gear: >0 forward, 0 neutral, <0 reverse
    uint32_t seq;        // bumps every frame_end
    uint32_t _pad;
};

// Plugin -> bridge: what ETS2 actually reports.
struct EtsTruckState {
    EtsPlacement placement;
    uint32_t     hasPlacement;
    uint32_t     paused;       // SDK paused/started events
    uint32_t     multiplayer;  // Convoy or TruckersMP detected: plugin is inert
    uint32_t     frames;       // frame_end count
    uint32_t     gameVersion;  // SDK game_version
    uint32_t     gameTimeMin;  // game.time: in-game minutes since 00:00 of day one
};

// Bridge -> plugin: the physics car's fuel (BeamNG fuel tanks, litres).
struct CarFuel {
    float    litres;
    float    capacity;
    uint32_t valid;
    uint32_t seq;
};

// Plugin -> bridge: the ETS2 truck's fuel. While the truck is refuelled at a station (its fuel
// rises, not by us), `refuelling` is 1 and the car should get `carTarget` litres.
struct EtsFuel {
    float    litres;      // telemetry truck.fuel.amount
    float    capacity;    // truck config fuel.capacity
    uint32_t linked;      // the fuel field was found: the truck's fuel follows the car's
    uint32_t refuelling;
    float    carTarget;   // litres the car should have now (refuelling only)
    uint32_t seq;
};

// Single-writer latest-value slot (seqlock). Readers retry on a torn read.
template <class T>
struct Slot {
    std::atomic<uint32_t> seq{0};
    uint32_t _pad{0};
    T v{};

    void write(const T& in) {
        uint32_t s = seq.load(std::memory_order_relaxed);
        seq.store(s + 1, std::memory_order_release);           // odd: writing
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(&v, &in, sizeof(T));
        std::atomic_thread_fence(std::memory_order_release);
        seq.store(s + 2, std::memory_order_release);           // even: stable
    }
    bool read(T& out) const {
        for (int i = 0; i < 64; ++i) {
            uint32_t a = seq.load(std::memory_order_acquire);
            if (a & 1) continue;
            std::atomic_thread_fence(std::memory_order_acquire);
            std::memcpy(&out, &v, sizeof(T));
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq.load(std::memory_order_acquire) == a) return a != 0;
        }
        return false;
    }
};

// Dev/RE command channel: es_ctl.exe -> plugin. The plugin runs a command on
// the game thread (frame_end) when `seq` changes, then sets result.seq = seq.
enum DevOp : uint32_t {
    DEV_NONE = 0,
    DEV_SCAN = 1,     // find (x,y,z) of truck placement in memory (double + float triples)
    DEV_FILTER = 2,   // keep candidates that still match the current placement
    DEV_POKE = 3,     // arg0 = candidate index, d[0..2] = delta; written for arg1 frames
    DEV_DUMP = 4,     // arg0 = address, arg1 = byte count (<= 1024)
    DEV_PUPPET = 5,   // arg0 = candidate index (or 0xFFFF.. to stop): follow EtsTarget
    DEV_LIST = 6,     // list candidates
    DEV_AUTOPOKE = 8, // poke every candidate by d[0..2] for arg1 frames, restore, report hits
    DEV_SCANROT = 9,  // find float yaw rotation matrices / quaternions matching truck heading
    DEV_SNAP = 11,    // snapshot +-256 bytes around every rotation hit, plus truck placement
    DEV_DIFF = 12,    // after the truck moved: candidates = triples whose change == telemetry change
    DEV_REMEMBER = 13,   // store every candidate's current value
    DEV_KEEP = 14,       // arg0 1: keep candidates that moved > d[0] m since REMEMBER; 0: that moved < d[0]
    DEV_SCANLOCAL = 15,  // all triples with y within 4 m of the truck's and |x|,|z| < 20 km (local frames)
    DEV_WATCH = 16,      // hardware write-watchpoint on candidate arg0 for d[0] ms: report writing code + registers
    DEV_SCANSECTOR = 17, // sector-encoded positions (3 floats + 2 int16, 512 m sectors) within d[0] m of
                         // candidate arg0's value (or the truck if arg0 is out of range)
    DEV_EXEC = 19,      // execute breakpoint at eurotrucks2.exe+arg1 for d[0] ms: report registers + stack (read only);
                        // d[1] = register index + 1 (rax rcx rdx rbx rsp rbp rsi rdi r8..r15) to record only hits
                        // where that register == d[2]
    DEV_WRITE = 20,     // write arg1 bytes of data[] to address arg0 (writable memory only)
    DEV_HOOKCAM = 18,    // execute breakpoint at eurotrucks2.exe+arg1 (an instruction copying the camera
                         // transform from [rsi+d[0]] into [rdi+d[1]]); when rdi+d[1] == candidate arg0,
                         // the camera target position is written into the source first. arg0 = ~0: off.
};
// DEV_PUPPET: arg1 = 0 follow the vehicle target, 1 follow the camera target.
// Puppet is relative: candidate = value at start + (target - target at start).
// Writes only ever go to addresses the probe found itself (poke/autopoke/puppet).
// DEV_SCAN: d[0] = tolerance in metres (0 = exact).
struct DevCmd {
    uint32_t op;
    uint32_t _pad;
    uint64_t arg0, arg1;
    double   d[4];
    uint8_t  data[64];
};
struct DevResult {
    uint32_t seq;
    int32_t  status;      // 0 ok, <0 error, 1 still running
    char     text[3072];
    uint8_t  bytes[1024];
    uint32_t nbytes;
    uint32_t _pad;
};

struct Shared {
    uint32_t magic;
    uint32_t version;
    std::atomic<uint32_t> bridgePid;
    std::atomic<uint32_t> pluginPid;
    std::atomic<uint64_t> bridgeHeartbeatMs;   // GetTickCount64()
    std::atomic<uint64_t> pluginHeartbeatMs;

    Slot<LfsCarState>   lfs;     // bridge writes
    Slot<EtsTarget>     target;  // bridge writes
    Slot<EtsTruckState> ets;     // plugin writes

    std::atomic<uint32_t> devSeq;   // es_ctl bumps after filling devCmd
    uint32_t  _pad2;
    DevCmd    devCmd;
    DevResult devResult;            // plugin writes

    Slot<BngState>     bng;         // bridge writes (from the BeamNG mod)
    Slot<EtsCamTarget> cam;         // bridge writes
    Slot<EtsInput>     input;       // plugin writes
    Slot<CarFuel>      carFuel;     // bridge writes (from the BeamNG mod)
    Slot<EtsFuel>      etsFuel;     // plugin writes
};

static_assert(sizeof(LfsCarState) == 104, "LfsCarState layout");
static_assert(sizeof(EtsPlacement) == 48, "EtsPlacement layout");
static_assert(sizeof(EtsTarget) == 56, "EtsTarget layout");
static_assert(sizeof(EtsTruckState) == 72, "EtsTruckState layout");

}  // namespace es

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
namespace es {
// Microseconds on the system-wide QueryPerformanceCounter clock (same in every process).
inline uint64_t NowUs() {
    static const uint64_t freq = [] { LARGE_INTEGER f; QueryPerformanceFrequency(&f); return uint64_t(f.QuadPart); }();
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    const uint64_t q = uint64_t(t.QuadPart);
    return (q / freq) * 1000000ull + (q % freq) * 1000000ull / freq;
}

// Opens or creates the mapping. Whoever creates it stamps magic/version.
// Returns nullptr if the existing mapping has a different version.
inline Shared* OpenShared(HANDLE* outMapping) {
    HANDLE h = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0,
                                  sizeof(Shared), kShmName);
    if (!h) return nullptr;
    bool created = GetLastError() != ERROR_ALREADY_EXISTS;
    auto* s = static_cast<Shared*>(MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Shared)));
    if (!s) { CloseHandle(h); return nullptr; }
    if (created || s->magic == 0) {
        s->version = kVersion;
        std::atomic_thread_fence(std::memory_order_release);
        s->magic = kMagic;
    }
    if (s->magic != kMagic || s->version != kVersion) {
        UnmapViewOfFile(s); CloseHandle(h); return nullptr;
    }
    *outMapping = h;
    return s;
}
inline void CloseShared(Shared* s, HANDLE h) {
    if (s) UnmapViewOfFile(s);
    if (h) CloseHandle(h);
}
}  // namespace es
#endif
