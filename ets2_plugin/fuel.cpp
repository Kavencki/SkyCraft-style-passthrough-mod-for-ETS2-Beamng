// FuelLink (see fuel.h).
//
// Finding the fuel value: floats equal to the telemetry fuel in the truck object (first 8 KB) and
// in the objects it points to (first 2 KB of each, pointers from its first 4 KB). Each candidate
// is tested by writing the fuel minus 1 l: the one the telemetry follows within a few frames is
// the truck's fuel; the others are put back at once.
//
// Then: the truck's fuel is kept at the car's (missing litres, or percentage). A rise that is not
// ours is an ETS2 refuel: no more writes until the car has caught up, the bridge fills the car.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "fuel.h"

using namespace es;

namespace fuel {
namespace {

bool SafeCopy(void* dst, const void* src, size_t n) {
    __try { memcpy(dst, src, n); return true; }
    __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

struct Path { int off1, off2; };   // truck+off1, or [truck+off1]+off2 (off2 >= 0)

Settings g_set;
float g_fuel = 0, g_cap = 0;
bool g_haveFuel = false;

enum class Phase { Idle, Testing, Linked, Failed };
Phase g_phase = Phase::Idle;
std::vector<Path> g_cands;
size_t g_next = 0;          // candidate under test
int g_waitFrames = 0;
float g_testValue = 0, g_testOrig = 0;
uintptr_t g_testAddr = 0;
uint32_t g_lastScanTruck = 0;
Path g_path{};

float g_lastWritten = NAN;
bool g_confirmed = true;
int g_unconfirmedFrames = 0;
bool g_refuelling = false;
uint64_t g_lastRiseMs = 0;
float g_lastFuelSeen = 0;
uint32_t g_seq = 0;
char g_msg[256];

uintptr_t Resolve(uintptr_t truck, const Path& p) {
    if (p.off2 < 0) return truck + p.off1;
    uintptr_t q = 0;
    if (!SafeCopy(&q, reinterpret_cast<const void*>(truck + p.off1), 8) || !q) return 0;
    return q + p.off2;
}
bool ReadF(uintptr_t a, float& v) { return a && SafeCopy(&v, reinterpret_cast<const void*>(a), 4) && std::isfinite(v); }
bool WriteF(uintptr_t a, float v) { return a && SafeCopy(reinterpret_cast<void*>(a), &v, 4); }

bool Near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// (ETS2 1.61.1.9s: nothing within the first 8 KB / 2 KB any more - the object grew; wider now.)
constexpr int kTruckBytes = 0x4000, kPointerBytes = 0x2000, kChildBytes = 0x1000;
constexpr size_t kMaxCands = 128;

void Scan(uintptr_t truck) {
    g_cands.clear();
    const float tol = 1e-4f * std::max(1.0f, g_fuel);
    std::vector<uint8_t> buf(kTruckBytes);
    if (!SafeCopy(buf.data(), reinterpret_cast<const void*>(truck), buf.size()))   // the end may be unreadable
        if (!SafeCopy(buf.data(), reinterpret_cast<const void*>(truck), buf.size() / 2)) return;
    for (int o = 0; o + 4 <= int(buf.size()); o += 4) {
        float v; memcpy(&v, &buf[o], 4);
        if (std::isfinite(v) && Near(v, g_fuel, tol)) g_cands.push_back({o, -1});
    }
    std::vector<uint8_t> b2(kChildBytes);
    for (int o = 0; o + 8 <= kPointerBytes && g_cands.size() < kMaxCands; o += 8) {
        uintptr_t q; memcpy(&q, &buf[o], 8);
        if (!q || (q & 7) || q < 0x10000 || q > 0x7FFFFFFFFFFFull) continue;
        if (!SafeCopy(b2.data(), reinterpret_cast<const void*>(q), b2.size())) continue;
        for (int o2 = 0; o2 + 4 <= int(b2.size()) && g_cands.size() < kMaxCands; o2 += 4) {
            float v; memcpy(&v, &b2[o2], 4);
            if (std::isfinite(v) && Near(v, g_fuel, tol)) g_cands.push_back({o, o2});
        }
    }
}

float TruckFromCar(float car, float carCap) {
    if (carCap <= 0 || g_cap <= 0) return g_fuel;
    const float t = g_set.percent ? car / carCap * g_cap : g_cap - (carCap - car);
    return std::max(0.0f, std::min(g_cap, t));
}
float CarFromTruck(float truck, float carCap) {
    if (carCap <= 0 || g_cap <= 0) return 0;
    const float c = g_set.percent ? truck / g_cap * carCap : carCap - (g_cap - truck);
    return std::max(0.0f, std::min(carCap, c));
}

}  // namespace

void Configure(const Settings& s) { g_set = s; }
void OnFuel(float litres) { g_fuel = litres; g_haveFuel = true; }
void OnCapacity(float litres) { g_cap = litres; }

const char* Update(Shared* sh, bool drive, uintptr_t truck) {
    const char* ret = nullptr;
    if (!g_set.enabled || !sh) return nullptr;
    const uint64_t now = GetTickCount64();

    // 1. Find the fuel value (once the truck object is known and the tank is not empty).
    if (g_phase == Phase::Idle && truck && g_haveFuel && g_fuel > 2 && g_cap > 0) {
        Scan(truck);
        g_next = 0;
        g_waitFrames = 0;
        g_testAddr = 0;
        g_phase = Phase::Testing;
        snprintf(g_msg, sizeof(g_msg), "fuel: %.1f of %.0f l, %zu places to test", g_fuel, g_cap, g_cands.size());
        return g_msg;   // (testing from the next frame: this line is not overwritten)
    }
    if (g_phase == Phase::Testing) {
        if (!truck) { g_phase = Phase::Idle; return ret; }
        if (g_testAddr) {
            if (++g_waitFrames < 4) return ret;
            float cur = 0;
            ReadF(g_testAddr, cur);
            if (Near(g_fuel, g_testValue, 0.05f)) {
                WriteF(g_testAddr, g_testOrig);   // back to where it was; FuelLink sets it from now on
                g_path = g_cands[g_next];
                g_phase = Phase::Linked;
                g_lastWritten = NAN;
                snprintf(g_msg, sizeof(g_msg), "fuel linked: truck+0x%x%s%x (%zu tested) - the truck's fuel now follows the car's",
                         g_path.off1, g_path.off2 >= 0 ? " -> +0x" : "", g_path.off2 >= 0 ? g_path.off2 : 0, g_next + 1);
                return g_msg;
            }
            if (Near(cur, g_testValue, 1e-4f)) WriteF(g_testAddr, g_testOrig);
            g_testAddr = 0;
            ++g_next;
        }
        if (g_next >= g_cands.size()) {
            g_phase = Phase::Failed;
            return "fuel: the truck's fuel value was not found - fuel is not linked this session";
        }
        const uintptr_t a = Resolve(truck, g_cands[g_next]);
        if (!ReadF(a, g_testOrig) || !Near(g_testOrig, g_fuel, 1e-4f * std::max(1.0f, g_fuel) + 0.01f)) { ++g_next; return ret; }
        g_testValue = g_testOrig - 1.0f;
        if (!WriteF(a, g_testValue)) { ++g_next; return ret; }
        g_testAddr = a;
        g_waitFrames = 0;
        return ret;
    }

    EtsFuel out{};
    out.litres = g_fuel;
    out.capacity = g_cap;
    out.linked = g_phase == Phase::Linked;

    // 2. Linked: follow the car; an ETS2 refuel fills the car.
    CarFuel car{};
    const bool haveCar = sh->carFuel.read(car) && car.valid && car.capacity > 0;
    if (g_phase == Phase::Linked && truck && drive && haveCar) {
        const uintptr_t a = Resolve(truck, g_path);
        if (g_fuel > g_lastFuelSeen + 0.05f) g_lastRiseMs = now;
        g_lastFuelSeen = g_fuel;
        if (!std::isnan(g_lastWritten) && !g_confirmed) {
            if (Near(g_fuel, g_lastWritten, 0.05f)) { g_confirmed = true; g_unconfirmedFrames = 0; }
            else if (++g_unconfirmedFrames > 120) {   // our writes no longer reach the telemetry
                g_phase = Phase::Idle;
                g_confirmed = true;
                g_unconfirmedFrames = 0;
                ret = "fuel: the link was lost, searching again";
            }
        }
        if (g_phase == Phase::Linked) {
            if (g_refuelling) {
                const float target = CarFromTruck(g_fuel, car.capacity);
                out.refuelling = 1;
                out.carTarget = target;
                if (now - g_lastRiseMs > 3000 && Near(car.litres, target, 1.0f)) {
                    g_refuelling = false;
                    g_lastWritten = g_fuel;
                    snprintf(g_msg, sizeof(g_msg), "fuel: refuelled - truck %.1f l, car %.1f l", g_fuel, car.litres);
                    ret = g_msg;
                } else if (now - g_lastRiseMs > 30000) {
                    g_refuelling = false;   // the car never caught up: back to following it
                    g_lastWritten = g_fuel;
                    ret = "fuel: the car did not take the refuel (bridge/BeamNG?) - following the car again";
                }
            } else if (!std::isnan(g_lastWritten) && g_confirmed && g_fuel > g_lastWritten + 0.3f) {
                g_refuelling = true;
                g_lastRiseMs = now;
                ret = "fuel: refuelling at an ETS2 station - the car is filled too";
            } else if (g_confirmed) {
                const float want = TruckFromCar(car.litres, car.capacity);
                if (!Near(g_fuel, want, 0.3f) && WriteF(a, want)) {
                    g_lastWritten = want;
                    g_confirmed = false;
                    g_unconfirmedFrames = 0;
                }
            }
        }
    }
    out.seq = ++g_seq;
    sh->etsFuel.write(out);
    return ret;
}

}  // namespace fuel
