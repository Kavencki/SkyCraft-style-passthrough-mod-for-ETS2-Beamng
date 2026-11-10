// codescan_test: finds the code the plugin hooks (ets2_plugin/ets2_code.h, as camhook.cpp and
// truckpuppet.cpp do) in an eurotrucks2.exe mapped as an image (not run), and checks the physics
// actor class the way truckpuppet does. Run it after an ETS2 update:
//   codescan_test <eurotrucks2.exe>
#include <chrono>
#include <cstdio>
#include "../ets2_plugin/codescan.h"
#include "../ets2_plugin/ets2_code.h"

using namespace ets2code;

int main(int argc, char** argv) {
    if (argc < 2) { printf("usage: codescan_test <eurotrucks2.exe>\n"); return 2; }
    codescan::g_module = LoadLibraryExA(argv[1], nullptr, DONT_RESOLVE_DLL_REFERENCES);
    if (!codescan::g_module) { printf("cannot map %s (%lu)\n", argv[1], GetLastError()); return 1; }
    const uintptr_t base = codescan::Base();
    int bad = 0;
    auto rel = [&](uintptr_t a) { return (unsigned long long)(a ? a - base : 0); };

    const auto t0 = std::chrono::steady_clock::now();
    const uintptr_t cam = FindCamera();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const char* how = "not found";
    for (uintptr_t k : kCameraKnown) if (cam == base + k) how = "a known place";
    if (cam && *how == 'n') how = "found by search (add it to kCameraKnown)";
    printf("camera            +%llx  %s (%.0f ms)\n", rel(cam), how, ms);
    if (!cam) ++bad;

    const uintptr_t loop = codescan::Locate(kLoopSig, kLoopKnown, Count(kLoopKnown));
    const uintptr_t call = loop ? codescan::FindUnique(kCallSig, loop, loop + 0x180) : 0;
    const uintptr_t states = loop ? codescan::FindUnique(kStatesSig, loop - 0x180, loop) : 0;
    int32_t slot = 0;
    if (call) memcpy(&slot, reinterpret_cast<const void*>(call + 15), 4);
    printf("state loop        +%llx, states array +%llx, setPlacement call +%llx, vtable slot +0x%x\n",
           rel(loop), rel(states), rel(call), unsigned(slot));
    if (!loop || !states || !call || slot <= 0) ++bad;
    const uintptr_t thunk = codescan::Locate(kThunkSig, kThunkKnown, Count(kThunkKnown));
    const uintptr_t getter = codescan::Locate(kGetterSig, kGetterKnown, Count(kGetterKnown));
    printf("telemetry thunk   +%llx, placement getter +%llx\n", rel(thunk), rel(getter));
    if (!thunk || !getter) ++bad;

    // The actor class: truckpuppet learns it from the player truck and accepts a vtable in .rdata
    // whose setPlacement slot holds a function starting like setPlacement. A vtable seen before must
    // pass that (else the puppet would never accept the truck: run sigscan for the new one).
    const codescan::Range text = codescan::Section(".text"), rdata = codescan::Section(".rdata");
    bool vtOk = false;
    for (uintptr_t v : kActorVtablesSeen) {
        const uintptr_t vt = base + v;
        uintptr_t fn = 0;
        if (slot > 0 && rdata.Has(vt)) memcpy(&fn, reinterpret_cast<const void*>(vt + slot), 8);
        const bool ok = text.Has(fn) && codescan::Matches(fn, kSetPlacementSig);
        if (ok) { printf("actor vtable      +%llx: setPlacement +%llx - class check ok\n", rel(vt), rel(fn)); vtOk = true; break; }
    }
    if (!vtOk) { printf("actor vtable      none of the known ones fits this version - find it with sigscan\n"); ++bad; }
    printf(bad ? "FAILED (%d)\n" : "all found\n", bad);
    return bad ? 1 : 0;
}
