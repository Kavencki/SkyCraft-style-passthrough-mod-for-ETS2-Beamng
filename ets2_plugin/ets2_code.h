// The ETS2 code the plugin hooks, in one place: what it looks like (bytes, ?? = any) and where it
// was in the versions seen. The plugin searches the running game for it (codescan.h), the known
// places first. After a game update: build\codescan_test.exe <eurotrucks2.exe> says what is still
// found; tools/re/sigscan.cpp helps find what is not.
//
//   1.61.1.1s, 1.61.1.9s: first versions (camera, truck puppet).
//   1.62.1: the camera code after the hooked read changed (xor r12d,r12d -> xor edx,edx); all of
//           it moved. Found by the context pattern; its own pattern added.
#pragma once
#include <cstdint>
#include "codescan.h"

namespace ets2code {

struct Sig { const char* bytes; int at; };   // the spot is `at` bytes into the match

// Camera: the read of the camera's placement, movsx eax, word [rbx+0x4e] (rbx = the camera; its
// placement at +0x40, sector at +0x4c, FOV at +0x20). The hook rewrites them just before.
constexpr Sig kCameraSigs[] = {
    {"0f bf 43 4e 45 33 e4 f3 44 0f 10 4b 44", 0},   // 1.61: then xor r12d,r12d; movss xmm9,[rbx+0x44]
    {"0f bf 43 4e 33 d2 f3 44 0f 10 4b 44", 0},      // 1.62: then xor edx,edx; movss xmm9,[rbx+0x44]
    // Any version where only what follows changed: the call just before it.
    {"48 8b 8f 38 2e 00 00 b2 01 e8 ?? ?? ?? ?? 0f bf 43 4e", 14},
};
constexpr uintptr_t kCameraKnown[] = {0x47eb5d, 0x473de5, 0x473d55};   // 1.62.1, 1.61.1.9s, 1.61.1.1s

// Truck puppet. The body state loop: S = [rsi]; dt = [S+0xc]; S+0x10 placement, S+0x3c angular
// velocity. Before it the states array (mov rbx,[rdi+0x218]); after it P->setPlacement(S+0x10), a
// call through the actor's vtable: mov rdx,[rsi]; mov rcx,[rdx]; add rdx,0x10; mov rax,[rcx]; call [rax+slot].
constexpr char kLoopSig[] = "48 8b 06 f3 0f 10 40 0c 48 8d 58 10 4c 8d 48 3c";
constexpr uintptr_t kLoopKnown[] = {0x85cec0, 0x86e100, 0x86d6d0};      // 1.62.1, 1.61.1.9s, 1.61.1.1s
constexpr char kStatesSig[] = "48 8b 9f 18 02 00 00 48";
constexpr char kCallSig[] = "48 8b 16 48 8b 0a 48 83 c2 10 48 8b 01 ff 90";
// The telemetry thunk (its `call [rax+0x28]` 13 bytes in) and the placement channel getter.
constexpr char kThunkSig[] = "48 63 49 30 48 8d 54 24 20 48 03 48 20 ff 50 28 48 8b d0";
constexpr uintptr_t kThunkKnown[] = {0x5a4a1c, 0x5a7b59, 0x5a79e9};
constexpr int kThunkCall = 13;
constexpr char kGetterSig[] = "48 8b 01 ff a0 f8 00 00 00";
constexpr uintptr_t kGetterKnown[] = {0x5a4f2c, 0x5a805c, 0x5a7eec};
// setPlacement's first bytes: the function the actor vtable's slot holds (the actor class is learned
// from the player truck; its vtable was +0x249dfb0 in 1.62.1, +0x2449d48 in 1.61.1.9s).
constexpr char kSetPlacementSig[] = "48 89 5c 24 08 48 89 7c";
constexpr uintptr_t kActorVtablesSeen[] = {0x249dfb0, 0x2449d48};

template <size_t N> constexpr int Count(const uintptr_t (&)[N]) { return int(N); }

// The camera read in this game (0: not found).
inline uintptr_t FindCamera() {
    const uintptr_t base = codescan::Base();
    for (uintptr_t off : kCameraKnown)
        for (const Sig& s : kCameraSigs)
            if (s.at == 0 && codescan::Matches(base + off, s.bytes)) return base + off;
    for (const Sig& s : kCameraSigs)
        if (const uintptr_t a = codescan::FindUnique(s.bytes)) return a + s.at;
    return 0;
}

}  // namespace ets2code
