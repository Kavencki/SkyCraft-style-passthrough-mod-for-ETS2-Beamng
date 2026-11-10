// Finds the game's code by its bytes. A game update moves code around (ETS2 1.61.1.1s -> 1.61.1.9s
// moved everything the plugin hooks by 0x90 - 0x1e30 bytes) but rarely changes the code itself, so
// the plugin looks for the code instead of fixed addresses (tools/re/sigscan.cpp does the same on
// the exe file, to check a new version).
#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace codescan {

struct Range { uintptr_t begin = 0, end = 0; bool Has(uintptr_t a) const { return a >= begin && a < end; } };

// The game's exe (tests set another image).
inline HMODULE g_module = nullptr;
inline uintptr_t Base() { return reinterpret_cast<uintptr_t>(g_module ? g_module : GetModuleHandleA(nullptr)); }

// A section of the game's exe in memory (".text", ".rdata").
inline Range Section(const char* name) {
    const auto* base = reinterpret_cast<const uint8_t*>(Base());
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    const IMAGE_SECTION_HEADER* s = IMAGE_FIRST_SECTION(nt);
    for (int i = 0; i < nt->FileHeader.NumberOfSections; ++i, ++s)
        if (strncmp(reinterpret_cast<const char*>(s->Name), name, 8) == 0)
            return {uintptr_t(base) + s->VirtualAddress, uintptr_t(base) + s->VirtualAddress + s->Misc.VirtualSize};
    return {};
}

constexpr int kMaxLen = 64;
struct Sig { int16_t b[kMaxLen]; int n = 0; };   // -1 = any byte

// "0f bf 43 ?? 45": ?? = any byte.
inline Sig Parse(const char* text) {
    Sig s;
    for (const char* c = text; *c && s.n < kMaxLen;) {
        if (*c == ' ') { ++c; continue; }
        if (c[0] == '?') { s.b[s.n++] = -1; c += 2; continue; }
        char hex[3] = {c[0], c[1], 0};
        s.b[s.n++] = int16_t(strtol(hex, nullptr, 16));
        c += 2;
    }
    return s;
}

namespace detail {
// Matches in [from, to); stops after `max`. (No C++ objects here: the scan runs under SEH, a page
// that cannot be read ends it.)
inline int Scan(uintptr_t from, uintptr_t to, const Sig* s, uintptr_t* out, int max) {
    int found = 0;
    __try {
        for (uintptr_t a = from; a + s->n <= to && found < max; ++a) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(a);
            if (s->b[0] >= 0 && p[0] != uint8_t(s->b[0])) continue;
            int i = 1;
            while (i < s->n && (s->b[i] < 0 || p[i] == uint8_t(s->b[i]))) ++i;
            if (i == s->n) out[found++] = a;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {}
    return found;
}
}  // namespace detail

// The code is at `at`.
inline bool Matches(uintptr_t at, const char* sig) {
    const Sig s = Parse(sig);
    uintptr_t hit = 0;
    return detail::Scan(at, at + s.n, &s, &hit, 1) == 1;
}

// Where the code is in [from, to) (the game's .text if 0): its address if there is exactly one
// place, else 0 (none, or several: not sure which).
inline uintptr_t FindUnique(const char* sig, uintptr_t from = 0, uintptr_t to = 0) {
    if (!from) { const Range t = Section(".text"); from = t.begin; to = t.end; }
    const Sig s = Parse(sig);
    uintptr_t hits[2] = {0, 0};
    return detail::Scan(from, to, &s, hits, 2) == 1 ? hits[0] : 0;
}

// Known places first (game versions seen), else a search. offs: offsets from the exe's base.
inline uintptr_t Locate(const char* sig, const uintptr_t* known, int nKnown) {
    const uintptr_t base = Base();
    for (int i = 0; i < nKnown; ++i)
        if (Matches(base + known[i], sig)) return base + known[i];
    return FindUnique(sig);
}

}  // namespace codescan
