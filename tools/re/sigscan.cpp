// sigscan: finds the code the ETS2 plugin hooks in an eurotrucks2.exe (after a game update, the
// addresses move). Prints each pattern's matches as RVAs, and pairs that keep their old distance.
//   sigscan <eurotrucks2.exe>
#include <windows.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

struct Section { char name[9]; uint32_t rva, vsize, raw, rawSize; };

static std::vector<uint8_t> img;          // the file
static std::vector<Section> secs;
static uint64_t imageBase = 0;

static bool Load(const char* path) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") || !f) return false;
    fseek(f, 0, SEEK_END);
    img.resize(size_t(ftell(f)));
    fseek(f, 0, SEEK_SET);
    fread(img.data(), 1, img.size(), f);
    fclose(f);
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(img.data());
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS64*>(img.data() + dos->e_lfanew);
    imageBase = nt->OptionalHeader.ImageBase;
    auto* s = IMAGE_FIRST_SECTION(nt);
    for (int i = 0; i < nt->FileHeader.NumberOfSections; ++i, ++s) {
        Section x{};
        memcpy(x.name, s->Name, 8);
        x.rva = s->VirtualAddress; x.vsize = s->Misc.VirtualSize; x.raw = s->PointerToRawData; x.rawSize = s->SizeOfRawData;
        secs.push_back(x);
    }
    return true;
}
static int64_t RvaToFile(uint64_t rva) {
    for (auto& s : secs) if (rva >= s.rva && rva < s.rva + s.rawSize) return int64_t(rva - s.rva + s.raw);
    return -1;
}
static int64_t FileToRva(uint64_t off) {
    for (auto& s : secs) if (off >= s.raw && off < s.raw + s.rawSize) return int64_t(off - s.raw + s.rva);
    return -1;
}
// "48 8b ?? 10": ?? = any byte.
static void Parse(const char* p, std::vector<int>& out) {
    out.clear();
    for (const char* c = p; *c;) {
        while (*c == ' ') ++c;
        if (!*c) break;
        if (c[0] == '?') { out.push_back(-1); c += 2; continue; }
        out.push_back(int(strtol(std::string(c, 2).c_str(), nullptr, 16)));
        c += 2;
    }
}
static bool MatchAt(uint64_t rva, const std::vector<int>& pat) {
    const int64_t o = RvaToFile(rva);
    if (o < 0 || size_t(o) + pat.size() > img.size()) return false;
    for (size_t i = 0; i < pat.size(); ++i) if (pat[i] >= 0 && img[size_t(o) + i] != uint8_t(pat[i])) return false;
    return true;
}
static std::vector<uint64_t> Find(const char* sig, const char* section = ".text") {
    std::vector<int> pat;
    Parse(sig, pat);
    std::vector<uint64_t> hits;
    for (auto& s : secs) {
        if (strcmp(s.name, section) != 0) continue;
        const size_t end = s.raw + s.rawSize - pat.size();
        for (size_t o = s.raw; o < end; ++o) {
            if (pat[0] >= 0 && img[o] != uint8_t(pat[0])) continue;
            size_t i = 1;
            while (i < pat.size() && (pat[i] < 0 || img[o + i] == uint8_t(pat[i]))) ++i;
            if (i == pat.size()) hits.push_back(uint64_t(FileToRva(o)));
        }
    }
    return hits;
}
static void Dump(uint64_t rva, int before, int after) {
    const int64_t o = RvaToFile(rva);
    printf("   %llx:", (unsigned long long)rva);
    for (int i = -before; i < after; ++i) printf(i == 0 ? " [%02x" : (i == 1 ? "] %02x" : " %02x"), img[size_t(o + i)]);
    printf("\n");
}
static void Report(const char* name, const char* sig, uint64_t oldRva) {
    auto h = Find(sig);
    printf("%-26s old %8llx  matches %zu:", name, (unsigned long long)oldRva, h.size());
    for (size_t i = 0; i < h.size() && i < 8; ++i) printf(" %llx (%+lld)", (unsigned long long)h[i], (long long)(h[i] - oldRva));
    printf("\n");
}

int main(int argc, char** argv) {
    if (argc < 2 || !Load(argv[1])) { printf("usage: sigscan <eurotrucks2.exe> [qwords <rva> <count>]\n"); return 2; }
    if (argc >= 5 && strcmp(argv[2], "qwords") == 0) {
        // The 8-byte values from rva on, as RVAs where they point into the image (.text / .rdata).
        const uint64_t at = strtoull(argv[3], nullptr, 16);
        const int n = atoi(argv[4]);
        for (int i = 0; i < n; ++i) {
            uint64_t v = 0;
            memcpy(&v, &img[size_t(RvaToFile(at + 8ull * i))], 8);
            const char* where = "";
            if (v >= imageBase) for (auto& s : secs) if (v - imageBase >= s.rva && v - imageBase < s.rva + s.vsize) where = s.name;
            printf("%llx: %016llx %s%s%llx\n", (unsigned long long)(at + 8ull * i), (unsigned long long)v, where,
                   *where ? " +" : "", (unsigned long long)(*where ? v - imageBase : 0));
        }
        return 0;
    }
    if (argc >= 5 && strcmp(argv[2], "bytes") == 0) {
        // The bytes around an RVA: bytes <rva> <before> [after]
        const uint64_t at = strtoull(argv[3], nullptr, 16);
        Dump(at, atoi(argv[4]), argc >= 6 ? atoi(argv[5]) : 48);
        return 0;
    }
    if (argc >= 4 && strcmp(argv[2], "find") == 0) {
        // Every match of the patterns in .text ("48 8b ?? 10").
        for (int a = 3; a < argc; ++a) {
            auto h = Find(argv[a]);
            printf("%zu matches of %s:", h.size(), argv[a]);
            for (size_t i = 0; i < h.size() && i < 10; ++i) printf(" %llx", (unsigned long long)h[i]);
            printf("\n");
        }
        return 0;
    }
    if (argc >= 4 && strcmp(argv[2], "refs") == 0) {
        // Code that addresses these RVAs RIP-relative (lea / mov [rip+disp32]): e.g. who installs a vtable.
        for (int a = 3; a < argc; ++a) {
            const uint64_t want = strtoull(argv[a], nullptr, 16);
            int n = 0;
            for (auto& s : secs) {
                if (strcmp(s.name, ".text") != 0) continue;
                for (size_t o = s.raw; o + 4 <= s.raw + s.rawSize; ++o) {
                    int32_t d;
                    memcpy(&d, &img[o], 4);
                    const uint64_t r = uint64_t(FileToRva(o));
                    if (r + 4 + int64_t(d) == want && o >= 3) {
                        if (n < 6) { printf("%llx <- ", (unsigned long long)want); Dump(r - 3, 0, 12); }
                        ++n;
                    }
                }
            }
            printf("%llx: %d references\n", (unsigned long long)want, n);
        }
        return 0;
    }
    printf("image base %llx, %zu sections\n", (unsigned long long)imageBase, secs.size());
    for (auto& s : secs) printf("  %-8s rva %8x size %8x\n", s.name, s.rva, s.vsize);

    // camhook: the camera placement read.
    Report("camera", "0f bf 43 4e 45 33 e4 f3 44 0f 10 4b 44", 0x473d55);
    // truckpuppet: the body state loop, one 16-byte run (0x86d6d0) and its neighbours.
    Report("state loop", "48 8b 06 f3 0f 10 40 0c 48 8d 58 10 4c 8d 48 3c", 0x86d6d0);
    Report("states array", "48 8b 9f 18 02 00 00 48", 0x86d655);
    Report("setPlacement call", "48 8b 16 48 8b 0a 48 83", 0x86d756);
    Report("telemetry thunk", "ff 50 28 48 8b d0 48 8b", 0x5a79f6);
    Report("placement getter", "48 8b 01 ff a0 f8 00 00", 0x5a7eec);
    Report("setPlacement", "48 89 5c 24 08 48 89 7c", 0x163bd40);

    // The loop's neighbours at their old distances from it.
    for (uint64_t loop : Find("48 8b 06 f3 0f 10 40 0c 48 8d 58 10 4c 8d 48 3c")) {
        std::vector<int> a, b;
        Parse("48 8b 9f 18 02 00 00 48", a);
        Parse("48 8b 16 48 8b 0a 48 83", b);
        printf("loop %llx: states array at -0x7b %s, setPlacement call at +0x86 %s\n", (unsigned long long)loop,
               MatchAt(loop - 0x7b, a) ? "yes" : "NO", MatchAt(loop + 0x86, b) ? "yes" : "NO");
        Dump(loop - 0x7b, 0, 16);
        Dump(loop + 0x86, 0, 32);
    }
    // Thunk + getter pairs 0x4f6 apart (as in 1.61.1.1s).
    {
        std::vector<int> g;
        Parse("48 8b 01 ff a0 f8 00 00", g);
        for (uint64_t t : Find("ff 50 28 48 8b d0 48 8b"))
            if (MatchAt(t + 0x4f6, g)) { printf("thunk %llx + getter %llx (0x4f6 apart)\n", (unsigned long long)t, (unsigned long long)(t + 0x4f6)); Dump(t, 24, 24); }
    }
    // The camera site's surroundings.
    for (uint64_t c : Find("0f bf 43 4e 45 33 e4 f3 44 0f 10 4b 44")) Dump(c, 32, 32);
    // The physics actor's vtable (.rdata): its +0x110 entry is setPlacement (1.61.1.1s: vtable
    // 0x2448928, setPlacement 0x163bd40). Every setPlacement-like function near the old one that
    // some .rdata slot points to, and the vtable that slot would belong to.
    for (uint64_t sp : Find("48 89 5c 24 08 48 89 7c")) {
        if (sp + 0x40000 < 0x163bd40 || sp > 0x163bd40 + 0x40000) continue;
        const uint64_t va = imageBase + sp;
        for (auto& s : secs) {
            if (strcmp(s.name, ".rdata") != 0) continue;
            for (size_t o = s.raw; o + 8 <= s.raw + s.rawSize; o += 8)
                if (memcmp(&img[o], &va, 8) == 0) {
                    const uint64_t slot = uint64_t(FileToRva(o));
                    printf("setPlacement? %llx (%+lld): pointed to from %llx -> vtable %llx (%+lld) if it is entry +0x110\n",
                           (unsigned long long)sp, (long long)(sp - 0x163bd40), (unsigned long long)slot,
                           (unsigned long long)(slot - 0x110), (long long)(slot - 0x110 - 0x2448928));
                }
        }
    }
    return 0;
}
