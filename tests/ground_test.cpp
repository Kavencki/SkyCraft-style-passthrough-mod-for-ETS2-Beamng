// ground_test: builds a ground block (bridge/ground.h) from a generator's height tiles and checks
// the .ter written: size, and heights at the samples against the tiles themselves.
//   ground_test <heightfield dir> <out dir> <ets2 x> <ets2 z>
#include <cstdio>
#include <cstdlib>
#include <string>
#include "../bridge/ground.h"

using namespace es;

int main(int argc, char** argv) {
    if (argc < 5) { printf("usage: ground_test <hf dir> <out dir> <x> <z>\n"); return 2; }
    Ground g;
    g.enabled = true;
    g.hfDir = argv[1];
    g.terDir = argv[2];
    g.LoadIndex();
    const double x = atof(argv[3]), z = atof(argv[4]);
    EtsPlacement truck{};
    truck.pos = {x, 0, z};
    Anchor a = MakeAnchor(Vec3d{0, 0, 500}, 0, truck);
    std::string msg;
    for (int i = 0; i < 5000 && !g.Update(x, z, a, msg); ++i) Sleep(2);
    if (msg.empty()) { printf("no block\n"); return 1; }
    printf("%s\n", msg.c_str());
    char path[256];
    double bx, by, bz, maxH, sq;
    unsigned seq;
    if (sscanf_s(msg.c_str(), "ESTR %255s %lf %lf %lf %lf %lf %u", path, unsigned(sizeof(path)), &bx, &by, &bz, &maxH, &sq, &seq) != 7) return 1;
    const std::string file = std::string(argv[2]) + "\\" + (strrchr(path, '/') + 1);
    FILE* f = nullptr;
    if (fopen_s(&f, file.c_str(), "rb") || !f) { printf("cannot open %s\n", file.c_str()); return 1; }
    uint8_t ver; uint32_t n;
    fread(&ver, 1, 1, f); fread(&n, 4, 1, f);
    std::vector<uint16_t> q(size_t(n) * n);
    std::vector<uint8_t> lay(size_t(n) * n);
    fread(q.data(), 2, q.size(), f); fread(lay.data(), 1, lay.size(), f);
    uint32_t mats; fread(&mats, 4, 1, f);
    printf(".ter v%u, %u x %u, %u materials, maxHeight %.1f, corner (%.1f, %.1f, %.2f)\n", ver, n, n, mats, maxH, bx, by, bz);
    // Every sample of the tiles in the block vs the .ter (BeamNG height minus the anchor's 500).
    const int cx = int(std::floor(x / 256 + 0.5)), cz = int(std::floor(z / 256 + 0.5));
    const int bx0 = (cx - 2) * 256, bz0 = (cz - 2) * 256;
    double maxErr = 0; long checked = 0; int asphalt = 0;
    for (int r = 0; r < int(n); ++r)
        for (int c = 0; c < int(n); ++c) {
            const double h = bz + q[size_t(r) * n + c] * maxH / 65536.0 - 500;   // ETS2 height
            const int ex = bx0 + c, ez = bz0 + int(n) - 1 - r;
            // The tile file's own value there.
            const int tx = int(std::floor(ex / 256.0)), tz = int(std::floor(ez / 256.0));
            static std::vector<int16_t> th; static float tb = 0; static std::string lastKey;
            const std::string key = std::to_string(tx) + " " + std::to_string(tz) + " ";
            if (key != lastKey) {
                lastKey = key; th.clear();
                // The tile's current file (hf.idx; older versions stay in the folder for a while).
                std::string fname;
                FILE* ix = nullptr;
                if (!fopen_s(&ix, (std::string(argv[1]) + "\\hf.idx").c_str(), "r") && ix) {
                    char line[512];
                    while (fgets(line, sizeof(line), ix))
                        if (strncmp(line, key.c_str(), key.size()) == 0) { fname = line + key.size(); break; }
                    fclose(ix);
                }
                while (!fname.empty() && (fname.back() == '\n' || fname.back() == '\r')) fname.pop_back();
                FILE* t = nullptr;
                if (!fname.empty() && !fopen_s(&t, (std::string(argv[1]) + "\\" + fname).c_str(), "rb") && t) {
                    fseek(t, 20, SEEK_SET); fread(&tb, 4, 1, t);
                    th.resize(65536); fread(th.data(), 2, th.size(), t); fclose(t);
                }
            }
            if (th.empty()) continue;
            const int16_t v = th[size_t(ez - tz * 256) * 256 + (ex - tx * 256)];
            if (v == INT16_MIN) continue;
            const double e = std::fabs(h - (tb + v / 100.0));
            if (e > maxErr) maxErr = e;
            ++checked;
            if (lay[size_t(r) * n + c] == 0) ++asphalt;
        }
    printf("checked %ld samples against the tiles: max error %.4f m; asphalt samples %d\n", checked, maxErr, asphalt);
    return maxErr < 0.02 ? 0 : 1;
}
