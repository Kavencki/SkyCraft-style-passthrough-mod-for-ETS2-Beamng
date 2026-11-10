// Ground: the drivable ground around the car as one BeamNG TerrainBlock (not drawn), the way
// BeamNG.Earth does it. The road generator writes a 1 m height grid per 256 m tile
// (--heightfield-dir, see tools/ets2_roads/Heightfield.cs); this stitches the 4 x 4 tiles around
// the car into a 1024 x 1024 block, fills where there is no ground (push-pull: smooth, no cliffs),
// and writes BeamNG's own terrain file (.ter v9). The mod swaps it into its TerrainBlock in a few
// ms, and the block's collision follows without a collision rebuild - unlike mesh tiles, whose
// every change rebuilt all static collision (a hitch). A new block every 256 m the car moves.
//
// Built on a worker thread: the bridge loop keeps feeding ETS2's camera meanwhile.
#pragma once
#include <windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "../protocol/mapping.h"

namespace es {

class Ground {
public:
    static constexpr int kTile = 256;    // samples (m) per generator tile
    static constexpr int kBlock = 1024;  // samples per block side (4 x 4 tiles)

    bool enabled = false;
    std::string hfDir;     // the generator's height tiles (outside BeamNG)
    std::string terDir;    // BeamNG user folder's temp\eurobeamng (real path)
    const char* terVfs = "/temp/eurobeamng/";

    ~Ground() { if (worker_.joinable()) worker_.join(); }

    // After a generation: the tile list (hf.idx). The next Update builds a new block.
    void LoadIndex() {
        std::lock_guard<std::mutex> lk(mu_);
        index_.clear();
        FILE* f = nullptr;
        if (fopen_s(&f, (hfDir + "\\hf.idx").c_str(), "r") || !f) return;
        char line[512];
        while (fgets(line, sizeof(line), f)) {
            int tx, tz;
            char name[400];
            if (sscanf_s(line, "%d %d %399s", &tx, &tz, name, unsigned(sizeof(name))) == 3) index_[{tx, tz}] = name;
        }
        fclose(f);
        dirty_ = true;
    }
    bool HasIndex() { std::lock_guard<std::mutex> lk(mu_); return !index_.empty(); }
    uint32_t Seq() const { return sentSeq_; }   // the last block sent (0: none yet)
    // A new anchor: the block BeamNG has is in the wrong place now; none counts until the next.
    void Invalidate() { dirty_ = true; sentSeq_ = 0; lastMsg_.clear(); }

    // Call every loop: starts building the block around (x, z) (ETS2) when needed; when one is
    // done, returns true with the message for BeamNG ("ESTR path x y z maxHeight square seq").
    bool Update(double x, double z, const Anchor& anchor, std::string& msg) {
        if (!enabled) return false;
        if (building_ && done_.load()) {
            worker_.join();
            building_ = false;
            done_ = false;
            if (!result_.empty()) { msg = result_; lastMsg_ = result_; sentSeq_ = resultSeq_; return true; }
        }
        if (building_) return false;
        const int cx = int(std::floor(x / kTile + 0.5)), cz = int(std::floor(z / kTile + 0.5));
        if (!dirty_ && cx == cx_ && cz == cz_) return false;
        if (!HasIndex()) return false;
        cx_ = cx; cz_ = cz;
        dirty_ = false;
        building_ = true;
        const uint32_t seq = ++seq_;
        worker_ = std::thread([this, cx, cz, anchor, seq] { result_ = Build(cx, cz, anchor, seq); resultSeq_ = seq; done_ = true; });
        return false;
    }
    // The last block's message (resent when BeamNG lost it, e.g. after a level or Lua reload).
    const std::string& LastMessage() const { return lastMsg_; }

private:
    struct Tile { float base = 0; std::vector<int16_t> h; std::vector<uint8_t> s; };
    std::mutex mu_;
    std::map<std::pair<int, int>, std::string> index_;
    std::map<std::string, Tile> cache_;   // by file name (content): a changed tile is a new name
    std::thread worker_;
    std::atomic<bool> done_{false};
    bool building_ = false, dirty_ = true;
    int cx_ = INT32_MIN, cz_ = INT32_MIN;
    uint32_t seq_ = 0, resultSeq_ = 0, sentSeq_ = 0;
    std::string result_, lastMsg_;

    const Tile* GetTile(int tx, int tz) {
        std::string name;
        {
            std::lock_guard<std::mutex> lk(mu_);
            auto it = index_.find({tx, tz});
            if (it == index_.end()) return nullptr;
            name = it->second;
        }
        auto c = cache_.find(name);
        if (c != cache_.end()) return &c->second;
        FILE* f = nullptr;
        if (fopen_s(&f, (hfDir + "\\" + name).c_str(), "rb") || !f) return nullptr;
        char magic[4];
        int32_t ver = 0, ftx = 0, ftz = 0, size = 0;
        float base = 0;
        Tile t;
        bool ok = fread(magic, 1, 4, f) == 4 && memcmp(magic, "ESHF", 4) == 0 && fread(&ver, 4, 1, f) == 1 && ver == 1 &&
                  fread(&ftx, 4, 1, f) == 1 && fread(&ftz, 4, 1, f) == 1 && fread(&size, 4, 1, f) == 1 && size == kTile &&
                  fread(&base, 4, 1, f) == 1;
        if (ok) {
            t.base = base;
            t.h.resize(size_t(kTile) * kTile);
            t.s.resize(size_t(kTile) * kTile);
            ok = fread(t.h.data(), 2, t.h.size(), f) == t.h.size() && fread(t.s.data(), 1, t.s.size(), f) == t.s.size();
        }
        fclose(f);
        if (!ok) return nullptr;
        if (cache_.size() > 96) cache_.clear();   // ~ 6 blocks' worth
        return &(cache_[name] = std::move(t));
    }

    // Fills NaN cells from the coarser levels of an averaging pyramid (smooth, no gaps).
    static void PushPull(std::vector<float>& v, int n) {
        std::vector<std::vector<float>> val{v}, wt;
        wt.emplace_back(v.size());
        for (size_t i = 0; i < v.size(); ++i) wt[0][i] = std::isnan(v[i]) ? 0.f : 1.f;
        for (size_t i = 0; i < v.size(); ++i) if (std::isnan(val[0][i])) val[0][i] = 0;
        std::vector<int> sizes{n};
        while (sizes.back() > 1) {
            const int s = sizes.back(), h = (s + 1) / 2;
            std::vector<float> nv(size_t(h) * h, 0.f), nw(size_t(h) * h, 0.f);
            const auto& pv = val.back();
            const auto& pw = wt.back();
            for (int y = 0; y < s; ++y)
                for (int x = 0; x < s; ++x) {
                    const float w = pw[size_t(y) * s + x];
                    if (w <= 0) continue;
                    const size_t k = size_t(y / 2) * h + x / 2;
                    nv[k] += pv[size_t(y) * s + x] * w;
                    nw[k] += w;
                }
            for (size_t k = 0; k < nv.size(); ++k) if (nw[k] > 0) nv[k] /= nw[k];
            val.push_back(std::move(nv));
            wt.push_back(std::move(nw));
            sizes.push_back(h);
        }
        // Down: a cell without data takes the coarser level, bilinearly.
        for (int L = int(sizes.size()) - 2; L >= 0; --L) {
            const int s = sizes[L], ps = sizes[L + 1];
            auto& cv = val[L];
            auto& cw = wt[L];
            const auto& pv = val[L + 1];
            for (int y = 0; y < s; ++y)
                for (int x = 0; x < s; ++x) {
                    const size_t k = size_t(y) * s + x;
                    if (cw[k] > 0) continue;
                    const float fx = std::fmax(0.f, std::fmin(float(ps - 1), (x + 0.5f) / 2 - 0.5f));
                    const float fy = std::fmax(0.f, std::fmin(float(ps - 1), (y + 0.5f) / 2 - 0.5f));
                    const int x0 = int(fx), y0 = int(fy), x1 = (std::min)(x0 + 1, ps - 1), yb = (std::min)(y0 + 1, ps - 1);
                    const float ax = fx - x0, ay = fy - y0;
                    cv[k] = (pv[size_t(y0) * ps + x0] * (1 - ax) + pv[size_t(y0) * ps + x1] * ax) * (1 - ay) +
                            (pv[size_t(yb) * ps + x0] * (1 - ax) + pv[size_t(yb) * ps + x1] * ax) * ay;
                    cw[k] = 1;
                }
        }
        v = std::move(val[0]);
    }

    std::string Build(int cx, int cz, const Anchor& anchor, uint32_t seq) {
        const int n = kBlock;
        const int tx0 = cx - 2, tz0 = cz - 2;
        const int bx0 = tx0 * kTile, bz0 = tz0 * kTile;   // ETS2 x, z of sample (0, 0)
        std::vector<float> h(size_t(n) * n, NAN);
        std::vector<uint8_t> s(size_t(n) * n, 1);
        int known = 0;
        for (int ti = 0; ti < 4; ++ti)
            for (int tj = 0; tj < 4; ++tj) {
                const Tile* t = GetTile(tx0 + ti, tz0 + tj);
                if (!t) continue;
                for (int z = 0; z < kTile; ++z)
                    for (int x = 0; x < kTile; ++x) {
                        const int16_t q = t->h[size_t(z) * kTile + x];
                        if (q == INT16_MIN) continue;
                        const size_t k = size_t(tj * kTile + z) * n + (ti * kTile + x);
                        h[k] = t->base + q / 100.f;
                        s[k] = t->s[size_t(z) * kTile + x] == 0 ? 0 : 1;
                        ++known;
                    }
            }
        if (known == 0) return "";
        PushPull(h, n);
        float lo = 1e30f, hi = -1e30f;
        for (float v : h) { lo = std::fmin(lo, v); hi = std::fmax(hi, v); }
        const double base = std::floor(lo) - 1;
        const double maxH = std::fmax(16.0, std::ceil(hi - base) + 1);
        // .ter v9: version, size, heights (u16), material per sample (u8), material names.
        // Row 0 = south: ETS2 z grows southwards, so row r is z = bz0 + n - 1 - r.
        std::vector<uint16_t> q(size_t(n) * n);
        std::vector<uint8_t> lay(size_t(n) * n);
        for (int r = 0; r < n; ++r) {
            const int j = n - 1 - r;
            for (int i = 0; i < n; ++i) {
                const double v = (h[size_t(j) * n + i] - base) * 65536.0 / maxH;
                q[size_t(r) * n + i] = uint16_t(std::fmax(0.0, std::fmin(65535.0, std::floor(v + 0.5))));
                lay[size_t(r) * n + i] = s[size_t(j) * n + i];
            }
        }
        CreateDirectoryA(terDir.c_str(), nullptr);
        char name[64];
        snprintf(name, sizeof(name), "ground_%u.ter", seq);
        const std::string path = terDir + "\\" + name, tmp = path + ".tmp";
        FILE* f = nullptr;
        if (fopen_s(&f, tmp.c_str(), "wb") || !f) return "";
        const uint8_t ver = 9;
        const uint32_t size = n, mats = 2;
        fwrite(&ver, 1, 1, f);
        fwrite(&size, 4, 1, f);
        fwrite(q.data(), 2, q.size(), f);
        fwrite(lay.data(), 1, lay.size(), f);
        fwrite(&mats, 4, 1, f);
        for (const char* m : {"eurobeamng_asphalt", "eurobeamng_grass"}) {
            const uint8_t len = uint8_t(strlen(m));
            fwrite(&len, 1, 1, f);
            fwrite(m, 1, len, f);
        }
        fclose(f);
        MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING);
        if (seq > 4) {   // older blocks are no longer used
            char old[64];
            snprintf(old, sizeof(old), "ground_%u.ter", seq - 4);
            DeleteFileA((terDir + "\\" + old).c_str());
        }
        // The block's south-west corner in BeamNG (TerrainBlock position).
        const Vec3d sw = EtsToSrc(anchor, Vec3d{double(bx0), base, double(bz0 + n - 1)});
        char msg[256];
        snprintf(msg, sizeof(msg), "ESTR %s%s %.3f %.3f %.3f %.3f 1 %u", terVfs, name, sw.x, sw.y, sw.z, maxH, seq);
        return msg;
    }
};

}  // namespace es
