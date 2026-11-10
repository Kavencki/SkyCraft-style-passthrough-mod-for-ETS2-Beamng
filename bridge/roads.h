// CollisionField for BeamNG: ETS2 road strips (from tools/ets2_roads) streamed
// around the car. BeamNG's mod turns each strip into a solid MeshRoad.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>
#include "../protocol/mapping.h"

namespace es {

struct RoadStrip {
    uint32_t kind;     // 0 road, 1 junction link, 2 barrier (points = barrier top)
    float    width;
    std::vector<Vec3d> pts;  // ETS2 world
    double   cx, cz, r;      // bounding circle (ETS2 x/z)
    int      tx = 0, tz = 0;  // mesh tiles: grid cell
    uint64_t hash = 0;        // mesh tiles: content hash (0 = unknown); equal = same tile
};

inline bool LoadRoads(const char* path, std::vector<RoadStrip>& out, std::string& err) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") || !f) { err = "cannot open"; return false; }
    char magic[4];
    uint32_t version = 0, count = 0;
    bool ok = fread(magic, 1, 4, f) == 4 && memcmp(magic, "ESRD", 4) == 0 && fread(&version, 4, 1, f) == 1 &&
              (version == 1 || version == 2) && fread(&count, 4, 1, f) == 1;
    if (!ok) { fclose(f); err = "not a roads.bin v1/v2"; return false; }
    out.clear();
    out.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        RoadStrip s{};
        uint32_t n = 0;
        if (fread(&s.kind, 4, 1, f) != 1 || fread(&s.width, 4, 1, f) != 1 || fread(&n, 4, 1, f) != 1 || n > 100000) {
            fclose(f); err = "truncated"; return false;
        }
        std::vector<float> raw(size_t(n) * 3);
        if (fread(raw.data(), 4, raw.size(), f) != raw.size()) { fclose(f); err = "truncated"; return false; }
        s.pts.resize(n);
        double minx = 1e30, maxx = -1e30, minz = 1e30, maxz = -1e30;
        for (uint32_t k = 0; k < n; ++k) {
            s.pts[k] = {raw[k * 3], raw[k * 3 + 1], raw[k * 3 + 2]};
            minx = std::fmin(minx, s.pts[k].x); maxx = std::fmax(maxx, s.pts[k].x);
            minz = std::fmin(minz, s.pts[k].z); maxz = std::fmax(maxz, s.pts[k].z);
        }
        s.cx = (minx + maxx) / 2; s.cz = (minz + maxz) / 2;
        s.r = std::sqrt((maxx - minx) * (maxx - minx) + (maxz - minz) * (maxz - minz)) / 2 + s.width;
        out.push_back(std::move(s));
    }
    fclose(f);
    return true;
}

// Road mesh tiles (tools/ets2_roads writes tiles.idx: "tx tz originX originZ vfsPath" per line).
// Each tile becomes a RoadStrip with no points (centre + radius only) so the same streamer works.
// groundMode: the index says "#ground" - its tiles need the ground block (made with height tiles).
// area: "#area x z radius", the circle generated (r = 0: not in the index).
struct TileArea { double x = 0, z = 0, r = 0; };
inline bool LoadTiles(const char* path, std::vector<RoadStrip>& out, std::vector<std::string>& paths, std::string& err,
                      bool* groundMode = nullptr, TileArea* area = nullptr) {
    if (groundMode) *groundMode = false;
    if (area) *area = TileArea{};
    FILE* f = nullptr;
    if (fopen_s(&f, path, "r") || !f) { err = "cannot open"; return false; }
    out.clear();
    paths.clear();
    char line[1024];
    double tileSize = 512;   // "#tile <size>" header (older indexes: 512 m)
    while (fgets(line, sizeof(line), f)) {
        if (sscanf_s(line, "#tile %lf", &tileSize) == 1) continue;
        if (strncmp(line, "#ground", 7) == 0) { if (groundMode) *groundMode = true; continue; }
        TileArea a;
        if (sscanf_s(line, "#area %lf %lf %lf", &a.x, &a.z, &a.r) == 3) { if (area) *area = a; continue; }
        int tx, tz;
        double ox, oz;
        char vfs[768];
        unsigned long long hash = 0;
        if (sscanf_s(line, "%d %d %lf %lf %767s %llx", &tx, &tz, &ox, &oz, vfs, unsigned(sizeof(vfs)), &hash) < 5) continue;
        RoadStrip s{};
        s.tx = tx;
        s.tz = tz;
        s.hash = hash;
        s.kind = 9;
        s.cx = ox;
        s.cz = oz;
        s.r = tileSize * 0.7072;  // half diagonal of the tile
        out.push_back(std::move(s));
        paths.emplace_back(vfs);
    }
    fclose(f);
    if (out.empty()) { err = "no tiles"; return false; }
    return true;
}

// Runs tools/ets2_roads in the background to generate road tiles around a point of the map
// (used when the truck is somewhere no tiles exist yet). Its output shows in the bridge console.
struct RoadExporter {
    HANDLE proc = nullptr;
    bool ok = false;
    std::string heightfieldDir;   // non-empty: the generator also writes ground height tiles there (Ground)
    bool lastWithHeightfields = false;

    bool Running() const { return proc != nullptr; }

    // tilesDir/tilesVfs: where the tiles go (empty = the generator's default folder).
    void Start(const std::string& exeDir, double x, double z, const std::string& outBin, double radius = 12000,
               const std::string& tilesDir = "", const std::string& tilesVfs = "") {
        // The generator's own exe (a release carries .NET inside it), else the .dll through dotnet.
        char cmd[3072];
        int len;
        const std::string exe = exeDir + "\\ets2_roads\\ets2_roads.exe";
        if (GetFileAttributesA(exe.c_str()) != INVALID_FILE_ATTRIBUTES) {
            len = snprintf(cmd, sizeof(cmd), "\"%s\" --x %.0f --z %.0f --radius %.0f --out \"%s\"", exe.c_str(), x, z, radius, outBin.c_str());
        } else {
            std::string dotnet = "C:\\Program Files\\dotnet\\dotnet.exe";
            if (GetFileAttributesA(dotnet.c_str()) == INVALID_FILE_ATTRIBUTES) dotnet = "dotnet";
            len = snprintf(cmd, sizeof(cmd), "\"%s\" \"%s\\ets2_roads\\ets2_roads.dll\" --x %.0f --z %.0f --radius %.0f --out \"%s\"",
                           dotnet.c_str(), exeDir.c_str(), x, z, radius, outBin.c_str());
        }
        if (!tilesDir.empty() && len > 0 && len < int(sizeof(cmd)))
            len += snprintf(cmd + len, sizeof(cmd) - len, " --tiles-dir \"%s\" --tiles-vfs \"%s\"", tilesDir.c_str(), tilesVfs.c_str());
        lastWithHeightfields = !heightfieldDir.empty();
        if (lastWithHeightfields && len > 0 && len < int(sizeof(cmd)))
            snprintf(cmd + len, sizeof(cmd) - len, " --heightfield-dir \"%s\"", heightfieldDir.c_str());
        STARTUPINFOA si{};
        si.cb = sizeof(si);
        PROCESS_INFORMATION pi{};
        printf("[bridge] generating roads around ETS2 (%.0f, %.0f), this takes ~20 s ...\n", x, z);
        // Below normal priority: generating must not starve BeamNG and ETS2 (frame drops, gaps).
        if (CreateProcessA(nullptr, cmd, nullptr, nullptr, FALSE, BELOW_NORMAL_PRIORITY_CLASS, nullptr, nullptr, &si, &pi)) {
            CloseHandle(pi.hThread);
            proc = pi.hProcess;
        } else {
            printf("[bridge] cannot start the road generator (%lu): %s\n", GetLastError(), cmd);
        }
    }

    // True once, when a started export has finished (ok = exit code 0).
    bool Done() {
        if (!proc || WaitForSingleObject(proc, 0) != WAIT_OBJECT_0) return false;
        DWORD code = 1;
        GetExitCodeProcess(proc, &code);
        CloseHandle(proc);
        proc = nullptr;
        ok = code == 0;
        return true;
    }
};

// Decides which strips BeamNG should hold around the car (hysteresis: load < unload).
// Updates come in batches - only after the car has moved `step` metres - so BeamNG builds
// and rebuilds its collision rarely, well before the car gets there.
struct RoadStreamer {
    // BeamNG rebuilds the collision of every loaded tile when one is added, a hitch that grows
    // with what is loaded: only what is near the car (256 m tiles within 450 m; dropped beyond
    // 650 m). Updates every 200 m come in batches (one rebuild per batch) and still leave
    // 250+ m of loaded road ahead (~5 s at 200 km/h).
    double loadRadius = 450, unloadRadius = 650, step = 200;
    double lastX = 1e30, lastZ = 1e30;
    std::unordered_set<uint32_t> loaded;

    bool Due(double x, double z) const {
        return (x - lastX) * (x - lastX) + (z - lastZ) * (z - lastZ) >= step * step;
    }
    void Reset() { loaded.clear(); lastX = lastZ = 1e30; }

    // Fills `toLoad` / `toUnload` (strip indices), at most `maxLoads` loads per call.
    void Update(const std::vector<RoadStrip>& strips, double x, double z, size_t maxLoads,
                std::vector<uint32_t>& toLoad, std::vector<uint32_t>& toUnload) {
        toLoad.clear();
        toUnload.clear();
        for (uint32_t i = 0; i < strips.size(); ++i) {
            const RoadStrip& s = strips[i];
            const double d = std::sqrt((s.cx - x) * (s.cx - x) + (s.cz - z) * (s.cz - z)) - s.r;
            const bool isLoaded = loaded.count(i) != 0;
            if (!isLoaded && d < loadRadius && toLoad.size() < maxLoads) toLoad.push_back(i);
            else if (isLoaded && d > unloadRadius) toUnload.push_back(i);
        }
        for (uint32_t i : toLoad) loaded.insert(i);
        for (uint32_t i : toUnload) loaded.erase(i);
        lastX = x;
        lastZ = z;
    }
};

}  // namespace es
