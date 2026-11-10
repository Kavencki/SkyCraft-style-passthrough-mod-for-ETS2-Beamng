// lfs_bridge.exe - the physics-side half of EuroSpeed.
//
// Receives the car from a physics game and publishes it to ETS2:
//   BeamNG  - eurospeed.lua mod, UDP 127.0.0.1:30001 (preferred when streaming)
//   LFS     - OutSim, UDP 127.0.0.1:30000 (LFS has no code loader)
// The car is anchored to the ETS2 truck's pose at link time; motion relative to
// the anchor becomes the ETS2 vehicle target and camera target.
//
//   lfs_bridge [--port 30000] [--bng-port 30001] [--opts 3f] [--seconds N] [--quiet]
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <conio.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "../protocol/eurospeed_protocol.h"
#include "../protocol/mapping.h"
#include "roads.h"
#include "wheel.h"
#include "keyrouter.h"
#include "ground.h"
#include <algorithm>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#pragma comment(lib, "ws2_32.lib")

using namespace es;

static bool PeerAlive(uint64_t hb) {
    return hb != 0 && GetTickCount64() - hb < kHeartbeatTimeoutMs;
}

static SOCKET BindUdp(int port) {
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<u_short>(port));
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        printf("[bridge] cannot bind UDP 127.0.0.1:%d (error %d)\n", port, WSAGetLastError());
        closesocket(s);
        return INVALID_SOCKET;
    }
    u_long nb = 1;
    ioctlsocket(s, FIONBIO, &nb);
    return s;
}

enum class Source { None, Lfs, BeamNG };

// ETS2's day on BeamNG's ([time] in wheel.ini). BeamNG's sun follows its clock evenly: it rises at
// 06:00 and sets at 18:00. ETS2's sun follows the map position and sets much later (19:00 is
// still golden hour), so ETS2's clock is mapped: its sunrise -> 06:00, its sunset -> 18:00.
// The key box glows less in the dark: BeamNG's auto exposure follows the bright box, which kept
// the cabin black at dusk; a dimmer box lets it open up, and the box itself looks the same.
struct DayMap {
    int sunrise = 5 * 60 + 30, sunset = 20 * 60 + 30;   // ETS2 minutes after midnight
    double dayNits = 600, nightNits = 40;
    int DayLength() const { return (sunset - sunrise + 1440) % 1440; }
    int ToBeamng(int m) const {
        const int day = DayLength(), d = (m - sunrise + 1440) % 1440;
        if (day <= 0) return m;
        if (d < day) return (360 + d * 720 / day) % 1440;                 // 06:00 .. 18:00
        return (1080 + (d - day) * 720 / (1440 - day)) % 1440;            // 18:00 .. 06:00
    }
    // Full day glow from an hour after sunrise to an hour before sunset, night glow from an hour
    // after sunset to an hour before sunrise, in between a smooth (geometric) change.
    double KeyNits(int m) const {
        const int day = DayLength(), d = (m - sunrise + 1440) % 1440;
        double light;
        if (d < day) light = 0.5 + 0.5 * (std::min)(1.0, (std::min)(d, day - d) / 60.0);
        else light = 0.5 - 0.5 * (std::min)(1.0, (std::min)(d - day, 1440 - d) / 60.0);
        return nightNits * std::pow(dayNits / nightNits, light);
    }
};

// Road left ahead of (x, z) moving along (dx, dz) inside the circle (cx, cz, r): where the line
// leaves it. No direction: to the nearest edge.
static double AheadInArea(double x, double z, double dx, double dz, double cx, double cz, double r) {
    const double px = x - cx, pz = z - cz, d2 = px * px + pz * pz;
    if (dx == 0 && dz == 0) return r - std::sqrt(d2);
    const double b = px * dx + pz * dz, disc = b * b - (d2 - r * r);
    return disc < 0 ? 0 : (std::max)(0.0, -b + std::sqrt(disc));
}

static DayMap ReadDayMap(const char* ini) {
    DayMap dm;
    auto time = [&](const char* key, int& out) {
        char v[32] = "";
        int h = 0, mi = 0;
        GetPrivateProfileStringA("time", key, "", v, sizeof(v), ini);
        if (sscanf_s(v, "%d:%d", &h, &mi) == 2 && h >= 0 && h < 24 && mi >= 0 && mi < 60) out = h * 60 + mi;
    };
    auto number = [&](const char* key, double& out) {
        char v[32] = "";
        GetPrivateProfileStringA("time", key, "", v, sizeof(v), ini);
        if (atof(v) > 0) out = atof(v);
    };
    time("ets2_sunrise", dm.sunrise);
    time("ets2_sunset", dm.sunset);
    number("key_nits_day", dm.dayNits);
    number("key_nits_night", dm.nightNits);
    return dm;
}

int main(int argc, char** argv) {
    int port = 30000, bngPort = 30001;
    uint32_t opts = kOutSimOpts;
    int seconds = 0;
    bool quiet = false;
    // Default: ETS2 keeps focus; the driver's ETS2 controls (wheel/keys/pedals, gears,
    // parking brake) drive the BeamNG car. --beamng-input: drive in the BeamNG window instead.
    bool ets2Input = true;
    const char* roadsPath = nullptr;  // ETS2 road strips for BeamNG collision (tools/ets2_roads)
    const char* tilesPath = nullptr;  // tiles.idx of road mesh tiles (preferred over --roads)
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--ets2-input")) { ets2Input = true; continue; }
        if (!strcmp(argv[i], "--beamng-input")) { ets2Input = false; continue; }
        if (!strcmp(argv[i], "--roads") && i + 1 < argc) { roadsPath = argv[++i]; continue; }
        if (!strcmp(argv[i], "--tiles") && i + 1 < argc) { tilesPath = argv[++i]; continue; }
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bng-port") && i + 1 < argc) bngPort = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--opts") && i + 1 < argc) opts = strtoul(argv[++i], nullptr, 16);
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--quiet")) quiet = true;
        else { printf("usage: lfs_bridge [--port N] [--bng-port N] [--opts hex] [--seconds N] [--quiet]\n"); return 2; }
    }

    setvbuf(stdout, nullptr, _IONBF, 0);  // logs appear immediately even when redirected

    HANDLE map = nullptr;
    Shared* sh = OpenShared(&map);
    if (!sh) { printf("[bridge] cannot open shared memory (version mismatch?)\n"); return 1; }
    sh->bridgePid.store(GetCurrentProcessId());
    sh->bridgeHeartbeatMs.store(GetTickCount64());

    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    SOCKET lfsSock = BindUdp(port);
    SOCKET bngSock = BindUdp(bngPort);
    if (lfsSock == INVALID_SOCKET || bngSock == INVALID_SOCKET) return 1;
    // ETS2 driver inputs -> BeamNG mod (it listens on bngPort + 1).
    SOCKET inSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in bngIn{};
    bngIn.sin_family = AF_INET;
    bngIn.sin_port = htons(static_cast<u_short>(bngPort + 1));
    inet_pton(AF_INET, "127.0.0.1", &bngIn.sin_addr);
    uint32_t lastInputSeq = 0, inputsSent = 0;
    // Freeze report: the last few messages to BeamNG (not the per-frame input), to name what was
    // going on when BeamNG's frames stopped coming.
    struct SentNote { uint64_t us; char tag[5]; };
    SentNote sentRing[16]{};
    size_t sentNext = 0;
    auto sendBng = [&](const std::string& msg) {
        sendto(inSock, msg.data(), int(msg.size()), 0, reinterpret_cast<sockaddr*>(&bngIn), sizeof(bngIn));
        if (msg.size() >= 4 && msg.compare(0, 4, "ESI1") != 0) {
            SentNote& s = sentRing[sentNext++ % 16];
            s.us = NowUs();
            memcpy(s.tag, msg.data(), 4);
            s.tag[4] = 0;
        }
    };

    // CollisionField: ETS2 roads become solid in BeamNG. The ETS2 truck's spot is pinned
    // to BeamNG (0, 0, 500), high above whatever level is loaded, facing BeamNG north.
    std::vector<RoadStrip> roads;
    RoadStreamer streamer;
    std::deque<uint32_t> sendQueue;                  // roads waiting to be sent (paced)
    std::unordered_map<uint32_t, uint64_t> unacked;  // sent roads not yet confirmed by the mod
    Vec3d lastGoodEts{};                             // last ETS2 spot where the car was on the roads
    const Vec3d kRoadAnchor{0, 0, 500};
    const double kStartAhead = 12.0;  // metres in front of the truck
    // Road axis strips (roads.bin), used to put the car on the nearest real road when the
    // truck is parked off-road (yard, parking) - even in tile mode.
    std::vector<RoadStrip> spawnStrips;
    Vec3d startSrc{};
    double startHeading = 0;
    bool haveStart = false;
    // Picks the start: nearest road axis point to the truck, facing the road direction closest
    // to the truck's heading. Falls back to kStartAhead metres ahead of the truck.
    // The start (and recovery) spot: the road nearest to `at` (the truck), facing its way.
    auto ChooseStartAt = [&](const Anchor& a, const EtsPlacement& at) {
        const double th = at.heading * kTau;                     // truck heading, CCW from north
        const double tfx = -std::sin(th), tfz = -std::cos(th);   // truck forward in ETS2 x/z
        // Nearest road within 300 m, else within 1.5 km (off-road far from the roads).
        double best = 0;
        haveStart = false;
        for (double reach : {300.0, 1500.0}) {
        best = reach * reach;
        for (const RoadStrip& s : spawnStrips) {
            if (s.kind != 0 || s.pts.size() < 2) continue;
            const double dc = (s.cx - at.pos.x) * (s.cx - at.pos.x) + (s.cz - at.pos.z) * (s.cz - at.pos.z);
            if (dc > (s.r + 1500) * (s.r + 1500)) continue;
            for (size_t k = 0; k + 1 < s.pts.size(); ++k) {
                const Vec3d& p = s.pts[k];
                const double d = (p.x - at.pos.x) * (p.x - at.pos.x) + (p.z - at.pos.z) * (p.z - at.pos.z);
                if (d >= best) continue;
                double fx = s.pts[k + 1].x - p.x, fz = s.pts[k + 1].z - p.z;
                if (fx * tfx + fz * tfz < 0) { fx = -fx; fz = -fz; }  // drive the way the truck faces
                best = d;
                startSrc = EtsToSrc(a, p);
                startHeading = std::atan2(-fx, -fz);  // ETS2: north = -z; CCW from north
                haveStart = true;
            }
        }
        if (haveStart) break;
        }
        if (!haveStart) {
            if (&at == &a.ets) {
                const double h = a.srcHeadingRad;
                startSrc = {kRoadAnchor.x - std::sin(h) * kStartAhead, kRoadAnchor.y + std::cos(h) * kStartAhead, kRoadAnchor.z};
                startHeading = h;
            } else {
                startSrc = EtsToSrc(a, Vec3d{at.pos.x + tfx * kStartAhead, at.pos.y, at.pos.z + tfz * kStartAhead});
                startHeading = th;
            }
        }
        printf("[bridge] start: %s, %.1f m from the truck\n", haveStart ? "nearest road" : "ahead of the truck",
               haveStart ? std::sqrt(best) : kStartAhead);
    };
    auto ChooseStart = [&](const Anchor& a) { ChooseStartAt(a, a.ets); };
    auto StartPoint = [&](const Anchor&) { return startSrc; };
    bool carPlaced = false;
    uint64_t lastStream = 0, lastTeleport = 0, lastClockSent = 0, lastLeashWarn = 0, lastFuelSent = 0;
    uint64_t lastKbMs = 0;
    double kbThrottle = 0, kbBrake = 0;   // keyboard pedals (ramped)
    uint32_t carFuelSeq = 0;
    bool carFuelLogged = false, refuelLogged = false;
    EtsCamTarget goodCam{};      // the last camera / target ETS2 may safely see (jump guard)
    EtsTarget goodTarget{};
    bool haveGoodCam = false;
    constexpr double kTruckLeash = 4000;  // m from the parked ETS2 truck (see the leash check)
    uint32_t roadsSent = 0;
    // Tile mode: each "road" entry is one mesh tile (pts empty, path in tilePaths).
    std::vector<std::string> tilePaths;
    const bool tileMode = tilesPath != nullptr;
    // The generator (build\ets2_roads) and its road strip output live next to this exe.
    std::string exeDir;
    {
        char p[MAX_PATH];
        GetModuleFileNameA(nullptr, p, MAX_PATH);
        exeDir = p;
        exeDir = exeDir.substr(0, exeDir.find_last_of("\\/"));
    }
    const std::string roadsOut = exeDir + "\\roads.bin";
    // Steering wheel / pedals (wheel.ini next to this exe).
    WheelConfig wheelCfg;
    wheelCfg.Load(exeDir + "\\wheel.ini");
    // The keyboard drives BeamNG while ETS2 has focus ([keys] in wheel.ini).
    KeyRouterConfig keyCfg;
    keyCfg.Load(exeDir + "\\wheel.ini");
    static KeyRouter keyRouter;
    bool bngMenuOpen = false;
    // (read as text: GetPrivateProfileInt makes a negative value 0, and negative = higher)
    char lowerText[32] = "";
    GetPrivateProfileStringA("view", "car_lower_cm", "20", lowerText, sizeof(lowerText), (exeDir + "\\wheel.ini").c_str());
    const double carLower = atof(lowerText) / 100.0;
    const DayMap dayMap = ReadDayMap((exeDir + "\\wheel.ini").c_str());
    printf("[bridge] ETS2 sunrise %02d:%02d, sunset %02d:%02d (BeamNG's sun follows them; key glow %.0f nits by day, %.0f at night)\n",
           dayMap.sunrise / 60, dayMap.sunrise % 60, dayMap.sunset / 60, dayMap.sunset % 60, dayMap.dayNits, dayMap.nightNits);
    keyRouter.Start(keyCfg, ntohs(bngIn.sin_port));
    Wheel wheel;
    bool wheelOk = false, pedalSeen[3] = {false, false, false};
    bool shifterSeen = false;   // a shifter gear was engaged once: from then on the shifter sets the gear
    double ffbForce = 0;
    uint64_t lastFfb = 0;
    if (wheelCfg.enabled) {
        wheelOk = wheel.Open(wheelCfg.device, wheelCfg.ffb);
        if (wheelOk) printf("[bridge] wheel: %s (drives the car whichever window has focus)\n", wheel.name.c_str());
        else printf("[bridge] no wheel found: keyboard input through ETS2 only\n");
        if (wheelOk && wheelCfg.ffb)
            printf(wheel.HasFfb() ? "[bridge] force feedback: BeamNG's steering force goes to the wheel\n"
                                  : "[bridge] force feedback unavailable: another program holds the wheel exclusively "
                                    "(turn force feedback off in ETS2's and BeamNG's controls, then restart the bridge)\n");
    }
    RoadExporter exporter;
    // Roads follow the car: when it nears the edge of the generated area, the next area is
    // generated in the background into the other of two tile folders, then swapped in while the
    // old tiles stay in BeamNG until the new ones under the car are built (ids differ by idBase).
    // Radius 7 km, next area at 3.5 km from the centre: the car always has >= 3.5 km of roads
    // (and ground) ahead, while BeamNG holds about a third of what 12 km took (less stutter).
    // Generated areas: 5 km around a point 3.5 km ahead of the car (half the work of 7 km around
    // it, and 8.5 km of road ahead); the next one when less than kRegenAhead (or 2 min of driving)
    // is left ahead.
    constexpr double kGenRadius = 5000, kGenLookAhead = 3500, kRegenAhead = 3000;
    Vec3d dirFrom{};                   // where the car's travel direction was last taken (ETS2)
    bool haveDirFrom = false;
    double travelX = 0, travelZ = 0;   // the car's travel direction (ETS2 x/z, unit; 0 = unknown)
    uint64_t lastEdgeWarn = 0;
    std::string tileDirs[2], tileVfs[2] = {"/art/eurospeed/tiles", "/art/eurospeed/tiles"};
    if (tilesPath) {
        std::string d = tilesPath;
        d = d.substr(0, d.find_last_of("\\/"));
        tileDirs[0] = d;
        // One folder for every area: tile files are named by their content, so the next area only
        // adds the tiles that changed (BeamNG watches this folder: few new files, no hitch).
        tileDirs[1] = d;
    }
    // Ground as a TerrainBlock (ground.h): [ground] terrain=1 in wheel.ini. Its .ter files go to
    // BeamNG's user folder temp\eurospeed (the tiles path runs through ...\current\...).
    Ground ground;
    bool groundCapable = true;        // the mod can have its TerrainBlock (no other one in the level)
    bool groundKnown = false;         // the mod said so (ESTM)
    uint32_t luaGroundSeq = 0;        // the block the mod has
    uint64_t lastGroundSent = 0, bngLiveSince = 0;
    if (tilesPath && GetPrivateProfileIntA("ground", "terrain", 1, (exeDir + "\\wheel.ini").c_str()) != 0) {
        std::string t = tilesPath;
        for (char& ch : t) if (ch == '/') ch = '\\';
        const size_t at = t.find("\\current\\");
        if (at != std::string::npos) {
            ground.enabled = true;
            ground.hfDir = exeDir + "\\heightfield";
            ground.terDir = t.substr(0, at) + "\\current\\temp\\eurospeed";
            exporter.heightfieldDir = ground.hfDir;
            printf("[bridge] ground: TerrainBlock mode (height tiles %s)\n", ground.hfDir.c_str());
        }
    }
    int tileSlot = 0;
    bool followGen = false, haveGen = false;
    double genX = 0, genZ = 0;
    uint32_t idBase = 0;
    // Each tile's id in BeamNG ("wire" id). Normally idBase + index; a tile the next area shares
    // with the previous one (same cell, same content) keeps the previous one's object.
    std::vector<uint32_t> wireOf;
    std::unordered_map<uint32_t, uint32_t> idxOfWire;
    auto Wire = [&](uint32_t id) { return id < wireOf.size() ? wireOf[id] : idBase + id; };
    auto ResetWires = [&]() {
        wireOf.resize(roads.size());
        idxOfWire.clear();
        for (uint32_t i = 0; i < wireOf.size(); ++i) { wireOf[i] = idBase + i; idxOfWire[wireOf[i]] = i; }
    };
    std::vector<uint32_t> retireIds;  // wire ids of the previous area's tiles, dropped after the swap
    uint64_t retireAt = 0;
    // The generated area: from the index ("#area"); older indexes: the tiles' mean centre and
    // the radius they were made with then.
    double genR = kGenRadius;
    TileArea tileArea;
    auto SetGenCentre = [&]() {
        haveGen = !roads.empty();
        if (tileArea.r > 0) { genX = tileArea.x; genZ = tileArea.z; genR = tileArea.r; return; }
        genX = genZ = 0;
        genR = 7000;
        for (const RoadStrip& s : roads) { genX += s.cx; genZ += s.cz; }
        if (haveGen) { genX /= double(roads.size()); genZ /= double(roads.size()); }
    };
    if (tileMode) {
        std::string err;
        bool groundTiles = false;
        if (LoadTiles(tilesPath, roads, tilePaths, err, &groundTiles, &tileArea)) {
            ResetWires();
            printf("[bridge] road mesh: %zu tiles from %s%s\n", roads.size(), tilesPath,
                   groundTiles ? " (with the ground block)" : "");
            SetGenCentre();
            // Tiles made for the ground block: its heights too (else only bridges and walls, no ground).
            exporter.lastWithHeightfields = groundTiles;
            if (groundTiles && ground.enabled) ground.LoadIndex();
        }
        else
            printf("[bridge] road mesh: cannot load %s (%s) - driving without road collision\n", tilesPath, err.c_str());
    } else if (roadsPath) {
        std::string err;
        if (LoadRoads(roadsPath, roads, err)) printf("[bridge] roads: %zu strips from %s\n", roads.size(), roadsPath);
        else printf("[bridge] roads: cannot load %s (%s) - driving without road collision\n", roadsPath, err.c_str());
    }
    {
        std::string err;
        const char* sp = roadsPath ? roadsPath : roadsOut.c_str();
        if (tileMode && !LoadRoads(sp, spawnStrips, err)) printf("[bridge] no %s for choosing the start road (%s)\n", sp, err.c_str());
        if (!tileMode) spawnStrips = roads;
    }

    printf("[bridge] EuroSpeed protocol v%u | BeamNG on :%d | LFS OutSim on :%d (opts %x)\n", kVersion,
           bngPort, port, opts);
    printf("[bridge] keys: R = re-anchor car to the ETS2 truck, Q = quit\n");

    LfsCarState car{};
    BngState bng{};
    uint32_t lfsPackets = 0, bngPackets = 0, badPackets = 0;
    uint64_t lastLfs = 0, lastBng = 0;
    uint64_t lfsRecvUs = 0, bngRecvUs = 0;  // sample times for the plugin's interpolation
    Source src = Source::None, anchoredSrc = Source::None;
    bool haveAnchor = false;
    Anchor anchor{};
    uint32_t anchoredPluginPid = 0;
    uint64_t start = GetTickCount64(), lastStatus = 0;

    for (;;) {
        fd_set fs;
        FD_ZERO(&fs);
        FD_SET(lfsSock, &fs);
        FD_SET(bngSock, &fs);
        timeval tv{0, 20000};
        select(0, &fs, nullptr, nullptr, &tv);
        const uint64_t now = GetTickCount64();
        sh->bridgeHeartbeatMs.store(now);

        uint8_t buf[8192];
        int n;
        while ((n = recv(lfsSock, reinterpret_cast<char*>(buf), sizeof(buf), 0)) > 0) {
            LfsCarState c{};
            if (ParseOutSim(buf, static_cast<size_t>(n), opts, c)) {
                c.packets = ++lfsPackets;
                car = c;
                lastLfs = now;
                lfsRecvUs = NowUs();
                sh->lfs.write(car);
            } else if (++badPackets == 1) {
                printf("[bridge] ignoring OutSim packet of %d bytes: expected %zu. "
                       "Is cfg.txt 'OutSim Opts' = %x?\n", n, OutSimSize(opts), opts);
            }
        }
        while ((n = recv(bngSock, reinterpret_cast<char*>(buf), sizeof(buf), 0)) > 0) {
            if (n >= 5 && memcmp(buf, "ESF1 ", 5) == 0) {  // BeamNG's steering force (-1..1)
                buf[n < int(sizeof(buf)) ? n : int(sizeof(buf)) - 1] = 0;
                ffbForce = atof(reinterpret_cast<const char*>(buf) + 5);
                lastFfb = GetTickCount64();
                continue;
            }
            if (n >= 6 && memcmp(buf, "ESTM ", 5) == 0) {  // the mod's ground: capable (1/0) and block seq
                buf[n < int(sizeof(buf)) ? n : int(sizeof(buf)) - 1] = 0;
                int cap = 1;
                unsigned seqHave = 0;
                if (sscanf_s(reinterpret_cast<const char*>(buf) + 5, "%d %u", &cap, &seqHave) >= 1) {
                    if (ground.enabled && (cap != 0) != groundCapable)
                        printf("[bridge] ground: %s\n", cap ? "BeamNG can take the TerrainBlock again"
                                                            : "this BeamNG level has its own terrain - roads as meshes (use smallgrid for the ground block)");
                    groundCapable = cap != 0;
                    groundKnown = true;
                    luaGroundSeq = seqHave;
                    exporter.heightfieldDir = ground.enabled && groundCapable ? ground.hfDir : "";
                }
                continue;
            }
            if (n >= 6 && memcmp(buf, "ESMN ", 5) == 0) {  // BeamNG's menu opened (1) / closed (0)
                const bool open = buf[5] == '1';
                keyRouter.SetMenu(open);
                if (open != bngMenuOpen && keyCfg.enabled) {
                    bngMenuOpen = open;
                    // The mouse needs BeamNG under the cursor: BeamNG goes over ETS2 for its menu.
                    if (open) focus::BeamngOverEts2();
                    else focus::Ets2Active();
                }
                continue;
            }
            if (n >= 5 && memcmp(buf, "ESU1 ", 5) == 0) {  // the car's fuel: litres capacity
                buf[n < int(sizeof(buf)) ? n : int(sizeof(buf)) - 1] = 0;
                float l = 0, c = 0;
                if (sscanf_s(reinterpret_cast<const char*>(buf) + 5, "%f %f", &l, &c) == 2 && c > 0) {
                    CarFuel cf{l, c, 1, ++carFuelSeq};
                    sh->carFuel.write(cf);
                    if (!carFuelLogged) {
                        carFuelLogged = true;
                        printf("[bridge] car fuel %.1f of %.0f l: linked with the ETS2 truck's fuel\n", l, c);
                    }
                }
                continue;
            }
            if (n >= 5 && memcmp(buf, "ESA1 ", 5) == 0) {  // the mod confirms roads it received
                buf[n < int(sizeof(buf)) ? n : int(sizeof(buf)) - 1] = 0;
                char* ctx = nullptr;
                for (char* tok = strtok_s(reinterpret_cast<char*>(buf) + 5, " ", &ctx); tok; tok = strtok_s(nullptr, " ", &ctx)) {
                    const uint32_t wire = static_cast<uint32_t>(strtoul(tok, nullptr, 10));
                    if (auto it = idxOfWire.find(wire); it != idxOfWire.end()) unacked.erase(it->second);
                    else if (wireOf.empty() && wire >= idBase) unacked.erase(wire - idBase);
                }
                continue;
            }
            BngState b{};
            if (ParseBng(reinterpret_cast<const char*>(buf), static_cast<size_t>(n), b)) {
                // The mod restarts its counter when BeamNG reloads Lua: its roads are gone.
                if (bngPackets > 0 && b.seq + 100 < bng.seq) {
                    printf("[bridge] BeamNG mod reloaded: resending roads\n");
                    retireIds.clear();
                    streamer.Reset();
                    sendQueue.clear();
                    unacked.clear();
                    carPlaced = false;
                }
                const uint64_t recvUs = NowUs();
                if (bngPackets > 200 && bngRecvUs && recvUs - bngRecvUs > 150000) {
                    // BeamNG stalled: what did we ask of it in the second before?
                    std::string before;
                    for (const SentNote& s : sentRing)
                        if (s.us && s.us + 1000000 > bngRecvUs && s.us <= recvUs) {
                            char e[32];
                            snprintf(e, sizeof(e), " %s(-%.0fms)", s.tag, (recvUs - s.us) / 1000.0);
                            before += e;
                        }
                    printf("[bridge] BeamNG froze %.0f ms;%s%s\n", (recvUs - bngRecvUs) / 1000.0,
                           before.empty() ? " nothing sent to it just before" : " sent before:", before.c_str());
                    if (exporter.Running()) printf("[bridge]   (the road generator was running)\n");
                }
                bng = b;
                ++bngPackets;
                lastBng = now;
                bngRecvUs = recvUs;
                sh->bng.write(bng);
            } else if (++badPackets == 1) {
                printf("[bridge] ignoring malformed BeamNG packet (%d bytes)\n", n);
            }
        }

        // Pick the live source; BeamNG wins when both stream.
        const bool bngLive = lastBng && now - lastBng < 1000;
        const bool lfsLive = lastLfs && now - lastLfs < 1000;
        src = bngLive ? Source::BeamNG : lfsLive ? Source::Lfs : (lastBng || lastLfs ? src : Source::None);
        // A mod that never says whether it can have the ground block (an older eurospeed.lua):
        // full mesh tiles, or the car would have no ground.
        if (!bngLive) bngLiveSince = 0;
        else if (!bngLiveSince) bngLiveSince = now;
        if (ground.enabled && !groundKnown && bngLiveSince && now - bngLiveSince > 8000) {
            printf("[bridge] ground: the BeamNG mod does not report the ground block (old eurospeed.lua?) - roads as meshes\n");
            groundKnown = true;
            groundCapable = false;
            exporter.heightfieldDir = "";
        }

        // ETS2 side.
        EtsTruckState ets{};
        const bool pluginAlive = PeerAlive(sh->pluginHeartbeatMs.load());
        const bool haveEts = pluginAlive && sh->ets.read(ets) && ets.hasPlacement && !ets.multiplayer;
        const uint32_t pluginPid = sh->pluginPid.load();

        // Forward the driver's ETS2 inputs to BeamNG (once per ETS2 frame).
        EtsInput in{};
        if (ets2Input && pluginAlive && !ets.multiplayer && sh->input.read(in) && in.seq != lastInputSeq) {
            lastInputSeq = in.seq;
            // The wheel and pedals, read directly (ETS2 may not see them, and this works with any
            // window focused). Keyboard input coming through ETS2 still works alongside.
            // The keyboard, read by the bridge itself (W/S/A/D, arrows): ETS2 1.62 reports only the
            // steering for the parked truck, throttle and brake stay 0. Pedals ramp like a game's
            // keyboard; steering goes as is (BeamNG smooths keyboard steering itself).
            {
                const uint32_t keys = keyCfg.enabled ? keyRouter.DriveKeys() : 0;
                const double dt = std::clamp((now - lastKbMs) / 1000.0, 0.0, 0.1);
                lastKbMs = now;
                auto ramp = [&](double v, bool on, double upS, double downS) {
                    return on ? (std::min)(1.0, v + dt / upS) : (std::max)(0.0, v - dt / downS);
                };
                kbThrottle = ramp(kbThrottle, (keys & KeyRouter::kKeyThrottle) != 0, 0.25, 0.08);
                kbBrake = ramp(kbBrake, (keys & KeyRouter::kKeyBrake) != 0, 0.2, 0.08);
                const double kbSteer = ((keys & KeyRouter::kKeyLeft) ? 1.0 : 0.0) - ((keys & KeyRouter::kKeyRight) ? 1.0 : 0.0);   // + = left
                if (std::fabs(kbSteer) > std::fabs(in.steering)) in.steering = float(kbSteer);
                in.throttle = (std::max)(in.throttle, float(kbThrottle));
                in.brake = (std::max)(in.brake, float(kbBrake));
            }
            WheelAxes wa{};
            int wheelSteering = 0;   // tells BeamNG to take steering 1:1 (no keyboard smoothing)
            int shifterGear = WheelConfig::kNoShifter;
            DWORD buttons = 0;
            if (wheelOk && wheel.Read(wa, &buttons)) {
                const int sg = wheelCfg.ShifterGear(buttons);
                if (sg != 0 && sg != WheelConfig::kNoShifter && !shifterSeen) {
                    shifterSeen = true;
                    printf("[bridge] H shifter in use: it now sets BeamNG's gear (set BeamNG's gearbox to manual)\n");
                }
                if (shifterSeen) shifterGear = sg;
                auto axis = [&](int idx, bool inv, int slot) -> double {
                    if (idx < 0) return 0;
                    const long raw = wa.v[idx];
                    if (raw != 32768) pedalSeen[slot] = true;   // DirectInput reports mid-range until the first real read
                    if (!pedalSeen[slot]) return 0;
                    const double u = raw / 65535.0;
                    return inv ? 1.0 - u : u;
                };
                double steer = 0;
                if (wheelCfg.steer >= 0) {
                    steer = -(wa.v[wheelCfg.steer] - 32767.5) / 32767.5;   // ETS2 convention: + = left
                    if (wheelCfg.steerInvert) steer = -steer;
                    if (std::fabs(steer) < wheelCfg.steerDeadzone) steer = 0;
                }
                const double thr = axis(wheelCfg.throttle, wheelCfg.throttleInvert, 0);
                const double brk = axis(wheelCfg.brake, wheelCfg.brakeInvert, 1);
                const double clu = axis(wheelCfg.clutch, wheelCfg.clutchInvert, 2);
                // Keyboard (through ETS2) wins only when it asks for more than the wheel.
                if (std::fabs(in.steering) < std::fabs(steer) + 0.05) { in.steering = float(steer); wheelSteering = 1; }
                in.throttle = float(std::max<double>(in.throttle, thr));
                in.brake = float(std::max<double>(in.brake, brk));
                in.clutch = float(std::max<double>(in.clutch, clu));
            }
            char line[160];
            const int len = snprintf(line, sizeof(line), "ESI1 %u %.4f %.4f %.4f %.4f %u %d %d %d", in.seq, in.steering,
                                     in.throttle, in.brake, in.clutch, in.parkingBrake, in.gear, wheelSteering, shifterGear);
            sendto(inSock, line, len, 0, reinterpret_cast<sockaddr*>(&bngIn), sizeof(bngIn));
            ++inputsSent;
        }

        // Force feedback: BeamNG's latest steering force; none when it stops coming.
        if (wheelOk && wheel.HasFfb()) {
            double f = (lastFfb && now - lastFfb < 300) ? ffbForce * wheelCfg.ffbGain : 0.0;
            wheel.SetForce(wheelCfg.ffbInvert ? -f : f);
        }

        // BeamNG's time of day follows ETS2's clock (the car's lighting matches the world).
        // Refuelling at an ETS2 station: the car gets the same fuel (the plugin says how much).
        {
            EtsFuel ef{};
            if (src == Source::BeamNG && sh->etsFuel.read(ef) && ef.linked) {
                if (ef.refuelling && now - lastFuelSent >= 250) {
                    lastFuelSent = now;
                    char fm[48];
                    snprintf(fm, sizeof(fm), "ESK1 %.2f", ef.carTarget);
                    sendBng(fm);
                    if (!refuelLogged) { refuelLogged = true; printf("[bridge] ETS2 refuel: filling the car\n"); }
                } else if (!ef.refuelling && refuelLogged) {
                    refuelLogged = false;
                    printf("[bridge] ETS2 refuel done: truck %.1f l\n", ef.litres);
                }
            }
        }
        // Every 60 s: each change of the time of day makes BeamNG redo its lighting (a hitch).
        if (haveEts && src == Source::BeamNG && ets.gameTimeMin && (lastClockSent == 0 || now - lastClockSent >= 10000)) {
            lastClockSent = now;
            const int m = int(ets.gameTimeMin % 1440);
            char clk[64];
            snprintf(clk, sizeof(clk), "ESC1 %d %.1f", dayMap.ToBeamng(m), dayMap.KeyNits(m));
            sendBng(clk);
        }

        bool reanchor = false;
        while (_kbhit()) {
            int k = _getch();
            if (k == 'q' || k == 'Q') goto done;
            if (k == 'r' || k == 'R') reanchor = true;
        }
        if (haveAnchor && (pluginPid != anchoredPluginPid || src != anchoredSrc)) reanchor = true;
        if (reanchor) haveAnchor = false;

        // Current source pose (the car).
        Vec3d srcPos{};
        double srcH = 0, srcP = 0, srcR = 0;
        if (src == Source::BeamNG) {
            srcPos = bng.vehPos;
            AnglesFromDirs(bng.vehFwd, bng.vehUp, srcH, srcP, srcR);
        } else if (src == Source::Lfs) {
            srcPos = car.pos; srcH = car.heading; srcP = car.pitch; srcR = car.roll;
        }

        // Road collision is on whenever tiles/roads were configured (tiles may be generated on demand).
        const bool roadMode = (tileMode || !roads.empty()) && src == Source::BeamNG;

        // Finished generating roads around the truck: load them and start streaming.
        if (followGen && exporter.Done()) {
            // The next area is ready: swap it in without dropping the roads under the car.
            followGen = false;
            std::vector<RoadStrip> nr;
            std::vector<std::string> np;
            std::string err;
            const int slot = 1 - tileSlot;
            if (exporter.ok && LoadTiles((tileDirs[slot] + "\\tiles.idx").c_str(), nr, np, err, nullptr, &tileArea)) {
                // Tiles loaded in BeamNG now, by cell: the new area keeps those that did not change.
                std::unordered_map<int64_t, std::pair<uint32_t, uint64_t>> loadedOld;   // cell -> (wire, hash)
                for (uint32_t id : streamer.loaded)
                    if (id < roads.size() && roads[id].hash && unacked.count(id) == 0)
                        loadedOld[(int64_t(roads[id].tx) << 32) ^ uint32_t(roads[id].tz)] = {Wire(id), roads[id].hash};
                retireAt = 0;
                idBase += 1000000;
                roads = std::move(nr);
                tilePaths = std::move(np);
                ResetWires();
                std::vector<uint32_t> adopted;
                std::unordered_set<uint32_t> kept;
                for (uint32_t i = 0; i < roads.size(); ++i) {
                    auto it = loadedOld.find((int64_t(roads[i].tx) << 32) ^ uint32_t(roads[i].tz));
                    if (it == loadedOld.end() || it->second.second != roads[i].hash) continue;
                    idxOfWire.erase(wireOf[i]);
                    wireOf[i] = it->second.first;
                    idxOfWire[wireOf[i]] = i;
                    kept.insert(wireOf[i]);
                    adopted.push_back(i);
                }
                for (auto& [cell, wh] : loadedOld) if (!kept.count(wh.first)) retireIds.push_back(wh.first);
                tileSlot = slot;
                SetGenCentre();
                if (exporter.lastWithHeightfields) ground.LoadIndex();
                std::string err2;
                LoadRoads(roadsOut.c_str(), spawnStrips, err2);
                streamer.Reset();
                for (uint32_t i : adopted) streamer.loaded.insert(i);
                sendQueue.clear();
                unacked.clear();
                printf("[bridge] roads ahead ready: %zu tiles around (%.0f, %.0f); %zu of the loaded ones kept, %zu to replace\n",
                       roads.size(), genX, genZ, adopted.size(), retireIds.size());
            } else {
                printf("[bridge] generating the roads ahead failed (%s)\n", err.c_str());
            }
        } else if (exporter.Done()) {
            std::string err;
            tileSlot = 0;
            if (exporter.ok && LoadTiles(tilesPath, roads, tilePaths, err, nullptr, &tileArea)) {
                ResetWires();
                SetGenCentre();
                if (exporter.lastWithHeightfields) ground.LoadIndex();
                std::string err2;
                LoadRoads(roadsOut.c_str(), spawnStrips, err2);
                printf("[bridge] roads ready: %zu tiles, %zu road strips\n", roads.size(), spawnStrips.size());
            } else {
                printf("[bridge] road generation failed (%s) - see ets2_roads output above\n", err.c_str());
            }
            streamer.Reset();
            sendQueue.clear();
            unacked.clear();
            carPlaced = false;
            if (haveAnchor) ChooseStart(anchor);
        }
        if (src != Source::None && haveEts && !haveAnchor) {
            // Road mode: BeamNG north = ETS2 north (no rotation between the worlds), so road mesh
            // tiles need no rotation; the car is placed facing the truck's heading instead.
            anchor = roadMode ? MakeAnchor(kRoadAnchor, ets.placement.heading * kTau, ets.placement)
                              : MakeAnchor(srcPos, srcH, ets.placement);
            if (roadMode) {
                streamer.Reset();
                sendQueue.clear();
                unacked.clear();
                carPlaced = false;
                sendBng("ESX1");  // drop roads from an earlier anchor
                retireIds.clear();
                ground.Invalidate();
                // No road tiles around the truck yet (new area of the map): generate them now.
                bool covered = false;
                for (const RoadStrip& s : roads)
                    if ((s.cx - ets.placement.pos.x) * (s.cx - ets.placement.pos.x) +
                        (s.cz - ets.placement.pos.z) * (s.cz - ets.placement.pos.z) < 1500.0 * 1500.0) { covered = true; break; }
                if (tileMode && !covered && !exporter.Running()) {
                    roads.clear();
                    tilePaths.clear();
                    spawnStrips.clear();
                    haveGen = false;
                    exporter.Start(exeDir, ets.placement.pos.x, ets.placement.pos.z, roadsOut, kGenRadius,
                                   tileDirs[0], tileVfs[0]);
                } else {
                    ChooseStart(anchor);
                }
            }
            anchoredPluginPid = pluginPid;
            anchoredSrc = src;
            haveAnchor = true;
            printf("[bridge] anchored %s car (%.2f, %.2f, %.2f) hdg %.3f rad -> ETS2 (%.2f, %.2f, %.2f) hdg %.4f\n",
                   src == Source::BeamNG ? "BeamNG" : "LFS", srcPos.x, srcPos.y, srcPos.z, srcH,
                   ets.placement.pos.x, ets.placement.pos.y, ets.placement.pos.z, ets.placement.heading);
        }

        // Stream ETS2 roads around the car into BeamNG, and put the car on the road once.
        if (roadMode && haveAnchor && now - lastStream >= 200) {
            lastStream = now;
            const Vec3d startPt = StartPoint(anchor);
            const double dx = bng.vehPos.x - startPt.x, dy = bng.vehPos.y - startPt.y, dz = bng.vehPos.z - startPt.z;
            const bool nearAnchor = std::sqrt(dx * dx + dy * dy + dz * dz) < 30;
            const bool groundReady = ground.enabled && groundCapable && ground.Seq() && luaGroundSeq == ground.Seq();
            // Tiles made for the ground block hold only bridges and walls: the car waits for the block.
            const bool needGround = exporter.lastWithHeightfields && ground.enabled && groundCapable;
            const bool solid = needGround ? groundReady : (streamer.loaded.size() > 0 || groundReady);
            if (!carPlaced && nearAnchor && solid && unacked.empty() && sendQueue.empty())
                carPlaced = true;
            const Vec3d carEts = SrcToEts(anchor, bng.vehPos, 0, 0, 0).pos;
            if (carPlaced && bng.vehPos.z > kRoadAnchor.z - 60) lastGoodEts = carEts;
            // Fell off the roads (or through a gap): say where, put the car back at the start.
            if (carPlaced && bng.vehPos.z < kRoadAnchor.z - 150) {
                int nearCount[3] = {0, 0, 0};
                for (uint32_t id : streamer.loaded) {
                    const RoadStrip& s = roads[id];
                    for (const Vec3d& p : s.pts)
                        if ((p.x - lastGoodEts.x) * (p.x - lastGoodEts.x) + (p.z - lastGoodEts.z) * (p.z - lastGoodEts.z) < 30 * 30) {
                            ++nearCount[s.kind < 3 ? s.kind : 0];
                            break;
                        }
                }
                printf("[bridge] car fell below the roads near ETS2 (%.1f, %.1f, %.1f): strips within 30 m: "
                       "%d road, %d junction, %d barrier; placing it back\n",
                       lastGoodEts.x, lastGoodEts.y, lastGoodEts.z, nearCount[0], nearCount[1], nearCount[2]);
                carPlaced = false;
                lastTeleport = 0;
                // Back onto the road where it was (the truck follows the car), not the journey's start.
                // From where it was last on solid ground: the truck follows the falling car down.
                if (haveEts) {
                    EtsPlacement at = ets.placement;
                    at.pos = lastGoodEts;
                    ChooseStartAt(anchor, at);
                }
            }
            // ETS2 loads its map around the (parked) truck, not the camera: far from the truck the
            // world is missing pieces and ETS2 can crash. Warn, then bring the car back.
            if (carPlaced && haveEts) {
                const double tdx = carEts.x - ets.placement.pos.x, tdz = carEts.z - ets.placement.pos.z;
                const double fromTruck = std::sqrt(tdx * tdx + tdz * tdz);
                if (fromTruck > kTruckLeash) {
                    printf("[bridge] %.1f km from the ETS2 truck: ETS2 has not loaded the map here - car back to the truck\n",
                           fromTruck / 1000);
                    carPlaced = false;
                    lastTeleport = 0;
                    ChooseStartAt(anchor, ets.placement);
                } else if (fromTruck > kTruckLeash - 1000 && now - lastLeashWarn > 10000) {
                    lastLeashWarn = now;
                    printf("[bridge] warning: %.1f km from the ETS2 truck (limit %.0f km)\n", fromTruck / 1000, kTruckLeash / 1000);
                }
            }
            // The area was made for the ground block, but this BeamNG level cannot have it: the full
            // mesh tiles instead.
            if (tileMode && haveGen && !exporter.Running() && exporter.lastWithHeightfields && (!ground.enabled || !groundCapable)) {
                printf("[bridge] ground block not possible here: generating the roads as full meshes\n");
                followGen = true;
                exporter.Start(exeDir, genX, genZ, roadsOut, kGenRadius, tileDirs[1 - tileSlot], tileVfs[1 - tileSlot]);
            }
            // Nearing the edge of the generated area: generate the next one, ahead of the car. Under
            // the games' load a generation takes ~1-2 min, so it starts when less road is left ahead
            // than the car covers in 2 minutes (a fast car outran it: past the edge the ground block
            // only guesses the ground, and the guess sloped off a bridge into the valley).
            // The travel direction: over the last 150+ m the car drove (not from where it was placed,
            // reset or teleported: a jump of over 1 km starts over, the direction unknown).
            if (!carPlaced) {
                haveDirFrom = false;
            } else if (!haveDirFrom) {
                dirFrom = carEts;
                haveDirFrom = true;
                travelX = travelZ = 0;
            } else {
                const double ddx = carEts.x - dirFrom.x, ddz = carEts.z - dirFrom.z, len2 = ddx * ddx + ddz * ddz;
                if (len2 > 1000.0 * 1000.0) {
                    dirFrom = carEts;
                    travelX = travelZ = 0;
                } else if (len2 > 150.0 * 150.0) {
                    const double len = std::sqrt(len2);
                    travelX = ddx / len;
                    travelZ = ddz / len;
                    dirFrom = carEts;
                }
            }
            if (tileMode && carPlaced && haveGen && !tileDirs[0].empty()) {
                const double ahead = AheadInArea(carEts.x, carEts.z, travelX, travelZ, genX, genZ, genR);
                const double needed = std::clamp(bng.speedMs * 120.0, kRegenAhead, 8000.0);
                if (!exporter.Running() && ahead < needed) {
                    followGen = true;
                    const bool dir = travelX != 0 || travelZ != 0;
                    exporter.Start(exeDir, carEts.x + (dir ? travelX * kGenLookAhead : 0), carEts.z + (dir ? travelZ * kGenLookAhead : 0),
                                   roadsOut, kGenRadius, tileDirs[1 - tileSlot], tileVfs[1 - tileSlot]);
                } else if (exporter.Running() && ahead < 400 && now - lastEdgeWarn > 10000) {
                    lastEdgeWarn = now;
                    printf("[bridge] warning: only %.0f m of generated road left ahead (the next area is still being made) - slow down\n", ahead);
                }
            }
            // After a swap: drop the previous area's tiles once the new ones around the car are in.
            if (!retireIds.empty() && streamer.loaded.size() > 0 && sendQueue.empty() && unacked.empty()) {
                if (!retireAt) retireAt = now + 3000;
                else if (now >= retireAt) {
                    for (uint32_t w : retireIds) sendBng("ESD1 " + std::to_string(w));
                    retireIds.clear();
                    retireAt = 0;
                }
            }
            // Until the car is on the roads, stream around the truck's spot (the anchor).
            const Vec3d centre = carPlaced ? carEts : SrcToEts(anchor, StartPoint(anchor), 0, 0, 0).pos;
            // The ground block around the car (or the start spot): a new one every 256 m; resent
            // when the mod does not have the current one (lost packet, level or Lua reload).
            if (ground.enabled && groundCapable) {
                std::string gm;
                if (ground.Update(centre.x, centre.z, anchor, gm)) { sendBng(gm); lastGroundSent = now; }
                else if (ground.Seq() && luaGroundSeq != ground.Seq() && !ground.LastMessage().empty() && now - lastGroundSent > 2500) {
                    sendBng(ground.LastMessage());
                    lastGroundSent = now;
                }
            }
            if (streamer.Due(centre.x, centre.z)) {
                std::vector<uint32_t> toLoad, toUnload;
                streamer.Update(roads, centre.x, centre.z, SIZE_MAX, toLoad, toUnload);
                // Nearest first, so the road under the car is solid soonest.
                std::sort(toLoad.begin(), toLoad.end(), [&](uint32_t a, uint32_t b) {
                    auto d = [&](uint32_t i) { return (roads[i].cx - centre.x) * (roads[i].cx - centre.x) +
                                                      (roads[i].cz - centre.z) * (roads[i].cz - centre.z); };
                    return d(a) < d(b);
                });
                for (uint32_t id : toLoad) sendQueue.push_back(id);
                for (uint32_t id : toUnload) { sendBng("ESD1 " + std::to_string(Wire(id))); unacked.erase(id); }
            }
        }

        // Resend roads BeamNG did not acknowledge (UDP to Lua can drop when its buffer is full).
        if (roadMode && haveAnchor) {
            for (auto& [id, sentAt] : unacked)
                if (now - sentAt > 1500 && streamer.loaded.count(id)) { sendQueue.push_front(id); sentAt = now; }
            // Pace sends: at most ~24 KB per loop iteration.
            size_t budget = 24 * 1024;
            while (!sendQueue.empty() && budget > 0) {
                const uint32_t id = sendQueue.front();
                sendQueue.pop_front();
                if (!streamer.loaded.count(id)) continue;
                const RoadStrip& s = roads[id];
                std::string msg;
                char num[64];
                if (tileMode) {
                    // ESG1 id path  pos(3)  yawRad  check(3): where the tile-local point (100,0,0) must land
                    const Vec3d pos = EtsToSrc(anchor, Vec3d{s.cx, 0, s.cz});
                    const Vec3d chk = EtsToSrc(anchor, Vec3d{s.cx + 100, 0, s.cz});
                    const double yaw = -(anchor.ets.heading * kTau - anchor.srcHeadingRad);
                    char head[512];
                    snprintf(head, sizeof(head), "ESG1 %u %s %.3f %.3f %.3f %.6f %.3f %.3f %.3f", Wire(id),
                             tilePaths[id].c_str(), pos.x, pos.y, pos.z, yaw, chk.x, chk.y, chk.z);
                    msg = head;
                } else {
                    msg = "ESR1 " + std::to_string(id) + " " + std::to_string(s.kind) + " ";
                    snprintf(num, sizeof(num), "%.2f %zu", s.width, s.pts.size());
                    msg += num;
                    for (const Vec3d& p : s.pts) {
                        const Vec3d b = EtsToSrc(anchor, p);
                        snprintf(num, sizeof(num), " %.2f %.2f %.2f", b.x, b.y, b.z);
                        msg += num;
                    }
                }
                sendBng(msg);
                unacked[id] = now;
                ++roadsSent;
                budget = msg.size() >= budget ? 0 : budget - msg.size();
            }
        }
        if (roadMode && haveAnchor) {
            // Teleport, facing the truck's heading, until the car is there - 12 m ahead of the
            // parked truck so the view doesn't start inside its cab.
            const bool groundIn = ground.enabled && groundCapable && ground.Seq() && luaGroundSeq == ground.Seq();
            const bool needGround = exporter.lastWithHeightfields && ground.enabled && groundCapable;
            if (!carPlaced && (needGround ? groundIn : (streamer.loaded.size() > 0 || groundIn)) && now - lastTeleport >= 1500) {
                lastTeleport = now;
                const Vec3d startPt = StartPoint(anchor);
                char tp[128];
                snprintf(tp, sizeof(tp), "EST1 %.2f %.2f %.2f %.6f", startPt.x, startPt.y, startPt.z + 1.0, startHeading);
                sendBng(tp);
            }
        }

        EtsTarget t{};
        EtsCamTarget cam{};
        if (haveAnchor && src != Source::None) {
            t.placement = SrcToEts(anchor, srcPos, srcH, srcP, srcR);
            t.valid = 1;
            t.lfsPackets = src == Source::BeamNG ? bng.seq : car.packets;
            if (src == Source::BeamNG) {
                double h, p, r;
                AnglesFromDirs(bng.camFwd, bng.camUp, h, p, r);
                cam.placement = SrcToEts(anchor, bng.camPos, h, p, r);
                // ETS2's camera a little lower than BeamNG's: BeamNG's road surface lies slightly above
                // the road ETS2 draws, so the car seemed to float ([view] car_lower in wheel.ini).
                cam.placement.pos.y -= carLower;
                cam.fovDeg = bng.fovDeg;
            } else {
                // LFS: no camera stream; sit 1.2 m above the car's origin.
                Vec3d eye{car.pos.x, car.pos.y, car.pos.z + 1.2};
                cam.placement = SrcToEts(anchor, eye, srcH, srcP, srcR);
                cam.fovDeg = 60;
            }
            cam.valid = 1;
            cam.sampleUs = src == Source::BeamNG ? bngRecvUs : lfsRecvUs;
        }
        // Jump guard: the car can leap kilometres in one frame (BeamNG's reset puts it back where it
        // was spawned). ETS2 must never see that: a camera where its map is not loaded crashes it,
        // and the truck puppet gives up. ETS2 keeps the last good view; the car is put back on the
        // road by the truck.
        if (cam.valid && haveEts && roadMode) {
            const Vec3d& cp = cam.placement.pos;
            const double fromTruck = std::hypot(cp.x - ets.placement.pos.x, cp.z - ets.placement.pos.z);
            const double jump = haveGoodCam ? std::hypot(cp.x - goodCam.placement.pos.x, cp.z - goodCam.placement.pos.z) : 0;
            const bool bad = fromTruck > kTruckLeash + 500 || (jump > 300 && fromTruck > 1000);
            if (bad) {
                if (carPlaced) {
                    printf("[bridge] the car jumped %.1f km (BeamNG reset?) - ETS2 keeps its view, car back on the road by the truck\n",
                           jump / 1000);
                    carPlaced = false;
                    lastTeleport = 0;
                    ChooseStartAt(anchor, ets.placement);
                }
                if (haveGoodCam) { t = goodTarget; cam = goodCam; }
                else { t.valid = 0; cam.valid = 0; }
            } else {
                goodCam = cam;
                goodTarget = t;
                haveGoodCam = true;
            }
        }
        sh->target.write(t);
        sh->cam.write(cam);

        if (!quiet && now - lastStatus >= 500) {
            lastStatus = now;
            const char* etsState = !pluginAlive ? "ETS2: not linked"
                                 : ets.multiplayer ? "ETS2: MULTIPLAYER - inert"
                                 : ets.paused ? "ETS2: paused" : "ETS2: linked";
            if (src == Source::BeamNG)
                printf("[bridge] BeamNG #%u veh (%.2f, %.2f, %.2f) %5.1f km/h | %s", bng.seq, bng.vehPos.x,
                       bng.vehPos.y, bng.vehPos.z, bng.speedMs * 3.6, etsState);
            else if (src == Source::Lfs)
                printf("[bridge] LFS #%u pos (%.2f, %.2f, %.2f) %5.1f km/h gear %d | %s", car.packets,
                       car.pos.x, car.pos.y, car.pos.z, car.speedMs * 3.6, car.gear - 1, etsState);
            else
                printf("[bridge] waiting for BeamNG or LFS | %s", etsState);
            if (cam.valid)
                printf(" | cam (%.2f, %.2f, %.2f) hdg %.4f", cam.placement.pos.x, cam.placement.pos.y,
                       cam.placement.pos.z, cam.placement.heading);
            if (!roads.empty())
                printf(" | roads %zu loaded, %u sent, %zu queued, %zu unconfirmed%s", streamer.loaded.size(), roadsSent,
                       sendQueue.size(), unacked.size(), carPlaced ? "" : ", placing car");
            if (pluginAlive && ets2Input)
                printf(" | ETS2 input steer %+.2f thr %.2f brk %.2f (%u sent)", in.steering, in.throttle, in.brake,
                       inputsSent);
            printf("\n");
        }
        if (seconds > 0 && now - start >= static_cast<uint64_t>(seconds) * 1000) break;
    }
done:
    printf("[bridge] exit: %u BeamNG + %u OutSim packets, %u rejected, anchor %s\n", bngPackets, lfsPackets,
           badPackets, haveAnchor ? "yes" : "no");
    sh->bridgeHeartbeatMs.store(0);
    closesocket(lfsSock);
    closesocket(bngSock);
    closesocket(inSock);
    WSACleanup();
    CloseShared(sh, map);
    return 0;
}
