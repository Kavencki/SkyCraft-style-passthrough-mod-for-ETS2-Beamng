// Unit tests: coordinate mapping, OutSim parsing, seqlock slot.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "../protocol/mapping.h"
#include "../bridge/roads.h"

using namespace es;

static int g_fail = 0, g_pass = 0;
#define CHECK_NEAR(a, b, eps)                                                               \
    do {                                                                                    \
        double _a = (a), _b = (b);                                                          \
        if (std::fabs(_a - _b) > (eps)) {                                                   \
            printf("FAIL %s:%d  %s = %.6f, expected %.6f\n", __FILE__, __LINE__, #a, _a, _b); \
            ++g_fail;                                                                       \
        } else ++g_pass;                                                                    \
    } while (0)
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } else ++g_pass; } while (0)

static LfsCarState Car(double x, double y, double z, double hdg) {
    LfsCarState c{};
    c.pos = {x, y, z};
    c.heading = float(hdg);
    return c;
}
static EtsPlacement Truck(double x, double y, double z, double hdgTurns) {
    EtsPlacement p{};
    p.pos = {x, y, z};
    p.heading = hdgTurns;
    return p;
}

int main() {
    // 1. The anchor point maps exactly onto the truck.
    {
        Anchor a = MakeAnchor(Car(10, 20, 3, 0.7), Truck(-31140.25, 45.5, -6420.75, 0.25));
        EtsPlacement e = LfsToEts(a, Car(10, 20, 3, 0.7));
        CHECK_NEAR(e.pos.x, -31140.25, 1e-9);
        CHECK_NEAR(e.pos.y, 45.5, 1e-9);
        CHECK_NEAR(e.pos.z, -6420.75, 1e-9);
        CHECK_NEAR(e.heading, 0.25, 1e-6);
    }
    // 2. Both facing north: 10 m north in LFS = 10 m north in ETS2 (-Z). Up = +Y.
    {
        Anchor a = MakeAnchor(Car(0, 0, 0, 0), Truck(100, 5, 200, 0));
        EtsPlacement e = LfsToEts(a, Car(0, 10, 2, 0));
        CHECK_NEAR(e.pos.x, 100, 1e-9);
        CHECK_NEAR(e.pos.y, 7, 1e-9);
        CHECK_NEAR(e.pos.z, 190, 1e-9);
        // 10 m east stays east.
        e = LfsToEts(a, Car(10, 0, 0, 0));
        CHECK_NEAR(e.pos.x, 110, 1e-9);
        CHECK_NEAR(e.pos.z, 200, 1e-9);
    }
    // 3. Car faces north, truck faces west: driving forward goes west in ETS2.
    {
        Anchor a = MakeAnchor(Car(0, 0, 0, 0), Truck(0, 0, 0, 0.25));
        EtsPlacement e = LfsToEts(a, Car(0, 10, 0, 0));
        CHECK_NEAR(e.pos.x, -10, 1e-9);
        CHECK_NEAR(e.pos.z, 0, 1e-9);
        CHECK_NEAR(e.heading, 0.25, 1e-6);
        // Turning left 90° in LFS -> facing south in ETS2 (0.5).
        e = LfsToEts(a, Car(0, 0, 0, kPi / 2));
        CHECK_NEAR(e.heading, 0.5, 1e-6);
    }
    // 4. Car faces east (heading -pi/2), truck faces north: forward (+X) -> north (-Z).
    //    Heading is a float in OutSim, so allow its rounding (~4e-8 rad * 10 m).
    {
        Anchor a = MakeAnchor(Car(5, 5, 0, -kPi / 2), Truck(0, 0, 0, 0));
        EtsPlacement e = LfsToEts(a, Car(15, 5, 0, -kPi / 2));
        CHECK_NEAR(e.pos.x, 0, 1e-5);
        CHECK_NEAR(e.pos.z, -10, 1e-5);
        CHECK_NEAR(e.heading, 0, 1e-6);
    }
    // 5. Heading wrap stays in [0,1).
    {
        Anchor a = MakeAnchor(Car(0, 0, 0, 0), Truck(0, 0, 0, 0.9));
        EtsPlacement e = LfsToEts(a, Car(0, 0, 0, 0.2 * kTau));
        CHECK_NEAR(e.heading, 0.1, 1e-6);
        CHECK(e.heading >= 0 && e.heading < 1);
    }
    // 6. OutSim parse of a hand-built packet.
    {
        std::vector<uint8_t> b;
        auto put = [&](const void* p, size_t n) { auto c = (const uint8_t*)p; b.insert(b.end(), c, c + n); };
        auto f = [&](float x) { put(&x, 4); };
        auto i = [&](int32_t x) { put(&x, 4); };
        put("LFST", 4); i(7); i(12345);
        f(1); f(2); f(3); f(0.5f); f(0.1f); f(-0.2f); f(0); f(0); f(0); f(3); f(4); f(0);
        i(-65536 * 12); i(65536 * 34 + 32768); i(65536 * 2);
        f(0.75f); f(0.25f); f(-0.1f); f(0); f(1);
        uint8_t d[4] = {3, 0, 0, 0}; put(d, 4); f(600); f(300);
        CHECK(b.size() == OutSimSize(kOutSimOpts));
        CHECK(OutSimSize(kOutSimOpts) == 104);
        LfsCarState c{};
        CHECK(ParseOutSim(b.data(), b.size(), kOutSimOpts, c));
        CHECK(c.lfsTimeMs == 12345);
        CHECK_NEAR(c.pos.x, -12, 1e-9);
        CHECK_NEAR(c.pos.y, 34.5, 1e-9);
        CHECK_NEAR(c.pos.z, 2, 1e-9);
        CHECK_NEAR(c.heading, 0.5, 1e-6);
        CHECK_NEAR(c.speedMs, 5, 1e-5);
        CHECK_NEAR(c.throttle, 0.75, 1e-6);
        CHECK_NEAR(c.handbrake, 1, 1e-6);
        CHECK(c.gear == 3);
        CHECK_NEAR(c.engineRadS, 600, 1e-3);
        // Wrong header / wrong size rejected.
        b[0] = 'X';
        CHECK(!ParseOutSim(b.data(), b.size(), kOutSimOpts, c));
        b[0] = 'L';
        CHECK(!ParseOutSim(b.data(), b.size() - 4, kOutSimOpts, c));
        // Full 0x1ff packet is 280 bytes, as LFS docs state.
        CHECK(OutSimSize(0x1ff) == 280);
    }
    // 7. Seqlock slot.
    {
        Slot<EtsTarget> s;
        EtsTarget t{};
        CHECK(!s.read(t));  // never written
        t.valid = 1; t.lfsPackets = 42; t.placement.pos.x = 1.5;
        s.write(t);
        EtsTarget u{};
        CHECK(s.read(u) && u.lfsPackets == 42 && u.placement.pos.x == 1.5);
    }
    // 8. Angles from forward/up vectors (BeamNG camera and vehicle).
    {
        double h, p, r;
        const float up[3] = {0, 0, 1};
        const float north[3] = {0, 1, 0}, west[3] = {-1, 0, 0}, east[3] = {1, 0, 0};
        AnglesFromDirs(north, up, h, p, r);
        CHECK_NEAR(h, 0, 1e-6); CHECK_NEAR(p, 0, 1e-6); CHECK_NEAR(r, 0, 1e-6);
        AnglesFromDirs(west, up, h, p, r);
        CHECK_NEAR(h, kPi / 2, 1e-6);   // CCW from north: west = +90°
        AnglesFromDirs(east, up, h, p, r);
        CHECK_NEAR(h, -kPi / 2, 1e-6);
        const float c30 = float(std::cos(kPi / 6)), s30 = float(std::sin(kPi / 6));
        const float climb[3] = {0, c30, s30}, climbUp[3] = {0, -s30, c30};
        AnglesFromDirs(climb, climbUp, h, p, r);
        CHECK_NEAR(p, kPi / 6, 1e-6); CHECK_NEAR(r, 0, 1e-6);
        // Facing north, body up tilted to the right (east): looking forward that's a
        // clockwise roll, so ETS2's CCW-positive roll is negative.
        const float rollRight[3] = {s30, 0, c30};
        AnglesFromDirs(north, rollRight, h, p, r);
        CHECK_NEAR(r, -kPi / 6, 1e-6);
    }
    // 9. BeamNG packet parsing + anchored mapping of the camera.
    {
        const char* line = "ESB1 12 3456 10.0000 20.0000 1.5000 0.0000 1.0000 0.0000 0.0000 0.0000 1.0000 65.00 "
                           "10.0000 18.0000 0.5000 0.0000 1.0000 0.0000 0.0000 0.0000 1.0000 13.889";
        BngState b{};
        CHECK(ParseBng(line, strlen(line), b));
        CHECK(b.seq == 12 && b.timeMs == 3456);
        CHECK_NEAR(b.camPos.y, 20, 1e-9);
        CHECK_NEAR(b.vehPos.y, 18, 1e-9);
        CHECK_NEAR(b.fovDeg, 65, 1e-4);
        CHECK_NEAR(b.speedMs, 13.889, 1e-3);
        CHECK(!ParseBng("ESB1 1 2 3", 10, b));
        // Car faces north, truck faces west: camera 2 m ahead of the car lands 2 m west.
        Anchor a = MakeAnchor(b.vehPos, 0.0, Truck(500, 10, 600, 0.25));
        EtsPlacement e = SrcToEts(a, b.camPos, 0, 0, 0);
        CHECK_NEAR(e.pos.x, 498, 1e-6);
        CHECK_NEAR(e.pos.y, 11, 1e-6);   // camera 1 m above the car origin
        CHECK_NEAR(e.pos.z, 600, 1e-6);
        CHECK_NEAR(e.heading, 0.25, 1e-6);
    }
    // 10. EtsToSrc inverts SrcToEts (roads go ETS2 -> BeamNG, the car comes back).
    {
        Anchor a = MakeAnchor(Vec3d{0, 0, 500}, 0.0, Truck(43618.7, 18.4, -25649.6, 0.5077));
        const Vec3d pts[] = {{0, 0, 500}, {12.5, -40, 503}, {-300, 250, 480}};
        for (const auto& p : pts) {
            EtsPlacement e = SrcToEts(a, p, 0, 0, 0);
            Vec3d back = EtsToSrc(a, e.pos);
            CHECK_NEAR(back.x, p.x, 1e-6);
            CHECK_NEAR(back.y, p.y, 1e-6);
            CHECK_NEAR(back.z, p.z, 1e-6);
        }
        // The truck's own spot maps to the anchor point.
        Vec3d s0 = EtsToSrc(a, Vec3d{43618.7, 18.4, -25649.6});
        CHECK_NEAR(s0.x, 0, 1e-9);
        CHECK_NEAR(s0.y, 0, 1e-9);
        CHECK_NEAR(s0.z, 500, 1e-9);
    }
    // 11. roads.bin round trip (same layout tools/ets2_roads writes) + streaming hysteresis.
    {
        const char* path = "test_roads.bin";
        FILE* f = nullptr;
        fopen_s(&f, path, "wb");
        CHECK(f != nullptr);
        if (f) {
            fwrite("ESRD", 1, 4, f);
            uint32_t v = 1, count = 2;
            fwrite(&v, 4, 1, f); fwrite(&count, 4, 1, f);
            // strip 0: road near origin, 3 points; strip 1: junction link 5 km away
            const float stripA[] = {0, 10, 0, 50, 11, 0, 100, 12, 0};
            const float stripB[] = {5000, 20, 0, 5010, 20, 0};
            uint32_t kind = 0, n = 3; float w = 26;
            fwrite(&kind, 4, 1, f); fwrite(&w, 4, 1, f); fwrite(&n, 4, 1, f); fwrite(stripA, 4, 9, f);
            kind = 1; n = 2; w = 18;
            fwrite(&kind, 4, 1, f); fwrite(&w, 4, 1, f); fwrite(&n, 4, 1, f); fwrite(stripB, 4, 6, f);
            fclose(f);
        }
        std::vector<RoadStrip> strips;
        std::string err;
        CHECK(LoadRoads(path, strips, err));
        CHECK(strips.size() == 2);
        if (strips.size() == 2) {
            CHECK(strips[0].pts.size() == 3 && strips[1].kind == 1);
            CHECK_NEAR(strips[0].pts[2].y, 12, 1e-6);
            CHECK_NEAR(strips[0].cx, 50, 1e-6);
            RoadStreamer rs;
            std::vector<uint32_t> load, unload;
            rs.Update(strips, 0, 0, 10, load, unload);
            CHECK(load.size() == 1 && load[0] == 0 && unload.empty());
            rs.Update(strips, 0, 0, 10, load, unload);
            CHECK(load.empty() && unload.empty());          // already loaded
            rs.Update(strips, 4900, 0, 10, load, unload);
            CHECK(load.size() == 1 && load[0] == 1);         // far strip comes in
            CHECK(unload.size() == 1 && unload[0] == 0);     // near strip leaves
        }
        remove(path);
    }
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
