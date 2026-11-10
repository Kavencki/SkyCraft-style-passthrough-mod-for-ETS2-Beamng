// LFS <-> ETS2 coordinate mapping. See docs/DESIGN.md §4.
//
// LFS world:  X east, Y north, Z up. Heading radians CCW from north.
// ETS2 world: X east, Y up, Z south. Heading turns CCW from north.
// Scale: 1 LFS metre = 1 ETS2 metre (ETS2 physics units are metres).
//
// The two worlds share no origin, so we anchor: at link time the LFS car's
// pose is pinned to the ETS2 truck's pose, and LFS motion relative to that
// anchor is rotated by the heading difference and applied in ETS2.
#pragma once
#include <cmath>
#include <cstdio>
#include "eurospeed_protocol.h"

namespace es {

constexpr double kPi  = 3.14159265358979323846;
constexpr double kTau = 2.0 * kPi;

inline double WrapTurns01(double t) {        // [0,1)
    t = std::fmod(t, 1.0);
    return t < 0 ? t + 1.0 : t;
}
inline double WrapTurnsHalf(double t) {      // [-0.5,0.5)
    t = WrapTurns01(t + 0.5) - 0.5;
    return t;
}

// Source worlds (LFS, BeamNG) both use X east, Y north, Z up.
struct Anchor {
    Vec3d        srcPos;
    double       srcHeadingRad;
    EtsPlacement ets;
};

inline Anchor MakeAnchor(const Vec3d& srcPos, double srcHeadingRad, const EtsPlacement& truck) {
    return Anchor{srcPos, srcHeadingRad, truck};
}
inline Anchor MakeAnchor(const LfsCarState& car, const EtsPlacement& truck) {
    return MakeAnchor(car.pos, car.heading, truck);
}

// Maps a source pose (heading CCW from north, pitch + nose up, roll in radians).
inline EtsPlacement SrcToEts(const Anchor& a, const Vec3d& p, double headingRad, double pitchRad, double rollRad) {
    const double dTheta = a.ets.heading * kTau - a.srcHeadingRad;  // CCW, radians
    const double c = std::cos(dTheta), s = std::sin(dTheta);

    const double dx = p.x - a.srcPos.x;         // east
    const double dy = p.y - a.srcPos.y;         // north
    const double dz = p.z - a.srcPos.z;         // up

    const double rx = dx * c - dy * s;          // rotated east
    const double ry = dx * s + dy * c;          // rotated north

    EtsPlacement out;
    out.pos.x = a.ets.pos.x + rx;
    out.pos.y = a.ets.pos.y + dz;
    out.pos.z = a.ets.pos.z - ry;               // ETS2 +Z is south
    out.heading = WrapTurns01((headingRad + dTheta) / kTau);
    out.pitch   = pitchRad / kTau;              // both: + is nose up
    out.roll    = WrapTurnsHalf(rollRad / kTau);
    return out;
}

// Inverse of SrcToEts for a point: ETS2 world -> source world (BeamNG/LFS).
inline Vec3d EtsToSrc(const Anchor& a, const Vec3d& e) {
    const double dTheta = a.ets.heading * kTau - a.srcHeadingRad;
    const double c = std::cos(dTheta), s = std::sin(dTheta);
    const double rx = e.x - a.ets.pos.x;        // rotated east
    const double ry = -(e.z - a.ets.pos.z);     // rotated north (ETS2 +Z is south)
    const double dz = e.y - a.ets.pos.y;        // up
    // Undo the rotation by dTheta.
    const double dx = rx * c + ry * s;
    const double dy = -rx * s + ry * c;
    return Vec3d{a.srcPos.x + dx, a.srcPos.y + dy, a.srcPos.z + dz};
}

inline EtsPlacement LfsToEts(const Anchor& a, const LfsCarState& car) {
    // LFS roll is CCW looking forward along +Y; ETS2 roll is CCW looking along
    // the roll axis. Assumed same sign; to be pinned against the real game.
    return SrcToEts(a, car.pos, car.heading, car.pitch, car.roll);
}

// Heading/pitch/roll from forward + up vectors (X east, Y north, Z up).
// Roll is CCW looking along the forward axis, matching ETS2's definition.
inline void AnglesFromDirs(const float f[3], const float u[3], double& heading, double& pitch, double& roll) {
    const double fx = f[0], fy = f[1], fz = f[2];
    heading = std::atan2(-fx, fy);
    pitch = std::asin(fz < -1 ? -1 : fz > 1 ? 1 : fz);
    // World up projected onto the plane perpendicular to forward, vs body up.
    double wx = -fz * fx, wy = -fz * fy, wz = 1 - fz * fz;            // worldUp - f*(f.z)
    const double ud = u[0] * fx + u[1] * fy + u[2] * fz;
    double bx = u[0] - fx * ud, by = u[1] - fy * ud, bz = u[2] - fz * ud;
    const double wl = std::sqrt(wx * wx + wy * wy + wz * wz), bl = std::sqrt(bx * bx + by * by + bz * bz);
    if (wl < 1e-6 || bl < 1e-6) { roll = 0; return; }
    wx /= wl; wy /= wl; wz /= wl; bx /= bl; by /= bl; bz /= bl;
    const double cx = wy * bz - wz * by, cy = wz * bx - wx * bz, cz = wx * by - wy * bx;
    // Right-hand rotation about f; looking along f that reads clockwise, so negate.
    roll = -std::atan2(cx * fx + cy * fy + cz * fz, wx * bx + wy * by + wz * bz);
}

// ---- OutSim (LFS docs/OutSimPack.txt) --------------------------------------

constexpr uint32_t OSO_HEADER = 1, OSO_ID = 2, OSO_TIME = 4, OSO_MAIN = 8,
                   OSO_INPUTS = 16, OSO_DRIVE = 32, OSO_DISTANCE = 64,
                   OSO_WHEELS = 128, OSO_EXTRA_1 = 256;
// What we ask LFS for (cfg.txt "OutSim Opts"). Header lets us validate.
constexpr uint32_t kOutSimOpts = OSO_HEADER | OSO_ID | OSO_TIME | OSO_MAIN |
                                 OSO_INPUTS | OSO_DRIVE;

inline size_t OutSimSize(uint32_t opts) {
    size_t n = 0;
    if (opts & OSO_HEADER)   n += 4;
    if (opts & OSO_ID)       n += 4;
    if (opts & OSO_TIME)     n += 4;
    if (opts & OSO_MAIN)     n += 60;
    if (opts & OSO_INPUTS)   n += 20;
    if (opts & OSO_DRIVE)    n += 12;
    if (opts & OSO_DISTANCE) n += 8;
    if (opts & OSO_WHEELS)   n += 4 * 40;
    if (opts & OSO_EXTRA_1)  n += 8;
    return n;
}

// Parses an OutSimPack2 built with `opts`. Returns false if the size or
// header doesn't match. `out.packets` is left to the caller.
inline bool ParseOutSim(const uint8_t* p, size_t len, uint32_t opts, LfsCarState& out) {
    if (len != OutSimSize(opts) || !(opts & OSO_MAIN)) return false;
    size_t o = 0;
    auto f32 = [&](float& v) { std::memcpy(&v, p + o, 4); o += 4; };
    auto i32 = [&]() { int32_t v; std::memcpy(&v, p + o, 4); o += 4; return v; };

    if (opts & OSO_HEADER) {
        if (std::memcmp(p, "LFST", 4) != 0) return false;
        o += 4;
    }
    if (opts & OSO_ID) o += 4;
    out.lfsTimeMs = 0;
    if (opts & OSO_TIME) out.lfsTimeMs = static_cast<uint32_t>(i32());

    f32(out.angVel[0]); f32(out.angVel[1]); f32(out.angVel[2]);
    f32(out.heading); f32(out.pitch); f32(out.roll);
    float accel[3]; f32(accel[0]); f32(accel[1]); f32(accel[2]);
    f32(out.vel[0]); f32(out.vel[1]); f32(out.vel[2]);
    out.pos.x = i32() / 65536.0;
    out.pos.y = i32() / 65536.0;
    out.pos.z = i32() / 65536.0;
    out.speedMs = std::sqrt(out.vel[0] * out.vel[0] + out.vel[1] * out.vel[1] +
                            out.vel[2] * out.vel[2]);

    out.throttle = out.brake = out.steer = out.clutch = out.handbrake = 0;
    if (opts & OSO_INPUTS) {
        f32(out.throttle); f32(out.brake); f32(out.steer); f32(out.clutch); f32(out.handbrake);
    }
    out.gear = 1; out.engineRadS = 0;
    if (opts & OSO_DRIVE) {
        out.gear = p[o]; o += 4;
        f32(out.engineRadS);
        o += 4;  // MaxTorqueAtVel
    }
    return true;
}

// ---- BeamNG mod packets (beamng_mod/lua/ge/extensions/eurospeed.lua) ------

inline bool ParseBng(const char* line, size_t len, BngState& b) {
    char buf[512];
    if (len == 0 || len >= sizeof(buf)) return false;
    std::memcpy(buf, line, len);
    buf[len] = 0;
    double cp[3], vp[3];
    unsigned seq = 0, t = 0;
    int n = sscanf_s(buf,
                     "ESB1 %u %u %lf %lf %lf %f %f %f %f %f %f %f %lf %lf %lf %f %f %f %f %f %f %f",
                     &seq, &t, &cp[0], &cp[1], &cp[2], &b.camFwd[0], &b.camFwd[1], &b.camFwd[2],
                     &b.camUp[0], &b.camUp[1], &b.camUp[2], &b.fovDeg, &vp[0], &vp[1], &vp[2],
                     &b.vehFwd[0], &b.vehFwd[1], &b.vehFwd[2], &b.vehUp[0], &b.vehUp[1], &b.vehUp[2],
                     &b.speedMs);
    if (n != 22) return false;
    b.seq = seq;
    b.timeMs = t;
    b.camPos = {cp[0], cp[1], cp[2]};
    b.vehPos = {vp[0], vp[1], vp[2]};
    return true;
}

}  // namespace es
