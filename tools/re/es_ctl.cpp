// es_ctl.exe - talks to the in-game probe in eurospeed.dll.
//   es_ctl scan                      find truck position candidates
//   es_ctl filter                    keep candidates matching the truck now
//   es_ctl list
//   es_ctl poke <i> <dx> <dy> <dz> [frames]
//   es_ctl dump <hexaddr> <bytes>    hex + float + double view
//   es_ctl write <hexaddr> f|d <v1> [v2 ...]
//   es_ctl puppet <i>|off [cam]      follow vehicle target (default) or camera target
//   es_ctl state                     ETS2 / BeamNG / camera target status
//   es_ctl scanlocal                 all local-frame triples near the truck's height
//   es_ctl remember                  store candidate values
//   es_ctl changed|unchanged [m]     keep candidates that moved more / less than m
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "../protocol/eurospeed_protocol.h"

using namespace es;

int main(int argc, char** argv) {
    if (argc < 2) { printf("see source header for usage\n"); return 2; }
    HANDLE map = nullptr;
    Shared* sh = OpenShared(&map);
    if (!sh) { printf("shared memory version mismatch\n"); return 1; }
    uint64_t hb = sh->pluginHeartbeatMs.load();
    if (!hb || GetTickCount64() - hb > kHeartbeatTimeoutMs) { printf("ETS2 plugin not running (or game paused in menu)\n"); return 1; }

    EtsTruckState ets{};
    if (sh->ets.read(ets))
        printf("[ets2] frames %u, paused %u, multiplayer %u, truck (%.3f, %.3f, %.3f) hdg %.4f\n", ets.frames,
               ets.paused, ets.multiplayer, ets.placement.pos.x, ets.placement.pos.y, ets.placement.pos.z,
               ets.placement.heading);
    BngState bs{};
    if (sh->bng.read(bs))
        printf("[bng]  #%u veh (%.2f, %.2f, %.2f) %.1f km/h, cam (%.2f, %.2f, %.2f) fov %.1f\n", bs.seq, bs.vehPos.x,
               bs.vehPos.y, bs.vehPos.z, bs.speedMs * 3.6, bs.camPos.x, bs.camPos.y, bs.camPos.z, bs.fovDeg);
    EtsCamTarget ct{};
    if (sh->cam.read(ct) && ct.valid)
        printf("[cam]  target (%.2f, %.2f, %.2f) hdg %.4f pitch %.4f roll %.4f\n", ct.placement.pos.x,
               ct.placement.pos.y, ct.placement.pos.z, ct.placement.heading, ct.placement.pitch, ct.placement.roll);
    if (!strcmp(argv[1], "state")) return 0;

    DevCmd c{};
    const char* op = argv[1];
    auto num = [&](int i) { return i < argc ? atof(argv[i]) : 0.0; };
    if (!strcmp(op, "scan")) { c.op = DEV_SCAN; c.d[0] = num(2); }
    else if (!strcmp(op, "autopoke")) { c.op = DEV_AUTOPOKE; c.d[0] = num(2); c.d[1] = num(3); c.d[2] = num(4); c.arg1 = argc > 5 ? atoi(argv[5]) : 4; }
    else if (!strcmp(op, "filter")) c.op = DEV_FILTER;
    else if (!strcmp(op, "scanrot")) c.op = DEV_SCANROT;
    else if (!strcmp(op, "snap")) c.op = DEV_SNAP;
    else if (!strcmp(op, "diff")) { c.op = DEV_DIFF; c.d[0] = num(2); }
    else if (!strcmp(op, "list")) c.op = DEV_LIST;
    else if (!strcmp(op, "poke")) { c.op = DEV_POKE; c.arg0 = atoi(argv[2]); c.d[0] = num(3); c.d[1] = num(4); c.d[2] = num(5); c.arg1 = argc > 6 ? atoi(argv[6]) : 30; }
    else if (!strcmp(op, "dump")) { c.op = DEV_DUMP; c.arg0 = strtoull(argv[2], nullptr, 16); c.arg1 = atoi(argv[3]); }
    else if (!strcmp(op, "puppet")) {
        c.op = DEV_PUPPET; c.arg0 = strcmp(argv[2], "off") ? atoi(argv[2]) : ~0ull;
        c.arg1 = argc > 3 && !strcmp(argv[3], "cam");
    }
    else if (!strcmp(op, "remember")) c.op = DEV_REMEMBER;
    else if (!strcmp(op, "changed")) { c.op = DEV_KEEP; c.arg0 = 1; c.d[0] = num(2); }
    else if (!strcmp(op, "unchanged")) { c.op = DEV_KEEP; c.arg0 = 0; c.d[0] = num(2); }
    else if (!strcmp(op, "scanlocal")) c.op = DEV_SCANLOCAL;
    else if (!strcmp(op, "scansector")) { c.op = DEV_SCANSECTOR; c.arg0 = argc > 2 && strcmp(argv[2], "truck") ? atoi(argv[2]) : ~0ull; c.d[0] = argc > 3 ? atof(argv[3]) : 1.0; }
    else if (!strcmp(op, "hookcam")) {  // hookcam <cand>|off [hexModuleOffset srcOff dstOff]
        c.op = DEV_HOOKCAM;
        c.arg0 = strcmp(argv[2], "off") ? atoi(argv[2]) : ~0ull;
        c.arg1 = argc > 3 ? strtoull(argv[3], nullptr, 16) : 0;
        c.d[0] = argc > 4 ? double(strtoul(argv[4], nullptr, 16)) : 0;
        c.d[1] = argc > 5 ? double(strtoul(argv[5], nullptr, 16)) : 0;
        c.d[2] = argc > 6 && !strcmp(argv[6], "posonly") ? 1 : 0;
    }
    else if (!strcmp(op, "watch")) { c.op = DEV_WATCH; c.arg0 = atoi(argv[2]); c.d[0] = argc > 3 ? atof(argv[3]) : 500; }
    else if (!strcmp(op, "watchaddr")) { c.op = DEV_WATCH; c.arg0 = ~0ull; c.arg1 = strtoull(argv[2], nullptr, 16); c.d[0] = argc > 3 ? atof(argv[3]) : 500; }
    else if (!strcmp(op, "exec")) {  // exec <hexModuleOffset> [ms] [reg hexValue]
        c.op = DEV_EXEC;
        c.arg1 = strtoull(argv[2], nullptr, 16);
        c.d[0] = argc > 3 ? atof(argv[3]) : 500;
        if (argc > 5) {
            static const char* rn[16] = {"rax", "rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi",
                                         "r8",  "r9",  "r10", "r11", "r12", "r13", "r14", "r15"};
            for (int i = 0; i < 16; ++i) if (!strcmp(argv[4], rn[i])) c.d[1] = i + 1;
            c.d[2] = double(strtoull(argv[5], nullptr, 16));
        }
    }
    else if (!strcmp(op, "write")) {  // write <hexaddr> f|d|i v1 [v2 ...]   (float, double, int32)
        c.op = DEV_WRITE;
        c.arg0 = strtoull(argv[2], nullptr, 16);
        const char t = argv[3][0];
        size_t n = 0;
        for (int i = 4; i < argc && n + 8 <= sizeof(c.data); ++i) {
            if (t == 'f') { float v = float(atof(argv[i])); memcpy(c.data + n, &v, 4); n += 4; }
            else if (t == 'd') { double v = atof(argv[i]); memcpy(c.data + n, &v, 8); n += 8; }
            else { int32_t v = int32_t(strtol(argv[i], nullptr, 0)); memcpy(c.data + n, &v, 4); n += 4; }
        }
        c.arg1 = n;
    }
    else { printf("unknown op\n"); return 2; }

    memcpy(&sh->devCmd, &c, sizeof(c));
    uint32_t seq = sh->devSeq.load() + 1;
    sh->devSeq.store(seq, std::memory_order_release);

    uint64_t start = GetTickCount64();
    while (sh->devResult.seq != seq) {
        if (GetTickCount64() - start > 900000) { printf("timeout (game paused? the plugin runs on frame_end)\n"); return 1; }
        Sleep(20);
    }
    printf("%s", sh->devResult.text);
    if (c.op == DEV_DUMP && sh->devResult.nbytes) {
        const uint8_t* b = sh->devResult.bytes;
        for (uint32_t o = 0; o < sh->devResult.nbytes; o += 16) {
            printf("%016llx +%03x: ", (unsigned long long)(c.arg0 + o), o);
            for (int i = 0; i < 16; ++i) printf("%02x ", b[o + i]);
            float f[4]; memcpy(f, b + o, 16);
            double d[2]; memcpy(d, b + o, 16);
            printf("| f %10.4g %10.4g %10.4g %10.4g | d %12.6g %12.6g\n", f[0], f[1], f[2], f[3], d[0], d[1]);
        }
    }
    int st = sh->devResult.status;
    CloseShared(sh, map);
    return st < 0 ? 1 : 0;
}
