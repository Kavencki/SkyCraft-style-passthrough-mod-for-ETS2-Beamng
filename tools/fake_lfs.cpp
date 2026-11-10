// fake_lfs.exe - stand-in for LFS's OutSim sender. Drives a 50 m circle at
// 72 km/h and sends OutSimPack2 packets (same layout LFS sends) at 100 Hz.
//
//   fake_lfs [--port 30000] [--seconds N]
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "../protocol/mapping.h"

#pragma comment(lib, "ws2_32.lib")

int main(int argc, char** argv) {
    int port = 30000, seconds = 5;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atoi(argv[++i]);
    }
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(static_cast<u_short>(port));
    inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);

    const double R = 50.0, v = 20.0, cx = -120.0, cy = 340.0, z = 2.5;
    const int hz = 100, total = seconds * hz;
    for (int i = 0; i < total; ++i) {
        const double t = double(i) / hz, a = v / R * t;  // CCW around (cx,cy)
        std::vector<uint8_t> b;
        auto put = [&](const void* p, size_t n) { auto c = (const uint8_t*)p; b.insert(b.end(), c, c + n); };
        auto f = [&](float x) { put(&x, 4); };
        auto n32 = [&](int32_t x) { put(&x, 4); };
        put("LFST", 4);
        n32(0);                                  // ID
        n32(int32_t(t * 1000));                  // Time
        f(0); f(0); f(float(v / R));             // AngVel
        // On a CCW circle starting east of centre the car faces north at a=0.
        f(float(a)); f(0.02f); f(-0.01f);        // Heading, Pitch, Roll
        f(0); f(0); f(0);                        // Accel
        f(float(-v * sin(a))); f(float(v * cos(a))); f(0);  // Vel
        n32(int32_t((cx + R * cos(a)) * 65536)); n32(int32_t((cy + R * sin(a)) * 65536)); n32(int32_t(z * 65536));
        f(0.6f); f(0); f(0.1f); f(0); f(0);      // Inputs
        uint8_t drive[4] = {4, 0, 0, 0}; put(drive, 4);  // 3rd gear
        f(500.0f); f(250.0f);
        if (b.size() != es::OutSimSize(es::kOutSimOpts)) { printf("size bug %zu\n", b.size()); return 1; }
        sendto(s, (const char*)b.data(), (int)b.size(), 0, (sockaddr*)&to, sizeof(to));
        Sleep(1000 / hz);
    }
    printf("[fake_lfs] sent %d OutSim packets to 127.0.0.1:%d\n", total, port);
    closesocket(s);
    WSACleanup();
    return 0;
}
