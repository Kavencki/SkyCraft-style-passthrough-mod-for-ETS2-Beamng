// lfs_insim.exe - tiny InSim client for single-player test automation.
// Sends LFS commands (e.g. "/sp", "/track BL1", "/ai") and reports state.
// Refuses to send anything if LFS says a multiplayer session is active (IS_ISM).
//
//   lfs_insim [--port 29999] [--wait N] "/cmd1" "/cmd2" ...
//     commands are sent ~1.5 s apart; "@N" sleeps N seconds instead.
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#pragma comment(lib, "ws2_32.lib")

enum : uint8_t { ISP_ISI = 1, ISP_VER = 2, ISP_TINY = 3, ISP_STA = 5, ISP_ISM = 10, ISP_MSO = 11, ISP_MST = 13, ISP_MSX = 39 };
enum : uint8_t { TINY_NONE = 0, TINY_VER = 1, TINY_CLOSE = 2, TINY_ISM = 10 };

static SOCKET g_s = INVALID_SOCKET;
static bool g_multiplayer = false;
static std::vector<uint8_t> g_rx;

static void SendPkt(std::vector<uint8_t> p) {
    while (p.size() % 4) p.push_back(0);
    p[0] = static_cast<uint8_t>(p.size() / 4);  // InSim 9+: Size is bytes/4
    send(g_s, reinterpret_cast<const char*>(p.data()), static_cast<int>(p.size()), 0);
}
static void Tiny(uint8_t subt, uint8_t reqi = 1) { SendPkt({0, ISP_TINY, reqi, subt}); }

static void Pump(int ms) {
    uint64_t end = GetTickCount64() + ms;
    while (GetTickCount64() < end) {
        fd_set fs; FD_ZERO(&fs); FD_SET(g_s, &fs);
        timeval tv{0, 50000};
        if (select(0, &fs, nullptr, nullptr, &tv) <= 0) continue;
        uint8_t buf[2048];
        int n = recv(g_s, reinterpret_cast<char*>(buf), sizeof(buf), 0);
        if (n <= 0) { printf("[insim] connection closed\n"); exit(1); }
        g_rx.insert(g_rx.end(), buf, buf + n);
        while (!g_rx.empty() && g_rx.size() >= size_t(g_rx[0]) * 4 && g_rx[0]) {
            size_t len = size_t(g_rx[0]) * 4;
            const uint8_t* p = g_rx.data();
            switch (p[1]) {
            case ISP_VER: printf("[insim] LFS %.8s %.6s, InSim v%u\n", (const char*)p + 4, (const char*)p + 12, p[18]); break;
            case ISP_TINY: if (p[3] == TINY_NONE) Tiny(TINY_NONE, 0); break;  // keepalive
            case ISP_ISM:  // LFS 0.8 also answers with an empty name when offline
                if (p[8]) { g_multiplayer = true; printf("[insim] MULTIPLAYER session '%.32s' - refusing\n", (const char*)p + 8); }
                break;
            case 21 /*ISP_NPL*/: printf("[insim] player joined: PLID %u%s\n", p[3], (p[5] & 2) ? " (AI)" : ""); break;
            case ISP_MSO: printf("[insim] msg: %s\n", (const char*)p + 8); break;
            case ISP_STA: {
                uint16_t flags; memcpy(&flags, p + 8, 2);
                if (flags & 512 /*ISS_MULTI*/) { g_multiplayer = true; printf("[insim] MULTIPLAYER (ISS_MULTI) - refusing\n"); }
                printf("[insim] state: flags %04x, ViewPLID %u, InGameCam %u, RaceInProg %u, Track %.6s\n",
                       flags, p[11], p[10], p[15], (const char*)p + 20);
                break;
            }
            }
            g_rx.erase(g_rx.begin(), g_rx.begin() + len);
        }
    }
}

int main(int argc, char** argv) {
    int port = 29999, wait = 30;
    std::vector<std::string> cmds;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--wait") && i + 1 < argc) wait = atoi(argv[++i]);
        else cmds.push_back(argv[i]);
    }
    WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons((u_short)port);
    inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    for (int t = 0;; ++t) {  // LFS may still be starting
        g_s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (connect(g_s, (sockaddr*)&a, sizeof(a)) == 0) break;
        closesocket(g_s);
        if (t >= wait) { printf("[insim] no InSim on port %d\n", port); return 1; }
        Sleep(1000);
    }
    // IS_ISI: InSimVer 9, local flag, name "EuroBeamNG".
    std::vector<uint8_t> isi(44, 0);
    isi[1] = ISP_ISI; isi[2] = 1; isi[6] = 4 /*ISF_LOCAL*/; isi[8] = 9;
    memcpy(&isi[28], "EuroBeamNG", 9);
    SendPkt(isi);
    Tiny(TINY_ISM, 2);
    Tiny(7 /*TINY_SST*/, 2);
    Pump(1000);
    if (g_multiplayer) { Tiny(TINY_CLOSE); return 3; }

    for (auto& c : cmds) {
        if (c[0] == '@') { Pump(atoi(c.c_str() + 1) * 1000); continue; }
        if (c[0] == '!') {  // !view N : IS_SCC, point the camera at PLID N
            SendPkt({0, 8 /*ISP_SCC*/, 0, 0, (uint8_t)atoi(c.c_str() + 5), 255, 0, 0});
            printf("[insim] viewing PLID %s\n", c.c_str() + 5);
            Pump(1000);
            continue;
        }
        Tiny(7 /*TINY_SST*/, 2);
        Pump(300);
        if (g_multiplayer) { Tiny(TINY_CLOSE); return 3; }
        std::vector<uint8_t> mst(68, 0);
        mst[1] = ISP_MST;
        strncpy_s(reinterpret_cast<char*>(&mst[4]), 64, c.c_str(), 63);
        SendPkt(mst);
        printf("[insim] sent %s\n", c.c_str());
        Pump(1500);
    }
    Tiny(7 /*TINY_SST*/, 3);
    Pump(500);
    Tiny(TINY_CLOSE);
    closesocket(g_s);
    return 0;
}
