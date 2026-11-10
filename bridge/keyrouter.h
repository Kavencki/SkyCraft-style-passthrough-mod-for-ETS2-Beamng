// KeyRouter: while ETS2 has focus, the keyboard drives BeamNG (Esc menu, R reset, camera keys...),
// so there is no need to switch windows. A low-level keyboard hook catches the keys, keeps them
// from ETS2 and sends them to the BeamNG mod ("ESKY 1 ctrl r"), which triggers whatever the
// player's BeamNG keyboard bindings say. A toggle key (default F12) hands the keyboard back to
// ETS2 (e.g. M for its map) and again to BeamNG.
//
// Always left to ETS2 / Windows: the driving keys (they reach the car through ETS2 already),
// Win keys, Alt+Tab, Alt+F4, Ctrl+Esc, Print Screen. Modifier keys go to both.
#pragma once
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <windows.h>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <thread>

namespace es {

struct KeyRouterConfig {
    bool enabled = true;
    bool startBeamng = true;
    UINT toggleVk = VK_F12;
    std::set<std::string> passthrough{"w", "a", "s", "d", "up", "down", "left", "right"};

    void Load(const std::string& path) {
        char v[512];
        auto str = [&](const char* key, const char* def) {
            GetPrivateProfileStringA("keys", key, def, v, sizeof(v), path.c_str());
            return std::string(v);
        };
        enabled = str("route", "1") != "0";
        startBeamng = str("start", "beamng") != "ets2";
        const std::string t = str("toggle", "f12");
        if (UINT vk = VkFromName(t)) toggleVk = vk;
        const std::string p = str("ets2_keys", "w,a,s,d,up,down,left,right");
        passthrough.clear();
        size_t i = 0;
        while (i <= p.size()) {
            size_t j = p.find(',', i);
            if (j == std::string::npos) j = p.size();
            std::string k = p.substr(i, j - i);
            while (!k.empty() && k.front() == ' ') k.erase(0, 1);
            while (!k.empty() && k.back() == ' ') k.pop_back();
            if (!k.empty()) passthrough.insert(k);
            i = j + 1;
        }
    }

    // BeamNG's names for keys (as in its inputmaps), from a virtual-key code.
    static std::string KeyName(UINT vk) {
        if (vk >= 'A' && vk <= 'Z') return std::string(1, char(vk - 'A' + 'a'));
        if (vk >= '0' && vk <= '9') return std::string(1, char(vk));
        if (vk >= VK_F1 && vk <= VK_F24) return "f" + std::to_string(vk - VK_F1 + 1);
        if (vk >= VK_NUMPAD0 && vk <= VK_NUMPAD9) return "numpad" + std::to_string(vk - VK_NUMPAD0);
        switch (vk) {
            case VK_ESCAPE: return "escape";
            case VK_RETURN: return "enter";
            case VK_SPACE: return "space";
            case VK_TAB: return "tab";
            case VK_BACK: return "backspace";
            case VK_INSERT: return "insert";
            case VK_DELETE: return "delete";
            case VK_HOME: return "home";
            case VK_END: return "end";
            case VK_PRIOR: return "pageup";
            case VK_NEXT: return "pagedown";
            case VK_UP: return "up";
            case VK_DOWN: return "down";
            case VK_LEFT: return "left";
            case VK_RIGHT: return "right";
            case VK_OEM_COMMA: return "comma";
            case VK_OEM_PERIOD: return "period";
            case VK_OEM_2: return "slash";
            case VK_OEM_4: return "[";
            case VK_OEM_6: return "]";
            case VK_OEM_3: return "tilde";
            case VK_OEM_1: return "semicolon";
            case VK_OEM_7: return "apostrophe";
            case VK_OEM_5: return "backslash";
            case VK_OEM_MINUS: return "minus";
            case VK_OEM_PLUS: return "equals";
            case VK_ADD: return "numpadadd";
            case VK_SUBTRACT: return "numpadminus";
            case VK_DIVIDE: return "numpaddivide";
            case VK_MULTIPLY: return "*";
            case VK_DECIMAL: return "numpadperiod";
            case VK_LSHIFT: return "lshift";
            case VK_RSHIFT: return "rshift";
            case VK_LCONTROL: return "lcontrol";
            case VK_RCONTROL: return "rcontrol";
            case VK_LMENU: return "lalt";
            case VK_RMENU: return "ralt";
            case VK_PAUSE: return "pause";
        }
        return "";
    }
    static UINT VkFromName(std::string n) {
        for (char& c : n) c = char(std::tolower(static_cast<unsigned char>(c)));
        for (UINT vk = 1; vk < 256; ++vk)
            if (KeyName(vk) == n) return vk;
        return 0;
    }
};

// Windows switching for BeamNG's menu: the mouse only reaches BeamNG under the cursor, so while
// its menu is open BeamNG's window lies exactly over ETS2's picture and is active (the overlay
// keeps showing ETS2 around the car); when the menu closes, ETS2 is active again.
namespace focus {

struct FindTitle { const char* text; HWND found; };
inline BOOL CALLBACK FindProc(HWND h, LPARAM lp) {
    auto* f = reinterpret_cast<FindTitle*>(lp);
    if (!IsWindowVisible(h) || GetWindow(h, GW_OWNER)) return TRUE;
    char title[256] = {}, cls[128] = {};
    GetWindowTextA(h, title, sizeof(title));
    GetClassNameA(h, cls, sizeof(cls));
    if (!strcmp(cls, "ConsoleWindowClass") || !strcmp(cls, "CASCADIA_HOSTING_WINDOW_CLASS")) return TRUE;
    if (strstr(title, f->text) == title) { f->found = h; return FALSE; }
    return TRUE;
}
inline HWND Find(const char* titleStart) {
    FindTitle f{titleStart, nullptr};
    EnumWindows(FindProc, reinterpret_cast<LPARAM>(&f));
    return f.found;
}
// Makes `h` the active window (allowed for us by joining the active window's input queue).
inline void Activate(HWND h) {
    HWND fg = GetForegroundWindow();
    if (fg == h) return;
    const DWORD fgThread = fg ? GetWindowThreadProcessId(fg, nullptr) : 0, me = GetCurrentThreadId();
    if (fgThread && fgThread != me) AttachThreadInput(me, fgThread, TRUE);
    BringWindowToTop(h);
    SetForegroundWindow(h);
    if (fgThread && fgThread != me) AttachThreadInput(me, fgThread, FALSE);
}
// BeamNG's picture exactly over ETS2's, BeamNG active.
inline bool BeamngOverEts2() {
    HWND ets = Find("Euro Truck Simulator 2"), bng = Find("BeamNG.drive");
    if (!ets || !bng) return false;
    RECT ec{}, bw{}, bc{};
    GetClientRect(ets, &ec);
    POINT eo{0, 0};
    ClientToScreen(ets, &eo);
    GetWindowRect(bng, &bw);
    GetClientRect(bng, &bc);
    POINT bo{0, 0};
    ClientToScreen(bng, &bo);
    const int offX = bo.x - bw.left, offY = bo.y - bw.top;
    const int extraW = (bw.right - bw.left) - bc.right, extraH = (bw.bottom - bw.top) - bc.bottom;
    SetWindowPos(bng, HWND_TOP, eo.x - offX, eo.y - offY, ec.right + extraW, ec.bottom + extraH, SWP_NOACTIVATE);
    Activate(bng);
    return true;
}
inline void Ets2Active() {
    HWND ets = Find("Euro Truck Simulator 2"), bng = Find("BeamNG.drive");
    if (ets && GetForegroundWindow() == bng) Activate(ets);
}

}  // namespace focus

class KeyRouter {
public:
    // Starts the hook thread. `bngPort`: the BeamNG mod's input port.
    void Start(const KeyRouterConfig& cfg, int bngPort) {
        cfg_ = cfg;
        if (!cfg_.enabled) return;
        toBeamng_ = cfg_.startBeamng;
        port_ = bngPort;
        self_ = this;
        std::thread([this] { Run(); }).detach();
    }
    bool ToBeamng() const { return toBeamng_.load(); }
    // BeamNG's menu is open: every key goes to it (arrows and Enter move through the menu).
    void SetMenu(bool open) { menu_ = open; }

    // The driving keys held while ETS2 has focus (W/Up, S/Down, A/Left, D/Right): the bridge drives
    // the car with them itself - ETS2 1.62 reports no throttle or brake for the parked truck.
    enum : uint32_t { kKeyThrottle = 1, kKeyBrake = 2, kKeyLeft = 4, kKeyRight = 8 };
    uint32_t DriveKeys() const { return driveKeys_.load(); }

private:
    static inline KeyRouter* self_ = nullptr;
    KeyRouterConfig cfg_;
    std::atomic<bool> toBeamng_{true};
    std::atomic<bool> menu_{false};
    std::atomic<uint32_t> driveKeys_{0};
    static uint32_t DriveBit(UINT vk) {
        switch (vk) {
            case 'W': case VK_UP: return kKeyThrottle;
            case 'S': case VK_DOWN: return kKeyBrake;
            case 'A': case VK_LEFT: return kKeyLeft;
            case 'D': case VK_RIGHT: return kKeyRight;
        }
        return 0;
    }
    int port_ = 0;
    SOCKET sock_ = INVALID_SOCKET;
    sockaddr_in to_{};
    std::set<UINT> held_;            // keys pressed into BeamNG (their release goes there too)
    bool ctrl_ = false, alt_ = false, shift_ = false;
    HWND ets2_ = nullptr;

    void Send(const std::string& m) {
        if (sock_ != INVALID_SOCKET) sendto(sock_, m.data(), int(m.size()), 0, reinterpret_cast<sockaddr*>(&to_), sizeof(to_));
    }

    // ETS2 (or the click-through overlay over it) is the active window.
    bool Ets2Active() {
        HWND fg = GetForegroundWindow();
        if (!fg) return false;
        if (fg == ets2_) return true;
        char title[256] = {}, cls[128] = {};
        GetWindowTextA(fg, title, sizeof(title));
        GetClassNameA(fg, cls, sizeof(cls));
        if (strstr(title, "Euro Truck Simulator 2")) { ets2_ = fg; return true; }
        return strcmp(cls, "EuroBeamNGOverlay") == 0;
    }

    void ReleaseAll() {
        for (UINT vk : held_) Send("ESKY 0 " + KeyRouterConfig::KeyName(vk));
        held_.clear();
    }

    void Run() {
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        to_.sin_family = AF_INET;
        to_.sin_port = htons(static_cast<u_short>(port_));
        to_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        HHOOK hook = SetWindowsHookExW(WH_KEYBOARD_LL, &Proc, GetModuleHandleW(nullptr), 0);
        if (!hook) { printf("[bridge] keyboard routing: cannot hook the keyboard (%lu)\n", GetLastError()); return; }
        printf("[bridge] keyboard: with ETS2 focused, keys go to %s - %s switches (driving keys stay with ETS2)\n",
               toBeamng_ ? "BeamNG" : "ETS2", KeyRouterConfig::KeyName(cfg_.toggleVk).c_str());
        MSG msg;
        while (GetMessageW(&msg, nullptr, 0, 0) > 0) {}
        UnhookWindowsHookEx(hook);
    }

    static LRESULT CALLBACK Proc(int code, WPARAM wp, LPARAM lp) {
        if (code == HC_ACTION && self_) {
            const auto* k = reinterpret_cast<const KBDLLHOOKSTRUCT*>(lp);
            const bool down = wp == WM_KEYDOWN || wp == WM_SYSKEYDOWN;
            if (!(k->flags & LLKHF_INJECTED) && self_->Handle(k->vkCode, down)) return 1;   // swallowed
        }
        return CallNextHookEx(nullptr, code, wp, lp);
    }

    // true: the key went to BeamNG (ETS2 does not get it).
    bool Handle(UINT vk, bool down) {
        const bool isMod = vk == VK_LCONTROL || vk == VK_RCONTROL || vk == VK_LMENU || vk == VK_RMENU ||
                           vk == VK_LSHIFT || vk == VK_RSHIFT;
        if (vk == VK_LCONTROL || vk == VK_RCONTROL) ctrl_ = down;
        if (vk == VK_LMENU || vk == VK_RMENU) alt_ = down;
        if (vk == VK_LSHIFT || vk == VK_RSHIFT) shift_ = down;

        // Driving keys: held while ETS2 has focus (not in BeamNG's menu); released anywhere.
        if (const uint32_t bit = DriveBit(vk)) {
            if (!down) driveKeys_ &= ~bit;
            else if (!menu_.load() && Ets2Active()) driveKeys_ |= bit;
        }

        // A key that went to BeamNG is released there, whatever has focus now.
        if (!down && held_.count(vk)) {
            held_.erase(vk);
            Send("ESKY 0 " + KeyRouterConfig::KeyName(vk));
            return !isMod;
        }
        if (!Ets2Active()) return false;

        if (vk == cfg_.toggleVk) {
            if (down) {
                const bool b = !toBeamng_.load();
                toBeamng_ = b;
                if (!b) ReleaseAll();
                Send(b ? "ESKM beamng" : "ESKM ets2");
                printf("[bridge] keyboard -> %s\n", b ? "BeamNG" : "ETS2");
                Beep(b ? 880 : 440, 60);
            }
            return true;
        }
        if (!toBeamng_.load()) return false;
        // Windows' own shortcuts.
        if (vk == VK_LWIN || vk == VK_RWIN || vk == VK_SNAPSHOT) return false;
        if (alt_ && (vk == VK_TAB || vk == VK_F4 || vk == VK_ESCAPE)) return false;
        if (ctrl_ && vk == VK_ESCAPE) return false;

        const std::string name = KeyRouterConfig::KeyName(vk);
        if (name.empty() || (!menu_.load() && cfg_.passthrough.count(name))) return false;
        if (down) {
            if (held_.count(vk)) return !isMod;   // auto-repeat: BeamNG already has it down
            held_.insert(vk);
            std::string control;
            if (!isMod) {
                if (ctrl_) control += "ctrl ";
                if (alt_) control += "alt ";
                if (shift_) control += "shift ";
            }
            Send("ESKY 1 " + control + name);
        }
        return !isMod;   // modifiers reach Windows too (Alt+Tab, Ctrl+Alt+Del keep working)
    }
};

}  // namespace es
