// Reads a game controller (steering wheel, pedals) with DirectInput in background,
// non-exclusive mode: works whichever window has focus and does not take the device from
// the games (ETS2 keeps its force feedback).
#pragma once
#define DIRECTINPUT_VERSION 0x0800
#include <windows.h>
#include <dinput.h>
#include <cstdio>
#include <cstring>
#include <string>

#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "user32.lib")

namespace es {

struct WheelAxes {
    long v[8];  // lX lY lZ lRx lRy lRz slider0 slider1, raw 0..65535
};
static const char* const kAxisNames[8] = {"x", "y", "z", "rx", "ry", "rz", "s0", "s1"};

struct Wheel {
    IDirectInput8A* di = nullptr;
    IDirectInputDevice8A* dev = nullptr;
    std::string name;
    std::string want;  // substring of the device name to pick ("" = first wheel / joystick)

    static BOOL CALLBACK Enum(const DIDEVICEINSTANCEA* inst, void* ctx) {
        Wheel* w = static_cast<Wheel*>(ctx);
        const std::string n = inst->tszProductName;
        if (!w->want.empty()) {
            std::string a = n, b = w->want;
            for (auto& c : a) c = char(tolower(c));
            for (auto& c : b) c = char(tolower(c));
            if (a.find(b) == std::string::npos) return DIENUM_CONTINUE;
        }
        if (FAILED(w->di->CreateDevice(inst->guidInstance, &w->dev, nullptr))) return DIENUM_CONTINUE;
        w->name = n;
        return DIENUM_STOP;
    }

    // Force feedback (constant force on the steering axis). Needs exclusive access to the device,
    // taken in the background through a hidden window of ours, so it works with ETS2 focused.
    IDirectInputEffect* eff = nullptr;
    HWND hidden = nullptr;
    bool exclusive = false;
    long lastForce = 0x7fffffff;

    bool Open(const std::string& match, bool wantFfb = false) {
        want = match;
        if (FAILED(DirectInput8Create(GetModuleHandleA(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8A,
                                      reinterpret_cast<void**>(&di), nullptr)))
            return false;
        di->EnumDevices(DI8DEVCLASS_GAMECTRL, Enum, this, DIEDFL_ATTACHEDONLY);
        if (!dev) di->EnumDevices(DI8DEVCLASS_GAMECTRL, Enum, this, DIEDFL_ALLDEVICES);
        if (!dev) return false;
        HWND hwnd = GetConsoleWindow();
        if (!hwnd) hwnd = GetDesktopWindow();
        dev->SetDataFormat(&c_dfDIJoystick2);
        if (wantFfb) {
            hidden = CreateWindowExA(WS_EX_TOOLWINDOW, "STATIC", "EuroSpeed FFB", WS_POPUP, 0, 0, 1, 1, nullptr, nullptr,
                                     GetModuleHandleA(nullptr), nullptr);
            exclusive = hidden && SUCCEEDED(dev->SetCooperativeLevel(hidden, DISCL_BACKGROUND | DISCL_EXCLUSIVE));
        }
        if (!exclusive) dev->SetCooperativeLevel(hwnd, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);
        // Same range on every axis.
        const DWORD offsets[] = {DWORD(DIJOFS_X),  DWORD(DIJOFS_Y),  DWORD(DIJOFS_Z),         DWORD(DIJOFS_RX),
                                 DWORD(DIJOFS_RY), DWORD(DIJOFS_RZ), DWORD(DIJOFS_SLIDER(0)), DWORD(DIJOFS_SLIDER(1))};
        for (DWORD ofs : offsets) {
            DIPROPRANGE r{};
            r.diph.dwSize = sizeof(r);
            r.diph.dwHeaderSize = sizeof(DIPROPHEADER);
            r.diph.dwObj = ofs;
            r.diph.dwHow = DIPH_BYOFFSET;
            r.lMin = 0;
            r.lMax = 65535;
            dev->SetProperty(DIPROP_RANGE, &r.diph);
        }
        if (exclusive) {
            // Hand-centering off: BeamNG's force is the only force.
            DIPROPDWORD ac{};
            ac.diph.dwSize = sizeof(ac);
            ac.diph.dwHeaderSize = sizeof(DIPROPHEADER);
            ac.diph.dwHow = DIPH_DEVICE;
            ac.dwData = DIPROPAUTOCENTER_OFF;
            dev->SetProperty(DIPROP_AUTOCENTER, &ac.diph);
        }
        dev->Acquire();
        if (exclusive) {
            DWORD axes[1] = {DWORD(DIJOFS_X)};
            LONG dir[1] = {0};
            DICONSTANTFORCE cf{0};
            DIEFFECT e{};
            e.dwSize = sizeof(e);
            e.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
            e.dwDuration = INFINITE;
            e.dwGain = DI_FFNOMINALMAX;
            e.dwTriggerButton = DIEB_NOTRIGGER;
            e.cAxes = 1;
            e.rgdwAxes = axes;
            e.rglDirection = dir;
            e.cbTypeSpecificParams = sizeof(cf);
            e.lpvTypeSpecificParams = &cf;
            if (FAILED(dev->CreateEffect(GUID_ConstantForce, &e, &eff, nullptr))) eff = nullptr;
            if (eff) eff->Start(1, 0);
        }
        return true;
    }

    bool HasFfb() const { return eff != nullptr; }

    // force: -1..1 (+ pushes the wheel to the right).
    void SetForce(double force) {
        if (!eff) return;
        force = force > 1 ? 1 : force < -1 ? -1 : force;
        const long mag = long(force * DI_FFNOMINALMAX);
        if (mag == lastForce) return;
        DICONSTANTFORCE cf{mag};
        DIEFFECT e{};
        e.dwSize = sizeof(e);
        e.cbTypeSpecificParams = sizeof(cf);
        e.lpvTypeSpecificParams = &cf;
        HRESULT hr = eff->SetParameters(&e, DIEP_TYPESPECIFICPARAMS | DIEP_START);
        if (FAILED(hr)) {   // lost the device (another program took it): take it back, restart the effect
            dev->Acquire();
            hr = eff->SetParameters(&e, DIEP_TYPESPECIFICPARAMS | DIEP_START);
        }
        if (SUCCEEDED(hr)) lastForce = mag;
    }

    bool Read(WheelAxes& a, DWORD* buttons = nullptr) {
        if (!dev) return false;
        if (FAILED(dev->Poll()) && FAILED(dev->Acquire())) return false;
        DIJOYSTATE2 s{};
        if (FAILED(dev->GetDeviceState(sizeof(s), &s))) { dev->Acquire(); return false; }
        const long v[8] = {s.lX, s.lY, s.lZ, s.lRx, s.lRy, s.lRz, s.rglSlider[0], s.rglSlider[1]};
        memcpy(a.v, v, sizeof(v));
        if (buttons) {
            *buttons = 0;
            for (int i = 0; i < 32; ++i) if (s.rgbButtons[i] & 0x80) *buttons |= 1u << i;
        }
        return true;
    }

    void Close() {
        if (eff) { eff->Stop(); eff->Release(); eff = nullptr; }
        if (dev) { dev->Unacquire(); dev->Release(); dev = nullptr; }
        if (di) { di->Release(); di = nullptr; }
    }
};

// Wheel settings (wheel.ini next to the bridge). Axis names: x y z rx ry rz s0 s1, "-" = none.
// Pedals: invert = 1 when the raw value is high with the pedal released.
struct WheelConfig {
    bool enabled = true;
    std::string device = "";          // part of the device name, "" = first game controller
    int steer = 0, throttle = 1, brake = 5, clutch = 6;   // G29 defaults: x, y, rz, slider0
    bool steerInvert = false, throttleInvert = true, brakeInvert = true, clutchInvert = true;
    double steerDeadzone = 0.0;
    bool ffb = true;            // force feedback from BeamNG, applied by the bridge
    double ffbGain = 1.0;
    bool ffbInvert = false;
    // H shifter (Logitech Driving Force Shifter on a G29: buttons 12-17 = gears 1-6, 18 =
    // reverse, counted from 0). It drives BeamNG's gearbox; ETS2's gear is not used.
    static constexpr int kShifterGears = 7;
    bool shifter = true;
    int gearButton[kShifterGears] = {12, 13, 14, 15, 16, 17, -1};
    int reverseButton = 18;

    static int Axis(const char* s) {
        for (int i = 0; i < 8; ++i) if (!_stricmp(s, kAxisNames[i])) return i;
        return -1;
    }
    void Load(const std::string& path) {
        char v[128];
        auto str = [&](const char* key, const char* def) {
            GetPrivateProfileStringA("wheel", key, def, v, sizeof(v), path.c_str());
            return std::string(v);
        };
        enabled = str("enabled", "1") != "0";
        device = str("device", "");
        steer = Axis(str("steering", "x").c_str());
        throttle = Axis(str("throttle", "y").c_str());
        brake = Axis(str("brake", "rz").c_str());
        clutch = Axis(str("clutch", "s0").c_str());
        steerInvert = str("steering_invert", "0") == "1";
        throttleInvert = str("throttle_invert", "1") == "1";
        brakeInvert = str("brake_invert", "1") == "1";
        clutchInvert = str("clutch_invert", "1") == "1";
        steerDeadzone = atof(str("steering_deadzone", "0").c_str());
        ffb = str("ffb", "1") != "0";
        ffbGain = atof(str("ffb_gain", "1").c_str());
        ffbInvert = str("ffb_invert", "0") == "1";
        shifter = str("shifter", "1") != "0";
        const char* defs[kShifterGears] = {"12", "13", "14", "15", "16", "17", "-1"};
        for (int g = 0; g < kShifterGears; ++g) {
            char key[16];
            snprintf(key, sizeof(key), "gear%d", g + 1);
            gearButton[g] = atoi(str(key, defs[g]).c_str());
        }
        reverseButton = atoi(str("reverse", "18").c_str());
    }

    // H shifter: the gear whose button is held (-1 reverse, 0 neutral = none held), or kNoShifter.
    static constexpr int kNoShifter = -9;
    int ShifterGear(DWORD buttons) const {
        if (!shifter) return kNoShifter;
        auto held = [&](int b) { return b >= 0 && b < 32 && (buttons >> b) & 1u; };
        if (held(reverseButton)) return -1;
        for (int g = 0; g < kShifterGears; ++g) if (held(gearButton[g])) return g + 1;
        return 0;
    }
};

}  // namespace es
