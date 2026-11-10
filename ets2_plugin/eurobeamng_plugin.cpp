// eurobeamng.dll - the ETS2 half of EuroBeamNG, loaded by the official SCS SDK
// plugin loader from <ETS2>\bin\win_x64\plugins\.
//
// Phase 0 (Link): publish the truck's real world placement, read the target
// pose computed from LFS physics, and log both. It does not move the truck yet:
// the SDK is read-only, so TruckPuppet (Phase 1) needs reverse engineering
// against the installed game version.
//
// Inert in multiplayer: Convoy (multiplayer.time.offset != 0) or TruckersMP
// (core_ets2mp.dll / core_atsmp.dll in the process).
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "scssdk_telemetry.h"
#include "scssdk_input.h"
#include "eurotrucks2/scssdk_eut2.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"
#include "amtrucks/scssdk_ats.h"
#include "common/scssdk_telemetry_common_channels.h"
#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_truck_common_channels.h"

#include "../protocol/eurobeamng_protocol.h"
#include "camhook.h"
#include "truckpuppet.h"
#include "fuel.h"

using namespace es;

namespace {

scs_log_t g_log = nullptr;
HANDLE    g_map = nullptr;
Shared*   g_sh = nullptr;

EtsTruckState g_state{};
int32_t   g_mpOffset = 0;
bool      g_truckersMp = false;
bool      g_wasInert = false;
uint64_t  g_lastLog = 0;
bool      g_linkedLogged = false;

void Log(scs_log_type_t type, const char* fmt, ...) {
    char msg[512];
    int n = snprintf(msg, sizeof(msg), "[EuroBeamNG] ");
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg + n, sizeof(msg) - n, fmt, ap);
    va_end(ap);
    if (g_log) g_log(type, msg);
    OutputDebugStringA(msg);
    OutputDebugStringA("\n");
}


// eurobeamng.ini next to the plugin, e.g.
//   [camera]
//   fov=vertical      ; vertical (default) | horizontal | off
int ReadFovMode() {
    char path[MAX_PATH];
    HMODULE me = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCSTR>(&ReadFovMode), &me);
    GetModuleFileNameA(me, path, MAX_PATH);
    char* slash = strrchr(path, '\\');
    if (!slash) return 1;
    strcpy_s(slash + 1, MAX_PATH - (slash + 1 - path), "eurobeamng.ini");
    char v[32];
    GetPrivateProfileStringA("camera", "fov", "vertical", v, sizeof(v), path);
    if (!_stricmp(v, "off")) return 0;
    if (!_stricmp(v, "horizontal")) return 2;
    return 1;
}

//   [truck]
//   puppet=1          ; the truck follows the car (needed to drive far from where the truck was)
//   height=1          ; metres above the road (negative = under it, out of sight)
//   back=0            ; metres behind the car, along the road it drove
bool g_lightsWanted = true;   // eurobeamng.ini [truck] lights: the truck's low beams on while driving (HeadLights)

truckpuppet::Settings ReadPuppetSettings() {
    char path[MAX_PATH];
    HMODULE me = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCSTR>(&ReadPuppetSettings), &me);
    GetModuleFileNameA(me, path, MAX_PATH);
    truckpuppet::Settings s;
    char* slash = strrchr(path, '\\');
    if (!slash) return s;
    strcpy_s(slash + 1, MAX_PATH - (slash + 1 - path), "eurobeamng.ini");
    char v[32];
    GetPrivateProfileStringA("truck", "puppet", "1", v, sizeof(v), path);
    s.enabled = strcmp(v, "0") != 0;
    GetPrivateProfileStringA("truck", "height", "1", v, sizeof(v), path);
    s.height = atof(v);
    GetPrivateProfileStringA("truck", "back", "0", v, sizeof(v), path);
    s.back = atof(v);
    GetPrivateProfileStringA("truck", "lights", "1", v, sizeof(v), path);
    g_lightsWanted = strcmp(v, "0") != 0;
    fuel::Settings f;
    GetPrivateProfileStringA("fuel", "link", "1", v, sizeof(v), path);
    f.enabled = strcmp(v, "0") != 0;
    GetPrivateProfileStringA("fuel", "mode", "litres", v, sizeof(v), path);
    f.percent = _stricmp(v, "percent") == 0;
    fuel::Configure(f);
    return s;
}

bool DetectTruckersMp() {
    return GetModuleHandleA("core_ets2mp.dll") || GetModuleHandleA("core_atsmp.dll");
}

SCSAPI_VOID OnPlacement(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
                        const scs_context_t) {
    if (!value || value->type != SCS_VALUE_TYPE_dplacement) {
        g_state.hasPlacement = 0;
        return;
    }
    const scs_value_dplacement_t& p = value->value_dplacement;
    g_state.placement.pos = {p.position.x, p.position.y, p.position.z};
    g_state.placement.heading = p.orientation.heading;
    g_state.placement.pitch = p.orientation.pitch;
    g_state.placement.roll = p.orientation.roll;
    g_state.hasPlacement = 1;
}

SCSAPI_VOID OnMpOffset(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
                       const scs_context_t) {
    g_mpOffset = (value && value->type == SCS_VALUE_TYPE_s32) ? value->value_s32.value : 0;
}

EtsInput g_input{};

SCSAPI_VOID OnInputFloat(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
                         const scs_context_t context) {
    float* dst = static_cast<float*>(context);
    *dst = (value && value->type == SCS_VALUE_TYPE_float) ? value->value_float.value : 0.0f;
}
// ---- TruckHold: while the bridge drives, the ETS2 truck stays parked. The plugin is also a
// "semantical" input device that can press the parking brake toggle. Whenever the driver's
// parking brake key releases the truck's brake, that press is taken as a toggle of the
// physics car's parking brake, and the truck's brake is set again at once.
bool     g_truckPb = false;         // the ETS2 truck's parking brake (telemetry)
bool     g_carPb = false;           // the physics car's parking brake, while holding
bool     g_holding = false;
bool     g_inputActive = false;     // our input device is being polled by the game
int      g_pressPhase = 0;          // 0 idle, 1 send "down", 2 send "up"
int      g_pressInput = 0;          // which of our inputs is pressed (kInParkingBrake, kInLight)
uint64_t g_pressCooldownUntil = 0;  // wait for telemetry to show the result of our press
enum { kInParkingBrake = 0, kInLight = 1 };

// HeadLights: the truck (invisible, see the invisible-truck mod) lights the road for the car, so
// its low beams are switched on while the bridge drives ("light" cycles off/parking/low beam).
bool     g_lowBeam = false;         // telemetry
uint64_t g_lightCooldownUntil = 0;

bool     g_wantTruckPb = true;      // while holding: parked (on), or released for TruckPuppet (off)

SCSAPI_VOID OnParkingBrake(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    const bool on = value && value->type == SCS_VALUE_TYPE_bool && value->value_bool.value;
    // It left the state we keep it in, and not by our press: the driver pressed the key.
    if (g_holding && g_truckPb == g_wantTruckPb && on != g_wantTruckPb) g_carPb = !g_carPb;
    g_truckPb = on;
}

// hold: the bridge drives - the truck's parking brake is ours. release: TruckPuppet drives the
// truck, so its brake must be off; otherwise it is kept on (the truck stays parked).
void UpdateTruckHold(bool hold, bool release, uint64_t now) {
    if (hold && !g_holding) g_carPb = false;   // start with the car's brake off
    g_holding = hold;
    g_wantTruckPb = !release;
    if (hold && g_truckPb != g_wantTruckPb && g_pressPhase == 0 && now >= g_pressCooldownUntil && g_inputActive) {
        g_pressPhase = 1;
        g_pressInput = kInParkingBrake;
        g_pressCooldownUntil = now + 400;
    }
    if (hold && g_lightsWanted && !g_lowBeam && g_pressPhase == 0 && now >= g_lightCooldownUntil &&
        now >= g_pressCooldownUntil && g_inputActive) {
        g_pressPhase = 1;
        g_pressInput = kInLight;
        g_lightCooldownUntil = now + 600;
    }
    g_input.parkingBrake = hold ? (g_carPb ? 1 : 0) : (g_truckPb ? 1 : 0);
}

SCSAPI_VOID OnInputActive(const scs_u8_t active, const scs_context_t) { g_inputActive = active != 0; }

SCSAPI_RESULT OnInputEvent(scs_input_event_t* const ev, const scs_u32_t flags, const scs_context_t) {
    // At most one event per frame, so the game sees a separate press and release.
    if (!(flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame) || g_pressPhase == 0) return SCS_RESULT_not_found;
    ev->input_index = g_pressInput;
    ev->value_bool.value = g_pressPhase == 1 ? 1 : 0;
    g_pressPhase = g_pressPhase == 1 ? 2 : 0;
    return SCS_RESULT_ok;
}
SCSAPI_VOID OnFuelAmount(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    if (value && value->type == SCS_VALUE_TYPE_float) fuel::OnFuel(value->value_float.value);
}
SCSAPI_VOID OnLowBeam(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    g_lowBeam = value && value->type == SCS_VALUE_TYPE_bool && value->value_bool.value;
}
SCSAPI_VOID OnGameTime(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    if (value && value->type == SCS_VALUE_TYPE_u32) g_state.gameTimeMin = value->value_u32.value;
}
SCSAPI_VOID OnGear(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    g_input.gear = (value && value->type == SCS_VALUE_TYPE_s32) ? value->value_s32.value : 0;
}

// Logs the player truck's identity (brand_id, id) - the invisible-truck mod is made for it.
SCSAPI_VOID OnConfig(const scs_event_t, const void* const event_info, const scs_context_t) {
    const auto* cfg = static_cast<const scs_telemetry_configuration_t*>(event_info);
    if (!cfg || !cfg->id || strcmp(cfg->id, SCS_TELEMETRY_CONFIG_truck) != 0) return;
    const char* brand = "?";
    const char* id = "?";
    for (const scs_named_value_t* a = cfg->attributes; a && a->name; ++a) {
        if (!strcmp(a->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_fuel_capacity) && a->value.type == SCS_VALUE_TYPE_float)
            fuel::OnCapacity(a->value.value_float.value);
        if (a->value.type != SCS_VALUE_TYPE_string) continue;
        if (!strcmp(a->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_brand_id)) brand = a->value.value_string.value;
        if (!strcmp(a->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_id)) id = a->value.value_string.value;
    }
    Log(SCS_LOG_TYPE_message, "player truck: brand_id %s, id %s", brand, id);
}

SCSAPI_VOID OnPause(const scs_event_t event, const void* const, const scs_context_t) {
    g_state.paused = (event == SCS_TELEMETRY_EVENT_paused) ? 1 : 0;
}

SCSAPI_VOID OnFrameEnd(const scs_event_t, const void* const, const scs_context_t) {
    if (!g_sh) return;
    const uint64_t now = GetTickCount64();

    if (!g_truckersMp && g_state.frames % 120 == 0) g_truckersMp = DetectTruckersMp();
    const bool inert = g_truckersMp || g_mpOffset != 0;
    if (inert != g_wasInert) {
        g_wasInert = inert;
        Log(SCS_LOG_TYPE_warning, inert ? "multiplayer detected (%s): EuroBeamNG is inert"
                                        : "multiplayer ended: EuroBeamNG active",
            g_truckersMp ? "TruckersMP" : "Convoy");
    }

    EtsTarget t{};
    EtsCamTarget cam{};
    const uint64_t bridgeHb = g_sh->bridgeHeartbeatMs.load();
    const bool bridgeAlive = bridgeHb && now - bridgeHb < kHeartbeatTimeoutMs;
    const bool haveTarget = !inert && bridgeAlive && g_sh->target.read(t) && t.valid;
    const bool haveCam = !inert && bridgeAlive && g_sh->cam.read(cam) && cam.valid;
    UpdateTruckHold(haveCam && !g_state.paused, truckpuppet::Active(), now);

    g_state.multiplayer = inert ? 1 : 0;
    g_state.frames++;
    g_sh->pluginPid.store(GetCurrentProcessId());
    g_sh->pluginHeartbeatMs.store(now);
    g_sh->ets.write(g_state);
    g_input.seq++;
    g_sh->input.write(g_input);

    // CameraDriver: ETS2 renders from the physics car's camera while the bridge streams.
    if (const char* msg = camhook::Update(g_sh, haveCam)) Log(SCS_LOG_TYPE_message, "%s", msg);
    // TruckPuppet: the truck's physics body follows the car, so ETS2 loads the map around it.
    if (const char* msg = truckpuppet::Update(g_sh, haveTarget && haveCam, g_state, haveTarget ? &t : nullptr))
        Log(SCS_LOG_TYPE_message, "%s", msg);
    // FuelLink: the truck's fuel follows the car's; an ETS2 refuel fills the car.
    if (!inert && !g_state.paused)
        if (const char* msg = fuel::Update(g_sh, haveTarget && haveCam, truckpuppet::TruckObject()))
            Log(SCS_LOG_TYPE_message, "%s", msg);
    if (inert) return;

    if (haveTarget && !g_linkedLogged) {
        g_linkedLogged = true;
        Log(SCS_LOG_TYPE_message, "linked to lfs_bridge (pid %u)", g_sh->bridgePid.load());
    }
    if (!bridgeAlive) g_linkedLogged = false;

    if (now - g_lastLog >= 10000) {
        g_lastLog = now;
        if (!bridgeAlive) {
            Log(SCS_LOG_TYPE_message, "waiting for lfs_bridge");
        } else if (!haveTarget) {
            Log(SCS_LOG_TYPE_message, "bridge up, no target yet (car not streaming or not anchored)");
        } else {
            const auto& b = cam.placement;
            Log(SCS_LOG_TYPE_message, "car #%u camera (%.2f, %.2f, %.2f) hdg %.4f, %u frames driven",
                t.lfsPackets, b.pos.x, b.pos.y, b.pos.z, b.heading, camhook::Applied());
        }
    }
}

}  // namespace

SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version, const scs_telemetry_init_params_t* const params) {
    if (version != SCS_TELEMETRY_VERSION_1_00 && version != SCS_TELEMETRY_VERSION_1_01)
        return SCS_RESULT_unsupported;
    const auto* p = static_cast<const scs_telemetry_init_params_v100_t*>(params);
    g_log = p->common.log;

    const bool ets2 = strcmp(p->common.game_id, SCS_GAME_ID_EUT2) == 0;
    const bool ats = strcmp(p->common.game_id, SCS_GAME_ID_ATS) == 0;
    if (!ets2 && !ats) {
        Log(SCS_LOG_TYPE_warning, "unsupported game '%s'", p->common.game_id);
        return SCS_RESULT_unsupported;
    }
    g_state = {};
    g_state.paused = 1;  // SDK starts paused
    g_state.gameVersion = p->common.game_version;

    g_sh = OpenShared(&g_map);
    if (!g_sh) {
        Log(SCS_LOG_TYPE_error, "cannot open shared memory %ls (protocol v%u mismatch?)", kShmName, kVersion);
        return SCS_RESULT_generic_error;
    }

    bool ok =
        p->register_for_event(SCS_TELEMETRY_EVENT_frame_end, OnFrameEnd, nullptr) == SCS_RESULT_ok &&
        p->register_for_event(SCS_TELEMETRY_EVENT_paused, OnPause, nullptr) == SCS_RESULT_ok &&
        p->register_for_event(SCS_TELEMETRY_EVENT_started, OnPause, nullptr) == SCS_RESULT_ok &&
        p->register_for_event(SCS_TELEMETRY_EVENT_configuration, OnConfig, nullptr) == SCS_RESULT_ok &&
        p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_U32_NIL,
                                SCS_VALUE_TYPE_dplacement, SCS_TELEMETRY_CHANNEL_FLAG_none,
                                OnPlacement, nullptr) == SCS_RESULT_ok;
    if (!ok) {
        Log(SCS_LOG_TYPE_error, "SDK registration failed");
        CloseShared(g_sh, g_map); g_sh = nullptr; g_map = nullptr;
        return SCS_RESULT_generic_error;
    }
    // Driver inputs (optional individually; the car just won't get that control).
    g_input = {};
    const struct { const char* name; float* dst; } floats[] = {
        {SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, &g_input.steering},
        {SCS_TELEMETRY_TRUCK_CHANNEL_input_throttle, &g_input.throttle},
        {SCS_TELEMETRY_TRUCK_CHANNEL_input_brake, &g_input.brake},
        {SCS_TELEMETRY_TRUCK_CHANNEL_input_clutch, &g_input.clutch},
    };
    for (const auto& f : floats)
        p->register_for_channel(f.name, SCS_U32_NIL, SCS_VALUE_TYPE_float, SCS_TELEMETRY_CHANNEL_FLAG_none,
                                OnInputFloat, f.dst);
    p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, SCS_U32_NIL, SCS_VALUE_TYPE_bool,
                            SCS_TELEMETRY_CHANNEL_FLAG_none, OnParkingBrake, nullptr);
    p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, SCS_U32_NIL, SCS_VALUE_TYPE_s32,
                            SCS_TELEMETRY_CHANNEL_FLAG_none, OnGear, nullptr);
    p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_fuel, SCS_U32_NIL, SCS_VALUE_TYPE_float,
                            SCS_TELEMETRY_CHANNEL_FLAG_each_frame, OnFuelAmount, nullptr);
    p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_light_low_beam, SCS_U32_NIL, SCS_VALUE_TYPE_bool,
                            SCS_TELEMETRY_CHANNEL_FLAG_none, OnLowBeam, nullptr);
    // In-game clock, so BeamNG's time of day (the car's lighting) follows ETS2's.
    p->register_for_channel(SCS_TELEMETRY_CHANNEL_game_time, SCS_U32_NIL, SCS_VALUE_TYPE_u32,
                            SCS_TELEMETRY_CHANNEL_FLAG_none, OnGameTime, nullptr);
    // Optional: older games don't have this channel.
    p->register_for_channel(SCS_TELEMETRY_CHANNEL_multiplayer_time_offset, SCS_U32_NIL,
                            SCS_VALUE_TYPE_s32, SCS_TELEMETRY_CHANNEL_FLAG_none, OnMpOffset, nullptr);

    g_truckersMp = DetectTruckersMp();
    const int fovMode = ReadFovMode();
    camhook::Configure(fovMode);
    truckpuppet::Configure(ReadPuppetSettings());
    Log(SCS_LOG_TYPE_message, "camera FOV: %s", fovMode == 0 ? "ETS2's own" : fovMode == 2 ? "BeamNG, as horizontal" : "BeamNG, vertical");
    Log(SCS_LOG_TYPE_message, "loaded: %s %u.%u, protocol v%u, shm %ls%s", p->common.game_name,
        SCS_GET_MAJOR_VERSION(p->common.game_version), SCS_GET_MINOR_VERSION(p->common.game_version),
        kVersion, kShmName, g_truckersMp ? " (TruckersMP present: inert)" : "");
    return SCS_RESULT_ok;
}

SCSAPI_VOID scs_telemetry_shutdown(void) {
    Log(SCS_LOG_TYPE_message, "shutdown");
    if (g_sh) g_sh->pluginHeartbeatMs.store(0);
    truckpuppet::Shutdown();
    camhook::Shutdown();
    CloseShared(g_sh, g_map);
    g_sh = nullptr; g_map = nullptr; g_log = nullptr;
}

// Input half: a semantical device with one input, the parking brake toggle (see TruckHold).
SCSAPI_RESULT scs_input_init(const scs_u32_t version, const scs_input_init_params_t* const params) {
    if (version != SCS_INPUT_VERSION_1_00) return SCS_RESULT_unsupported;
    const auto* p = static_cast<const scs_input_init_params_v100_t*>(params);
    if (!g_log) g_log = p->common.log;
    static const scs_input_device_input_t inputs[] = {
        {"parkingbrake", "Parking brake", SCS_VALUE_TYPE_bool},   // kInParkingBrake
        {"light", "Light modes", SCS_VALUE_TYPE_bool},            // kInLight
    };
    scs_input_device_t dev{};
    dev.name = "eurobeamng";
    dev.display_name = "EuroBeamNG";
    dev.type = SCS_INPUT_DEVICE_TYPE_semantical;
    dev.input_count = 2;
    dev.inputs = inputs;
    dev.callback_context = nullptr;
    dev.input_active_callback = OnInputActive;
    dev.input_event_callback = OnInputEvent;
    if (p->register_device(&dev) != SCS_RESULT_ok) {
        Log(SCS_LOG_TYPE_warning, "input device registration failed: the truck is not held while driving");
        return SCS_RESULT_generic_error;
    }
    Log(SCS_LOG_TYPE_message, "input device on: the truck stays parked while the physics car drives");
    return SCS_RESULT_ok;
}

SCSAPI_VOID scs_input_shutdown(void) {}

BOOL APIENTRY DllMain(HMODULE, DWORD, LPVOID) { return TRUE; }
