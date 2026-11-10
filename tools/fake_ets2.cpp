// fake_ets2.exe - stand-in for eurotrucks2.exe's SCS SDK plugin host.
// Loads eurobeamng.dll through the real SDK ABI (scs_telemetry_init v1.01),
// feeds it a parked truck, and pumps frame events at 60 Hz.
//
//   fake_ets2 <path\to\eurobeamng.dll> [--seconds N] [--convoy] [--truckersmp] [--follow]
//     --convoy      report multiplayer.time.offset != 0 (plugin must go inert)
//     --follow      move the fake truck to the plugin's target each frame
//                   (what Phase 1 TruckPuppet will do in the real game)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

#include "scssdk_telemetry.h"
#include "eurotrucks2/scssdk_eut2.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"
#include "../protocol/eurobeamng_protocol.h"

struct ChannelReg { scs_telemetry_channel_callback_t cb; scs_context_t ctx; scs_value_type_t type; };
static std::map<scs_event_t, std::pair<scs_telemetry_event_callback_t, scs_context_t>> g_events;
static std::map<std::string, ChannelReg> g_channels;
static int g_logLines = 0;

static SCSAPI_VOID HostLog(const scs_log_type_t type, const scs_string_t msg) {
    ++g_logLines;
    printf("[ets2 log%s] %s\n", type == SCS_LOG_TYPE_error ? " ERROR" : type == SCS_LOG_TYPE_warning ? " warn" : "", msg);
}
static SCSAPI_RESULT RegEvent(const scs_event_t e, const scs_telemetry_event_callback_t cb, const scs_context_t ctx) {
    if (!cb) return SCS_RESULT_invalid_parameter;
    if (g_events.count(e)) return SCS_RESULT_already_registered;
    g_events[e] = {cb, ctx};
    return SCS_RESULT_ok;
}
static SCSAPI_RESULT UnregEvent(const scs_event_t e) { return g_events.erase(e) ? SCS_RESULT_ok : SCS_RESULT_not_found; }
static SCSAPI_RESULT RegChannel(const scs_string_t name, const scs_u32_t index, const scs_value_type_t type,
                                const scs_u32_t, const scs_telemetry_channel_callback_t cb, const scs_context_t ctx) {
    if (index != SCS_U32_NIL) return SCS_RESULT_not_found;
    std::string n = name;
    if (n == "truck.world.placement" && type != SCS_VALUE_TYPE_dplacement && type != SCS_VALUE_TYPE_fplacement)
        return SCS_RESULT_unsupported_type;
    if (n == "multiplayer.time.offset" && type != SCS_VALUE_TYPE_s32) return SCS_RESULT_unsupported_type;
    if (n != "truck.world.placement" && n != "multiplayer.time.offset") return SCS_RESULT_not_found;
    g_channels[n] = {cb, ctx, type};
    return SCS_RESULT_ok;
}
static SCSAPI_RESULT UnregChannel(const scs_string_t name, const scs_u32_t, const scs_value_type_t) {
    return g_channels.erase(name) ? SCS_RESULT_ok : SCS_RESULT_not_found;
}
static void FireEvent(scs_event_t e, const void* info) {
    auto it = g_events.find(e);
    if (it != g_events.end()) it->second.first(e, info, it->second.second);
}
static void FireChannel(const char* name, const scs_value_t* v) {
    auto it = g_channels.find(name);
    if (it != g_channels.end()) it->second.cb(name, SCS_U32_NIL, v, it->second.ctx);
}

int main(int argc, char** argv) {
    if (argc < 2) { printf("usage: fake_ets2 <eurobeamng.dll> [--seconds N] [--convoy] [--follow]\n"); return 2; }
    int seconds = 5;
    bool convoy = false, follow = false;
    for (int i = 2; i < argc; ++i) {
        if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--convoy")) convoy = true;
        else if (!strcmp(argv[i], "--follow")) follow = true;
    }

    HMODULE dll = LoadLibraryA(argv[1]);
    if (!dll) { printf("[fake_ets2] LoadLibrary failed: %lu\n", GetLastError()); return 1; }
    auto init = reinterpret_cast<scs_result_t(SCSAPIFUNC*)(scs_u32_t, const scs_telemetry_init_params_t*)>(
        GetProcAddress(dll, "scs_telemetry_init"));
    auto shutdown = reinterpret_cast<void(SCSAPIFUNC*)()>(GetProcAddress(dll, "scs_telemetry_shutdown"));
    if (!init || !shutdown) { printf("[fake_ets2] missing SDK exports\n"); return 1; }

    scs_telemetry_init_params_v101_t params{};
    params.common.game_name = "Euro Truck Simulator 2 (fake)";
    params.common.game_id = SCS_GAME_ID_EUT2;
    params.common.game_version = SCS_TELEMETRY_EUT2_GAME_VERSION_CURRENT;
    params.common.log = HostLog;
    params.register_for_event = RegEvent;
    params.unregister_from_event = UnregEvent;
    params.register_for_channel = RegChannel;
    params.unregister_from_channel = UnregChannel;

    // The engine tries newest API first, like the real game.
    scs_result_t r = init(SCS_TELEMETRY_VERSION_1_01, &params);
    printf("[fake_ets2] scs_telemetry_init(1.01) = %d, events %zu, channels %zu\n", r, g_events.size(), g_channels.size());
    if (r != SCS_RESULT_ok) return 1;

    // A truck parked somewhere in ETS2, facing west.
    scs_value_t place{};
    place.type = SCS_VALUE_TYPE_dplacement;
    place.value_dplacement.position = {-31140.25, 45.5, -6420.75};
    place.value_dplacement.orientation = {0.25f, 0.0f, 0.0f};
    scs_value_t mp{};
    mp.type = SCS_VALUE_TYPE_s32;
    mp.value_s32.value = convoy ? 37 : 0;

    FireEvent(SCS_TELEMETRY_EVENT_started, nullptr);
    FireChannel("multiplayer.time.offset", &mp);

    HANDLE map = nullptr;
    es::Shared* sh = es::OpenShared(&map);
    uint32_t followed = 0;
    const int frames = seconds * 60;
    for (int f = 0; f < frames; ++f) {
        if (follow && sh) {
            es::EtsTarget t{};
            if (sh->target.read(t) && t.valid) {
                place.value_dplacement.position = {t.placement.pos.x, t.placement.pos.y, t.placement.pos.z};
                place.value_dplacement.orientation = {(float)t.placement.heading, (float)t.placement.pitch,
                                                      (float)t.placement.roll};
                ++followed;
            }
        }
        scs_telemetry_frame_start_t fs{};
        fs.render_time = f * 16667ull;
        fs.simulation_time = fs.render_time;
        fs.paused_simulation_time = fs.render_time;
        FireEvent(SCS_TELEMETRY_EVENT_frame_start, &fs);
        FireChannel("truck.world.placement", &place);
        FireEvent(SCS_TELEMETRY_EVENT_frame_end, nullptr);
        Sleep(16);
    }
    auto& p = place.value_dplacement.position;
    printf("[fake_ets2] done: %d frames, followed %u, final truck (%.2f, %.2f, %.2f) hdg %.4f, %d log lines\n",
           frames, followed, p.x, p.y, p.z, place.value_dplacement.orientation.heading, g_logLines);
    FireEvent(SCS_TELEMETRY_EVENT_paused, nullptr);
    shutdown();
    es::CloseShared(sh, map);
    FreeLibrary(dll);
    return 0;
}
