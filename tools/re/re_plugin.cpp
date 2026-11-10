// eurospeed_re.dll - a separate, temporary ETS2 SDK plugin for finding where ETS2 keeps the
// player truck's physics position (TruckPuppet research). It only hosts tools/re/probe.cpp,
// driven by es_ctl.exe through the shared memory command channel. Install it for research
// sessions only (install_re.bat), remove it afterwards (uninstall_re.bat).
// Inert in multiplayer, like the main plugin.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdio>
#include <cstring>

#include "scssdk_telemetry.h"
#include "eurotrucks2/scssdk_eut2.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"
#include "common/scssdk_telemetry_common_channels.h"
#include "common/scssdk_telemetry_truck_common_channels.h"

#include "../../protocol/eurospeed_protocol.h"
#include "probe.h"

using namespace es;

namespace {
scs_log_t g_log = nullptr;
HANDLE g_map = nullptr;
Shared* g_sh = nullptr;
EtsTruckState g_state{};
int32_t g_mpOffset = 0;

SCSAPI_VOID OnPlacement(const scs_string_t, const scs_u32_t, const scs_value_t* const v, const scs_context_t) {
    if (!v || v->type != SCS_VALUE_TYPE_dplacement) { g_state.hasPlacement = 0; return; }
    const auto& p = v->value_dplacement;
    g_state.placement.pos = {p.position.x, p.position.y, p.position.z};
    g_state.placement.heading = p.orientation.heading;
    g_state.placement.pitch = p.orientation.pitch;
    g_state.placement.roll = p.orientation.roll;
    g_state.hasPlacement = 1;
}
SCSAPI_VOID OnMpOffset(const scs_string_t, const scs_u32_t, const scs_value_t* const v, const scs_context_t) {
    g_mpOffset = (v && v->type == SCS_VALUE_TYPE_s32) ? v->value_s32.value : 0;
}
SCSAPI_VOID OnFrameEnd(const scs_event_t, const void* const, const scs_context_t) {
    if (!g_sh) return;
    if (g_mpOffset != 0 || GetModuleHandleA("core_ets2mp.dll")) return;   // never in multiplayer
    EtsTarget t{};
    const bool haveTarget = g_sh->target.read(t) && t.valid;
    probe::Tick(g_sh, g_state, haveTarget ? &t : nullptr);
}
}  // namespace

SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version, const scs_telemetry_init_params_t* const params) {
    if (version != SCS_TELEMETRY_VERSION_1_00 && version != SCS_TELEMETRY_VERSION_1_01) return SCS_RESULT_unsupported;
    const auto* p = static_cast<const scs_telemetry_init_params_v100_t*>(params);
    g_log = p->common.log;
    if (strcmp(p->common.game_id, SCS_GAME_ID_EUT2) != 0) return SCS_RESULT_unsupported;
    g_sh = OpenShared(&g_map);
    if (!g_sh) return SCS_RESULT_generic_error;
    p->register_for_event(SCS_TELEMETRY_EVENT_frame_end, OnFrameEnd, nullptr);
    p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_U32_NIL, SCS_VALUE_TYPE_dplacement,
                            SCS_TELEMETRY_CHANNEL_FLAG_none, OnPlacement, nullptr);
    p->register_for_channel(SCS_TELEMETRY_CHANNEL_multiplayer_time_offset, SCS_U32_NIL, SCS_VALUE_TYPE_s32,
                            SCS_TELEMETRY_CHANNEL_FLAG_none, OnMpOffset, nullptr);
    if (g_log) g_log(SCS_LOG_TYPE_message, "[EuroSpeed RE] research probe loaded (use es_ctl.exe)");
    return SCS_RESULT_ok;
}

SCSAPI_VOID scs_telemetry_shutdown(void) {
    probe::Shutdown();
    CloseShared(g_sh, g_map);
    g_sh = nullptr;
    g_map = nullptr;
}

BOOL APIENTRY DllMain(HMODULE, DWORD, LPVOID) { return TRUE; }
