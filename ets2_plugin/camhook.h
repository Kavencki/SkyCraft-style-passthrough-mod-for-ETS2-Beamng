// CameraDriver: makes ETS2 render from the camera pose the bridge publishes
// (EtsCamTarget, computed from the BeamNG/LFS car).
//
// ETS2 1.61 builds each frame's view from the active camera object (rbx) in a
// function at eurotrucks2.exe+0x473c60. A hardware execute breakpoint (DR2) on
// its first read of the object, +0x473d55 `movsx eax, word [rbx+0x4e]`, lets us
// write the target pose into the object just before the game reads it:
//   +0x40 float x, y, z (local to sector)   +0x4c int16 sectorX, sectorZ (512 m)
//   +0x50 float quaternion w, x, y, z
// No code is patched. The hook arms only if the instruction bytes match.
#pragma once
#include "../protocol/eurospeed_protocol.h"

namespace camhook {

// Call every frame. `want` = drive the camera this frame (target valid, not multiplayer).
// Returns a short status string for logging when the state changes, else nullptr.
const char* Update(es::Shared* sh, bool want);

uint32_t Applied();   // frames the pose was written

// FOV handling: 0 = leave ETS2's FOV, 1 = BeamNG's vertical FOV as is, 2 = converted to horizontal.
void Configure(int fovMode);
void Shutdown();      // disarm on all threads (call before unload)

}  // namespace camhook
