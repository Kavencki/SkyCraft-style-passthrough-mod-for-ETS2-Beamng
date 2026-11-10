// In-game RE probe: finds where eurotrucks2.exe keeps the truck's world
// position by searching memory for the placement telemetry reports, then
// verifies a candidate by nudging it and watching telemetry respond.
#pragma once
#include <cstdint>
#include "../protocol/eurobeamng_protocol.h"

namespace probe {

// ETS2 sector encoding: world = sector * kSectorSize + local (floats x,y,z then int16 sx, sz).
constexpr double kSectorSize = 512.0;

struct Candidate {
    uintptr_t addr;
    bool      isDouble;        // else float
    bool      isSector = false; // float x,y,z at addr, int16 sectorX at +12, sectorZ at +14
};

// Called from frame_end on the game thread. `truck` is this frame's telemetry.
void Tick(es::Shared* sh, const es::EtsTruckState& truck, const es::EtsTarget* target);
bool Busy();      // a background scan is running: don't unload
void Shutdown();  // disarm watchpoints, wait for scans; call before the DLL unloads

// Writes a position into a candidate. Returns false if the memory is gone.
bool WritePos(const Candidate& c, double x, double y, double z);
bool ReadPos(const Candidate& c, double& x, double& y, double& z);

}  // namespace probe
