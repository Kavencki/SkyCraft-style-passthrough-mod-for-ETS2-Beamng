// TruckPuppet: the ETS2 player truck's physics body follows the physics car, so ETS2 streams its
// map around the car (and the world reacts to "the player" where the car is). ETS2 1.61.1.1s only:
// every code location used is checked byte for byte first.
#pragma once
#include "../protocol/eurospeed_protocol.h"

namespace truckpuppet {

struct Settings {
    bool   enabled = true;
    double height = 1.0; // metres above the road (negative: under it, out of sight)
    double back = 0.0;   // metres behind the car, along the road the car drove
};

void Configure(const Settings& s);
// Call every frame_end. `want`: the bridge is driving. Returns a log message on state changes.
const char* Update(es::Shared* sh, bool want, const es::EtsTruckState& truck, const es::EtsTarget* target);
void Shutdown();
bool Active();   // driving the truck right now (its parking brake must be off)
uintptr_t TruckObject();   // the player truck object once found (0 before), for fuel.cpp

}  // namespace truckpuppet
