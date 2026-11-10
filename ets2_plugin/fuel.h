// FuelLink: the ETS2 truck's fuel follows the physics car's, and a refuel at an ETS2 station
// fills the car. The SDK only reports the fuel, so the truck's fuel value is found in the truck
// object (found by TruckPuppet) once per session: a float equal to the telemetry fuel that the
// telemetry follows when it is changed.
#pragma once
#include "../protocol/eurobeamng_protocol.h"

namespace fuel {

struct Settings {
    bool enabled = true;
    bool percent = false;   // false: the truck misses as many litres as the car (refuel cost as for
                            // the car); true: same percentage (ETS2's gauge shows the car's level)
};

void Configure(const Settings& s);
void OnFuel(float litres);          // telemetry truck.fuel.amount
void OnCapacity(float litres);      // truck config fuel.capacity
// Every frame_end while not paused. `drive`: the bridge drives (car fuel may be applied).
// Returns a log message on events.
const char* Update(es::Shared* sh, bool drive, uintptr_t truck);

}  // namespace fuel
