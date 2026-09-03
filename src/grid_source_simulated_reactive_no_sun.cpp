#include "grid_data_source.h"
#include "config.h"

// Reactive simulated grid source (see grid_source_simulated_reactive_moderate.cpp
// for the general rationale) - the "no sun" member of the family. Zero PV
// capacity means it always reports a deficit (at least the house load), so
// it should never clear the 6A floor and never enter charging regardless of
// how long it's selected - a pure night-time/no-solar scenario.
static const float SIMULATED_PV_CAPACITY_W = 0.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;

static bool simulated_reactive_no_sun_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                      float &outWatts, float &outVoltageV) {
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W; // negative = exporting
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_NO_SUN = {
    "simulated_reactive_no_sun",
    "Simulated - Reactive, No Sun (0W PV, 800W house load)",
    simulated_reactive_no_sun_read_power_w,
};
