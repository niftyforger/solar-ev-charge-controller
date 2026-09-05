#include "grid_data_source.h"
#include "config.h"

// Reactive simulated source (see _moderate.cpp for the general rationale) - "no sun"
// member: zero PV means always a deficit, so it should never clear the 6A floor.
static const float SIMULATED_PV_CAPACITY_W = 0.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;

static bool simulated_reactive_no_sun_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                      float &outWatts, float &outVoltageV,
                                                      float &outBatteryW, bool &outBatteryDataValid) {
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W; // negative = exporting
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    outBatteryW = 0.0f; // no battery modeled
    outBatteryDataValid = true; // "no battery" is itself a known, trustworthy state
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_NO_SUN = {
    "simulated_reactive_no_sun",
    "Simulated - Reactive, No Sun (0W PV, 800W house load)",
    simulated_reactive_no_sun_read_power_w,
};
