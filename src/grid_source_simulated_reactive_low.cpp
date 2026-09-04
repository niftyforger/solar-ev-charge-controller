#include "grid_data_source.h"
#include "config.h"

// Reactive simulated source (see _moderate.cpp for the general rationale) - "low sun"
// member: net surplus (1400W) is ~40W under the 1440W the 6A floor needs at 240V,
// deliberately just below the entry threshold to confirm it never enters charging.
static const float SIMULATED_PV_CAPACITY_W = 2200.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;

static bool simulated_reactive_low_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                   float &outWatts, float &outVoltageV,
                                                   float &outBatteryW) {
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W; // negative = exporting
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    outBatteryW = 0.0f; // no battery modeled
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_LOW = {
    "simulated_reactive_low",
    "Simulated - Reactive, Low Sun (2200W PV, 800W house load)",
    simulated_reactive_low_read_power_w,
};
