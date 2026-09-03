#include "grid_data_source.h"
#include "config.h"

// Reactive simulated grid source (see grid_source_simulated_reactive_moderate.cpp
// for the general rationale) - the "high sun" member of the family. Net
// surplus (capacity - load) is 8200W (34.2A at the fixed 240V simulated
// mains voltage), deliberately more than MAX_CURRENT_A (32A) can use -
// confirms the control loop converges to and holds at the ceiling
// (DECISION_CAPPED_MAX) rather than exceeding it.
static const float SIMULATED_PV_CAPACITY_W = 9000.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;

static bool simulated_reactive_high_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                    float &outWatts, float &outVoltageV) {
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W; // negative = exporting
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_HIGH = {
    "simulated_reactive_high",
    "Simulated - Reactive, High Sun (9000W PV, 800W house load)",
    simulated_reactive_high_read_power_w,
};
