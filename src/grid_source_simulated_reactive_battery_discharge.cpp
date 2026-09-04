#include "grid_data_source.h"
#include "config.h"

// Reactive simulated grid source (see grid_source_simulated_reactive_moderate.cpp
// for the general reactive-source rationale) - models a home battery
// discharging to cover house load and then some, pushing the surplus figure
// into apparent export with little PV behind it. Unlike the four PV-only
// reactive sources, this is what exercises the battery-discharge exclusion
// in solar_control_task (see grid_data_source.h's outBatteryW doc comment):
// outWatts alone looks exactly like a healthy 6600W solar surplus (27.5A),
// but outBatteryW correctly reports that surplus as coming from the battery,
// not PV, so the control loop should hold target amps near the 6A floor
// instead of climbing anywhere near what outWatts alone would suggest.
static const float SIMULATED_PV_CAPACITY_W = 0.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;
static const float SIMULATED_BATTERY_DISCHARGE_W = 7400.0f; // battery covers load + pushes export

static bool simulated_reactive_battery_discharge_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                                 float &outWatts, float &outVoltageV,
                                                                 float &outBatteryW) {
    // Same currentDrawW feedback as every other reactive source - the
    // battery's discharge behaves exactly like extra "PV" from the grid
    // meter's point of view, so it nets against currentDrawW the same way.
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W - SIMULATED_BATTERY_DISCHARGE_W;
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    outBatteryW = -SIMULATED_BATTERY_DISCHARGE_W; // negative = discharging
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_BATTERY_DISCHARGE = {
    "simulated_reactive_battery_discharge",
    "Simulated - Reactive, Battery Discharge (0W PV, 800W house load, 7400W battery discharge)",
    simulated_reactive_battery_discharge_read_power_w,
};
