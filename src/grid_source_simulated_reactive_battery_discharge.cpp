#include "grid_data_source.h"
#include "config.h"

// Reactive simulated source (see _moderate.cpp for the general rationale) - models a home
// battery discharging to cover house load and then some, pushing outWatts into apparent
// export (looks like a healthy 27.5A solar surplus) while outBatteryW correctly reports it
// as battery-sourced - exercises the battery-discharge exclusion in solar_control_task
// (see grid_data_source.h's outBatteryW doc comment). Target amps should stay near the 6A
// floor despite outWatts.
static const float SIMULATED_PV_CAPACITY_W = 0.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;
static const float SIMULATED_BATTERY_DISCHARGE_W = 7400.0f; // battery covers load + pushes export

static bool simulated_reactive_battery_discharge_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                                 float &outWatts, float &outVoltageV,
                                                                 float &outBatteryW, bool &outBatteryDataValid) {
    // Battery discharge behaves like extra "PV" from the grid meter's point of view.
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W - SIMULATED_BATTERY_DISCHARGE_W;
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    outBatteryW = -SIMULATED_BATTERY_DISCHARGE_W; // negative = discharging
    outBatteryDataValid = true; // simulated reading is always fresh by construction
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_BATTERY_DISCHARGE = {
    "simulated_reactive_battery_discharge",
    "Simulated - Reactive, Battery Discharge (0W PV, 800W house load, 7400W battery discharge)",
    simulated_reactive_battery_discharge_read_power_w,
};
