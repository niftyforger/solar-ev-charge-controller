#include "grid_data_source.h"
#include "config.h"

// Reactive simulated grid source: models a fixed PV capacity and house load, then
// subtracts currentDrawW (the wattage currently commanded for charging) each poll, so
// reported export shrinks toward zero as target amps rises - letting the self-referential
// control loop actually converge in bench testing (a constant-value source never does,
// since it never accounts for the EV's own draw - see CLAUDE.md "Resolved").
//
// "Moderate sun" member of the reactive family (see also _no_sun/_low/_high.cpp) - values
// chosen so the equilibrium point lands well inside the 6-32A range (~13.3A) rather than
// at either extreme.
static const float SIMULATED_PV_CAPACITY_W = 4000.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;

static bool simulated_reactive_moderate_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                        float &outWatts, float &outVoltageV,
                                                        float &outBatteryW, bool &outBatteryDataValid) {
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W; // negative = exporting
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    outBatteryW = 0.0f; // no battery modeled
    outBatteryDataValid = true; // "no battery" is itself a known, trustworthy state
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_MODERATE = {
    "simulated_reactive_moderate",
    "Simulated - Reactive, Moderate Sun (4000W PV, 800W house load)",
    simulated_reactive_moderate_read_power_w,
};
