#include "grid_data_source.h"
#include "config.h"

// Reactive simulated grid source - not a real meter/inverter integration,
// but unlike a fixed-value source, this one models the EV's own draw
// feeding back into the reading, the way a real meter always does. A fixed
// source reports the same wattage forever regardless of how much current is
// already committed, so the control loop just keeps adding more until it
// hits MAX_CURRENT_A - it never converges, because there's no real physical
// constraint behind the number (see CLAUDE.md "Resolved", 2026-09-03). This
// source instead assumes a fixed PV capacity and house load, then subtracts
// currentDrawW (the wattage solar_control.cpp is currently commanding for
// charging) each poll, so reported export genuinely shrinks toward zero as
// target amps rises - letting the self-referential control loop (see
// CLAUDE.md "Control loop is self-referential") actually be bench-tested
// end-to-end without real hardware.
//
// This is the "moderate sun" member of the reactive family (see also
// grid_source_simulated_reactive_no_sun.cpp / _low.cpp / _high.cpp) - values
// chosen so the equilibrium point, (capacity - load) / mains voltage, lands
// well inside the 6-32A range at the fixed 240V simulated mains voltage
// (~13.3A) rather than at either extreme.
static const float SIMULATED_PV_CAPACITY_W = 4000.0f;
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;

static bool simulated_reactive_moderate_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                        float &outWatts, float &outVoltageV) {
    outWatts = currentDrawW + SIMULATED_HOUSE_LOAD_W - SIMULATED_PV_CAPACITY_W; // negative = exporting
    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_MODERATE = {
    "simulated_reactive_moderate",
    "Simulated - Reactive, Moderate Sun (4000W PV, 800W house load)",
    simulated_reactive_moderate_read_power_w,
};
