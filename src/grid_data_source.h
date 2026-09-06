#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <stddef.h>

// Abstraction over "where does the grid export/import figure come from", so the control
// loop doesn't need to know the specific meter/inverter wiring behind it - see CLAUDE.md
// "Solar data source".
//
// Reads current grid power in watts (negative = exporting) and grid/mains voltage in
// volts. currentDrawW is the wattage currently commanded for charging - a real meter's
// reading already nets this out on its own, so real sources ignore it; simulated sources
// use it to model that feedback themselves. outVoltageV lets each source report its own
// grid voltage instead of relying on one manually-configured global value.
//
// outBatteryW reports the home battery's own flow, independent of outWatts: positive =
// charging, negative = discharging, 0 = idle/no battery modeled (same "negative = leaving
// the point of reference" convention as the grid reading). Lets solar_control_task exclude
// battery-discharge power from EV-charging surplus, since grid power alone can't tell
// fresh-PV export from battery-discharge export. A source with no battery just reports 0.0f.
//
// outBatteryDataValid reports whether outBatteryW is fresh enough to trust for that
// exclusion - false on a source that has never yet obtained a real battery reading, or
// whose reading has gone stale (a failed/rejected register read silently reusing a stale
// last-known value would otherwise look identical to a confirmed-idle battery, defeating
// the exclusion exactly when it matters). A source with no battery just reports true
// unconditionally - "no battery" is itself a known, trustworthy state.
//
// Returns true on success.
typedef bool (*GridDataSourceReadFn)(IPAddress host, float currentDrawW, float &outWatts,
                                       float &outVoltageV, float &outBatteryW,
                                       bool &outBatteryDataValid);

struct GridDataSource {
    // Short, stable machine key (e.g. "sungrow_winet") - persisted to NVS as the selected
    // source, so must stay stable across firmware revisions and unique in the registry.
    const char *id;
    const char *name;
    GridDataSourceReadFn read_power_w;
};

// See grid_source_sungrow_winet.cpp for the sourcing rationale.
extern const GridDataSource GRID_SOURCE_SUNGROW_WINET;

// Reactive simulated sources: each models a fixed PV capacity and house load, then
// subtracts currentDrawW each poll, so reported export shrinks as target amps rises (a
// constant-value source never converges - see CLAUDE.md "Resolved"). One file per source,
// spanning no sun through more than MAX_CURRENT_A can use: _no_sun (0W PV, never charges),
// _low (just under the 6A floor), _moderate (comfortably mid-range), _high (exceeds
// MAX_CURRENT_A, exercises the ceiling clamp).
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_NO_SUN;
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_LOW;
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_MODERATE;
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_HIGH;

// Models a home battery discharging into apparent grid export with little/no PV behind it
// (outWatts negative/exporting) while outBatteryW correctly reports the discharge - the
// only way to bench-verify the battery-discharge exclusion without real hardware. Target
// amps should stay pinned near the 6A floor with this source selected, despite outWatts
// showing a large export.
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_BATTERY_DISCHARGE;

// Models measurement lag (vehicle current ramp + meter/register transport delay) and
// cloud-driven surplus variation on top of the reactive model above. The rest of the
// family reflects currentDrawW back instantaneously and noise-free, which is the one case
// a deadbeat control law is unconditionally stable against - so this is the only source
// that can reproduce the loop's lag-induced oscillation, and the regression test for the
// damping that fixes it. See its .cpp and CLAUDE.md "Control-loop damping".
extern const GridDataSource GRID_SOURCE_SIMULATED_LAGGED_CLOUDY;

// All sources compiled into this build, selectable at runtime from the HTTP control page,
// not a compile-time #define. Index 0 is the default/fallback source.
extern const GridDataSource *const GRID_SOURCE_REGISTRY[];
extern const size_t GRID_SOURCE_REGISTRY_COUNT;

// Looks up a source by id, falling back to GRID_SOURCE_REGISTRY[0] for an unrecognized id
// (e.g. a persisted id from a source removed in a later build) - never needs a null check.
const GridDataSource &grid_data_source_lookup(const char *id);
