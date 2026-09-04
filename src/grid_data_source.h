#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <stddef.h>

// Abstraction over "where does the grid export/import figure come from",
// so the control loop doesn't need to know the specific meter/inverter
// wiring behind it - see CLAUDE.md "Solar data source".
//
// Reads current grid power in watts, negative = exporting (same convention
// used everywhere else in this codebase), and the AC grid/mains voltage in
// volts. currentDrawW is the wattage the control loop is currently
// commanding for charging (targetAmps * mainsVoltageV) - a real meter's
// power reading already nets this out on its own, so real sources ignore
// it; it exists for simulated sources that need to model that feedback
// themselves (see grid_source_simulated_reactive_moderate.cpp and siblings).
// outVoltageV lets each source report its own grid voltage rather than
// relying on one manually-configured global value - GRID_SOURCE_SUNGROW_WINET
// reads the real value from the inverter, every simulated source reports a
// fixed MAINS_VOLTAGE_FIXED_V (config.h).
//
// outBatteryW reports the home battery's own power flow, independent of
// currentDrawW/outWatts above: positive = charging (energy entering the
// battery), negative = discharging (energy leaving it), 0 = idle or no
// battery modeled - the same "negative = leaving the point of reference"
// convention as the grid reading. This exists so solar_control_task can
// exclude battery-discharge power from the surplus it hands to the EV -
// grid power alone can't distinguish "the meter is exporting because of
// fresh PV" from "the meter is exporting because the home battery is
// discharging", and diverting the latter into EV charging would mean
// charging the car from the house battery instead of solar (see CLAUDE.md
// "Solar data source"). A source with no battery (every simulated source
// except GRID_SOURCE_SIMULATED_REACTIVE_BATTERY_DISCHARGE) just reports
// 0.0f, making the exclusion a no-op. Returns true on success.
typedef bool (*GridDataSourceReadFn)(IPAddress host, float currentDrawW, float &outWatts,
                                       float &outVoltageV, float &outBatteryW);

struct GridDataSource {
    // Short, stable machine key (e.g. "sungrow_winet"). This is what gets
    // persisted to NVS as the selected source, so it must stay stable across
    // firmware revisions and unique across the registry - unlike name below,
    // it's never shown to the user.
    const char *id;
    const char *name;
    GridDataSourceReadFn read_power_w;
};

// See grid_source_sungrow_winet.cpp for the sourcing rationale (why the
// WiNet-S, not the DTSU666-20 meter, is polled directly).
extern const GridDataSource GRID_SOURCE_SUNGROW_WINET;

// Reactive simulated sources - not real meter integrations, but each models
// a fixed PV capacity and house load, then subtracts currentDrawW each poll,
// so reported export genuinely shrinks as target amps rises (unlike a
// constant-value fixed source, which never accounts for the EV's own draw
// and so never converges - see CLAUDE.md "Resolved", 2026-09-03). One file
// per source, spanning no sun through more sun than MAX_CURRENT_A can use:
// grid_source_simulated_reactive_no_sun.cpp (0W PV - always a deficit,
// never charges), _low.cpp (just under the 6A floor's wattage requirement),
// _moderate.cpp (comfortably charging, mid-range), _high.cpp (exceeds
// MAX_CURRENT_A - exercises the ceiling clamp).
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_NO_SUN;
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_LOW;
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_MODERATE;
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_HIGH;

// Models a home battery discharging into apparent grid export with little or
// no PV behind it, so the reading looks like solar surplus (outWatts
// negative/exporting) while outBatteryW correctly reports the discharge -
// the only way to bench-verify the battery-discharge exclusion in
// solar_control_task without real hardware (see grid_data_source.h's
// outBatteryW doc comment above and CLAUDE.md "Solar data source"). Target
// amps should stay pinned near the 6A floor with this source selected,
// despite outWatts showing a large export.
extern const GridDataSource GRID_SOURCE_SIMULATED_REACTIVE_BATTERY_DISCHARGE;

// All sources compiled into this build, selectable at runtime from the HTTP
// control page (see solar_control.cpp) instead of a compile-time #define.
// Index 0 is the default/fallback source - see grid_data_source_registry.cpp.
extern const GridDataSource *const GRID_SOURCE_REGISTRY[];
extern const size_t GRID_SOURCE_REGISTRY_COUNT;

// Looks up a source by GridDataSource::id. Falls back to
// GRID_SOURCE_REGISTRY[0] if id doesn't match anything currently registered
// (e.g. a persisted id from a source removed in a later build) - always
// returns a valid reference, never needs a null check at the call site.
const GridDataSource &grid_data_source_lookup(const char *id);
