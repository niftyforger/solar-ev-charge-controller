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
// fixed MAINS_VOLTAGE_FIXED_V (config.h). Returns true on success.
typedef bool (*GridDataSourceReadFn)(IPAddress host, float currentDrawW, float &outWatts, float &outVoltageV);

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
