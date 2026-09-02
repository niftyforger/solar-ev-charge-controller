#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <stddef.h>

// Abstraction over "where does the grid export/import figure come from",
// so the control loop doesn't need to know the specific meter/inverter
// wiring behind it - see CLAUDE.md "Solar data source".
//
// Reads current grid power in watts, negative = exporting (same convention
// used everywhere else in this codebase). Returns true on success.
typedef bool (*GridDataSourceReadFn)(IPAddress host, float &outWatts);

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

// Fixed-value simulated sources - not real meter integrations. One reports
// a fixed export figure, the other a fixed import figure, so both the
// surplus and deficit control-loop paths can be exercised without real
// solar. Each lives in its own file, one source per file like
// grid_source_sungrow_winet.cpp, so either can double as a minimal
// reference when adding a real second source later. See
// grid_source_simulated_export.cpp / grid_source_simulated_import.cpp.
extern const GridDataSource GRID_SOURCE_SIMULATED_EXPORT;
extern const GridDataSource GRID_SOURCE_SIMULATED_IMPORT;

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
