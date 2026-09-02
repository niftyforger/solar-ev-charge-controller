#pragma once

#include <Arduino.h>
#include <IPAddress.h>

// Abstraction over "where does the grid export/import figure come from",
// so the control loop doesn't need to know the specific meter/inverter
// wiring behind it - see CLAUDE.md "Solar data source".
//
// Reads current grid power in watts, negative = exporting (same convention
// used everywhere else in this codebase). Returns true on success.
typedef bool (*GridDataSourceReadFn)(IPAddress host, float &outWatts);

struct GridDataSource {
    const char *name;
    GridDataSourceReadFn read_power_w;
};

// See grid_source_sungrow_winet.cpp for the sourcing rationale (why the
// WiNet-S, not the DTSU666-20 meter, is polled directly).
extern const GridDataSource GRID_SOURCE_SUNGROW_WINET;

// Compile-time selection - point this at a different GridDataSource if the
// installed meter/inverter ever changes.
#define ACTIVE_GRID_DATA_SOURCE GRID_SOURCE_SUNGROW_WINET
