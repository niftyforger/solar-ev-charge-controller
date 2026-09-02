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

// The Sungrow SH8.0RS's WiNet-S Modbus TCP gateway is the true source of
// this figure. The DTSU666-20 meter behind it is wired via RS485 directly
// to the inverter and isn't polled directly (RTU only supports one bus
// master, and the inverter already is one) - see grid_source_sungrow_winet.cpp
// and CLAUDE.md "Solar data source".
extern const GridDataSource GRID_SOURCE_SUNGROW_WINET;

// Compile-time selection - point this at a different GridDataSource if the
// installed meter/inverter ever changes.
#define ACTIVE_GRID_DATA_SOURCE GRID_SOURCE_SUNGROW_WINET
