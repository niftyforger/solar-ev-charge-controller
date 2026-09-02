#include "grid_data_source.h"

// Fixed-value simulated grid source - not a real meter/inverter
// integration. Reports a constant export figure so the control loop's
// surplus path can be exercised on the bench without real solar. Also
// serves as a minimal reference for adding a new GridDataSource - see
// grid_data_source.h.

static bool simulated_export_read_power_w(IPAddress /*host*/, float &outWatts) {
    outWatts = -500.0f; // negative = exporting
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_EXPORT = {
    "simulated_export",
    "Simulated - Export (-500 W)",
    simulated_export_read_power_w,
};
