#include "grid_data_source.h"

// Fixed-value simulated grid source - not a real meter/inverter
// integration. Reports a constant import figure so the control loop's
// deficit path can be exercised on the bench without real solar. Also
// serves as a minimal reference for adding a new GridDataSource - see
// grid_data_source.h.

static bool simulated_import_read_power_w(IPAddress /*host*/, float &outWatts) {
    outWatts = 500.0f; // positive = importing
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_IMPORT = {
    "simulated_import",
    "Simulated - Import (+500 W)",
    simulated_import_read_power_w,
};
