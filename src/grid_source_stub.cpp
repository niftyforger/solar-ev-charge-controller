#include "grid_data_source.h"

// Minimal second GridDataSource implementation whose only purpose is to
// prove the registry/selector mechanism works with more than one entry -
// not a real second meter/inverter integration. Ignores the host argument
// entirely and always reports a fixed synthetic export figure. Useful on
// the bench to exercise the control loop without a real solar link.
static bool stub_read_power_w(IPAddress /*host*/, float &outWatts) {
    outWatts = -500.0f; // fixed, negative = exporting (see grid_data_source.h)
    return true;
}

const GridDataSource GRID_SOURCE_STUB = {
    "stub",
    "Stub (test source, fixed value)",
    stub_read_power_w,
};
