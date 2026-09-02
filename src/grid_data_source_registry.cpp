#include "grid_data_source.h"
#include <string.h>

// Index 0 is load-bearing: it's both the compile-time default and the
// fallback target grid_data_source_lookup() returns for an unrecognized id
// (e.g. persisted NVS state from a source removed in a later build). Keep
// GRID_SOURCE_SUNGROW_WINET first if this list is ever reordered/extended.
const GridDataSource *const GRID_SOURCE_REGISTRY[] = {
    &GRID_SOURCE_SUNGROW_WINET,
    &GRID_SOURCE_SIMULATED_EXPORT,
    &GRID_SOURCE_SIMULATED_IMPORT,
};
const size_t GRID_SOURCE_REGISTRY_COUNT =
    sizeof(GRID_SOURCE_REGISTRY) / sizeof(GRID_SOURCE_REGISTRY[0]);

const GridDataSource &grid_data_source_lookup(const char *id) {
    for (size_t i = 0; i < GRID_SOURCE_REGISTRY_COUNT; i++) {
        if (strcmp(GRID_SOURCE_REGISTRY[i]->id, id) == 0) {
            return *GRID_SOURCE_REGISTRY[i];
        }
    }
    return *GRID_SOURCE_REGISTRY[0];
}
