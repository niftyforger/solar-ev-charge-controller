#include <Arduino.h>
#include "config.h"
#include "shared_state.h"
#include "solar_control.h"
#include "ble_config.h"
#include "cp_interceptor.h"

void setup() {
    Serial.begin(115200);

    shared_state_init();

    // BLE config server: independent of WiFi state, must come up even on a
    // fully unprovisioned board - see ble_config.cpp.
    ble_config_init();

    // Core 1: real-time CP interception, pinned + highest priority.
    xTaskCreatePinnedToCore(cp_interceptor_task, "cp_interceptor", 8192,
                             nullptr, TASK_PRIO_CP, nullptr, CORE_CP);

    // Core 0: WiFi/Modbus control loop.
    xTaskCreatePinnedToCore(solar_control_task, "solar_control", 8192,
                             nullptr, TASK_PRIO_SOLAR, nullptr, CORE_SOLAR);
}

void loop() {
    vTaskDelete(NULL);
}
