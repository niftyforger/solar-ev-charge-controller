#pragma once

// Core 0 task: WiFi + Modbus TCP polling of the SH8.0RS, and the
// surplus-power -> target-current control loop. Started pinned to core 0
// from setup().
void solar_control_task(void *pvParameters);
