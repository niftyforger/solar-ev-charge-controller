#pragma once

// BLE config server: a Nordic UART Service (NUS) RX/TX characteristic pair
// carrying a small text command protocol (AUTH/SSID/PASS/IP/COMMIT/STATUS/
// HELP), so any BLE serial-terminal app (Serial Bluetooth Terminal, nRF
// Connect's UART plugin, etc.) auto-detects it and just works as a
// terminal - no per-characteristic GATT browsing needed. Provisions WiFi
// SSID/password and the inverter IP, and reports back the device's current
// WiFi IP address via STATUS. Runs on Core 0, independent of WiFi/
// solar_control_task state - see CLAUDE.md "BLE configuration". Must be
// called once from setup(), before solar_control_task starts, since that
// task reads its WiFi/inverter config out of what this loads from NVS.
void ble_config_init();
