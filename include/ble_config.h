#pragma once

// BLE config server: a Nordic UART Service (NUS) RX/TX pair carrying a small
// text command protocol (AUTH/SET/GET/COMMIT/HELP) that any BLE
// serial-terminal app auto-detects and works with directly. Provisions WiFi
// SSID/password and the inverter IP, and reports the device's current WiFi
// IP via STATUS - see CLAUDE.md "BLE configuration" for the full protocol.
// Runs on Core 0, independent of WiFi/solar_control_task state. Must be
// called once from setup(), before solar_control_task starts, since that
// task reads its WiFi/inverter config out of what this loads from NVS.
void ble_config_init();
