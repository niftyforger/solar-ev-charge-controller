#pragma once

#include <Arduino.h>

enum SystemMode {
    MODE_BYPASS = 0,  // clamp scheduling disabled in cp_sense_isr(); CLAMP_DRIVE never asserts
    MODE_ACTIVE = 1,  // CP task is following target current; clamp may assert per duty state
};

enum CpDutyState {
    // Under MODE_ACTIVE: surplus is below the 6A floor (with hysteresis) - the disconnect
    // relay opens, isolating the vehicle's CP termination so the EVSE reads "unplugged".
    // Under MODE_BYPASS: just bookkeeping - the relay stays closed regardless, since a
    // fault must never trigger a disconnect (see cp_interceptor_task()).
    CP_STANDBY = 0,
    CP_OSCILLATING = 1, // relay closed, clamp active per current surplus
};

enum ConnectorState {
    CONN_STATE_A = 0,
    CONN_STATE_B = 1,
    CONN_STATE_C = 2,
    CONN_STATE_FAULT = 3,
};

struct SolarStatus {
    float grid_power_w;          // negative = exporting
    float battery_power_w;       // positive = charging, negative = discharging, 0 = idle/no battery
    // False whenever battery_power_w can't be trusted for the discharge-exclusion
    // calculation - never yet read, or the source's own reading has gone stale/been
    // rejected - so a real discharge that isn't being measured can't silently look like a
    // confirmed-idle battery. See grid_data_source.h's outBatteryDataValid doc comment.
    bool battery_data_valid;
    uint32_t last_poll_success_ms;
    bool wifi_connected;
    bool modbus_ok;
    // Independent liveness path for scheduled (fixed-current) charging - schedule_active
    // is whether the most recent loop tick found an enabled schedule whose UTC window
    // covers "now"; last_schedule_confirm_ms only advances while that's true, so it ages
    // past STALE_DATA_TIMEOUT_MS on its own once the window ends. Lets MODE_ACTIVE stay
    // reachable from a fresh schedule confirmation even while the grid-data poll is stale -
    // a fixed-current schedule must not depend on the solar pipeline.
    bool schedule_active;
    uint32_t last_schedule_confirm_ms;
};

struct CpStatus {
    SystemMode mode;
    CpDutyState duty_state;
    ConnectorState connector_state;
    float native_duty_pct;
    float applied_duty_pct;
};

struct RuntimeConfig {
    char ssid[33];
    char password[64];
    char inverter_ip[16];
    bool provisioned;
    uint32_t generation;
    // BLE-config gate and HTTP control-page passwords - independent of OTA_PASSWORD and of
    // each other, empty string meaning "never set" (see CLAUDE.md "BLE configuration").
    char ble_password[64];
    char web_password[64];
};

// Must be called once from setup() before either core's task starts.
void shared_state_init();

// --- Core 0 -> Core 1: latest commanded target current -------------------
// Queue of length 1 used with xQueueOverwrite: always holds only the most
// recent value, never a backlog of stale commands.
extern QueueHandle_t g_target_amps_queue;
void shared_state_set_target_amps(float amps);

// --- Core 0 -> Core 1 / Core 0-display: inverter poll health -------------
// Core 0 publishes with a blocking mutex take (it is not real-time).
// Core 1 reads with a non-blocking take so a contended mutex can never
// stall the CP task.
void shared_state_publish_solar_status(const SolarStatus &s);
bool shared_state_try_get_solar_status(SolarStatus &out); // non-blocking

// --- Core 1 -> Core 0-display: CP telemetry -------------------------------
// Core 1 publishes with a non-blocking mutex take so a display refresh that
// happens to be mid-I2C-transaction can never delay the CP task.
bool shared_state_try_publish_cp_status(const CpStatus &s); // non-blocking
CpStatus shared_state_get_cp_status_blocking();

// --- Core 0 -> Core 1: solar_control_task liveness heartbeat -------------
// A plain volatile word (millis() timestamp), not mutex-protected - a 32-bit aligned
// read/write is already atomic here, same pattern as cp_interceptor.cpp's mode/duty-state
// volatiles. Core 0 sets it once per loop; Core 1 polls it (SOLAR_TASK_HEARTBEAT_TIMEOUT_MS)
// to detect a wedged Core 0 and recover via esp_restart().
void shared_state_heartbeat_solar_task();
uint32_t shared_state_solar_task_heartbeat_age_ms();

// --- BLE (ble_config.cpp) -> Core 0: WiFi + inverter runtime config ------
// Provisioned entirely over BLE, no compile-time fallback (see CLAUDE.md "BLE
// configuration"). Setters copy into fixed-size buffers (bounded, null-terminated).
//
// `generation` increments only on shared_state_set_wifi_config() - solar_control_task
// reconnects WiFi / re-parses the inverter IP only when it changes. The password setters
// deliberately don't touch it, since a password change is not a reason to bounce WiFi.
void shared_state_init_runtime_config(const RuntimeConfig &initial);
void shared_state_set_wifi_config(const char *ssid, const char *password,
                                   const char *inverter_ip);
void shared_state_set_ble_password(const char *password);
void shared_state_set_web_password(const char *password);
RuntimeConfig shared_state_get_runtime_config();
