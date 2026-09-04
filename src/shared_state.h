#pragma once

#include <Arduino.h>

enum SystemMode {
    MODE_BYPASS = 0,  // clamp scheduling disabled in cp_sense_isr(); CLAMP_DRIVE never asserts
    MODE_ACTIVE = 1,  // CP task is following target current; clamp may assert per duty state
};

enum CpDutyState {
    // While MODE_ACTIVE: surplus is genuinely below the 6A floor (with
    // hysteresis) - the CP disconnect relay is driven open, isolating the
    // vehicle's CP termination so the EVSE reads "unplugged" and stops
    // charging on its own. While MODE_BYPASS: just internal bookkeeping: the
    // relay stays closed regardless (see cp_interceptor_task()) - a fault
    // must never trigger a disconnect.
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
    uint32_t last_poll_success_ms;
    bool wifi_connected;
    bool modbus_ok;
    // Independent liveness path for scheduled (fixed-current) charging - see
    // CLAUDE.md "Solar data source" and cp_interceptor_task()'s staleness
    // check. schedule_active is whether solar_control_task's most recent
    // loop tick found an enabled schedule whose UTC window covers "now";
    // last_schedule_confirm_ms only advances while that's true (carried
    // forward otherwise), so it ages past STALE_DATA_TIMEOUT_MS on its own
    // once the window ends or NTP/WiFi/Core 0 stops confirming it - no
    // separate timeout constant or transition-handling code needed. This
    // lets MODE_ACTIVE stay reachable purely from a fresh schedule
    // confirmation even while last_poll_success_ms is stale (e.g. the
    // inverter/Modbus link is down), which is the whole point: a fixed-
    // current schedule must not depend on the solar grid-data pipeline.
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
    // BLE-config gate and HTTP control-page passwords - each independent of
    // OTA_PASSWORD and of each other, empty string meaning "never set" (same
    // convention as ssid/provisioned above). See CLAUDE.md "BLE
    // configuration".
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
// A plain volatile word (millis() timestamp), not mutex-protected: a 32-bit
// aligned read/write is already atomic on this platform, and this is a
// liveness signal, not data that needs a consistent multi-field snapshot -
// the same pattern cp_interceptor.cpp already uses for its Core0/Core1
// mode/duty-state volatiles. Core 0 calls the setter once per loop
// iteration; Core 1 polls the getter on its own cadence (see
// SOLAR_TASK_HEARTBEAT_TIMEOUT_MS in config.h) to detect a wedged Core 0
// (WiFi/lwIP/WebServer hang) and recover via esp_restart() - see
// cp_interceptor_task().
void shared_state_heartbeat_solar_task();
uint32_t shared_state_solar_task_heartbeat_age_ms();

// --- BLE (ble_config.cpp) -> Core 0: WiFi + inverter runtime config ------
// WiFi SSID/password, the inverter IP, and the BLE/web-page passwords are
// all provisioned entirely over BLE and have no compile-time fallback - see
// CLAUDE.md "BLE configuration". All setters copy into fixed-size buffers
// (bounded, always null-terminated) so callers don't need to match
// RuntimeConfig's exact field sizes.
//
// `generation` increments only on shared_state_set_wifi_config() -
// solar_control_task compares it once per loop iteration and only
// reconnects WiFi / re-parses the inverter IP when it changes. The two
// password setters deliberately do NOT touch `generation`: a BLE- or
// web-page-password change is not a reason for Core 0 to bounce WiFi.
void shared_state_init_runtime_config(const RuntimeConfig &initial);
void shared_state_set_wifi_config(const char *ssid, const char *password,
                                   const char *inverter_ip);
void shared_state_set_ble_password(const char *password);
void shared_state_set_web_password(const char *password);
RuntimeConfig shared_state_get_runtime_config();
