#pragma once

#include <Arduino.h>

enum SystemMode {
    MODE_BYPASS = 0,  // clamp scheduling disabled in cp_sense_isr(); CLAMP_DRIVE never asserts
    MODE_ACTIVE = 1,  // CP task is following target current; clamp may assert per duty state
};

enum CpDutyState {
    CP_STANDBY = 0,     // clamp fully released, no PWM touched
    CP_OSCILLATING = 1, // clamp active per current surplus
};

enum ConnectorState {
    CONN_STATE_A = 0,
    CONN_STATE_B = 1,
    CONN_STATE_C = 2,
    CONN_STATE_FAULT = 3,
};

struct SolarStatus {
    float grid_power_w;          // negative = exporting
    uint32_t last_poll_success_ms;
    bool wifi_connected;
    bool modbus_ok;
    bool simulated;               // true when fed from the sim-mode control page, not real Modbus
};

struct CpStatus {
    SystemMode mode;
    CpDutyState duty_state;
    ConnectorState connector_state;
    float native_duty_pct;
    float applied_duty_pct;
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
SolarStatus shared_state_get_solar_status_blocking();

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
