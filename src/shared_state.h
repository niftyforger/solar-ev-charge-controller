#pragma once

#include <Arduino.h>

enum SystemMode {
    MODE_BYPASS = 0,  // K1 de-energized, clamp leg physically disconnected
    MODE_ACTIVE = 1,  // K1 energized, CP task is following target current
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
