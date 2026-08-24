#include "shared_state.h"

QueueHandle_t g_target_amps_queue = nullptr;

static SemaphoreHandle_t s_status_mutex = nullptr;
static SolarStatus s_solar_status = {0.0f, 0, false, false};
static CpStatus s_cp_status = {MODE_BYPASS, CP_STANDBY, CONN_STATE_FAULT, 0.0f, 0.0f};
static volatile uint32_t s_solar_task_heartbeat_ms = 0;

void shared_state_init() {
    g_target_amps_queue = xQueueCreate(1, sizeof(float));
    s_status_mutex = xSemaphoreCreateMutex();
    // Baseline to "now", not 0: solar_control_task hasn't run its first loop
    // iteration yet at this point (called from setup(), before
    // xTaskCreatePinnedToCore()), so a 0 baseline would read as an already
    // ancient heartbeat and could trip the Core 1 watchdog check before
    // Core 0 ever gets a chance to run.
    s_solar_task_heartbeat_ms = millis();
}

void shared_state_set_target_amps(float amps) {
    xQueueOverwrite(g_target_amps_queue, &amps);
}

void shared_state_publish_solar_status(const SolarStatus &s) {
    if (xSemaphoreTake(s_status_mutex, portMAX_DELAY) == pdTRUE) {
        s_solar_status = s;
        xSemaphoreGive(s_status_mutex);
    }
}

bool shared_state_try_get_solar_status(SolarStatus &out) {
    if (xSemaphoreTake(s_status_mutex, 0) == pdTRUE) {
        out = s_solar_status;
        xSemaphoreGive(s_status_mutex);
        return true;
    }
    return false;
}

SolarStatus shared_state_get_solar_status_blocking() {
    SolarStatus out;
    if (xSemaphoreTake(s_status_mutex, portMAX_DELAY) == pdTRUE) {
        out = s_solar_status;
        xSemaphoreGive(s_status_mutex);
    }
    return out;
}

bool shared_state_try_publish_cp_status(const CpStatus &s) {
    if (xSemaphoreTake(s_status_mutex, 0) == pdTRUE) {
        s_cp_status = s;
        xSemaphoreGive(s_status_mutex);
        return true;
    }
    return false;
}

CpStatus shared_state_get_cp_status_blocking() {
    CpStatus out;
    if (xSemaphoreTake(s_status_mutex, portMAX_DELAY) == pdTRUE) {
        out = s_cp_status;
        xSemaphoreGive(s_status_mutex);
    }
    return out;
}

void shared_state_heartbeat_solar_task() {
    s_solar_task_heartbeat_ms = millis();
}

uint32_t shared_state_solar_task_heartbeat_age_ms() {
    return millis() - s_solar_task_heartbeat_ms;
}
