#include "solar_control.h"
#include "config.h"
#include "secrets.h"
#include "shared_state.h"
#include "modbus_tcp_client.h"
#include <WiFi.h>

// Step-limited, settling-gated integration from surplus power to target
// current. The EV's own draw is part of what the meter measures, so we
// deliberately do not chase every single reading: each accepted change
// starts a settle window, and readings during that window are ignored by
// the control law (still displayed/logged, just not acted on) so the loop
// doesn't fight its own effect on the next poll.
static float compute_next_target_amps(float currentTargetA, float gridPowerW,
                                       uint32_t nowMs, uint32_t &lastChangeMs) {
    if (nowMs - lastChangeMs < SETTLE_MS) {
        return currentTargetA;
    }

    float surplusW = -gridPowerW; // positive = exporting
    float deltaA = surplusW / MAINS_VOLTAGE_V;
    deltaA = constrain(deltaA, -STEP_MAX_A_PER_POLL, STEP_MAX_A_PER_POLL);

    float newTargetA = constrain(currentTargetA + deltaA, 0.0f, MAX_CURRENT_A);
    if (newTargetA != currentTargetA) {
        lastChangeMs = nowMs;
    }
    return newTargetA;
}

static void connect_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void solar_control_task(void *pvParameters) {
    (void)pvParameters;

    connect_wifi();

    IPAddress inverterIp;
    inverterIp.fromString(INVERTER_IP_ADDR);

    float targetAmps = 0.0f;
    uint32_t lastChangeMs = millis();
    uint32_t lastPollMs = 0;
    uint32_t lastWifiAttemptMs = 0;
    uint32_t lastPollSuccessMs = 0;

    for (;;) {
        uint32_t now = millis();

        if (WiFi.status() != WL_CONNECTED) {
            if (now - lastWifiAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
                lastWifiAttemptMs = now;
                connect_wifi();
            }
        }

        if (now - lastPollMs >= POLL_INTERVAL_MS) {
            lastPollMs = now;

            SolarStatus status;
            status.wifi_connected = (WiFi.status() == WL_CONNECTED);
            status.modbus_ok = false;
            status.grid_power_w = 0.0f;
            status.last_poll_success_ms = lastPollSuccessMs;

            if (status.wifi_connected) {
                float gridPowerW;
                if (modbus_read_grid_power_w(inverterIp, gridPowerW)) {
                    status.modbus_ok = true;
                    status.grid_power_w = gridPowerW;
                    lastPollSuccessMs = millis();
                    status.last_poll_success_ms = lastPollSuccessMs;

                    targetAmps = compute_next_target_amps(targetAmps, gridPowerW, now, lastChangeMs);
                    shared_state_set_target_amps(targetAmps);
                }
            }

            shared_state_publish_solar_status(status);
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
