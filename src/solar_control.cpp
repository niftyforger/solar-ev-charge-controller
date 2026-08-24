#include "solar_control.h"
#include "config.h"
#include "secrets.h"
#include "shared_state.h"
#include "modbus_tcp_client.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

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
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// --- Sim mode: runtime real/simulated toggle + control page ---------------
// Only ever touched from this task's own loop (the WebServer handler runs
// synchronously inside handleClient(), called from here), so no mutex is
// needed for these two.
static bool s_sim_mode_active = false;
static float s_simulated_grid_power_w = SIM_DEFAULT_GRID_POWER_W;

static WebServer s_web_server(SIM_HTTP_PORT);

static bool require_auth() {
    if (!s_web_server.authenticate("admin", OTA_PASSWORD)) {
        s_web_server.requestAuthentication();
        return false;
    }
    return true;
}

static void handle_root() {
    if (!require_auth()) {
        return;
    }
    char body[768];
    snprintf(body, sizeof(body),
        "<!DOCTYPE html><html><head><title>geely-charger-controller</title></head><body>"
        "<h1>Solar control</h1>"
        "<p>Mode: <b>%s</b></p>"
        "<p>Simulated grid power: %.0f W (negative = exporting)</p>"
        "<form action=\"/set\" method=\"get\">"
        "<label><input type=\"radio\" name=\"mode\" value=\"real\" %s>Real</label><br>"
        "<label><input type=\"radio\" name=\"mode\" value=\"sim\" %s>Simulated</label><br>"
        "Simulated watts: <input type=\"number\" name=\"w\" value=\"%.0f\" step=\"50\"><br>"
        "<input type=\"submit\" value=\"Apply\">"
        "</form></body></html>",
        s_sim_mode_active ? "Simulated" : "Real",
        s_simulated_grid_power_w,
        s_sim_mode_active ? "" : "checked",
        s_sim_mode_active ? "checked" : "",
        s_simulated_grid_power_w);
    s_web_server.send(200, "text/html", body);
}

static void handle_set() {
    if (!require_auth()) {
        return;
    }
    if (s_web_server.hasArg("mode")) {
        s_sim_mode_active = (s_web_server.arg("mode") == "sim");
    }
    if (s_web_server.hasArg("w")) {
        s_simulated_grid_power_w = s_web_server.arg("w").toFloat();
    }
    s_web_server.sendHeader("Location", "/");
    s_web_server.send(302, "text/plain", "");
}

static void start_network_services() {
    ArduinoOTA.setHostname(WIFI_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() { Serial.println("OTA: update starting"); });
    ArduinoOTA.onEnd([]() { Serial.println("OTA: update complete"); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA: error %u\n", error); });
    ArduinoOTA.begin();

    s_web_server.on("/", HTTP_GET, handle_root);
    s_web_server.on("/set", HTTP_GET, handle_set);
    s_web_server.begin();
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
    bool servicesStarted = false;

    for (;;) {
        uint32_t now = millis();

        if (WiFi.status() != WL_CONNECTED) {
            if (now - lastWifiAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
                lastWifiAttemptMs = now;
                connect_wifi();
            }
        } else if (!servicesStarted) {
            start_network_services();
            servicesStarted = true;
        }

        if (servicesStarted) {
            ArduinoOTA.handle();
            s_web_server.handleClient();
        }

        if (now - lastPollMs >= POLL_INTERVAL_MS) {
            lastPollMs = now;

            SolarStatus status;
            status.wifi_connected = (WiFi.status() == WL_CONNECTED);
            status.modbus_ok = false;
            status.grid_power_w = 0.0f;
            status.simulated = s_sim_mode_active;
            status.last_poll_success_ms = lastPollSuccessMs;

            if (s_sim_mode_active) {
                status.modbus_ok = true;
                status.grid_power_w = s_simulated_grid_power_w;
                lastPollSuccessMs = millis();
                status.last_poll_success_ms = lastPollSuccessMs;

                targetAmps = compute_next_target_amps(targetAmps, s_simulated_grid_power_w, now, lastChangeMs);
                shared_state_set_target_amps(targetAmps);
            } else if (status.wifi_connected) {
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
