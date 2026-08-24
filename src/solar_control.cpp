#include "solar_control.h"
#include "config.h"
#include "secrets.h"
#include "shared_state.h"
#include "modbus_tcp_client.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <math.h>

// What the control law just did, kept alongside the numeric target so the
// web UI can explain the decision in words instead of just showing a number.
enum ControlDecision {
    DECISION_SETTLING,     // still inside the settle window from the last change
    DECISION_STEP_UP,      // surplus supports more current, stepped up
    DECISION_STEP_DOWN,    // surplus dropped, stepped down
    DECISION_HOLD,         // already matched to surplus, no change needed
    DECISION_CAPPED_MAX,   // would go higher, but MAX_CURRENT_A is the ceiling
    DECISION_CAPPED_ZERO,  // no surplus at all, held at the floor
};

static const char *decision_reason_str(ControlDecision d) {
    switch (d) {
        case DECISION_SETTLING:    return "Holding - settling after last change";
        case DECISION_STEP_UP:     return "Increasing - surplus supports more current";
        case DECISION_STEP_DOWN:   return "Decreasing - surplus dropped";
        case DECISION_CAPPED_MAX:  return "Holding at max - surplus exceeds charger's limit";
        case DECISION_CAPPED_ZERO: return "Holding at 0A - no surplus available";
        case DECISION_HOLD:        return "Holding - matched to surplus";
        default:                   return "";
    }
}

// Step-limited, settling-gated integration from surplus power to target
// current. The EV's own draw is part of what the meter measures, so we
// deliberately do not chase every single reading: each accepted change
// starts a settle window, and readings during that window are ignored by
// the control law (still displayed/logged, just not acted on) so the loop
// doesn't fight its own effect on the next poll.
static ControlDecision compute_next_target_amps(float currentTargetA, float gridPowerW,
                                                  uint32_t nowMs, uint32_t &lastChangeMs,
                                                  float &outTargetA) {
    if (nowMs - lastChangeMs < SETTLE_MS) {
        outTargetA = currentTargetA;
        return DECISION_SETTLING;
    }

    float surplusW = -gridPowerW; // positive = exporting
    float deltaA = surplusW / MAINS_VOLTAGE_V;
    deltaA = constrain(deltaA, -STEP_MAX_A_PER_POLL, STEP_MAX_A_PER_POLL);

    float newTargetA = constrain(currentTargetA + deltaA, 0.0f, MAX_CURRENT_A);

    ControlDecision decision;
    if (newTargetA > currentTargetA) {
        decision = DECISION_STEP_UP;
    } else if (newTargetA < currentTargetA) {
        decision = DECISION_STEP_DOWN;
    } else if (newTargetA >= MAX_CURRENT_A) {
        decision = DECISION_CAPPED_MAX;
    } else if (newTargetA <= 0.0f) {
        decision = DECISION_CAPPED_ZERO;
    } else {
        decision = DECISION_HOLD;
    }

    if (newTargetA != currentTargetA) {
        lastChangeMs = nowMs;
    }
    outTargetA = newTargetA;
    return decision;
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

// Latest status, cached here purely for the web UI so handle_root() can
// render the same picture the control loop just acted on rather than
// re-deriving it. Written once per poll from solar_control_task's own loop,
// read once per page request from the same task via handleClient() - no
// cross-task access, so no mutex needed for these either.
static SolarStatus s_last_solar_status = {};
static float s_last_target_amps = 0.0f;
static ControlDecision s_last_decision = DECISION_HOLD;
static uint32_t s_last_lastChangeMs = 0;

static WebServer s_web_server(SIM_HTTP_PORT);

static bool require_auth() {
    if (!s_web_server.authenticate("admin", OTA_PASSWORD)) {
        s_web_server.requestAuthentication();
        return false;
    }
    return true;
}

static const char *connector_state_str(ConnectorState s) {
    switch (s) {
        case CONN_STATE_A: return "A (idle, no vehicle)";
        case CONN_STATE_B: return "B (vehicle connected, not charging)";
        case CONN_STATE_C: return "C (charging)";
        default:           return "fault / disconnected";
    }
}

// Reused as the scratch buffer for the rendered page: kept file-scope
// (rather than a stack local) so a deeply nested call chain (WiFi/OTA/
// WebServer internals all run inside this same task) never has to find a
// couple of KB of extra stack for it.
static char s_page_body[3072];

static void handle_root() {
    if (!require_auth()) {
        return;
    }

    CpStatus cp = shared_state_get_cp_status_blocking();

    uint32_t nowMs = millis();
    bool everPolled = (s_last_solar_status.last_poll_success_ms != 0);
    uint32_t pollAgeS = everPolled ? (nowMs - s_last_solar_status.last_poll_success_ms) / 1000 : 0;
    bool stale = !everPolled || (nowMs - s_last_solar_status.last_poll_success_ms >= STALE_DATA_TIMEOUT_MS);

    float surplusW = -s_last_solar_status.grid_power_w;
    uint32_t settleRemainingS = 0;
    if (s_last_decision == DECISION_SETTLING) {
        uint32_t elapsed = nowMs - s_last_lastChangeMs;
        settleRemainingS = (elapsed < SETTLE_MS) ? (SETTLE_MS - elapsed) / 1000 : 0;
    }

    char settleNote[64] = "";
    if (s_last_decision == DECISION_SETTLING) {
        snprintf(settleNote, sizeof(settleNote), " (%lus left)", (unsigned long)settleRemainingS);
    }

    char pollAgeStr[24];
    if (everPolled) {
        snprintf(pollAgeStr, sizeof(pollAgeStr), "%lus ago", (unsigned long)pollAgeS);
    } else {
        snprintf(pollAgeStr, sizeof(pollAgeStr), "never");
    }

    const char *cpModeStr = (cp.mode == MODE_BYPASS)
        ? "Bypass - clamp physically disconnected, charger running at its native rate"
        : "Active - following the solar target below";
    const char *cpDutyStr = (cp.mode != MODE_ACTIVE)
        ? "-"
        : (cp.duty_state == CP_OSCILLATING)
            ? "Oscillating - clamping every cycle to the target duty"
            : "Standby - surplus below the 6A floor, clamp released to native pass-through";

    snprintf(s_page_body, sizeof(s_page_body),
        "<!DOCTYPE html><html><head><title>geely-charger-controller</title>"
        "<style>body{font-family:sans-serif;max-width:640px;margin:2em auto;line-height:1.5}"
        "h2{margin-bottom:0.2em}.warn{color:#b00}.ok{color:#080}</style>"
        "</head><body>"
        "<h1>Solar control</h1>"

        "<h2>What it's doing right now</h2>"
        "<p><b>%s</b></p>"
        "<p>Grid: <b>%.0f W %s</b>%s &mdash; commanded target: <b>%.1f A</b></p>"

        "<h2>Charge point interceptor</h2>"
        "<p>%s</p>"
        "<p>%s</p>"
        "<p>Connector state: <b>%s</b></p>"

        "<h2>Link health</h2>"
        "<p>WiFi: <span class=\"%s\">%s</span></p>"
        "<p>Data source: <b>%s</b>%s</p>"
        "<p>Last successful poll: <b>%s</b>%s</p>"

        "<h2>Simulation mode</h2>"
        "<p>Mode: <b>%s</b></p>"
        "<p>Simulated grid power: %.0f W (negative = exporting)</p>"
        "<form action=\"/set\" method=\"get\">"
        "<label><input type=\"radio\" name=\"mode\" value=\"real\" %s>Real</label><br>"
        "<label><input type=\"radio\" name=\"mode\" value=\"sim\" %s>Simulated</label><br>"
        "Simulated watts: <input type=\"number\" name=\"w\" value=\"%.0f\" step=\"50\"><br>"
        "<input type=\"submit\" value=\"Apply\">"
        "</form>"

        "<script>"
        "var wattsFieldFocused=false;"
        "document.addEventListener('DOMContentLoaded',function(){"
        "var w=document.querySelector('input[name=w]');"
        "w.addEventListener('focus',function(){wattsFieldFocused=true;});"
        "w.addEventListener('blur',function(){wattsFieldFocused=false;});"
        "});"
        "setTimeout(function(){if(!wattsFieldFocused){location.reload();}},5000);"
        "</script>"

        "</body></html>",

        decision_reason_str(s_last_decision),
        fabsf(surplusW), surplusW >= 0 ? "exporting" : "importing", settleNote,
        s_last_target_amps,

        cpModeStr,
        cpDutyStr,
        connector_state_str(cp.connector_state),

        s_last_solar_status.wifi_connected ? "ok" : "warn",
        s_last_solar_status.wifi_connected ? "connected" : "disconnected, retrying",
        s_last_solar_status.simulated ? "Simulated" : "Real (Modbus)",
        (!s_last_solar_status.simulated && !s_last_solar_status.modbus_ok) ? " - last poll failed" : "",
        pollAgeStr,
        stale ? " <span class=\"warn\">- STALE, CP fail-safe will bypass to native pass-through</span>" : "",

        s_sim_mode_active ? "Simulated" : "Real",
        s_simulated_grid_power_w,
        s_sim_mode_active ? "" : "checked",
        s_sim_mode_active ? "checked" : "",
        s_simulated_grid_power_w);
    s_web_server.send(200, "text/html", s_page_body);
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

// Route/callback registration: allocates handler objects (WebServer::on()
// appends a new FunctionRequestHandler to an internal linked list every
// call, never replacing an existing one), so this must run exactly once for
// the process lifetime - repeating it on every WiFi reconnect would leak a
// handler each time. Safe to call before WiFi/services are up; it only
// registers callbacks, it doesn't bind any socket.
static void register_network_services() {
    ArduinoOTA.setHostname(WIFI_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() { Serial.println("OTA: update starting"); });
    ArduinoOTA.onEnd([]() { Serial.println("OTA: update complete"); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA: error %u\n", error); });

    s_web_server.on("/", HTTP_GET, handle_root);
    s_web_server.on("/set", HTTP_GET, handle_set);
}

// Rebinds ArduinoOTA's UDP listener and the WebServer's TCP listener to
// whatever WiFi connection is current. Unlike register_network_services()
// above, this IS meant to be called again after every WiFi drop/reconnect -
// see the call site. ArduinoOTA.begin() is a no-op if it thinks it's
// already initialized (guarded by an internal _initialized flag - checked
// directly in this version's ArduinoOTA.cpp), so without an explicit end()
// first, its UDP socket stays bound to the old (torn-down) network
// interface forever after a reconnect - the web UI/OTA becoming
// permanently unreachable after any WiFi hiccup, until a full power cycle,
// while Modbus (a fresh short-lived socket every poll) keeps working, was
// traced to exactly this. WebServer::begin() already self-closes its
// previous listener internally, so it doesn't strictly need the explicit
// close() below, but it's included for symmetry/clarity and because
// close() on a never-started server is a harmless no-op either way.
static void start_network_services() {
    ArduinoOTA.end();
    s_web_server.close();

    ArduinoOTA.begin();
    s_web_server.begin();
}

void solar_control_task(void *pvParameters) {
    (void)pvParameters;

    connect_wifi();
    register_network_services();

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

        // Liveness signal for Core 1's independent recovery watchdog - see
        // shared_state_heartbeat_solar_task() and the check in
        // cp_interceptor_task(). Placed at the top of the loop so a hang
        // anywhere below it (WiFi/OTA/WebServer/Modbus) is what actually
        // gets caught, rather than being masked by a heartbeat that kept
        // updating right up until the hang.
        shared_state_heartbeat_solar_task();

        if (WiFi.status() != WL_CONNECTED) {
            // Drop servicesStarted so the reconnect branch below re-runs
            // start_network_services() once WiFi comes back - see that
            // function's comment for why a stale ArduinoOTA/WebServer
            // binding left over from before the drop won't recover on its
            // own otherwise. Harmless to set repeatedly while already
            // disconnected.
            servicesStarted = false;
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

                s_last_decision = compute_next_target_amps(targetAmps, s_simulated_grid_power_w, now, lastChangeMs, targetAmps);
                shared_state_set_target_amps(targetAmps);
            } else if (status.wifi_connected) {
                float gridPowerW;
                if (modbus_read_grid_power_w(inverterIp, gridPowerW)) {
                    status.modbus_ok = true;
                    status.grid_power_w = gridPowerW;
                    lastPollSuccessMs = millis();
                    status.last_poll_success_ms = lastPollSuccessMs;

                    s_last_decision = compute_next_target_amps(targetAmps, gridPowerW, now, lastChangeMs, targetAmps);
                    shared_state_set_target_amps(targetAmps);
                }
            }

            shared_state_publish_solar_status(status);
            s_last_solar_status = status;
            s_last_target_amps = targetAmps;
            s_last_lastChangeMs = lastChangeMs;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
