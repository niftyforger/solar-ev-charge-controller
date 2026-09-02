#include "solar_control.h"
#include "config.h"
#include "secrets.h"
#include "shared_state.h"
#include "grid_data_source.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>
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

// WiFi SSID/password have no compile-time fallback - they're provisioned
// entirely over BLE (see ble_config.cpp) and read here out of shared_state.
// If the device hasn't been provisioned yet (or the SSID is empty), this
// deliberately does not call WiFi.begin() at all - retried on the normal
// WIFI_RECONNECT_INTERVAL_MS cadence until the RuntimeConfig indicates a
// real SSID exists. Core 1's existing stale-data fail-safe (MODE_BYPASS)
// already makes "no working network" safe: native CP pass-through, never a
// guessed setpoint or a disconnect - see CLAUDE.md.
static void connect_wifi(const RuntimeConfig &cfg) {
    if (!cfg.provisioned || cfg.ssid[0] == '\0') {
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(cfg.ssid, cfg.password);
}

// --- Sim mode: runtime real/simulated toggle + control page ---------------
// Only ever touched from this task's own loop (the WebServer handler runs
// synchronously inside handleClient(), called from here), so no mutex is
// needed for these two.
static bool s_sim_mode_active = false;
static float s_simulated_grid_power_w = SIM_DEFAULT_GRID_POWER_W;

// --- Grid data source: runtime-selectable, separate from sim mode ---------
// Same ownership as the sim-mode statics above: only Core 0's solar_control_
// task ever reads or writes this (poll loop + its synchronous WebServer
// handlers), so no mutex is needed. Always a valid pointer - either this
// compile-time default or whatever grid_data_source_lookup() returns, which
// itself never returns null (falls back to index 0) - so no call site needs
// a null check.
static const GridDataSource *s_active_grid_source = &GRID_SOURCE_SUNGROW_WINET;

static const char *GRID_SOURCE_NVS_NAMESPACE = "gridsrc";

// Independent of ble_config.cpp's "netcfg" namespace - this isn't a
// BLE-provisioned setting, it's chosen from the HTTP control page.
static void load_grid_source_from_nvs() {
    Preferences prefs;
    prefs.begin(GRID_SOURCE_NVS_NAMESPACE, true);
    String id = prefs.getString("id", "");
    prefs.end();
    if (id.length() > 0) {
        s_active_grid_source = &grid_data_source_lookup(id.c_str());
    }
    // else: nothing saved yet (first boot / fresh NVS) - keep the
    // compile-time default, preserving today's real-hardware behavior.
}

static void save_grid_source_to_nvs(const char *id) {
    Preferences prefs;
    prefs.begin(GRID_SOURCE_NVS_NAMESPACE, false);
    prefs.putString("id", id);
    prefs.end();
}

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

// web_password is set entirely over BLE (SET WEB_PASS command, see
// ble_config.cpp) and has no compile-time fallback - OTA_PASSWORD is used
// only for OTA flashing (see register_network_services() below), never for
// this page. Until a web password has been committed at least once, the
// page refuses every request outright rather than falling back to an
// empty/guessable credential.
static bool require_auth() {
    RuntimeConfig cfg = shared_state_get_runtime_config();
    if (cfg.web_password[0] == '\0') {
        s_web_server.send(403, "text/plain",
            "No control-page password set yet - set one over BLE with SET WEB_PASS <password>.");
        return false;
    }
    if (!s_web_server.authenticate("admin", cfg.web_password)) {
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

// Status-badge color class for each connector state - shared between the
// JSON API (below) and nothing else server-side; the SPA's CSS defines what
// each class actually looks like.
static const char *connector_state_cls(ConnectorState s) {
    switch (s) {
        case CONN_STATE_A: return "muted";
        case CONN_STATE_B: return "info";
        case CONN_STATE_C: return "ok";
        default:           return "bad";
    }
}

// Short badge labels - connector_state_str()/the cp mode/duty sentences
// above are full explanations, too long to sit inside a pill-shaped badge
// without wrapping (confirmed visually), so the SPA shows these short forms
// in the badge itself and the full sentence as a hover tooltip instead.
static const char *connector_state_label(ConnectorState s) {
    switch (s) {
        case CONN_STATE_A: return "A - Idle";
        case CONN_STATE_B: return "B - Connected";
        case CONN_STATE_C: return "C - Charging";
        default:           return "Fault / Disconnected";
    }
}

// --- Static SPA shell -------------------------------------------------
// Contains no per-request data - every dynamic value is fetched by the
// page's own JS from /api/status after load, so this can be a plain
// compile-time PROGMEM constant instead of an snprintf'd buffer. Self-
// contained (no external CSS/JS/fonts): this device has no internet path
// worth relying on for a LAN control page.
static const char PAGE_HTML[] PROGMEM = R"PAGE(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>solar-ev-charger</title>
<style>
:root{
  --bg:#f3f4f6;--bg-elev:#ffffff;--border:#e3e5ea;--text:#1a1d23;--text-muted:#6b7280;
  --accent:#2563eb;--ok:#15803d;--ok-bg:#dcfce7;--warn:#b45309;--warn-bg:#fef3c7;
  --bad:#b91c1c;--bad-bg:#fee2e2;--info:#1d4ed8;--info-bg:#dbeafe;
  --muted:#4b5563;--muted-bg:#e5e7eb;--radius:14px;
  --shadow:0 1px 2px rgba(16,24,40,.06),0 1px 6px rgba(16,24,40,.06);
}
@media (prefers-color-scheme:dark){
  :root{
    --bg:#0e1116;--bg-elev:#171b22;--border:#2a2f3a;--text:#e6e9ef;--text-muted:#9aa3b2;
    --accent:#5b8def;--ok:#34d399;--ok-bg:rgba(52,211,153,.15);--warn:#fbbf24;--warn-bg:rgba(251,191,36,.15);
    --bad:#f87171;--bad-bg:rgba(248,113,113,.15);--info:#7ea6f7;--info-bg:rgba(126,166,247,.15);
    --muted:#9aa3b2;--muted-bg:rgba(154,163,178,.15);
    --shadow:0 1px 2px rgba(0,0,0,.4),0 1px 10px rgba(0,0,0,.4);
  }
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);
  font:15px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif}
.wrap{max-width:900px;margin:0 auto;padding:24px 20px 60px}
.topbar{display:flex;align-items:center;justify-content:space-between;margin-bottom:20px;flex-wrap:wrap;gap:10px}
.topbar h1{font-size:1.3rem;margin:0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px}
.card{background:var(--bg-elev);border:1px solid var(--border);border-radius:var(--radius);
  padding:20px;box-shadow:var(--shadow)}
.card h2{margin:0 0 14px;font-size:.72rem;font-weight:700;letter-spacing:.06em;
  text-transform:uppercase;color:var(--text-muted)}
.card-settings{border-style:dashed}
.stat-value{font-size:2.6rem;font-weight:700;line-height:1.1;letter-spacing:-.02em}
.stat-value-sub{font-size:1.1rem;font-weight:500;color:var(--text-muted)}
.stat-sub{color:var(--text-muted);margin-top:4px;font-size:.9rem}
.flow-row{display:flex;align-items:center;gap:8px;margin-top:14px;font-size:.95rem}
.flow-dot{width:10px;height:10px;border-radius:50%;flex:none}
.flow-export{background:var(--ok)}
.flow-import{background:var(--warn)}
.kv{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:8px 0;
  border-top:1px solid var(--border)}
.kv:first-of-type{border-top:none;padding-top:0}
.kv>span:first-child{color:var(--text-muted)}
.badge{display:inline-block;padding:3px 10px;border-radius:999px;font-size:.78rem;font-weight:600}
.badge-ok{background:var(--ok-bg);color:var(--ok)}
.badge-warn{background:var(--warn-bg);color:var(--warn)}
.badge-bad{background:var(--bad-bg);color:var(--bad)}
.badge-info{background:var(--info-bg);color:var(--info)}
.badge-muted{background:var(--muted-bg);color:var(--muted)}
.banner{border-radius:10px;padding:12px 16px;font-size:.9rem;margin-bottom:16px}
.banner-warn{background:var(--warn-bg);color:var(--warn)}
.banner-bad{background:var(--bad-bg);color:var(--bad)}
select,input[type=number],button{
  font:inherit;color:var(--text);background:var(--bg);border:1px solid var(--border);
  border-radius:8px;padding:6px 10px}
button{background:var(--accent);color:#fff;border-color:var(--accent);cursor:pointer;font-weight:600}
button:hover{filter:brightness(1.08)}
select{width:100%}
.field{padding:10px 0;border-top:1px solid var(--border)}
.field:first-of-type{border-top:none;padding-top:0}
.field label{display:block;color:var(--text-muted);margin-bottom:6px}
.watts-input-group{display:flex;gap:8px}
.watts-input-group input[type=number]{flex:1;min-width:0}
.switch{position:relative;display:inline-block;width:42px;height:24px;flex:none}
.switch input{opacity:0;width:0;height:0}
.slider{position:absolute;inset:0;background:var(--muted-bg);border-radius:999px;
  transition:background .15s;cursor:pointer}
.slider::before{content:"";position:absolute;width:18px;height:18px;left:3px;top:3px;
  background:var(--bg-elev);border-radius:50%;transition:transform .15s;box-shadow:var(--shadow)}
.switch input:checked+.slider{background:var(--accent)}
.switch input:checked+.slider::before{transform:translateX(18px)}
</style>
</head>
<body>
<div class="wrap">
  <div class="topbar">
    <h1>Solar EV Charger Controller</h1>
    <span id="wifiPill"></span>
  </div>

  <div id="errorBanner" class="banner banner-bad" style="display:none"></div>

  <div id="dashboard" style="display:none">
    <div class="grid">
      <section class="card">
        <h2>Target current</h2>
        <div class="stat-value"><span id="targetAmps">-- A</span> <span class="stat-value-sub" id="targetWatts">(-- W)</span></div>
        <div class="stat-sub" id="decision">Loading&hellip;</div>
        <div class="flow-row"><span class="flow-dot" id="gridFlow"></span><span id="gridWatts">-- W</span></div>
      </section>

      <section class="card">
        <h2>Charge point</h2>
        <div class="kv"><span>Mode</span><span id="cpMode">--</span></div>
        <div class="kv"><span>Clamp</span><span id="cpDuty">--</span></div>
        <div class="kv"><span>Connector</span><span id="connector">--</span></div>
      </section>

      <section class="card">
        <h2>Link health</h2>
        <div class="kv"><span>Data source</span><span id="sourceName">--</span></div>
        <div class="kv"><span>Last poll</span><span id="pollAge">--</span></div>
        <div id="staleWarn" class="banner banner-warn" style="display:none;margin:12px 0 0">
          Data is stale &mdash; CP will fail safe to native pass-through.
        </div>
      </section>

      <section class="card card-settings">
        <h2>Settings</h2>
        <div class="field">
          <label for="sourceSelect">Grid source</label>
          <select id="sourceSelect"></select>
        </div>
        <div class="kv">
          <span>Simulation mode</span>
          <label class="switch"><input type="checkbox" id="simToggle"><span class="slider"></span></label>
        </div>
        <div class="field" id="wattsRow" style="display:none">
          <label for="wattsInput">Simulated watts</label>
          <span class="watts-input-group">
            <input type="number" id="wattsInput" step="50">
            <button id="wattsApply" type="button">Apply</button>
          </span>
        </div>
      </section>
    </div>
  </div>
</div>
<script>
(function(){
  function $(id){ return document.getElementById(id); }
  function badge(cls, text, title){
    return '<span class="badge badge-' + cls + '"' + (title ? ' title="' + title + '"' : '') + '>' + text + '</span>';
  }

  function render(d){
    $('targetAmps').textContent = d.target_a.toFixed(1) + ' A';
    $('targetWatts').textContent = '(' + Math.round(d.target_w) + ' W)';
    $('decision').textContent = d.decision + (d.settle_s > 0 ? ' (' + d.settle_s + 's left)' : '');
    $('gridWatts').textContent = Math.abs(d.grid_w).toFixed(0) + ' W ' + (d.exporting ? 'exporting' : 'importing');
    $('gridFlow').className = 'flow-dot ' + (d.exporting ? 'flow-export' : 'flow-import');

    $('cpMode').innerHTML = badge(d.cp_mode_cls, d.cp_mode, d.cp_mode_title);
    $('cpDuty').innerHTML = badge(d.duty_cls, d.cp_duty, d.cp_duty_title);
    $('connector').innerHTML = badge(d.connector_cls, d.connector, d.connector_title);

    $('wifiPill').innerHTML = badge(d.wifi ? 'ok' : 'bad', d.wifi ? 'Wi-Fi connected' : 'Wi-Fi disconnected');
    $('sourceName').textContent = d.source + (d.poll_failed ? ' (last poll failed)' : '');
    $('pollAge').textContent = d.poll_age;
    $('staleWarn').style.display = d.stale ? 'block' : 'none';

    if (document.activeElement !== $('sourceSelect')) {
      $('sourceSelect').innerHTML = d.sources.map(function(s){
        return '<option value="' + s.id + '"' + (s.active ? ' selected' : '') + '>' + s.name + '</option>';
      }).join('');
    }

    $('simToggle').checked = d.sim_active;
    if (document.activeElement !== $('wattsInput')) {
      $('wattsInput').value = d.sim_w;
    }
    $('wattsRow').style.display = d.sim_active ? 'block' : 'none';
  }

  function showError(msg){
    $('errorBanner').textContent = msg;
    $('errorBanner').style.display = 'block';
    $('dashboard').style.display = 'none';
  }

  function poll(){
    fetch('/api/status', {cache:'no-store'}).then(function(r){
      if (!r.ok) { return r.text().then(function(t){ throw new Error(t || ('HTTP ' + r.status)); }); }
      return r.json();
    }).then(function(d){
      $('errorBanner').style.display = 'none';
      $('dashboard').style.display = '';
      render(d);
    }).catch(function(e){ showError(e.message); });
  }

  function applyAndRender(url){
    fetch(url, {cache:'no-store'}).then(function(r){ return r.json(); }).then(render).catch(function(){});
  }

  document.addEventListener('DOMContentLoaded', function(){
    $('sourceSelect').addEventListener('change', function(){
      applyAndRender('/api/set_source?id=' + encodeURIComponent(this.value));
    });
    $('simToggle').addEventListener('change', function(){
      applyAndRender('/api/set?mode=' + (this.checked ? 'sim' : 'real'));
    });
    $('wattsApply').addEventListener('click', function(){
      applyAndRender('/api/set?w=' + encodeURIComponent($('wattsInput').value));
    });
    poll();
    setInterval(poll, 5000);
  });
})();
</script>
</body>
</html>
)PAGE";

static void handle_root() {
    if (!require_auth()) {
        return;
    }
    s_web_server.send_P(200, "text/html", PAGE_HTML);
}

// --- JSON status API ---------------------------------------------------
// Kept file-scope (rather than a stack local) so a deeply nested call chain
// (WiFi/OTA/WebServer internals all run inside this same task) never has to
// find room for it on the stack. Much smaller than the old full-HTML buffer
// since the static shell above no longer needs to be re-rendered per
// request - this only ever holds compile-time-constant strings plus a
// couple of numbers, never user-controllable text, so no JSON-escaping is
// needed.
static char s_json_body[1536];

static size_t append_sources_json(char *buf, size_t bufSize, size_t offset) {
    for (size_t i = 0; i < GRID_SOURCE_REGISTRY_COUNT && offset < bufSize; i++) {
        const GridDataSource *src = GRID_SOURCE_REGISTRY[i];
        offset += snprintf(buf + offset, bufSize - offset,
            "%s{\"id\":\"%s\",\"name\":\"%s\",\"active\":%s}",
            (i == 0) ? "" : ",",
            src->id, src->name,
            (src == s_active_grid_source) ? "true" : "false");
    }
    return offset;
}

static const char *build_status_json() {
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

    char pollAgeStr[24];
    if (everPolled) {
        snprintf(pollAgeStr, sizeof(pollAgeStr), "%lus ago", (unsigned long)pollAgeS);
    } else {
        snprintf(pollAgeStr, sizeof(pollAgeStr), "never");
    }

    const char *dataSourceStr = s_last_solar_status.simulated
        ? "Simulated"
        : s_active_grid_source->name;

    const char *cpModeStr = (cp.mode == MODE_BYPASS)
        ? "Bypass - stale/unavailable data, clamp released, charger running at its native rate (disconnect relay stays closed)"
        : "Active - following the solar target below";
    const char *cpModeLabel = (cp.mode == MODE_BYPASS) ? "Bypass" : "Active";
    const char *cpModeCls = (cp.mode == MODE_BYPASS) ? "warn" : "ok";
    const char *cpDutyStr = (cp.mode != MODE_ACTIVE)
        ? "Clamp inactive - CP fail-safe bypass, charger running at its native rate"
        : (cp.duty_state == CP_OSCILLATING)
            ? "Oscillating - clamping every cycle to the target duty"
            : "Disconnected - surplus below the 6A floor, CP relay open, not charging";
    const char *cpDutyLabel = (cp.mode != MODE_ACTIVE)
        ? "N/A"
        : (cp.duty_state == CP_OSCILLATING) ? "Oscillating" : "Standby";
    const char *cpDutyCls = (cp.mode != MODE_ACTIVE)
        ? "muted"
        : (cp.duty_state == CP_OSCILLATING) ? "ok" : "warn";

    size_t offset = snprintf(s_json_body, sizeof(s_json_body),
        "{"
        "\"decision\":\"%s\","
        "\"grid_w\":%.0f,"
        "\"exporting\":%s,"
        "\"settle_s\":%lu,"
        "\"target_a\":%.1f,"
        "\"target_w\":%.0f,"
        "\"cp_mode\":\"%s\","
        "\"cp_mode_title\":\"%s\","
        "\"cp_mode_cls\":\"%s\","
        "\"cp_duty\":\"%s\","
        "\"cp_duty_title\":\"%s\","
        "\"duty_cls\":\"%s\","
        "\"connector\":\"%s\","
        "\"connector_title\":\"%s\","
        "\"connector_cls\":\"%s\","
        "\"wifi\":%s,"
        "\"source\":\"%s\","
        "\"poll_failed\":%s,"
        "\"poll_age\":\"%s\","
        "\"stale\":%s,"
        "\"sim_active\":%s,"
        "\"sim_w\":%.0f,"
        "\"sources\":[",

        decision_reason_str(s_last_decision),
        fabsf(surplusW),
        surplusW >= 0 ? "true" : "false",
        (unsigned long)settleRemainingS,
        s_last_target_amps,
        s_last_target_amps * MAINS_VOLTAGE_V,
        cpModeLabel, cpModeStr, cpModeCls,
        cpDutyLabel, cpDutyStr, cpDutyCls,
        connector_state_label(cp.connector_state), connector_state_str(cp.connector_state),
        connector_state_cls(cp.connector_state),
        s_last_solar_status.wifi_connected ? "true" : "false",
        dataSourceStr,
        (!s_last_solar_status.simulated && !s_last_solar_status.modbus_ok) ? "true" : "false",
        pollAgeStr,
        stale ? "true" : "false",
        s_sim_mode_active ? "true" : "false",
        s_simulated_grid_power_w);

    offset = append_sources_json(s_json_body, sizeof(s_json_body), offset);
    if (offset < sizeof(s_json_body)) {
        offset += snprintf(s_json_body + offset, sizeof(s_json_body) - offset, "]}");
    }
    return s_json_body;
}

static void handle_api_status() {
    if (!require_auth()) {
        return;
    }
    s_web_server.send(200, "application/json", build_status_json());
}

static void handle_api_set() {
    if (!require_auth()) {
        return;
    }
    if (s_web_server.hasArg("mode")) {
        s_sim_mode_active = (s_web_server.arg("mode") == "sim");
    }
    if (s_web_server.hasArg("w")) {
        s_simulated_grid_power_w = s_web_server.arg("w").toFloat();
    }
    s_web_server.send(200, "application/json", build_status_json());
}

static void handle_api_set_source() {
    if (!require_auth()) {
        return;
    }
    if (s_web_server.hasArg("id")) {
        s_active_grid_source = &grid_data_source_lookup(s_web_server.arg("id").c_str());
        // Persist the resolved id, not the raw request arg - an unrecognized
        // id in the request self-heals to the default rather than saving a
        // value that would need re-resolving (with a fallback) on next boot.
        save_grid_source_to_nvs(s_active_grid_source->id);
    }
    s_web_server.send(200, "application/json", build_status_json());
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
    s_web_server.on("/api/status", HTTP_GET, handle_api_status);
    s_web_server.on("/api/set", HTTP_GET, handle_api_set);
    s_web_server.on("/api/set_source", HTTP_GET, handle_api_set_source);
}

// Rebinds ArduinoOTA's UDP listener and the WebServer's TCP listener to
// whatever WiFi connection is current. Unlike register_network_services()
// above, this IS meant to be called again after every WiFi drop/reconnect -
// see the call site. ArduinoOTA.begin() is a no-op once it thinks it's
// already initialized, so without an explicit end() first, its UDP socket
// stays bound to the old (torn-down) interface forever after a reconnect -
// OTA/the web UI going permanently unreachable until a power cycle, while
// Modbus (a fresh socket every poll) keeps working. The explicit
// s_web_server.close() isn't strictly needed (begin() self-closes), but is
// harmless and kept for symmetry.
static void start_network_services() {
    ArduinoOTA.end();
    s_web_server.close();

    ArduinoOTA.begin();
    s_web_server.begin();
}

void solar_control_task(void *pvParameters) {
    (void)pvParameters;

    RuntimeConfig cfg = shared_state_get_runtime_config();
    uint32_t lastAppliedGeneration = cfg.generation;
    load_grid_source_from_nvs();

    connect_wifi(cfg);
    register_network_services();

    IPAddress inverterIp;
    if (cfg.provisioned) {
        inverterIp.fromString(cfg.inverter_ip);
    }

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

        // Picks up a new SSID/password/inverter IP committed over BLE (see
        // ble_config.cpp). RuntimeConfig's generation only advances on a
        // fully-applied BLE COMMIT, so cfg here is always a consistent
        // snapshot - never a new SSID paired with a stale password.
        RuntimeConfig freshCfg = shared_state_get_runtime_config();
        if (freshCfg.generation != lastAppliedGeneration) {
            lastAppliedGeneration = freshCfg.generation;
            cfg = freshCfg;
            if (cfg.provisioned) {
                inverterIp.fromString(cfg.inverter_ip);
            }
            if (WiFi.status() == WL_CONNECTED) {
                WiFi.disconnect();
            }
            lastWifiAttemptMs = 0; // reconnect with the new credentials on the next check below
        }

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
                connect_wifi(cfg);
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
                if (s_active_grid_source->read_power_w(inverterIp, gridPowerW)) {
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
