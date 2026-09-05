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
#include <time.h>

// What the control law just did, kept alongside the numeric target so the
// web UI can explain the decision in words instead of just showing a number.
enum ControlDecision {
    DECISION_SETTLING,     // still inside the settle window from the last change
    DECISION_STEP_UP,      // surplus supports more current, stepped up
    DECISION_STEP_DOWN,    // surplus dropped, stepped down
    DECISION_HOLD,         // already matched to surplus, no change needed
    DECISION_CAPPED_MAX,   // would go higher, but MAX_CURRENT_A is the ceiling
    DECISION_CAPPED_ZERO,  // no surplus at all, held at the floor
    DECISION_SCHEDULE_OVERRIDE, // fixed-current schedule window active - solar reading ignored
};

static const char *decision_reason_str(ControlDecision d) {
    switch (d) {
        case DECISION_SETTLING:    return "Holding - settling after last change";
        case DECISION_STEP_UP:     return "Increasing - surplus supports more current";
        case DECISION_STEP_DOWN:   return "Decreasing - surplus dropped";
        case DECISION_CAPPED_MAX:  return "Holding at max - surplus exceeds charger's limit";
        case DECISION_CAPPED_ZERO: return "Holding at 0A - no surplus available";
        case DECISION_HOLD:        return "Holding - matched to surplus";
        case DECISION_SCHEDULE_OVERRIDE: return "Scheduled charging - fixed current, solar reading ignored";
        default:                   return "";
    }
}

// Settling-gated integration from surplus power to target current. The EV's own draw is
// part of what the meter measures, so each accepted change starts a settle window and
// readings during it are ignored (still displayed/logged, not acted on) so the loop
// doesn't fight its own effect on the next poll.
static ControlDecision compute_next_target_amps(float currentTargetA, float gridPowerW,
                                                  uint32_t nowMs, uint32_t &lastChangeMs,
                                                  float &outTargetA, float mainsVoltageV) {
    if (nowMs - lastChangeMs < SETTLE_MS) {
        outTargetA = currentTargetA;
        return DECISION_SETTLING;
    }

    float surplusW = -gridPowerW; // positive = exporting

    // Full correction from the current surplus reading, no per-step magnitude limit -
    // nothing in the CP protocol requires gradual changes. SETTLE_MS alone gates reaction
    // rate. rawTargetA is the exact amps the current surplus supports, even from a cold
    // start - a large surplus reaches equilibrium in one settled poll, not the floor first.
    float deltaA = surplusW / mainsVoltageV;
    float rawTargetA = constrain(currentTargetA + deltaA, 0.0f, MAX_CURRENT_A);

    // Mirrors cp_interceptor.cpp's own STANDBY/OSCILLATING thresholds: bare
    // MIN_CURRENT_A floor on entry (no margin), HYSTERESIS_A margin only on exit.
    bool wasCharging = currentTargetA >= MIN_CURRENT_A;
    float floorThreshold = wasCharging ? (MIN_CURRENT_A - HYSTERESIS_A) : MIN_CURRENT_A;

    float newTargetA;
    if (rawTargetA < floorThreshold) {
        newTargetA = 0.0f; // not enough surplus (or a real deficit) - don't charge / drop out
    } else if (rawTargetA < MIN_CURRENT_A) {
        newTargetA = MIN_CURRENT_A; // exit-hysteresis band; duty floor holds actual output at 6A anyway
    } else {
        newTargetA = rawTargetA;
    }

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

// WiFi SSID/password are provisioned entirely over BLE (ble_config.cpp), read here from
// shared_state. Skips WiFi.begin() entirely if unprovisioned/empty - retried on the normal
// WIFI_RECONNECT_INTERVAL_MS cadence. Core 1's stale-data fail-safe already makes "no
// working network" safe (native pass-through).
static void connect_wifi(const RuntimeConfig &cfg) {
    if (!cfg.provisioned || cfg.ssid[0] == '\0') {
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(cfg.ssid, cfg.password);
}

// --- Grid data source: runtime-selectable from the control page -----------
// Only Core 0's solar_control_task ever touches this (poll loop + its synchronous
// WebServer handlers), so no mutex needed. Always valid - grid_data_source_lookup()
// never returns null (falls back to index 0), so no call site needs a null check.
static const GridDataSource *s_active_grid_source = &GRID_SOURCE_SUNGROW_WINET;

static const char *GRID_SOURCE_NVS_NAMESPACE = "gridsrc";

// Independent of ble_config.cpp's "netcfg" namespace - chosen from the HTTP control page.
static void load_grid_source_from_nvs() {
    Preferences prefs;
    prefs.begin(GRID_SOURCE_NVS_NAMESPACE, true);
    String id = prefs.getString("id", "");
    prefs.end();
    if (id.length() > 0) {
        s_active_grid_source = &grid_data_source_lookup(id.c_str());
    }
    // else: nothing saved yet - keep the compile-time default.
}

static void save_grid_source_to_nvs(const char *id) {
    Preferences prefs;
    prefs.begin(GRID_SOURCE_NVS_NAMESPACE, false);
    prefs.putString("id", id);
    prefs.end();
}

// --- Scheduled (fixed-current) charging ------------------------------------
// Control-page-owned, same ownership/mutex-free reasoning as s_active_grid_source above,
// own NVS namespace independent of "netcfg"/GRID_SOURCE_NVS_NAMESPACE.
//
// start_min/end_min are UTC minutes-since-midnight - storing/evaluating in UTC keeps
// timezone/DST conversion in the browser (PAGE_HTML's JS), no TZ rule needed on-device.
struct ScheduleConfig {
    bool enabled;
    uint16_t start_min;
    uint16_t end_min;
    float amps;
};
static ScheduleConfig s_schedule = { false, 0, 0, MIN_CURRENT_A };

static const char *SCHEDULE_NVS_NAMESPACE = "schedule";

static void load_schedule_from_nvs() {
    Preferences prefs;
    prefs.begin(SCHEDULE_NVS_NAMESPACE, true);
    s_schedule.enabled = prefs.getBool("en", false);
    s_schedule.start_min = prefs.getUShort("sm", 0);
    s_schedule.end_min = prefs.getUShort("em", 0);
    s_schedule.amps = prefs.getFloat("amps", MIN_CURRENT_A);
    prefs.end();
}

static void save_schedule_to_nvs() {
    Preferences prefs;
    prefs.begin(SCHEDULE_NVS_NAMESPACE, false);
    prefs.putBool("en", s_schedule.enabled);
    prefs.putUShort("sm", s_schedule.start_min);
    prefs.putUShort("em", s_schedule.end_min);
    prefs.putFloat("amps", s_schedule.amps);
    prefs.end();
}

// False until NTP has genuinely synced at least once (see NTP_MIN_VALID_EPOCH
// in config.h) - a schedule must never activate against a guessed/default
// clock. configTime() is kicked off in start_network_services() below.
static bool get_utc_now(struct tm &outTm) {
    time_t nowEpoch = time(nullptr);
    if (nowEpoch < (time_t)NTP_MIN_VALID_EPOCH) {
        return false;
    }
    gmtime_r(&nowEpoch, &outTm);
    return true;
}

// [start_min, end_min) in UTC minutes-since-midnight, wrapping past midnight
// when end_min <= start_min (e.g. a 23:00-06:00 off-peak window). Equal
// start/end is treated as "never active" rather than "always active" - a
// full-day schedule is just 00:00-23:59.
static bool is_schedule_window_now(const ScheduleConfig &s, const struct tm &utcNow) {
    if (s.start_min == s.end_min) {
        return false;
    }
    uint16_t nowMin = (uint16_t)(utcNow.tm_hour * 60 + utcNow.tm_min);
    if (s.start_min < s.end_min) {
        return nowMin >= s.start_min && nowMin < s.end_min;
    }
    return nowMin >= s.start_min || nowMin < s.end_min;
}

// Grid voltage comes from the active GridDataSource each poll, not a manually-configured
// global. This is a status cache only (like s_last_target_amps below), so
// handle_root()/build_status_json() can show the most recently reported figure between polls.
static float s_last_mains_voltage_v = MAINS_VOLTAGE_FIXED_V;

// Latest status, cached for the web UI. Written once per poll from solar_control_task's own
// loop, read once per page request from the same task via handleClient() - no cross-task
// access, so no mutex needed.
static SolarStatus s_last_solar_status = {};
static float s_last_target_amps = 0.0f;
static ControlDecision s_last_decision = DECISION_HOLD;
static uint32_t s_last_lastChangeMs = 0;

// Server-side power history for the control page's chart - a fixed ring of 5-minute
// buckets (not raw per-poll samples) so every browser session sees the same rolling 24h
// window instead of each tab reconstructing its own from /api/status polls. RAM-only,
// resets on reboot - see CLAUDE.md/plan notes; no NVS persistence, matching the project's
// pattern of only persisting user-driven settings, never periodic telemetry. Only ever
// touched from solar_control_task (poll loop writes, HTTP handler reads), so no mutex.
struct HistoryBucket {
    uint32_t start_epoch_s;
    float target_sum_w;
    float grid_sum_w;
    uint16_t sample_count;
};
static HistoryBucket s_history[HISTORY_BUCKET_CAPACITY];
static size_t s_history_created = 0; // total buckets ever started (monotonic)

static WebServer s_web_server(SIM_HTTP_PORT);

// web_password is set entirely over BLE (SET WEB_PASS, ble_config.cpp) - never
// OTA_PASSWORD. Until one has been committed, the page refuses every request outright
// rather than falling back to an empty/guessable credential.
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
#historyCard{grid-column:1/-1}
#historyChart{display:block;width:100%;height:220px}
.chart-legend{display:flex;gap:16px;margin-top:8px;font-size:.8rem;color:var(--text-muted)}
.chart-legend .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px;vertical-align:middle}
.dot-target{background:var(--accent)}
.dot-grid{background:var(--warn)}
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
        <h2>Charge point</h2>
        <div class="stat-value"><span id="targetWatts">-- W</span> <span class="stat-value-sub" id="targetAmps">(-- A)</span></div>
        <div class="stat-sub" id="decision">Loading&hellip;</div>
        <div class="kv"><span>Solar control</span><span id="cpMode">--</span></div>
        <div class="kv"><span>Charging status</span><span id="cpDuty">--</span></div>
        <div class="kv"><span>Connector</span><span id="connector">--</span></div>
      </section>

      <section class="card card-settings">
        <h2>Data source</h2>
        <div class="field">
          <select id="sourceSelect"></select>
          <div class="stat-sub" id="pollFailedNote" style="display:none;color:var(--bad)">Last poll failed</div>
        </div>
        <div class="kv"><span>Grid Power</span><span><span class="flow-dot" id="gridFlow" style="display:inline-block;vertical-align:middle;margin-right:6px"></span><span id="gridWatts">--</span></span></div>
        <div class="kv"><span>Battery</span><span id="batteryWatts">--</span></div>
        <div class="kv"><span>Grid voltage</span><span id="voltageDisplay">--</span></div>
        <div class="kv"><span>Last poll</span><span id="pollAge">--</span></div>
        <div id="batteryExcludedNote" class="stat-sub" style="display:none"></div>
        <div id="staleWarn" class="banner banner-warn" style="display:none;margin:12px 0 0">
          Data is stale &mdash; CP will fail safe to native pass-through.
        </div>
      </section>

      <section class="card card-settings">
        <h2>Schedule</h2>
        <div class="field">
          <label><input type="checkbox" id="scheduleEnabled"> Enabled</label>
        </div>
        <div class="field">
          <label for="scheduleStart">Start (local)</label>
          <input type="time" id="scheduleStart">
        </div>
        <div class="field">
          <label for="scheduleEnd">End (local)</label>
          <input type="time" id="scheduleEnd">
        </div>
        <div class="field">
          <label for="scheduleAmps">Current (A)</label>
          <input type="number" id="scheduleAmps" min="6" max="32" step="0.1">
        </div>
        <div class="field">
          <button id="scheduleSaveBtn" type="button">Save</button>
          <span id="scheduleActiveBadge"></span>
        </div>
        <div id="timeSyncWarn" class="stat-sub" style="display:none;color:var(--warn)">Waiting for time sync&hellip;</div>
      </section>

      <section class="card" id="historyCard">
        <h2>Power history</h2>
        <canvas id="historyChart"></canvas>
        <div class="chart-legend">
          <span><i class="dot dot-target"></i>Target</span>
          <span><i class="dot dot-grid"></i>Grid</span>
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
  function pad2(n){ return String(n).padStart(2, '0'); }

  // Schedule times are stored/sent as "HH:MM" UTC; these fields are what
  // the <input type=time> shows/accepts, which is always local time - the
  // browser's own Date knows the current offset (DST included), so no
  // timezone/DST logic needs to live on the device at all.
  function utcHHMMToLocal(utcHHMM){
    var parts = utcHHMM.split(':');
    var d = new Date(Date.UTC(1970, 0, 1, +parts[0], +parts[1]));
    return pad2(d.getHours()) + ':' + pad2(d.getMinutes());
  }
  function localHHMMToUtc(localHHMM){
    var parts = localHHMM.split(':');
    var d = new Date();
    d.setHours(+parts[0], +parts[1], 0, 0);
    return pad2(d.getUTCHours()) + ':' + pad2(d.getUTCMinutes());
  }
  var SCHEDULE_FIELD_IDS = ['scheduleEnabled', 'scheduleStart', 'scheduleEnd', 'scheduleAmps'];
  function scheduleFieldFocused(){
    return SCHEDULE_FIELD_IDS.indexOf(document.activeElement && document.activeElement.id) !== -1;
  }

  // Populated from /api/history (see fetchHistory below) - the device itself owns this
  // rolling window so every browser session shows the same data, not a per-tab
  // reconstruction from /api/status polls.
  var history = [];

  function fetchHistory(){
    fetch('/api/history', {cache:'no-store'}).then(function(r){ return r.json(); }).then(function(d){
      history = d.points.map(function(p){
        return {t: p.t * 1000, target: p.target_w, grid: p.grid_w};
      });
      drawChart();
    }).catch(function(){});
  }

  function drawChart(){
    var canvas = $('historyChart');
    var cssWidth = canvas.clientWidth, cssHeight = canvas.clientHeight;
    if (cssWidth === 0 || cssHeight === 0) { return; }
    var dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(cssWidth * dpr) || canvas.height !== Math.round(cssHeight * dpr)) {
      canvas.width = Math.round(cssWidth * dpr);
      canvas.height = Math.round(cssHeight * dpr);
    }
    var ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssWidth, cssHeight);
    if (history.length === 0) { return; }

    var style = getComputedStyle(document.documentElement);
    var accentColor = style.getPropertyValue('--accent').trim();
    var warnColor = style.getPropertyValue('--warn').trim();
    var borderColor = style.getPropertyValue('--border').trim();

    var textMutedColor = style.getPropertyValue('--text-muted').trim();

    var values = [0];
    for (var i = 0; i < history.length; i++) {
      values.push(history[i].target, history[i].grid);
    }
    var min = Math.min.apply(null, values);
    var max = Math.max.apply(null, values);
    if (min === max) { min -= 100; max += 100; }

    function niceStep(rawStep){
      var mag = Math.pow(10, Math.floor(Math.log10(rawStep)));
      var f = rawStep / mag;
      var niceF = f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10;
      return niceF * mag;
    }
    var step = niceStep((max - min) / 5);
    var niceMin = Math.floor(min / step) * step;
    var niceMax = Math.ceil(max / step) * step;

    var tMin = history[0].t, tMax = history[history.length - 1].t;
    if (tMax === tMin) { tMax = tMin + 1; }

    var marginLeft = 52, marginRight = 12, marginTop = 8, marginBottom = 26;
    var plotLeft = marginLeft, plotRight = cssWidth - marginRight;
    var plotTop = marginTop, plotBottom = cssHeight - marginBottom;
    var plotWidth = plotRight - plotLeft, plotHeight = plotBottom - plotTop;

    function xOf(t){ return plotLeft + (t - tMin) / (tMax - tMin) * plotWidth; }
    function yOf(v){ return plotBottom - (v - niceMin) / (niceMax - niceMin) * plotHeight; }

    ctx.font = '10px ' + getComputedStyle(document.body).fontFamily;

    // Y-axis: nice-number gridlines/labels. niceMin/niceMax are exact multiples of
    // step spanning the (always-included-via `values=[0]`) zero point, so 0 always
    // lands on a tick here - no separate zero-reference-line special case needed.
    ctx.lineWidth = 1;
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'right';
    for (var tick = niceMin; tick <= niceMax + 1e-9; tick += step) {
      var y = yOf(tick);
      ctx.strokeStyle = borderColor;
      ctx.beginPath();
      ctx.moveTo(plotLeft, y);
      ctx.lineTo(plotRight, y);
      ctx.stroke();
      ctx.fillStyle = textMutedColor;
      ctx.fillText(Math.round(tick).toLocaleString() + ' W', plotLeft - 6, y);
    }

    // X-axis: evenly spaced ticks across the actual retained time span - not
    // snapped to calendar boundaries, since history can span minutes or a full
    // 24h. Skipped entirely with <2 points (nothing meaningful yet after boot).
    if (history.length >= 2) {
      var xTickCount = 5;
      ctx.textBaseline = 'top';
      for (var k = 0; k < xTickCount; k++) {
        var tickT = tMin + k * (tMax - tMin) / (xTickCount - 1);
        var x = xOf(tickT);
        ctx.strokeStyle = borderColor;
        ctx.beginPath();
        ctx.moveTo(x, plotBottom);
        ctx.lineTo(x, plotBottom + 4);
        ctx.stroke();
        ctx.textAlign = (k === 0) ? 'left' : (k === xTickCount - 1) ? 'right' : 'center';
        var d = new Date(tickT);
        ctx.fillStyle = textMutedColor;
        ctx.fillText(pad2(d.getHours()) + ':' + pad2(d.getMinutes()), x, plotBottom + 6);
      }
    }

    function drawSeries(key, color){
      if (history.length < 2) { return; }
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      for (var j = 0; j < history.length; j++) {
        var x = xOf(history[j].t), y = yOf(history[j][key]);
        if (j === 0) { ctx.moveTo(x, y); } else { ctx.lineTo(x, y); }
      }
      ctx.stroke();
    }
    drawSeries('target', accentColor);
    drawSeries('grid', warnColor);
  }

  function render(d){
    $('targetWatts').textContent = Math.round(d.target_w) + ' W';
    $('targetAmps').textContent = '(' + d.target_a.toFixed(1) + ' A)';
    $('decision').textContent = d.decision + (d.settle_s > 0 ? ' (' + d.settle_s + 's left)' : '');
    $('gridWatts').textContent = Math.abs(d.grid_w).toFixed(0) + ' W ' + (d.exporting ? 'exporting' : 'importing');
    $('gridFlow').className = 'flow-dot ' + (d.exporting ? 'flow-export' : 'flow-import');

    $('batteryWatts').textContent = d.battery_w.toFixed(0) + ' W ' + d.battery_state;
    if (d.surplus_excluded_w > 0) {
      $('batteryExcludedNote').textContent = Math.round(d.surplus_excluded_w) + ' W excluded from surplus - home battery discharging';
      $('batteryExcludedNote').style.display = '';
    } else {
      $('batteryExcludedNote').style.display = 'none';
    }

    $('cpMode').innerHTML = badge(d.cp_mode_cls, d.cp_mode, d.cp_mode_title);
    $('cpDuty').innerHTML = badge(d.duty_cls, d.cp_duty, d.cp_duty_title);
    $('connector').innerHTML = badge(d.connector_cls, d.connector, d.connector_title);

    $('wifiPill').innerHTML = badge(d.wifi ? 'ok' : 'bad', d.wifi ? 'Wi-Fi connected' : 'Wi-Fi disconnected');
    $('pollFailedNote').style.display = d.poll_failed ? '' : 'none';
    $('pollAge').textContent = d.poll_age;
    $('staleWarn').style.display = d.stale ? 'block' : 'none';

    if (document.activeElement !== $('sourceSelect')) {
      $('sourceSelect').innerHTML = d.sources.map(function(s){
        return '<option value="' + s.id + '"' + (s.active ? ' selected' : '') + '>' + s.name + '</option>';
      }).join('');
    }

    $('voltageDisplay').textContent = d.voltage_v.toFixed(0) + ' V';

    if (!scheduleFieldFocused()) {
      $('scheduleEnabled').checked = d.schedule_enabled;
      $('scheduleStart').value = utcHHMMToLocal(d.schedule_start_utc);
      $('scheduleEnd').value = utcHHMMToLocal(d.schedule_end_utc);
      $('scheduleAmps').value = d.schedule_amps;
    }
    $('scheduleActiveBadge').innerHTML = d.schedule_active ? badge('ok', 'Active now') : '';
    $('timeSyncWarn').style.display = d.time_synced ? 'none' : '';

    drawChart();
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
    $('scheduleSaveBtn').addEventListener('click', function(){
      var url = '/api/set_schedule'
        + '?enabled=' + ($('scheduleEnabled').checked ? '1' : '0')
        + '&start=' + encodeURIComponent(localHHMMToUtc($('scheduleStart').value))
        + '&end=' + encodeURIComponent(localHHMMToUtc($('scheduleEnd').value))
        + '&amps=' + encodeURIComponent($('scheduleAmps').value);
      applyAndRender(url);
    });
    window.addEventListener('resize', drawChart);
    poll();
    setInterval(poll, 5000);
    fetchHistory();
    setInterval(fetchHistory, 30000);
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
// File-scope (not a stack local) so a deeply nested call chain (WiFi/OTA/WebServer all run
// in this same task) never has to find room for it on the stack. Only ever holds
// compile-time-constant strings plus a few numbers, never user-controllable text, so no
// JSON-escaping is needed.
static char s_json_body[1792];

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
    float batteryPowerW = s_last_solar_status.battery_power_w;
    // How much of the raw surplus above is being excluded because it's
    // attributable to the home battery discharging rather than PV - mirrors
    // the fmaxf(0.0f, -batteryPowerW) adjustment solar_control_task applies
    // before compute_next_target_amps() ever sees the reading.
    float surplusExcludedW = fmaxf(0.0f, -batteryPowerW);
    const char *batteryStateStr = (batteryPowerW < 0.0f) ? "discharging"
                                    : (batteryPowerW > 0.0f) ? "charging" : "idle";
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

    const char *dataSourceStr = s_active_grid_source->name;

    char scheduleStartStr[6], scheduleEndStr[6];
    snprintf(scheduleStartStr, sizeof(scheduleStartStr), "%02u:%02u", s_schedule.start_min / 60, s_schedule.start_min % 60);
    snprintf(scheduleEndStr, sizeof(scheduleEndStr), "%02u:%02u", s_schedule.end_min / 60, s_schedule.end_min % 60);
    struct tm utcNowForStatus;
    bool timeSyncedForStatus = get_utc_now(utcNowForStatus);

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
        "\"battery_w\":%.0f,"
        "\"battery_state\":\"%s\","
        "\"surplus_excluded_w\":%.0f,"
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
        "\"voltage_v\":%.0f,"
        "\"schedule_enabled\":%s,"
        "\"schedule_start_utc\":\"%s\","
        "\"schedule_end_utc\":\"%s\","
        "\"schedule_amps\":%.1f,"
        "\"schedule_active\":%s,"
        "\"time_synced\":%s,"
        "\"sources\":[",

        decision_reason_str(s_last_decision),
        fabsf(surplusW),
        surplusW >= 0 ? "true" : "false",
        fabsf(batteryPowerW),
        batteryStateStr,
        surplusExcludedW,
        (unsigned long)settleRemainingS,
        s_last_target_amps,
        s_last_target_amps * s_last_mains_voltage_v,
        cpModeLabel, cpModeStr, cpModeCls,
        cpDutyLabel, cpDutyStr, cpDutyCls,
        connector_state_label(cp.connector_state), connector_state_str(cp.connector_state),
        connector_state_cls(cp.connector_state),
        s_last_solar_status.wifi_connected ? "true" : "false",
        dataSourceStr,
        !s_last_solar_status.modbus_ok ? "true" : "false",
        pollAgeStr,
        stale ? "true" : "false",
        s_last_mains_voltage_v,
        s_schedule.enabled ? "true" : "false",
        scheduleStartStr,
        scheduleEndStr,
        s_schedule.amps,
        s_last_solar_status.schedule_active ? "true" : "false",
        timeSyncedForStatus ? "true" : "false");

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

// start/end arrive as "HH:MM" UTC (the control page's JS converts from the
// viewer's local time before sending - see PAGE_HTML). A malformed value is
// simply ignored, leaving the previously-saved field in place, same
// leave-prior-value-on-bad-input pattern as SET INVERTER_IP over BLE.
static bool parse_hhmm_utc(const String &s, uint16_t &outMin) {
    int h, m;
    if (sscanf(s.c_str(), "%d:%d", &h, &m) != 2) {
        return false;
    }
    if (h < 0 || h > 23 || m < 0 || m > 59) {
        return false;
    }
    outMin = (uint16_t)(h * 60 + m);
    return true;
}

static void handle_api_set_schedule() {
    if (!require_auth()) {
        return;
    }
    if (s_web_server.hasArg("enabled")) {
        s_schedule.enabled = s_web_server.arg("enabled") == "1";
    }
    if (s_web_server.hasArg("start")) {
        uint16_t m;
        if (parse_hhmm_utc(s_web_server.arg("start"), m)) {
            s_schedule.start_min = m;
        }
    }
    if (s_web_server.hasArg("end")) {
        uint16_t m;
        if (parse_hhmm_utc(s_web_server.arg("end"), m)) {
            s_schedule.end_min = m;
        }
    }
    if (s_web_server.hasArg("amps")) {
        float a = s_web_server.arg("amps").toFloat();
        s_schedule.amps = constrain(a, MIN_CURRENT_A, MAX_CURRENT_A);
    }
    save_schedule_to_nvs();
    s_web_server.send(200, "application/json", build_status_json());
}

// File-scope for the same reason as s_json_body above. Sized for
// HISTORY_BUCKET_CAPACITY points at ~45 bytes/point plus overhead.
static char s_history_json_body[16384];

static const char *build_history_json() {
    size_t offset = snprintf(s_history_json_body, sizeof(s_history_json_body),
        "{\"bucket_span_s\":%d,\"points\":[", HISTORY_BUCKET_SPAN_S);

    size_t count = (s_history_created < HISTORY_BUCKET_CAPACITY) ? s_history_created : HISTORY_BUCKET_CAPACITY;
    size_t oldest = s_history_created - count;
    for (size_t i = 0; i < count && offset < sizeof(s_history_json_body); i++) {
        const HistoryBucket &b = s_history[(oldest + i) % HISTORY_BUCKET_CAPACITY];
        offset += snprintf(s_history_json_body + offset, sizeof(s_history_json_body) - offset,
            "%s{\"t\":%lu,\"target_w\":%.0f,\"grid_w\":%.0f}",
            (i == 0) ? "" : ",",
            (unsigned long)b.start_epoch_s,
            b.target_sum_w / b.sample_count,
            b.grid_sum_w / b.sample_count);
    }
    if (offset < sizeof(s_history_json_body)) {
        offset += snprintf(s_history_json_body + offset, sizeof(s_history_json_body) - offset, "]}");
    }
    return s_history_json_body;
}

static void handle_api_history() {
    if (!require_auth()) {
        return;
    }
    s_web_server.send(200, "application/json", build_history_json());
}

// Route/callback registration: WebServer::on() appends to an internal linked list every
// call rather than replacing, so this must run exactly once for the process lifetime -
// repeating it on reconnect would leak a handler. Safe before WiFi/services are up; only
// registers callbacks, doesn't bind a socket.
static void register_network_services() {
    ArduinoOTA.setHostname(WIFI_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() { Serial.println("OTA: update starting"); });
    ArduinoOTA.onEnd([]() { Serial.println("OTA: update complete"); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA: error %u\n", error); });

    s_web_server.on("/", HTTP_GET, handle_root);
    s_web_server.on("/api/status", HTTP_GET, handle_api_status);
    s_web_server.on("/api/set_source", HTTP_GET, handle_api_set_source);
    s_web_server.on("/api/set_schedule", HTTP_GET, handle_api_set_schedule);
    s_web_server.on("/api/history", HTTP_GET, handle_api_history);
}

// Rebinds ArduinoOTA's UDP listener and the WebServer's TCP listener to whatever WiFi
// connection is current - unlike register_network_services() above, meant to be called
// again on every reconnect. ArduinoOTA.begin() is a no-op once already initialized, so
// without an explicit end() first its UDP socket stays bound to the torn-down interface,
// leaving OTA/the web UI unreachable until a power cycle. s_web_server.close() isn't
// strictly needed (begin() self-closes) but kept for symmetry.
static void start_network_services() {
    ArduinoOTA.end();
    s_web_server.close();

    ArduinoOTA.begin();
    s_web_server.begin();

    // UTC only, zero offset - timezone/DST conversion lives in the browser (config.h).
    // Re-issued on every reconnect so SNTP re-resolves against the current connection's DNS.
    configTime(0, 0, NTP_SERVER);
}

void solar_control_task(void *pvParameters) {
    (void)pvParameters;

    RuntimeConfig cfg = shared_state_get_runtime_config();
    uint32_t lastAppliedGeneration = cfg.generation;
    load_grid_source_from_nvs();
    load_schedule_from_nvs();

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

        // Liveness signal for Core 1's recovery watchdog - placed at the top of the loop so
        // a hang anywhere below (WiFi/OTA/WebServer/Modbus) is what actually gets caught.
        shared_state_heartbeat_solar_task();

        // Picks up a new SSID/password/inverter IP committed over BLE. generation only
        // advances on a fully-applied COMMIT, so cfg here is always a consistent snapshot.
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
            // Re-runs start_network_services() once WiFi comes back (see that function's
            // comment for why a stale binding won't recover on its own).
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

            // Evaluated every poll tick regardless of WiFi/Modbus outcome
            // below - a scheduled fixed-current charge must not depend on
            // the grid-data pipeline (see CLAUDE.md "Solar data source" and
            // cp_interceptor.cpp's staleness check).
            struct tm utcNow;
            bool timeSynced = get_utc_now(utcNow);
            bool scheduleActive = timeSynced && s_schedule.enabled && is_schedule_window_now(s_schedule, utcNow);

            SolarStatus status;
            status.wifi_connected = (WiFi.status() == WL_CONNECTED);
            status.modbus_ok = false;
            status.grid_power_w = 0.0f;
            status.battery_power_w = 0.0f;
            status.last_poll_success_ms = lastPollSuccessMs;
            status.schedule_active = scheduleActive;
            // Carried forward (not reset to 0) when inactive, so it ages
            // past STALE_DATA_TIMEOUT_MS on its own once the window ends -
            // see the field's comment in shared_state.h.
            status.last_schedule_confirm_ms = scheduleActive ? millis() : s_last_solar_status.last_schedule_confirm_ms;

            if (scheduleActive) {
                targetAmps = constrain(s_schedule.amps, MIN_CURRENT_A, MAX_CURRENT_A);
                shared_state_set_target_amps(targetAmps);
                // Fresh settle window for solar control's own hysteresis the
                // moment the schedule ends - targetAmps was just set from
                // the schedule, not derived from a surplus reading.
                lastChangeMs = now;
                s_last_decision = DECISION_SCHEDULE_OVERRIDE;
            }

            if (status.wifi_connected) {
                float gridPowerW;
                float voltageV;
                float batteryPowerW;
                // Derived from the previous poll's voltage - this poll's own voltage isn't
                // known until the read call below returns it; negligible staleness.
                float currentDrawW = targetAmps * s_last_mains_voltage_v;
                if (s_active_grid_source->read_power_w(inverterIp, currentDrawW, gridPowerW, voltageV, batteryPowerW)) {
                    status.modbus_ok = true;
                    status.grid_power_w = gridPowerW;
                    status.battery_power_w = batteryPowerW;
                    lastPollSuccessMs = millis();
                    status.last_poll_success_ms = lastPollSuccessMs;
                    s_last_mains_voltage_v = voltageV;

                    // Exclude home-battery discharge from EV-charging surplus - the meter
                    // alone can't tell fresh-PV export from battery-discharge export.
                    // -batteryPowerW is positive exactly when discharging, so adding it
                    // back to gridPowerW cancels its contribution; battery charging adds 0
                    // and is left alone (that PV was already consumed). See
                    // grid_data_source.h's outBatteryW doc comment.
                    float effectiveGridPowerW = gridPowerW + fmaxf(0.0f, -batteryPowerW);

                    // Polling still runs (and still updates status/display)
                    // during a schedule window - it just stops feeding the
                    // control law while the schedule is in charge.
                    if (!scheduleActive) {
                        s_last_decision = compute_next_target_amps(targetAmps, effectiveGridPowerW, now, lastChangeMs, targetAmps, voltageV);
                        shared_state_set_target_amps(targetAmps);
                    }
                }
            }

            shared_state_publish_solar_status(status);
            s_last_solar_status = status;
            s_last_target_amps = targetAmps;
            s_last_lastChangeMs = lastChangeMs;

            // Only record a real reading, never a synthetic zero from a WiFi/Modbus outage
            // or a pre-NTP-sync boot - an outage then shows as an honest time gap between
            // two real points on the chart, not a misleading dip to zero.
            if (timeSynced && status.modbus_ok) {
                uint32_t nowEpoch = (uint32_t)time(nullptr);
                uint32_t bucketStartEpoch = nowEpoch - (nowEpoch % HISTORY_BUCKET_SPAN_S);
                bool needNewBucket = (s_history_created == 0);
                HistoryBucket *cur = nullptr;
                if (!needNewBucket) {
                    cur = &s_history[(s_history_created - 1) % HISTORY_BUCKET_CAPACITY];
                    needNewBucket = (cur->start_epoch_s != bucketStartEpoch);
                }
                if (needNewBucket) {
                    cur = &s_history[s_history_created % HISTORY_BUCKET_CAPACITY];
                    cur->start_epoch_s = bucketStartEpoch;
                    cur->target_sum_w = 0.0f;
                    cur->grid_sum_w = 0.0f;
                    cur->sample_count = 0;
                    s_history_created++;
                }
                cur->target_sum_w += targetAmps * s_last_mains_voltage_v;
                cur->grid_sum_w += status.grid_power_w;
                cur->sample_count++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
