#include "solar_control.h"
#include "config.h"
#include "secrets.h"
#include "shared_state.h"
#include "grid_data_source.h"
#include "build_info.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <math.h>
#include <time.h>
#include <string.h>

// What the control law just did, kept alongside the numeric target so the
// web UI can explain the decision in words instead of just showing a number.
enum ControlDecision {
    DECISION_BLANKING,     // inside the blanking window - readings discarded, not averaged
    DECISION_AVERAGING,    // past blanking, still collecting samples for the mean
    DECISION_STEP_UP,      // surplus supports more current, stepped up
    DECISION_STEP_DOWN,    // surplus dropped, stepped down
    DECISION_FAST_DROP,    // real import seen - cut immediately, skipping both windows
    DECISION_HOLD,         // already matched to surplus, no change needed
    DECISION_CAPPED_MAX,   // would go higher, but MAX_CURRENT_A is the ceiling
    DECISION_CAPPED_ZERO,  // no surplus at all, held at the floor
    DECISION_SCHEDULE_OVERRIDE, // fixed-current schedule window active - solar reading ignored
};

// Keep every string at or under the length of the longest one below (the
// SCHEDULE_OVERRIDE line, 57 chars) - build_status_json()'s worst-case size accounting
// assumes it, and that buffer truncates silently.
static const char *decision_reason_str(ControlDecision d) {
    switch (d) {
        case DECISION_BLANKING:    return "Holding - waiting for the last change to reach the meter";
        case DECISION_AVERAGING:   return "Sampling - averaging surplus before the next change";
        case DECISION_STEP_UP:     return "Increasing - surplus supports more current";
        case DECISION_STEP_DOWN:   return "Decreasing - surplus dropped";
        case DECISION_FAST_DROP:   return "Decreasing now - grid import detected, cutting early";
        case DECISION_CAPPED_MAX:  return "Holding at max - surplus exceeds charger's limit";
        case DECISION_CAPPED_ZERO: return "Holding at 0A - no surplus available";
        case DECISION_HOLD:        return "Holding - matched to surplus";
        case DECISION_SCHEDULE_OVERRIDE: return "Scheduled charging - fixed current, solar reading ignored";
        default:                   return "";
    }
}

// Runtime-tunable control-loop damping. Defaults in config.h, overridden from the HTTP
// control page and persisted in the "control" NVS namespace - the right values depend on
// this install's actual inverter and vehicle lag, which can only be measured against real
// weather, so they must be adjustable without a reflash. Only ever touched from
// solar_control_task (poll loop writes, its own synchronous handlers write), so no mutex.
struct ControlTuning {
    uint32_t blank_ms;
    uint16_t avg_samples;
    float    gain_up;
    float    deadband_a;
    float    fast_drop_a;
};
static ControlTuning s_tuning = {
    CONTROL_BLANK_MS_DEFAULT,
    CONTROL_AVG_SAMPLES_DEFAULT,
    CONTROL_GAIN_UP_DEFAULT,
    CONTROL_DEADBAND_A_DEFAULT,
    CONTROL_FAST_DROP_A_DEFAULT,
};

// Clamped on both load and set, so neither a corrupt NVS blob nor a hand-crafted query
// arg can produce a divergent loop. gain_up in particular must never exceed 1.0.
static void control_tuning_clamp(ControlTuning &t) {
    t.blank_ms    = (uint32_t)constrain((long)t.blank_ms, 0L, 120000L);
    t.avg_samples = (uint16_t)constrain((int)t.avg_samples, 1, 12);
    t.gain_up     = constrain(t.gain_up, 0.1f, 1.0f);
    t.deadband_a  = constrain(t.deadband_a, 0.0f, 5.0f);
    t.fast_drop_a = constrain(t.fast_drop_a, 0.5f, MAX_CURRENT_A);
}

static const char *CONTROL_NVS_NAMESPACE = "control";

static void load_control_tuning_from_nvs() {
    Preferences prefs;
    prefs.begin(CONTROL_NVS_NAMESPACE, true);
    s_tuning.blank_ms    = prefs.getULong("blank", CONTROL_BLANK_MS_DEFAULT);
    s_tuning.avg_samples = prefs.getUShort("avgn", CONTROL_AVG_SAMPLES_DEFAULT);
    s_tuning.gain_up     = prefs.getFloat("gup", CONTROL_GAIN_UP_DEFAULT);
    s_tuning.deadband_a  = prefs.getFloat("db", CONTROL_DEADBAND_A_DEFAULT);
    s_tuning.fast_drop_a = prefs.getFloat("fda", CONTROL_FAST_DROP_A_DEFAULT);
    prefs.end();
    control_tuning_clamp(s_tuning);
}

static void save_control_tuning_to_nvs() {
    Preferences prefs;
    prefs.begin(CONTROL_NVS_NAMESPACE, false);
    prefs.putULong("blank", s_tuning.blank_ms);
    prefs.putUShort("avgn", s_tuning.avg_samples);
    prefs.putFloat("gup", s_tuning.gain_up);
    prefs.putFloat("db", s_tuning.deadband_a);
    prefs.putFloat("fda", s_tuning.fast_drop_a);
    prefs.end();
}

// All control-law state that has to survive across polls. Kept as a solar_control_task
// local passed by reference (like the bare lastChangeMs it replaces) rather than a
// file-scope static, so the law stays a function of its arguments with no hidden globals
// and the schedule-override path can reset it explicitly.
struct ControlLoopState {
    uint32_t lastChangeMs;      // start of the blanking window
    bool     lastChangeWasDown; // gates the fast path during blanking - see the sign
                                // argument in compute_next_target_amps()
    float    surplusSumW;       // averaging accumulator; blanking samples never enter it
    uint16_t sampleCount;
    bool     batteryValidAll;   // AND across the window, not just the newest sample
};

static void control_state_reset(ControlLoopState &st, uint32_t nowMs) {
    st.lastChangeMs = nowMs;
    st.lastChangeWasDown = false;
    st.surplusSumW = 0.0f;
    st.sampleCount = 0;
    st.batteryValidAll = true;
}

// Shared tail for both paths that actually reach a decision (the averaged one and the
// fast drop): floor/hysteresis banding, deadband, accumulator reset, and the bookkeeping
// that must happen exactly once per decision. Factored out so the two can't drift apart.
static ControlDecision finish_decision(float currentTargetA, float rawTargetA,
                                         uint32_t nowMs, ControlLoopState &st,
                                         float &outTargetA, bool fastDrop) {
    rawTargetA = constrain(rawTargetA, 0.0f, MAX_CURRENT_A);

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

    // Deadband, applied AFTER the banding above so it can never suppress a floor crossing
    // (those move the target by at least MIN_CURRENT_A). A sub-threshold correction must
    // not restart the blanking window either - a 0.05A nudge the clamp would ignore
    // outright used to cost a full settle window of responsiveness.
    if (fabsf(newTargetA - currentTargetA) < s_tuning.deadband_a) {
        newTargetA = currentTargetA;
    }

    // A decision that actually moved the target invalidates every sample taken at the old
    // operating point, and an averaged decision has consumed its window either way - so both
    // reset. The one case that must NOT is a fast-path check that changed nothing: the
    // banding above can compress a cut back below the deadband (e.g. 6.2A with a 1.0A
    // fast-drop lands in the hysteresis band and snaps back to 6.0A, a 0.2A move), and with
    // sustained import that repeats every poll - resetting there would restart the averaging
    // window forever and leave the target stuck while importing.
    if (!fastDrop || newTargetA != currentTargetA) {
        st.surplusSumW = 0.0f;
        st.sampleCount = 0;
        st.batteryValidAll = true;
    }

    ControlDecision decision;
    if (newTargetA != currentTargetA) {
        // Only an accepted CHANGE restarts blanking: a hold has no plant transient to wait
        // out, so an idle loop simply re-decides every avg_samples polls.
        st.lastChangeMs = nowMs;
        st.lastChangeWasDown = (newTargetA < currentTargetA);
        decision = fastDrop ? DECISION_FAST_DROP
                 : (newTargetA > currentTargetA) ? DECISION_STEP_UP
                 : DECISION_STEP_DOWN;
    } else if (newTargetA >= MAX_CURRENT_A) {
        decision = DECISION_CAPPED_MAX;
    } else if (newTargetA <= 0.0f) {
        decision = DECISION_CAPPED_ZERO;
    } else {
        decision = DECISION_HOLD;
    }

    outTargetA = newTargetA;
    return decision;
}

// Damped, averaged integration from surplus power to target current.
//
// The EV's own draw is part of what the meter measures, so this is a closed loop against a
// plant that answers late. At gain 1 with one decision-period of unabsorbed lag the error
// recurrence is e[n+1] = e[n] - e[n-1], whose poles sit exactly ON the unit circle: an
// undamped ring with a period of six decisions, which is what showed up as the charge
// current oscillating on cloudy days. Damping the correction to gain g moves those poles
// to magnitude sqrt(g), and because the law is still an integrator the steady-state error
// stays zero - large errors still move far, so this is NOT the per-step rate cap that was
// removed earlier. See CLAUDE.md "Control-loop damping".
static ControlDecision compute_next_target_amps(float currentTargetA, float gridPowerW,
                                                  uint32_t nowMs, ControlLoopState &st,
                                                  float &outTargetA, float mainsVoltageV,
                                                  bool batteryDataValid) {
    const float surplusW = -gridPowerW; // positive = exporting
    const float instantDeltaA = surplusW / mainsVoltageV;
    const bool blanking = (nowMs - st.lastChangeMs) < s_tuning.blank_ms;

    // Accumulated before the fast-path check below, not after, so a sample still counts
    // toward the mean on a poll where the fast path also looks at it. Otherwise a fast-path
    // check that ends up changing nothing would consume the poll without advancing the
    // averaging window, and sustained import could stall the loop entirely.
    if (!blanking) {
        st.surplusSumW += surplusW;
        st.sampleCount++;
        st.batteryValidAll = st.batteryValidAll && batteryDataValid;
    }

    // --- Fast path: real import cuts now, full gain, no waiting for the mean ------------
    // Armed during the blanking window that follows an UP change: there the un-ramped car
    // makes the meter read MORE export than reality, so an import reading understates the
    // deficit and a full-gain cut is an under-cut - safe. Suppressed during the window
    // after a DOWN change: there the residual import IS our own previous cut, not yet
    // absorbed, and acting on it is the exact double-count this function exists to avoid -
    // it would cut through the floor and open the CP disconnect relay. Gated on a nonzero
    // target because there is nothing left to cut at 0A, and an evening's house load alone
    // (800W = 3.3A) would otherwise fire this every poll and starve the averaging window
    // forever.
    if (currentTargetA > 0.0f && !(blanking && st.lastChangeWasDown) &&
        instantDeltaA <= -s_tuning.fast_drop_a) {
        return finish_decision(currentTargetA, currentTargetA + instantDeltaA,
                               nowMs, st, outTargetA, true);
    }

    if (blanking) {
        // Discarded, NOT accumulated. These readings still show the plant's previous
        // operating point, so averaging them in would bias the mean toward the pre-ramp,
        // apparently-larger surplus - making the oscillation worse rather than better.
        outTargetA = currentTargetA;
        return DECISION_BLANKING;
    }

    // --- Averaging phase ----------------------------------------------------------------
    // The sample was accumulated above. The target is constant for the whole of this window,
    // so every sample measures the same operating point and the only variance left is
    // genuine PV/house variance - which makes the mean unbiased. Counting samples rather
    // than milliseconds means a failed Modbus poll delays the decision instead of shrinking
    // the sample set.
    if (st.sampleCount < s_tuning.avg_samples) {
        outTargetA = currentTargetA;
        return DECISION_AVERAGING;
    }

    float deltaA = (st.surplusSumW / (float)st.sampleCount) / mainsVoltageV;

    // Damp increases only, and only once already charging. At target 0 the EV contributes
    // nothing to the meter, so this reading is the true supportable current rather than an
    // error term carrying our own lag - damping it would move the effective entry
    // threshold from MIN_CURRENT_A to MIN_CURRENT_A/gain_up (6A -> 8.6A at 0.7) and stop
    // charging ever starting in that band.
    if (deltaA > 0.0f && currentTargetA >= MIN_CURRENT_A) {
        deltaA *= s_tuning.gain_up;
    }
    // gridPowerW is already battery-discharge-excluded (see call site), but that exclusion
    // is only trustworthy when the battery reading behind it is fresh. When it isn't, never
    // increase the target on unverified data - holding or backing off is always safe, but an
    // increase could secretly be drawing on undetected battery discharge. ANDed across the
    // whole window, not just the newest sample: one untrusted reading anywhere in it must
    // forbid an averaged increase, or smoothing would walk straight past this guard. Kept
    // last so it stays unconditionally final regardless of the gain above.
    if (!st.batteryValidAll) {
        deltaA = fminf(deltaA, 0.0f);
    }

    return finish_decision(currentTargetA, currentTargetA + deltaA,
                           nowMs, st, outTargetA, false);
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
static uint16_t s_last_sample_count = 0;

// Set by the handlers that change what the control loop is looking at (the grid source, or
// the tuning values themselves) and consumed once by the poll loop. A half-full averaging
// accumulator would otherwise span two different plants, or hold more samples than the new
// avg_samples asks for, and produce one wrong decision. Matters in practice because
// switching sources mid-run is how the damping gets bench-tested.
static bool s_control_reset_requested = false;

// Server-side power history for the control page's chart - a fixed ring of 5-minute
// buckets (not raw per-poll samples) so every browser session sees the same rolling 24h
// window instead of each tab reconstructing its own from /api/status polls. Persisted to
// LittleFS (history_load_from_fs()/history_save_to_fs() below) so it survives reboots and
// reflashes - the buffer is loaded once at task start and the whole file is rewritten each
// time a bucket completes (~288 writes/day), relying on LittleFS's wear leveling across the
// otherwise-unused 1.5MB filesystem partition rather than a more complex append-only journal.
// Only ever touched from solar_control_task (poll loop writes, HTTP handler reads), so no
// mutex.
struct HistoryBucket {
    uint32_t start_epoch_s;
    float target_sum_w;
    float grid_sum_w;
    uint16_t sample_count;
};
static HistoryBucket s_history[HISTORY_BUCKET_CAPACITY];
static size_t s_history_created = 0; // total buckets ever started (monotonic)

struct HistoryFileHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t history_created;
};

// Loads a previously-persisted history buffer, if any. Any failure (no filesystem, missing
// file, wrong size, magic/version mismatch from an older firmware's HistoryBucket layout) is
// treated as "no history" and silently leaves s_history/s_history_created at their
// zero-initialized defaults - never blocks or fails boot over this.
static void history_load_from_fs() {
    if (!LittleFS.begin(true)) {
        return;
    }
    File f = LittleFS.open(HISTORY_FILE_PATH, "r");
    if (!f) {
        return;
    }
    HistoryFileHeader header;
    size_t expectedSize = sizeof(header) + sizeof(s_history);
    if (f.size() != expectedSize ||
        f.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) != sizeof(header) ||
        header.magic != HISTORY_FILE_MAGIC || header.version != HISTORY_FILE_VERSION) {
        f.close();
        return;
    }
    if (f.read(reinterpret_cast<uint8_t *>(s_history), sizeof(s_history)) != sizeof(s_history)) {
        f.close();
        // Partial/corrupt read - don't trust a half-populated buffer.
        memset(s_history, 0, sizeof(s_history));
        return;
    }
    f.close();
    s_history_created = header.history_created;
}

// Overwrites the persisted history file with the current buffer. Called only from the
// bucket-rollover path (~once per 5 minutes), not per-poll, to keep flash writes infrequent.
static void history_save_to_fs() {
    File f = LittleFS.open(HISTORY_FILE_PATH, "w");
    if (!f) {
        return;
    }
    HistoryFileHeader header;
    header.magic = HISTORY_FILE_MAGIC;
    header.version = HISTORY_FILE_VERSION;
    header.history_created = (uint32_t)s_history_created;
    f.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    f.write(reinterpret_cast<const uint8_t *>(s_history), sizeof(s_history));
    f.close();
}

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

// WebServer sends no cache headers of its own (its _prepareHeader emits only the status
// line, Content-Type, Content-Length and Connection), so without this browsers fall back
// to heuristic freshness and can keep serving a cached PAGE_HTML indefinitely after a
// reflash - old JS talking to a new /api/status, with nothing on screen to say so. This
// is also what makes the page's build_id auto-reload (see PAGE_HTML) actually converge:
// a reload that got handed the same cached shell would just loop.
//
// _responseHeaders is consumed and cleared per response, so this has to be called from
// each handler rather than once at setup.
static void send_no_cache_headers() {
    s_web_server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
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
/* Width is coupled to the grid below: the four settings cards only form four columns if
   the content box fits 4x260px plus three 16px gaps = 1088px, so this must stay at or
   above 1088 + 40 of padding. Narrowing it silently drops back to a 3-column layout. */
.wrap{max-width:1180px;margin:0 auto;padding:24px 20px 60px}
.topbar{display:flex;align-items:center;justify-content:space-between;margin-bottom:20px;flex-wrap:wrap;gap:10px}
.topbar h1{font-size:1.3rem;margin:0}
/* auto-fit, so the four cards sit in one row on a desktop and reflow to 3/2/1 columns on
   narrower viewports without any media query. See .wrap's max-width above. */
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px}
.card{background:var(--bg-elev);border:1px solid var(--border);border-radius:var(--radius);
  padding:20px;box-shadow:var(--shadow)}
.card h2{margin:0 0 14px;font-size:.72rem;font-weight:700;letter-spacing:.06em;
  text-transform:uppercase;color:var(--text-muted)}
.card-settings{border-style:dashed}
.stat-value{font-size:2.6rem;font-weight:700;line-height:1.1;letter-spacing:-.02em}
.stat-value-sub{font-size:1.1rem;font-weight:500;color:var(--text-muted)}
.stat-sub{color:var(--text-muted);margin-top:4px;font-size:.9rem}
.foot{color:var(--text-muted);margin-top:24px;font-size:.8rem;text-align:center}
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
        <div class="stat-sub" title="The ceiling offered to the car via CP - it may draw less">Target (ceiling)</div>
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
        <div id="batteryStaleNote" class="stat-sub" style="display:none;color:var(--bad)">Battery reading unavailable, stale, or of unknown direction - target amps on hold (can decrease, won't increase) until it's confirmed fresh</div>
        <div id="staleWarn" class="banner banner-warn" style="display:none;margin:12px 0 0">
          Data is stale &mdash; CP will fail safe to native pass-through.
        </div>
      </section>

      <!-- Control-loop damping against measurement lag. Tunable here rather than compiled
           in because the right values depend on this install's inverter and vehicle lag,
           which can only be measured against real weather. Labels are kept short so they
           don't wrap in a narrow grid column - the tooltips carry the full meaning. -->
      <section class="card card-settings">
        <h2>Control tuning</h2>
        <div class="field">
          <label for="tuneBlank" title="Readings this soon after a change are discarded - the meter and the car haven't caught up yet">Blanking (s)</label>
          <input type="number" id="tuneBlank" min="0" max="120" step="0.5">
        </div>
        <div class="field">
          <label for="tuneAvgN" title="Readings averaged together before each decision, instead of acting on one sample">Samples per decision</label>
          <input type="number" id="tuneAvgN" min="1" max="12" step="1">
        </div>
        <div class="field">
          <label for="tuneGainUp" title="Fraction of the correction applied when increasing. 1.0 = undamped; decreases always use full gain">Gain on increases</label>
          <input type="number" id="tuneGainUp" min="0.1" max="1" step="0.05">
        </div>
        <div class="field">
          <label for="tuneDeadband" title="Changes smaller than this are ignored, so tiny corrections don't restart the timing window">Deadband (A)</label>
          <input type="number" id="tuneDeadband" min="0" max="5" step="0.1">
        </div>
        <div class="field">
          <label for="tuneFastDrop" title="Grid import above this cuts current immediately, without waiting for the average">Fast-drop import (A)</label>
          <input type="number" id="tuneFastDrop" min="0.5" max="32" step="0.5">
        </div>
        <div class="field">
          <button id="tuningSaveBtn" type="button">Save</button>
        </div>
        <div class="stat-sub">Lower gain and more samples damp harder; raise blanking first if the current still oscillates.</div>
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

  <!-- Outside #dashboard on purpose: showError() hides that, and the running
       firmware is still worth showing while the device is unreachable. -->
  <div class="foot" id="buildInfo"></div>
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

  // Same reason as the schedule guard above: the 5s poll must not overwrite a value the
  // user is part-way through typing.
  var TUNING_FIELD_IDS = ['tuneBlank', 'tuneAvgN', 'tuneGainUp', 'tuneDeadband', 'tuneFastDrop'];
  function tuningFieldFocused(){
    return TUNING_FIELD_IDS.indexOf(document.activeElement && document.activeElement.id) !== -1;
  }

  // Populated from /api/history (see fetchHistory below) - the device itself owns this
  // rolling window so every browser session shows the same data, not a per-tab
  // reconstruction from /api/status polls.
  var history = [];

  // build_id of the firmware this page was served by, captured from the first
  // status response. Any later poll reporting a different one means the board
  // has been reprogrammed under us, so this shell is stale - see render().
  var seenBuildId = null;

  function fetchHistory(){
    fetch('/api/history', {cache:'no-store'}).then(function(r){ return r.json(); }).then(function(d){
      history = d.points.map(function(p){
        return {t: p.t * 1000, target: p.target_w, grid: p.grid_w};
      });
      drawChart();
    }).catch(function(e){
      // Keep the last good chart rather than blanking it, but never swallow the reason -
      // an empty catch here once hid a malformed-JSON response (a NaN average) as a
      // silently empty chart with nothing in the console to go on.
      console.error('history fetch/parse failed:', e);
    });
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
    // Before touching anything else: if the firmware changed, this page's JS is
    // older than the API it's talking to, so reload rather than rendering against
    // it. The no-store header on "/" (see send_no_cache_headers) is what stops
    // this looping - without it the reload could be handed the same stale shell.
    if (d.build_id) {
      if (seenBuildId === null) {
        seenBuildId = d.build_id;
      } else if (d.build_id !== seenBuildId) {
        location.reload();
        return;
      }
      $('buildInfo').textContent = 'Firmware ' + d.build_id;
    }

    $('targetWatts').textContent = Math.round(d.target_w) + ' W';
    $('targetAmps').textContent = '(' + d.target_a.toFixed(1) + ' A)';
    // Averaging shows sample progress as well as the estimated countdown, since a failed
    // poll stretches the wall clock but doesn't lose a sample.
    $('decision').textContent = d.decision
      + (d.avg_n > 0 ? ' (' + d.avg_n + '/' + d.avg_of + ' samples)'
        : d.settle_s > 0 ? ' (' + d.settle_s + 's left)' : '');
    $('gridWatts').textContent = Math.abs(d.grid_w).toFixed(0) + ' W ' + (d.exporting ? 'exporting' : 'importing');
    $('gridFlow').className = 'flow-dot ' + (d.exporting ? 'flow-export' : 'flow-import');

    $('batteryWatts').textContent = d.battery_w.toFixed(0) + ' W ' + d.battery_state;
    if (d.surplus_excluded_w > 0) {
      $('batteryExcludedNote').textContent = Math.round(d.surplus_excluded_w) + ' W excluded from surplus - home battery discharging';
      $('batteryExcludedNote').style.display = '';
    } else {
      $('batteryExcludedNote').style.display = 'none';
    }
    $('batteryStaleNote').style.display = d.battery_data_valid ? 'none' : '';

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

    if (!tuningFieldFocused()) {
      $('tuneBlank').value = d.tune_blank_s;
      $('tuneAvgN').value = d.tune_avg_n;
      $('tuneGainUp').value = d.tune_gain_up;
      $('tuneDeadband').value = d.tune_deadband_a;
      $('tuneFastDrop').value = d.tune_fast_drop_a;
    }

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
    $('tuningSaveBtn').addEventListener('click', function(){
      var url = '/api/set_tuning'
        + '?blank_s=' + encodeURIComponent($('tuneBlank').value)
        + '&avg_n=' + encodeURIComponent($('tuneAvgN').value)
        + '&gain_up=' + encodeURIComponent($('tuneGainUp').value)
        + '&deadband_a=' + encodeURIComponent($('tuneDeadband').value)
        + '&fast_drop_a=' + encodeURIComponent($('tuneFastDrop').value);
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
    send_no_cache_headers();
    s_web_server.send_P(200, "text/html", PAGE_HTML);
}

// --- JSON status API ---------------------------------------------------
// File-scope (not a stack local) so a deeply nested call chain (WiFi/OTA/WebServer all run
// in this same task) never has to find room for it on the stack. Only ever holds
// compile-time-constant strings plus a few numbers, never user-controllable text, so no
// JSON-escaping is needed.
//
// Sized against a computed worst case of 2048 bytes: the longest cp_mode_title/
// cp_duty_title/connector_title/decision strings, the longest source name, all 7 registry
// entries, build_id, the avg_n/avg_of averaging progress and the five tune_* values.
// Re-check that arithmetic before adding another field or another grid source -
// build_status_json() truncates SILENTLY, and a truncated body is invalid JSON, which
// makes the page's JSON.parse() throw and blanks the whole dashboard (the same failure
// mode a NaN history bucket once produced).
static char s_json_body[3072];

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
    bool batteryDataValid = s_last_solar_status.battery_data_valid;
    // "unknown" ahead of the sign test so an untrusted reading is never rendered as a
    // confident direction - the direction is derived from a separate register that can
    // fail or read ambiguously on its own (see grid_source_sungrow_winet.cpp).
    const char *batteryStateStr = !batteryDataValid ? "unknown"
                                    : (batteryPowerW < 0.0f) ? "discharging"
                                    : (batteryPowerW > 0.0f) ? "charging" : "idle";
    // Both waiting phases feed the same countdown the page already renders. The averaging
    // figure is an estimate by design: it assumes every remaining poll succeeds, and a
    // failed one legitimately stretches the window rather than shrinking the sample set.
    uint32_t settleRemainingS = 0;
    if (s_last_decision == DECISION_BLANKING) {
        uint32_t elapsed = nowMs - s_last_lastChangeMs;
        settleRemainingS = (elapsed < s_tuning.blank_ms) ? (s_tuning.blank_ms - elapsed) / 1000 : 0;
    } else if (s_last_decision == DECISION_AVERAGING && s_last_sample_count < s_tuning.avg_samples) {
        settleRemainingS = (uint32_t)(s_tuning.avg_samples - s_last_sample_count) * (POLL_INTERVAL_MS / 1000);
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
            ? "Clamping the CP line every cycle to hold the target duty cycle (charging)"
            : "CP relay open - surplus below the 6A floor, vehicle isolated, not charging";
    const char *cpDutyLabel = (cp.mode != MODE_ACTIVE)
        ? "N/A"
        : (cp.duty_state == CP_OSCILLATING) ? "Clamping CP" : "CP Disconnected";
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
        "\"battery_data_valid\":%s,"
        "\"surplus_excluded_w\":%.0f,"
        "\"settle_s\":%lu,"
        "\"avg_n\":%u,"
        "\"avg_of\":%u,"
        "\"tune_blank_s\":%.1f,"
        "\"tune_avg_n\":%u,"
        "\"tune_gain_up\":%.2f,"
        "\"tune_deadband_a\":%.2f,"
        "\"tune_fast_drop_a\":%.2f,"
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
        // Identifies the running firmware. The page compares this across polls and
        // reloads itself when it changes (see PAGE_HTML), and shows it in the footer.
        "\"build_id\":\"%s\","
        "\"sources\":[",

        decision_reason_str(s_last_decision),
        fabsf(surplusW),
        surplusW >= 0 ? "true" : "false",
        fabsf(batteryPowerW),
        batteryStateStr,
        batteryDataValid ? "true" : "false",
        surplusExcludedW,
        (unsigned long)settleRemainingS,
        (unsigned)s_last_sample_count,
        (unsigned)s_tuning.avg_samples,
        (float)s_tuning.blank_ms / 1000.0f,
        (unsigned)s_tuning.avg_samples,
        s_tuning.gain_up,
        s_tuning.deadband_a,
        s_tuning.fast_drop_a,
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
        timeSyncedForStatus ? "true" : "false",
        firmware_build_id());

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
    send_no_cache_headers();
    s_web_server.send(200, "application/json", build_status_json());
}

static void handle_api_set_source() {
    if (!require_auth()) {
        return;
    }
    send_no_cache_headers();
    if (s_web_server.hasArg("id")) {
        s_active_grid_source = &grid_data_source_lookup(s_web_server.arg("id").c_str());
        // Persist the resolved id, not the raw request arg - an unrecognized
        // id in the request self-heals to the default rather than saving a
        // value that would need re-resolving (with a fallback) on next boot.
        save_grid_source_to_nvs(s_active_grid_source->id);
        // The plant the control loop is closing around just changed; don't average the
        // new source's readings together with the old one's.
        s_control_reset_requested = true;
    }
    s_web_server.send(200, "application/json", build_status_json());
}

// Control-loop damping tuning. Same leave-prior-value-on-bad-input pattern as
// handle_api_set_schedule() below - a malformed value is ignored rather than rejecting the
// whole request. Every field is clamped by control_tuning_clamp(), so no argument here can
// put the loop outside its stable range regardless of what is sent.
static void handle_api_set_tuning() {
    if (!require_auth()) {
        return;
    }
    send_no_cache_headers();
    if (s_web_server.hasArg("blank_s")) {
        s_tuning.blank_ms = (uint32_t)(s_web_server.arg("blank_s").toFloat() * 1000.0f);
    }
    if (s_web_server.hasArg("avg_n")) {
        s_tuning.avg_samples = (uint16_t)s_web_server.arg("avg_n").toInt();
    }
    if (s_web_server.hasArg("gain_up")) {
        s_tuning.gain_up = s_web_server.arg("gain_up").toFloat();
    }
    if (s_web_server.hasArg("deadband_a")) {
        s_tuning.deadband_a = s_web_server.arg("deadband_a").toFloat();
    }
    if (s_web_server.hasArg("fast_drop_a")) {
        s_tuning.fast_drop_a = s_web_server.arg("fast_drop_a").toFloat();
    }
    control_tuning_clamp(s_tuning);
    save_control_tuning_to_nvs();
    // A shrunk avg_samples could otherwise leave the accumulator holding more samples than
    // the new window wants.
    s_control_reset_requested = true;
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
    send_no_cache_headers();
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
    bool anyEmitted = false;
    for (size_t i = 0; i < count && offset < sizeof(s_history_json_body); i++) {
        const HistoryBucket &b = s_history[(oldest + i) % HISTORY_BUCKET_CAPACITY];
        // A bucket can legitimately hold no samples - e.g. a persisted buffer restored from
        // flash whose newest bucket never received a reading before the reboot. Averaging it
        // divides by zero and prints "nan", which is NOT valid JSON, so a single such bucket
        // makes the browser's JSON.parse() throw and blanks the entire chart. Skip it: a gap
        // reads honestly, exactly as an outage already does.
        if (b.sample_count == 0) {
            continue;
        }
        offset += snprintf(s_history_json_body + offset, sizeof(s_history_json_body) - offset,
            "%s{\"t\":%lu,\"target_w\":%.0f,\"grid_w\":%.0f}",
            anyEmitted ? "," : "",
            (unsigned long)b.start_epoch_s,
            b.target_sum_w / b.sample_count,
            b.grid_sum_w / b.sample_count);
        anyEmitted = true;
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
    send_no_cache_headers();
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
    s_web_server.on("/api/set_tuning", HTTP_GET, handle_api_set_tuning);
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
    load_control_tuning_from_nvs();
    history_load_from_fs();

    connect_wifi(cfg);
    register_network_services();

    IPAddress inverterIp;
    if (cfg.provisioned) {
        inverterIp.fromString(cfg.inverter_ip);
    }

    float targetAmps = 0.0f;
    ControlLoopState ctl;
    control_state_reset(ctl, millis());
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

            // The grid source or the tuning values changed under us since the last poll -
            // start a clean window rather than deciding on a part-full accumulator.
            if (s_control_reset_requested) {
                s_control_reset_requested = false;
                control_state_reset(ctl, now);
            }

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
            // Carried forward alongside battery_data_valid below, rather than reset - a
            // dropped poll doesn't mean the battery went idle, and zeroing it here would
            // render a confident "0 W idle" while validity still said the reading was good.
            status.battery_power_w = s_last_solar_status.battery_power_w;
            status.battery_data_valid = s_last_solar_status.battery_data_valid;
            status.last_poll_success_ms = lastPollSuccessMs;
            status.schedule_active = scheduleActive;
            // Carried forward (not reset to 0) when inactive, so it ages
            // past STALE_DATA_TIMEOUT_MS on its own once the window ends -
            // see the field's comment in shared_state.h.
            status.last_schedule_confirm_ms = scheduleActive ? millis() : s_last_solar_status.last_schedule_confirm_ms;

            if (scheduleActive) {
                targetAmps = constrain(s_schedule.amps, MIN_CURRENT_A, MAX_CURRENT_A);
                shared_state_set_target_amps(targetAmps);
                // Fresh blanking window the moment the schedule ends - targetAmps was just
                // set from the schedule, not derived from a surplus reading, so there is no
                // accumulated sample worth keeping either. Marked as a downward change
                // because the schedule may just have imposed a large cut (a fixed 6A after
                // solar had been running at 20A), which is a down-transient like any other:
                // that suppresses the fast path for one blanking window on exit, so the
                // loop doesn't act on its own not-yet-absorbed reduction.
                control_state_reset(ctl, now);
                ctl.lastChangeWasDown = true;
                s_last_decision = DECISION_SCHEDULE_OVERRIDE;
            }

            if (status.wifi_connected) {
                float gridPowerW;
                float voltageV;
                float batteryPowerW;
                bool batteryDataValid;
                // Derived from the previous poll's voltage - this poll's own voltage isn't
                // known until the read call below returns it; negligible staleness.
                float currentDrawW = targetAmps * s_last_mains_voltage_v;
                if (s_active_grid_source->read_power_w(inverterIp, currentDrawW, gridPowerW, voltageV, batteryPowerW, batteryDataValid)) {
                    status.modbus_ok = true;
                    status.grid_power_w = gridPowerW;
                    status.battery_power_w = batteryPowerW;
                    status.battery_data_valid = batteryDataValid;
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
                        // millis() rather than the loop-top `now`, which was taken before a
                        // Modbus read that can block for seconds - blanking is a
                        // physical-time guard, so it should be measured from the moment the
                        // decision is actually made.
                        float prevTargetA = targetAmps;
                        s_last_decision = compute_next_target_amps(targetAmps, effectiveGridPowerW, millis(), ctl, targetAmps, voltageV, batteryDataValid);
                        shared_state_set_target_amps(targetAmps);
                        // The only per-poll trace in this file. Bench-tuning blank_ms
                        // otherwise has to be done through the page's 5s poll, which is too
                        // coarse to see a decision land.
                        Serial.printf("control: grid=%.0fW eff=%.0fW batv=%d n=%u/%u %.2fA -> %.2fA (%s)\n",
                            gridPowerW, effectiveGridPowerW, batteryDataValid ? 1 : 0,
                            (unsigned)ctl.sampleCount, (unsigned)s_tuning.avg_samples,
                            prevTargetA, targetAmps, decision_reason_str(s_last_decision));
                    }
                }
            }

            shared_state_publish_solar_status(status);
            s_last_solar_status = status;
            s_last_target_amps = targetAmps;
            s_last_lastChangeMs = ctl.lastChangeMs;
            s_last_sample_count = ctl.sampleCount;

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
                // Persisted only after the first sample lands, never on the bare bucket -
                // saving in between leaves a zero-sample bucket in the file that a reboot
                // restores and that can never be filled afterwards (its window has passed).
                if (needNewBucket) {
                    history_save_to_fs();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
