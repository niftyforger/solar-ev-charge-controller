#include "cp_interceptor.h"
#include "config.h"
#include "shared_state.h"

#include <Arduino.h>
#include <driver/rmt.h>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_intr_alloc.h>
#include <esp_system.h>

// Low-level (register-only, force-inlined) HAL headers, used only inside the
// two hot-path ISRs (cp_sense_isr/assert_timer_isr and the
// schedule_clamp_ticks() helper they call) - those run with
// ESP_INTR_FLAG_IRAM (see cp_hw_init()) and so cannot call the flash-resident
// driver/gpio.h, driver/timer.h, driver/rmt.h APIs used everywhere else.
#include <hal/gpio_ll.h>
#include <hal/rmt_ll.h>
#include <hal/timer_ll.h>

// Legacy hardware timer group used to schedule the clamp-assert instant.
// Fires assert_timer_isr() directly in interrupt context - see the comment
// above assert_timer_isr() for why this replaced an esp_timer one-shot.
#define CP_CLAMP_TIMER_GROUP    TIMER_GROUP_0
#define CP_CLAMP_TIMER_IDX      TIMER_0

// --- Task ("planning") -> ISR ("execution") handoff, all Core 1-local ---
// cp_interceptor_task recomputes these on its own cadence (CP_NOTIFY_WAIT_MS)
// from targetAmps/nativePeriodUs/nativeHighUs, doing all the floating-point
// duty/timing math. cp_sense_isr() just applies them - see cp_sense_isr()'s
// comment for why no floating point runs there. Write order on the task side
// is ticks/clampEnabled first, dutyState last, so the ISR can never observe
// dutyState==CP_OSCILLATING paired with stale/zero clamp parameters.
static volatile SystemMode s_mode = MODE_BYPASS;
static volatile CpDutyState s_dutyState = CP_STANDBY;
static volatile bool s_clampEnabled = false;
static volatile uint32_t s_assertOffsetTicks = 0; // hw timer ticks, 1us each
static volatile uint32_t s_holdTicksRmt = 0;       // RMT ticks, 0.1us each

// --- ISR -> task direction: opportunistically-learned native waveform ---
// Updated only by cp_sense_isr() (pure integer arithmetic on edge
// timestamps); read by the task each planning pass.
static volatile uint32_t s_nativePeriodUs = 1000; // 1kHz nominal until measured
static volatile uint32_t s_nativeHighUs = 0;
static volatile bool s_nativeHighKnown = false;

// ISR-internal only - touched solely within cp_sense_isr()'s own sequential
// execution (GPIO ISR isn't reentrant here), so these don't need volatile.
static int64_t s_lastRiseUs = -1;
static bool s_clampedThisCycle = false;

// Truncated low 32 bits of the last rising-edge timestamp, published
// volatile for read_connector_state() (task context, ~CP_STATUS_PUBLISH_MS
// cadence) to check activity recency against CP_ACTIVITY_TIMEOUT_US.
// Separate from s_lastRiseUs (int64_t, ISR-internal only) rather than making
// that one volatile: a 64-bit value isn't atomically read/written on this
// 32-bit core, and a plain 32-bit microsecond delta is all a few-millisecond
// recency check needs - same reasoning as s_nativePeriodUs/s_nativeHighUs
// above.
static volatile uint32_t s_lastRiseUs32 = 0;

// Set by schedule_clamp_ticks() just before arming the alarm, read back by
// assert_timer_isr() when it fires. Single in-flight clamp at a time, same
// as the RMT channel itself only ever holds one pending waveform.
static volatile uint32_t s_pendingHoldTicks = 0;

static bool IRAM_ATTR assert_timer_isr(void *arg);
static void IRAM_ATTR schedule_clamp_ticks(uint32_t assertOffsetTicks, uint32_t holdTicksRmt);

// Rising edge: measure the native period (pure integer), then - if the
// task's last planning pass says to clamp this cycle - arm the hardware
// timer immediately, right here in interrupt context. This is the entire
// timing-critical path now: no queue, no task wakeup, nothing between the
// real GPIO transition and the timer being armed. Falling edge: opportunistic
// native high-time learning, same rule as before (skip cycles we clamped,
// since that edge is our own MOSFET, not the EVSE's).
//
// CP_PWM_SENSE_GPIO's comparator/opto stage is inverted from the naive
// assumption (same as CP_CONNECTED_SENSE_GPIO): raw GPIO reads LOW during
// CP's positive lobe, HIGH during the negative lobe. So the real rising edge
// is level==0 and the real falling edge is level==1 below - get this wrong
// and the clamp timer arms off the real falling edge instead, asserting into
// the negative half-cycle (bench-reproduced pre-fix; see CLAUDE.md's
// "Resolved" section).
//
// Deliberately does ZERO floating-point/double arithmetic. ESP32-S3's Xtensa
// FreeRTOS port saves/restores the floating-point coprocessor lazily, per
// task, on context switch - not on arbitrary interrupt entry/exit. Using
// hardware FP here could silently corrupt in-flight FP state belonging to
// whatever this ISR interrupts (cp_interceptor_task's own double math, most
// likely), an intermittent, hard-to-trace corruption bug rather than a clean
// crash. All duty%/timing-offset math (which genuinely needs doubles) is
// done in cp_interceptor_task instead, at its own cadence, producing plain
// integer tick counts this ISR just applies.
static void IRAM_ATTR cp_sense_isr(void *arg) {
    (void)arg;
    int64_t nowUs = esp_timer_get_time();
    // Raw register read, not gpio_get_level(): this ISR now runs with
    // ESP_INTR_FLAG_IRAM (see cp_hw_init()), so it must never call into
    // flash-resident driver code - see the "fourth deviation" comment below.
    int level = gpio_ll_get_level(&GPIO, (gpio_num_t)CP_PWM_SENSE_GPIO);

    if (level == 0) { // real rising edge - see polarity note above
        if (s_lastRiseUs > 0) {
            int64_t period = nowUs - s_lastRiseUs;
            // NATIVE_PERIOD_MIN_US/MAX_US are compile-time double literals;
            // the (int64_t) casts fold at compile time, so this stays a
            // pure integer comparison at runtime.
            if (period > (int64_t)NATIVE_PERIOD_MIN_US && period < (int64_t)NATIVE_PERIOD_MAX_US) {
                s_nativePeriodUs = (uint32_t)period;
            }
        }
        s_lastRiseUs = nowUs;
        s_lastRiseUs32 = (uint32_t)nowUs;

        s_clampedThisCycle = false;

        if (s_mode == MODE_ACTIVE && s_dutyState == CP_OSCILLATING && s_clampEnabled) {
            schedule_clamp_ticks(s_assertOffsetTicks, s_holdTicksRmt);
            s_clampedThisCycle = true;
        }
    } else { // real falling edge - see polarity note above
        if (!s_clampedThisCycle && s_lastRiseUs > 0) {
            int64_t high = nowUs - s_lastRiseUs;
            if (high > 0 && high < (int64_t)s_nativePeriodUs) {
                s_nativeHighUs = (uint32_t)high;
                s_nativeHighKnown = true;
            }
        }
    }
}

static void cp_hw_init() {
    gpio_config_t senseConf = {};
    senseConf.pin_bit_mask = (1ULL << CP_PWM_SENSE_GPIO);
    senseConf.mode = GPIO_MODE_INPUT;
    senseConf.pull_up_en = GPIO_PULLUP_DISABLE;
    senseConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    senseConf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&senseConf);

    // ESP_INTR_FLAG_IRAM: safe here because this ISR and everything it calls
    // (schedule_clamp_ticks()) are raw register access only (hal/gpio_ll.h,
    // hal/timer_ll.h) - no flash-resident code remains on the path for the
    // flag to expose. Same reasoning applies to the clamp timer ISR below.
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t)CP_PWM_SENSE_GPIO, cp_sense_isr, nullptr);

    rmt_config_t rmtConf = {};
    rmtConf.rmt_mode = RMT_MODE_TX;
    rmtConf.channel = CP_RMT_TX_CHANNEL;
    rmtConf.gpio_num = (gpio_num_t)CLAMP_DRIVE_GPIO;
    rmtConf.clk_div = RMT_CLK_DIV;
    rmtConf.mem_block_num = 1;
    rmtConf.tx_config.loop_en = false;
    rmtConf.tx_config.carrier_en = false;
    rmtConf.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    rmtConf.tx_config.idle_output_en = true;
    rmtConf.tx_config.idle_level = RMT_IDLE_LEVEL_LOW; // clamp released = safe default
    // NOT setting RMT_CHANNEL_FLAGS_AWARE_DFS: this build has no CONFIG_PM_ENABLE
    // (stock Arduino-ESP32, no sdkconfig override), so the driver's PM-lock
    // acquisition for that flag has nothing to attach to - caused a hard
    // fault / boot loop on real hardware when tried. rmt_set_source_clk()
    // below still pins APB as the source; DFS isn't active in this build
    // anyway, so the flag isn't needed to keep the 10MHz tick assumption valid.
    rmt_config(&rmtConf);
    rmt_driver_install(CP_RMT_TX_CHANNEL, 0, 0);
    rmt_set_source_clk(CP_RMT_TX_CHANNEL, RMT_BASECLK_APB); // pin the clock the RMT tick math assumes

    // Free-running hardware timer, alarm-triggered to fire assert_timer_isr()
    // directly in interrupt context at the intended assert instant.
    // Supersedes an esp_timer one-shot, which dispatched via a FreeRTOS task
    // wakeup (ESP_TIMER_ISR dispatch isn't available in this build) and
    // carried real jitter (110-338us, bench-measured) since that task isn't
    // core-pinned and could land on Core 0 while it's busy with WiFi/Modbus/
    // OTA. One-time setup here (timer_init() etc.) stays on the regular
    // driver/timer.h API since it's task-context, not time-critical; the
    // ESP_INTR_FLAG_IRAM alloc flag below is safe for the same reason as the
    // gpio_install_isr_service call above.
    timer_config_t clampTimerConfig = {};
    clampTimerConfig.alarm_en = TIMER_ALARM_DIS;
    clampTimerConfig.counter_en = TIMER_PAUSE;
    clampTimerConfig.intr_type = TIMER_INTR_LEVEL;
    clampTimerConfig.counter_dir = TIMER_COUNT_UP;
    clampTimerConfig.auto_reload = TIMER_AUTORELOAD_DIS;
    clampTimerConfig.divider = 80; // 80MHz APB / 80 = 1MHz -> 1us/tick
    timer_init(CP_CLAMP_TIMER_GROUP, CP_CLAMP_TIMER_IDX, &clampTimerConfig);
    timer_set_counter_value(CP_CLAMP_TIMER_GROUP, CP_CLAMP_TIMER_IDX, 0);
    timer_isr_callback_add(CP_CLAMP_TIMER_GROUP, CP_CLAMP_TIMER_IDX, assert_timer_isr, nullptr, ESP_INTR_FLAG_IRAM);
    timer_start(CP_CLAMP_TIMER_GROUP, CP_CLAMP_TIMER_IDX);

    gpio_config_t connectedConf = {};
    connectedConf.pin_bit_mask = (1ULL << CP_CONNECTED_SENSE_GPIO);
    connectedConf.mode = GPIO_MODE_INPUT;
    connectedConf.pull_up_en = GPIO_PULLUP_DISABLE;
    connectedConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    connectedConf.intr_type = GPIO_INTR_DISABLE; // polled from the task, not interrupt-driven
    gpio_config(&connectedConf);

    // Disconnect relay drive - plain digital output, task-context only (no
    // ISR involvement, unlike CLAMP_DRIVE - the relay has no sub-cycle
    // timing requirement). Explicitly driven low before anything else can
    // run, matching the de-energized/closed/pass-through default described
    // in config.h.
    gpio_config_t relayConf = {};
    relayConf.pin_bit_mask = (1ULL << CP_DISCONNECT_RELAY_GPIO);
    relayConf.mode = GPIO_MODE_OUTPUT;
    relayConf.pull_up_en = GPIO_PULLUP_DISABLE;
    relayConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    relayConf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&relayConf);
    gpio_set_level((gpio_num_t)CP_DISCONNECT_RELAY_GPIO, 0);
}

// Task-context only (called at CP_STATUS_PUBLISH_MS cadence, not from an
// ISR), so plain driver/gpio.h calls are fine here - unlike cp_sense_isr()/
// assert_timer_isr(), nothing here is IRAM-restricted.
//
// Bench-confirmed 2026-08-30: CP_CONNECTED_SENSE_GPIO reads LOW when
// CONNECTED_IN's comparator (U2B) sees SENSE above REF_CONN, i.e. state A
// (~12V, no vehicle); HIGH for every connected level (B/C/D/E/F all sit
// below REF_CONN) - inverted from the originally-assumed polarity, per the
// comparator/opto stage actually installed. CP_PWM_SENSE_GPIO is the same
// story (see cp_sense_isr()'s polarity note): raw GPIO reads LOW while the
// real CP line is high, HIGH while it's low - inverted from the originally-
// assumed "HIGH during the driven-high portion" convention, so the
// idle-level check below is inverted to match.
static ConnectorState read_connector_state() {
    bool notConnected = gpio_get_level((gpio_num_t)CP_CONNECTED_SENSE_GPIO) == 0;
    if (notConnected) return CONN_STATE_A;

    // Connected. dutyState is our own clamp *decision*, not a measurement
    // of what the EVSE is actually driving, so it can't tell us whether
    // the line is really oscillating right now - use EDGE_IN activity
    // recency instead (see CP_ACTIVITY_TIMEOUT_US in config.h).
    uint32_t nowUs32 = (uint32_t)esp_timer_get_time();
    bool oscillatingNow = (nowUs32 - s_lastRiseUs32) < (uint32_t)CP_ACTIVITY_TIMEOUT_US;
    if (oscillatingNow) return CONN_STATE_C; // covers C and D - see config.h

    // Connected, no recent EDGE_IN toggling: either B (steady ~9V) or a
    // dead/fault line (E at 0V, F clamped near 0V) - CONNECTED_IN's single
    // threshold can't tell these apart (both read "connected"), but
    // EDGE_IN's idle *level* can, since B's ~9V sits above REF_EDGE while
    // E/F's ~0V sits below it.
    bool edgeIdleHigh = gpio_get_level((gpio_num_t)CP_PWM_SENSE_GPIO) == 0;
    return edgeIdleHigh ? CONN_STATE_B : CONN_STATE_FAULT;
}

// Beginning-of-struct pointer for TIMER_GROUP_0/1, matching CP_CLAMP_TIMER_GROUP.
// TIMER_LL_GET_HW() is a compile-time ternary on a constant here, not a
// runtime branch/lookup.
#define CP_CLAMP_TIMER_HW  (TIMER_LL_GET_HW((int)CP_CLAMP_TIMER_GROUP))

// Fires at the intended assert instant (armed by schedule_clamp_ticks()
// below), directly in interrupt context - no FreeRTOS task wakeup in this
// path.
//
// Registered with ESP_INTR_FLAG_IRAM (see cp_hw_init()), so this and
// cp_sense_isr() keep running even inside a flash-cache-disable window
// (WiFi/NVS activity elsewhere) instead of being deferred until it ends -
// that deferral was the dominant source of dispatch jitter (75-166us,
// bench-measured) in an earlier, non-IRAM design. An IRAM handler cannot
// call flash-resident driver APIs, so every call in this function and in
// schedule_clamp_ticks()/cp_sense_isr() uses the force-inlined, register-only
// hal/*_ll.h equivalents instead of driver/rmt.h and driver/timer.h.
//
// Writes a single RMT item that asserts (HIGH) immediately and releases
// after holdTicks - the hold duration lives in the item's *second* segment
// (duration1), which bench testing showed (under the pre-IRAM design) is
// honored accurately even when re-armed every ~1ms; the *first* segment is
// kept to the minimum 1 tick rather than depended on for timing, since that
// slot was the one found unreliable. A trailing zeroed item marks
// end-of-data for the RMT hardware (mirrors what rmt_write_items() itself
// writes past the caller's items). Idle-level release is still
// hardware-guaranteed (idle_output_en + idle_level=LOW) even if this ISR is
// ever deferred past the native falling edge.
static bool IRAM_ATTR assert_timer_isr(void *arg) {
    (void)arg;
    timer_ll_clear_intr_status(CP_CLAMP_TIMER_HW, CP_CLAMP_TIMER_IDX);
    // One-shot: the alarm stays disabled after firing (auto_reload is off)
    // until schedule_clamp_ticks() arms it again for the next cycle -
    // nothing to re-enable here.

    rmt_item32_t items[2];
    items[0].level0 = 0; // matches idle (released) - minimal, not depended on for timing
    items[0].duration0 = 1;
    items[0].level1 = 1; // assert - the duration that matters, kept in the reliable slot
    items[0].duration1 = s_pendingHoldTicks;
    items[1].val = 0; // end-of-data marker for the RMT hardware

    rmt_ll_tx_reset_pointer(&RMT, CP_RMT_TX_CHANNEL);
    rmt_ll_write_memory(&RMTMEM, CP_RMT_TX_CHANNEL, items, 2, 0);
    rmt_ll_tx_start(&RMT, CP_RMT_TX_CHANNEL);

    return false; // no FreeRTOS task to wake - nothing queued/notified from here
}

// Arms the clamp for holdTicksRmt (RMT ticks, 0.1us each), timed to land
// assertOffsetTicks (hardware timer ticks, 1us each) from now. Pure integer -
// called from cp_sense_isr(), see its comment for why. See assert_timer_isr()
// for why the delay itself is a hardware timer alarm rather than encoded in
// the RMT item, and for why this now uses hal/timer_ll.h instead of
// driver/timer.h.
static void IRAM_ATTR schedule_clamp_ticks(uint32_t assertOffsetTicks, uint32_t holdTicksRmt) {
    s_pendingHoldTicks = holdTicksRmt;

    timer_ll_set_alarm_enable(CP_CLAMP_TIMER_HW, CP_CLAMP_TIMER_IDX, false); // disarm any still-pending previous alarm first
    uint64_t now = 0;
    timer_ll_get_counter_value(CP_CLAMP_TIMER_HW, CP_CLAMP_TIMER_IDX, &now);
    uint64_t alarmVal = now + (uint64_t)assertOffsetTicks;
    timer_ll_set_alarm_value(CP_CLAMP_TIMER_HW, CP_CLAMP_TIMER_IDX, alarmVal);
    timer_ll_set_alarm_enable(CP_CLAMP_TIMER_HW, CP_CLAMP_TIMER_IDX, true);
}

void cp_interceptor_task(void *pvParameters) {
    (void)pvParameters;

    cp_hw_init();
    esp_task_wdt_add(NULL);

    SystemMode mode = MODE_BYPASS;
    CpDutyState dutyState = CP_STANDBY;
    bool bootstrapped = false;
    bool stale = true;
    float targetAmps = 0.0f;
    uint32_t lastDutyStateChangeMs = 0; // dwell-time bookkeeping for the disconnect relay, see RELAY_MIN_DWELL_MS

    int64_t lastStaleCheckUs = 0;
    int64_t lastStatusPublishUs = 0;

    for (;;) {
        float qAmps;
        if (xQueueReceive(g_target_amps_queue, &qAmps, 0) == pdTRUE) {
            targetAmps = qAmps;
            bootstrapped = true;
        }

        int64_t now = esp_timer_get_time();

        if (now - lastStaleCheckUs >= (int64_t)CP_STALE_CHECK_MS * 1000) {
            lastStaleCheckUs = now;
            SolarStatus solar;
            bool have = shared_state_try_get_solar_status(solar);
            // Two independent freshness paths, either of which is enough to
            // stay MODE_ACTIVE: a recently-successful grid-data poll (solar-
            // following), or a recently-confirmed scheduled fixed-current
            // window (see SolarStatus's schedule_active/last_schedule_confirm_ms
            // comment in shared_state.h) - a scheduled charge must not
            // depend on the inverter/Modbus link being up. Only when BOTH
            // are stale does this fall back to native pass-through, same
            // STALE_DATA_TIMEOUT_MS budget and MODE_BYPASS mechanism as
            // before - no new fail-safe timeout.
            bool gridStale = !have || (millis() - solar.last_poll_success_ms > STALE_DATA_TIMEOUT_MS);
            bool scheduleStale = !have || (millis() - solar.last_schedule_confirm_ms > STALE_DATA_TIMEOUT_MS);
            stale = gridStale && scheduleStale;

            // Independent recovery watchdog for a wedged Core 0 - see
            // shared_state_heartbeat_solar_task()'s comment in shared_state.h.
            if (shared_state_solar_task_heartbeat_age_ms() > SOLAR_TASK_HEARTBEAT_TIMEOUT_MS) {
                Serial.println("solar_control_task heartbeat stale - Core 0 appears wedged, restarting");
                Serial.flush();
                esp_restart();
            }
        }

        mode = (bootstrapped && !stale) ? MODE_ACTIVE : MODE_BYPASS;
        s_mode = mode;

        // Planning pass: all floating-point duty/timing math lives here, at
        // task cadence, never in cp_sense_isr(). nativePeriodUs/nativeHighUs
        // are ISR-measured and may be up to one tick (CP_NOTIFY_WAIT_MS)
        // stale here - immaterial, since a real ~1kHz oscillator's period
        // doesn't move meaningfully on that timescale (same tolerance this
        // codebase already accepts for Core 0's multi-second target-amps
        // cadence).
        uint32_t nativePeriodUs = s_nativePeriodUs;
        uint32_t nativeHighUs = s_nativeHighUs;
        bool nativeHighKnown = s_nativeHighKnown;

        // MIN_CURRENT_A (10% duty) is itself a fully valid, spec-legal CP
        // signal, so entry into OSCILLATING uses the bare floor with no
        // extra margin. The hysteresis margin sits only on the *exit* side
        // (don't drop back to STANDBY until target falls meaningfully below
        // the floor), so noise sitting right at MIN_CURRENT_A can't flap the
        // clamp every planning pass - putting the margin on entry instead
        // caused settling exactly at the floor to never leave STANDBY at all
        // (see CLAUDE.md's "Resolved" section).
        //
        // CP_STANDBY while MODE_ACTIVE drives the disconnect relay open (see
        // relayShouldOpen below) rather than releasing the clamp to native
        // pass-through, which would hand the vehicle full uncapped current
        // at exactly the moment surplus was lowest. Opening the relay is
        // additionally gated by its own dwell timer (RELAY_MIN_DWELL_MS)
        // below, since a real disconnect forces a fresh unplug/replug
        // handshake and shouldn't retrigger faster than that can complete.
        CpDutyState prevDutyState = dutyState;
        uint32_t nowMs = (uint32_t)(now / 1000);
        if (mode == MODE_ACTIVE) {
            CpDutyState candidateDutyState =
                (dutyState == CP_OSCILLATING)
                    ? ((targetAmps < MIN_CURRENT_A - HYSTERESIS_A) ? CP_STANDBY : CP_OSCILLATING)
                    : ((targetAmps >= MIN_CURRENT_A) ? CP_OSCILLATING : CP_STANDBY);
            if (candidateDutyState != dutyState && (nowMs - lastDutyStateChangeMs) >= RELAY_MIN_DWELL_MS) {
                dutyState = candidateDutyState;
                lastDutyStateChangeMs = nowMs;
            }
        } else {
            // A fault must engage immediately, never wait out the relay
            // dwell timer - but this doesn't actually move the relay either
            // way, since relayShouldOpen below is unconditionally false
            // whenever mode != MODE_ACTIVE.
            dutyState = CP_STANDBY;
        }

        // Only a fresh, confident MODE_ACTIVE "surplus is genuinely below
        // the floor" decision opens the relay - MODE_BYPASS (stale data, or
        // any other fault) always keeps it closed, favoring continued
        // native-rate charging over guessing during a data fault, same
        // tradeoff already accepted for the clamp itself.
        bool relayShouldOpen = (mode == MODE_ACTIVE && dutyState == CP_STANDBY);
        gpio_set_level((gpio_num_t)CP_DISCONNECT_RELAY_GPIO, relayShouldOpen ? 1 : 0);

        // Native duty was learned against the pre-disconnect waveform; once
        // the relay reopens (below) the EVSE will renegotiate from
        // scratch with the vehicle, quite possibly at a different native
        // duty. Force a fresh learn after reconnect rather than clamping
        // against stale pre-disconnect timing - reuses the existing
        // "clamping skipped until native duty has been learned at least
        // once" gate a few lines down, no new gating logic needed.
        if (prevDutyState == CP_OSCILLATING && dutyState == CP_STANDBY) {
            s_nativeHighKnown = false;
        }

        bool clampEnabled = false;
        uint32_t assertOffsetTicks = 0;
        uint32_t holdTicksRmt = 0;
        float appliedDutyPct = 0.0f;

        if (mode == MODE_ACTIVE && dutyState == CP_OSCILLATING && nativeHighKnown) {
            float targetDutyPct = constrain(targetAmps * CP_DUTY_PCT_PER_AMP, CP_MIN_DUTY_PCT, CP_MAX_DUTY_PCT);
            double nativeDutyPct = (double)nativeHighUs * 100.0 / (double)nativePeriodUs;
            double effectiveDutyPct = min((double)targetDutyPct, nativeDutyPct);

            if (nativeDutyPct - effectiveDutyPct >= MIN_CLAMP_MARGIN_PCT) {
                // assertTargetUs is the real-world instant we want the clamp
                // to visibly assert at; assertOffsetUs pulls the *armed*
                // timer earlier by CLAMP_DISPATCH_LATENCY_US so dispatch
                // delay lands the actual assert back on assertTargetUs. Hold
                // duration must be measured from that same real-world
                // target, not from the already-pulled-earlier assertOffsetUs
                // - the RMT hold is a pure hardware countdown starting at
                // the (compensated) actual assert instant, so re-subtracting
                // the latency here would make the release land
                // CLAMP_DISPATCH_LATENCY_US late, past the native falling
                // edge and into the negative half-cycle.
                double assertTargetUs = (double)nativePeriodUs * effectiveDutyPct / 100.0;
                double assertOffsetUs = assertTargetUs - CLAMP_DISPATCH_LATENCY_US;
                if (assertOffsetUs < 0) assertOffsetUs = 0;
                double holdUs = (double)nativeHighUs - assertTargetUs;
                if (holdUs < 0) holdUs = 0;

                uint32_t ht = (uint32_t)(holdUs * 10.0); // RMT: 0.1us/tick
                if (ht < 1) ht = 1;
                if (ht > 32767) ht = 32767;

                assertOffsetTicks = (uint32_t)assertOffsetUs; // hw timer: 1us/tick
                holdTicksRmt = ht;
                clampEnabled = true;
                appliedDutyPct = (float)effectiveDutyPct;
            } else {
                appliedDutyPct = (float)nativeDutyPct;
            }
        } else if (mode == MODE_ACTIVE && dutyState == CP_OSCILLATING) {
            // Native duty not learned yet - fail toward pass-through rather
            // than guessing a clamp point.
            appliedDutyPct = 0.0f;
        } else {
            appliedDutyPct = nativeHighKnown ? (float)((double)nativeHighUs * 100.0 / (double)nativePeriodUs) : 0.0f;
        }

        // Publish ticks/clampEnabled before dutyState - see the file-level
        // comment on this handoff for why the order matters.
        s_assertOffsetTicks = assertOffsetTicks;
        s_holdTicksRmt = holdTicksRmt;
        s_clampEnabled = clampEnabled;
        s_dutyState = dutyState;

        if (now - lastStatusPublishUs >= (int64_t)CP_STATUS_PUBLISH_MS * 1000) {
            lastStatusPublishUs = now;
            CpStatus status;
            status.mode = mode;
            status.duty_state = dutyState;
            status.connector_state = read_connector_state();
            status.native_duty_pct = nativeHighKnown ? (float)((double)nativeHighUs * 100.0 / (double)nativePeriodUs) : 0.0f;
            status.applied_duty_pct = appliedDutyPct;
            shared_state_try_publish_cp_status(status);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(CP_NOTIFY_WAIT_MS));
    }
}
