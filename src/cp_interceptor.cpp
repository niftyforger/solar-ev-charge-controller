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

// Register-only HAL headers for the two hot-path ISRs (cp_sense_isr/assert_timer_isr and
// schedule_clamp_ticks()) - those run with ESP_INTR_FLAG_IRAM (see cp_hw_init()), so they
// can't call the flash-resident driver/gpio.h, driver/timer.h, driver/rmt.h APIs.
#include <hal/gpio_ll.h>
#include <hal/rmt_ll.h>
#include <hal/timer_ll.h>

// Legacy timer group scheduling the clamp-assert instant; fires assert_timer_isr() directly
// in interrupt context (replaces an earlier esp_timer one-shot - see CLAUDE.md "Resolved").
#define CP_CLAMP_TIMER_GROUP    TIMER_GROUP_0
#define CP_CLAMP_TIMER_IDX      TIMER_0

// Task ("planning") -> ISR ("execution") handoff, Core 1-local. cp_interceptor_task
// recomputes these each cadence (CP_NOTIFY_WAIT_MS); cp_sense_isr() just applies them.
// Written ticks/clampEnabled first, dutyState last, so the ISR never sees CP_OSCILLATING
// paired with stale/zero clamp parameters.
static volatile SystemMode s_mode = MODE_BYPASS;
static volatile CpDutyState s_dutyState = CP_STANDBY;
static volatile bool s_clampEnabled = false;
static volatile uint32_t s_assertOffsetTicks = 0; // hw timer ticks, 1us each
static volatile uint32_t s_holdTicksRmt = 0;       // RMT ticks, 0.1us each

// ISR -> task direction: opportunistically-learned native waveform, updated only by
// cp_sense_isr() (integer-only); read by the task each planning pass.
static volatile uint32_t s_nativePeriodUs = 1000; // 1kHz nominal until measured
static volatile uint32_t s_nativeHighUs = 0;
static volatile bool s_nativeHighKnown = false;

// ISR-internal only (GPIO ISR isn't reentrant here), so plain (not volatile) is fine.
static int64_t s_lastRiseUs = -1;
static bool s_clampedThisCycle = false;

// Low 32 bits of the last rising-edge timestamp, for read_connector_state() (task context)
// to check activity recency - separate from s_lastRiseUs since a 64-bit value isn't
// atomic on this core and a 32-bit delta is all a millisecond-scale check needs.
static volatile uint32_t s_lastRiseUs32 = 0;

// Set by schedule_clamp_ticks() just before arming the alarm, read by assert_timer_isr()
// when it fires - single in-flight clamp at a time, like the RMT channel itself.
static volatile uint32_t s_pendingHoldTicks = 0;

static bool IRAM_ATTR assert_timer_isr(void *arg);
static void IRAM_ATTR schedule_clamp_ticks(uint32_t assertOffsetTicks, uint32_t holdTicksRmt);

// Rising edge: measure the native period (integer-only), then arm the hardware timer
// immediately if the last planning pass called for a clamp this cycle - no queue, no task
// wakeup between the GPIO transition and the timer arming. Falling edge: opportunistic
// native high-time learning, skipped on cycles we clamped (that edge is our own MOSFET).
//
// CP_PWM_SENSE_GPIO's comparator/opto stage is inverted: raw GPIO reads LOW during CP's
// positive lobe, HIGH during the negative lobe, so real rising edge is level==0 below -
// get this backwards and the clamp arms off the real falling edge instead, asserting into
// the negative half-cycle (bench-reproduced before this fix - see CLAUDE.md "Resolved").
//
// Zero floating-point here deliberately: the Xtensa FreeRTOS port saves/restores the FPU
// lazily per task, not on arbitrary interrupt entry/exit, so hardware FP in an ISR risks
// silently corrupting whatever task (likely cp_interceptor_task's own double math) this
// interrupts. All duty%/timing math lives in cp_interceptor_task instead, producing plain
// integer ticks this ISR applies.
static void IRAM_ATTR cp_sense_isr(void *arg) {
    (void)arg;
    int64_t nowUs = esp_timer_get_time();
    // Raw register read, not gpio_get_level() - this ISR runs IRAM-only (see cp_hw_init()).
    int level = gpio_ll_get_level(&GPIO, (gpio_num_t)CP_PWM_SENSE_GPIO);

    if (level == 0) { // real rising edge - see polarity note above
        if (s_lastRiseUs > 0) {
            int64_t period = nowUs - s_lastRiseUs;
            // NATIVE_PERIOD_MIN_US/MAX_US casts fold at compile time - pure integer compare.
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

    // ESP_INTR_FLAG_IRAM is safe here: this ISR and schedule_clamp_ticks() only touch raw
    // registers (hal/gpio_ll.h, hal/timer_ll.h), no flash-resident code on the path.
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
    // Not setting RMT_CHANNEL_FLAGS_AWARE_DFS: this build has no CONFIG_PM_ENABLE, so the
    // driver's PM-lock acquisition for that flag has nothing to attach to (caused a hard
    // fault on real hardware when tried); rmt_set_source_clk() below pins APB directly.
    rmt_config(&rmtConf);
    rmt_driver_install(CP_RMT_TX_CHANNEL, 0, 0);
    rmt_set_source_clk(CP_RMT_TX_CHANNEL, RMT_BASECLK_APB); // pin the clock the RMT tick math assumes

    // Free-running hardware timer, alarm-triggered to fire assert_timer_isr() directly in
    // interrupt context. Replaces an esp_timer one-shot, which carried real dispatch
    // jitter (110-338us bench-measured) since that task isn't core-pinned. Setup here is
    // task-context/not time-critical, so it stays on the regular driver/timer.h API.
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

    // Plain digital output, task-context only - the relay has no sub-cycle timing
    // requirement, unlike CLAMP_DRIVE. Driven low up front to match the
    // de-energized/closed/pass-through default.
    gpio_config_t relayConf = {};
    relayConf.pin_bit_mask = (1ULL << CP_DISCONNECT_RELAY_GPIO);
    relayConf.mode = GPIO_MODE_OUTPUT;
    relayConf.pull_up_en = GPIO_PULLUP_DISABLE;
    relayConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    relayConf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&relayConf);
    gpio_set_level((gpio_num_t)CP_DISCONNECT_RELAY_GPIO, 0);
}

// Task-context only (CP_STATUS_PUBLISH_MS cadence), not IRAM-restricted like the ISRs above.
//
// Both sense GPIOs are inverted from the naively-assumed polarity (bench-confirmed
// 2026-08-30): CP_CONNECTED_SENSE_GPIO reads LOW for state A (no vehicle), HIGH for every
// connected level; CP_PWM_SENSE_GPIO reads LOW while the real CP line is high, HIGH while
// low (see cp_sense_isr()'s polarity note) - the idle-level check below matches that.
static ConnectorState read_connector_state() {
    bool notConnected = gpio_get_level((gpio_num_t)CP_CONNECTED_SENSE_GPIO) == 0;
    if (notConnected) return CONN_STATE_A;

    // dutyState is our own clamp decision, not a measurement of what the EVSE is actually
    // driving - use EDGE_IN activity recency instead (CP_ACTIVITY_TIMEOUT_US, config.h).
    uint32_t nowUs32 = (uint32_t)esp_timer_get_time();
    bool oscillatingNow = (nowUs32 - s_lastRiseUs32) < (uint32_t)CP_ACTIVITY_TIMEOUT_US;
    if (oscillatingNow) return CONN_STATE_C; // covers C and D - see config.h

    // No recent EDGE_IN toggling: B (steady ~9V) or a dead/fault line (E/F, ~0V) -
    // CONNECTED_IN alone can't tell these apart, but EDGE_IN's idle level can.
    bool edgeIdleHigh = gpio_get_level((gpio_num_t)CP_PWM_SENSE_GPIO) == 0;
    return edgeIdleHigh ? CONN_STATE_B : CONN_STATE_FAULT;
}

// Beginning-of-struct pointer for TIMER_GROUP_0/1, matching CP_CLAMP_TIMER_GROUP - a
// compile-time ternary on a constant here, not a runtime lookup.
#define CP_CLAMP_TIMER_HW  (TIMER_LL_GET_HW((int)CP_CLAMP_TIMER_GROUP))

// Fires at the intended assert instant (armed by schedule_clamp_ticks()), directly in
// interrupt context - registered IRAM so it keeps running inside a flash-cache-disable
// window instead of being deferred (that deferral was the dominant source of dispatch
// jitter, 75-166us bench-measured, in an earlier non-IRAM design), so everything here and
// in schedule_clamp_ticks()/cp_sense_isr() uses the hal/*_ll.h register-only equivalents.
//
// Writes one RMT item: assert (HIGH) immediately, release after holdTicks. Hold duration
// lives in the item's second segment - bench testing showed the first segment isn't
// reliably timed when re-armed every ~1ms, so it's kept to the minimum 1 tick. A trailing
// zeroed item marks end-of-data, mirroring rmt_write_items(). Idle-level release is still
// hardware-guaranteed even if this ISR is ever deferred past the native falling edge.
static bool IRAM_ATTR assert_timer_isr(void *arg) {
    (void)arg;
    timer_ll_clear_intr_status(CP_CLAMP_TIMER_HW, CP_CLAMP_TIMER_IDX);
    // One-shot: alarm stays disabled until schedule_clamp_ticks() arms it again.

    rmt_item32_t items[2];
    items[0].level0 = 0; // matches idle (released) - not depended on for timing
    items[0].duration0 = 1;
    items[0].level1 = 1; // assert - the duration that matters, in the reliable slot
    items[0].duration1 = s_pendingHoldTicks;
    items[1].val = 0; // end-of-data marker for the RMT hardware

    rmt_ll_tx_reset_pointer(&RMT, CP_RMT_TX_CHANNEL);
    rmt_ll_write_memory(&RMTMEM, CP_RMT_TX_CHANNEL, items, 2, 0);
    rmt_ll_tx_start(&RMT, CP_RMT_TX_CHANNEL);

    return false; // no FreeRTOS task to wake
}

// Arms the clamp for holdTicksRmt (RMT ticks, 0.1us each) at assertOffsetTicks (hw timer
// ticks, 1us each) from now - integer-only, called from cp_sense_isr().
static void IRAM_ATTR schedule_clamp_ticks(uint32_t assertOffsetTicks, uint32_t holdTicksRmt) {
    s_pendingHoldTicks = holdTicksRmt;

    timer_ll_set_alarm_enable(CP_CLAMP_TIMER_HW, CP_CLAMP_TIMER_IDX, false); // disarm any pending previous alarm
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
    uint32_t lastDutyStateChangeMs = 0; // dwell-time bookkeeping, see RELAY_MIN_DWELL_MS

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
            // Two independent freshness paths keep MODE_ACTIVE: a recent successful grid
            // poll, or a recently-confirmed scheduled fixed-current window - a scheduled
            // charge must not depend on the inverter/Modbus link being up. Only when both
            // are stale does this fall back to MODE_BYPASS.
            bool gridStale = !have || (millis() - solar.last_poll_success_ms > STALE_DATA_TIMEOUT_MS);
            bool scheduleStale = !have || (millis() - solar.last_schedule_confirm_ms > STALE_DATA_TIMEOUT_MS);
            stale = gridStale && scheduleStale;

            // Independent recovery watchdog for a wedged Core 0.
            if (shared_state_solar_task_heartbeat_age_ms() > SOLAR_TASK_HEARTBEAT_TIMEOUT_MS) {
                Serial.println("solar_control_task heartbeat stale - Core 0 appears wedged, restarting");
                Serial.flush();
                esp_restart();
            }
        }

        mode = (bootstrapped && !stale) ? MODE_ACTIVE : MODE_BYPASS;
        s_mode = mode;

        // Planning pass: all floating-point duty/timing math lives here, never in
        // cp_sense_isr(). nativePeriodUs/nativeHighUs may be up to one CP_NOTIFY_WAIT_MS
        // stale - immaterial at ~1kHz.
        uint32_t nativePeriodUs = s_nativePeriodUs;
        uint32_t nativeHighUs = s_nativeHighUs;
        bool nativeHighKnown = s_nativeHighKnown;

        // MIN_CURRENT_A (10% duty) is itself a valid CP signal, so entry uses the bare
        // floor with no margin; HYSTERESIS_A applies only on exit, so noise at the floor
        // can't flap the clamp every pass (margin on entry instead left settling exactly
        // at the floor never leaving STANDBY - see CLAUDE.md "Resolved").
        //
        // CP_STANDBY under MODE_ACTIVE opens the disconnect relay (relayShouldOpen below)
        // rather than releasing to native pass-through, which would hand the vehicle full
        // uncapped current exactly when surplus is lowest. Gated by RELAY_MIN_DWELL_MS
        // since a real disconnect forces a fresh unplug/replug handshake.
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
            // A fault engages immediately, never waits out the dwell timer - though this
            // doesn't move the relay either way, since relayShouldOpen is false whenever
            // mode != MODE_ACTIVE.
            dutyState = CP_STANDBY;
        }

        // Only a fresh, confident MODE_ACTIVE "below floor" decision opens the relay -
        // MODE_BYPASS always keeps it closed, favoring native-rate charging over guessing
        // during a data fault.
        bool relayShouldOpen = (mode == MODE_ACTIVE && dutyState == CP_STANDBY);
        gpio_set_level((gpio_num_t)CP_DISCONNECT_RELAY_GPIO, relayShouldOpen ? 1 : 0);

        // Native duty was learned against the pre-disconnect waveform; force a relearn
        // after reconnect rather than clamping against stale timing, reusing the
        // "clamping skipped until native duty known" gate below.
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
                // assertTargetUs is the real-world assert instant; the armed timer is
                // pulled CLAMP_DISPATCH_LATENCY_US earlier so dispatch delay lands the
                // actual assert back on target. Hold duration is measured from that same
                // real-world target, not the pulled-earlier offset - the RMT hold counts
                // down from the (compensated) actual assert instant, so re-subtracting the
                // latency here would land the release CLAMP_DISPATCH_LATENCY_US late, into
                // the negative half-cycle.
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
            // Native duty not learned yet - fail toward pass-through rather than guessing.
            appliedDutyPct = 0.0f;
        } else {
            appliedDutyPct = nativeHighKnown ? (float)((double)nativeHighUs * 100.0 / (double)nativePeriodUs) : 0.0f;
        }

        // Publish ticks/clampEnabled before dutyState - see the handoff comment at top of file.
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
