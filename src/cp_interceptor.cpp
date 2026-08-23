#include "cp_interceptor.h"
#include "config.h"
#include "shared_state.h"

#include <Arduino.h>
#include <driver/rmt.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

struct EdgeEvent {
    int64_t t_us;
    uint8_t level; // 1 = rising, 0 = falling
};

static QueueHandle_t s_edge_queue = nullptr;

static void IRAM_ATTR cp_sense_isr(void *arg) {
    EdgeEvent ev;
    ev.t_us = esp_timer_get_time();
    ev.level = gpio_get_level((gpio_num_t)CP_PWM_SENSE_GPIO);
    BaseType_t higherPriorityWoken = pdFALSE;
    xQueueSendFromISR(s_edge_queue, &ev, &higherPriorityWoken);
    if (higherPriorityWoken == pdTRUE) {
        portYIELD_FROM_ISR();
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

    gpio_config_t relayConf = {};
    relayConf.pin_bit_mask = (1ULL << RELAY_K1_GPIO);
    relayConf.mode = GPIO_MODE_OUTPUT;
    relayConf.pull_up_en = GPIO_PULLUP_DISABLE;
    relayConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    relayConf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&relayConf);
    gpio_set_level((gpio_num_t)RELAY_K1_GPIO, 0); // de-energized until proven safe to run

    gpio_install_isr_service(0);
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
    rmt_config(&rmtConf);
    rmt_driver_install(CP_RMT_TX_CHANNEL, 0, 0);

    analogReadResolution(12);
    analogSetPinAttenuation(CP_STATE_ADC_GPIO, ADC_11db);
}

static ConnectorState read_connector_state() {
    uint32_t mv = analogReadMilliVolts(CP_STATE_ADC_GPIO);
    if (mv >= ADC_MV_STATE_A_MIN) return CONN_STATE_A;
    if (mv >= ADC_MV_STATE_B_MIN) return CONN_STATE_B;
    if (mv >= ADC_MV_STATE_C_MIN) return CONN_STATE_C;
    return CONN_STATE_FAULT;
}

// Arms the RMT-generated clamp pulse for the cycle currently in progress:
// held low (released) for assertOffsetUs after the rising edge, then high
// (clamped) for holdUs, then released again by the driver's configured idle
// output - the release edge is hardware-guaranteed even if this task is
// delayed before the next cycle.
static void arm_clamp(double assertOffsetUs, double holdUs) {
    const double ticksPerUs = 10.0; // RMT_CLK_DIV=8 -> 80MHz/8 = 10MHz

    if (assertOffsetUs < 0) assertOffsetUs = 0;
    if (holdUs < 0) holdUs = 0;

    uint32_t assertTicks = (uint32_t)(assertOffsetUs * ticksPerUs);
    uint32_t holdTicks = (uint32_t)(holdUs * ticksPerUs);
    if (assertTicks < 1) assertTicks = 1;
    if (holdTicks < 1) holdTicks = 1;
    if (assertTicks > 32767) assertTicks = 32767;
    if (holdTicks > 32767) holdTicks = 32767;

    rmt_item32_t items[1];
    items[0].level0 = 0;
    items[0].duration0 = assertTicks;
    items[0].level1 = 1;
    items[0].duration1 = holdTicks;
    rmt_write_items(CP_RMT_TX_CHANNEL, items, 1, false);
}

void cp_interceptor_task(void *pvParameters) {
    (void)pvParameters;

    s_edge_queue = xQueueCreate(CP_EDGE_QUEUE_LEN, sizeof(EdgeEvent));
    cp_hw_init();
    esp_task_wdt_add(NULL);

    SystemMode mode = MODE_BYPASS;
    CpDutyState dutyState = CP_STANDBY;
    bool bootstrapped = false;
    bool stale = true;

    float targetAmps = 0.0f;
    int64_t lastRiseUs = -1;
    double nativePeriodUs = 1000.0;
    double nativeHighUs = 0.0;
    bool nativeHighKnown = false;
    bool clampedThisCycle = false;
    float appliedDutyPct = 0.0f;

    int64_t lastStaleCheckUs = 0;
    int64_t lastStatusPublishUs = 0;

    for (;;) {
        EdgeEvent ev;
        BaseType_t got = xQueueReceive(s_edge_queue, &ev, pdMS_TO_TICKS(CP_NOTIFY_WAIT_MS));

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
            stale = !have || (millis() - solar.last_poll_success_ms > STALE_DATA_TIMEOUT_MS);
        }

        SystemMode desiredMode = (bootstrapped && !stale) ? MODE_ACTIVE : MODE_BYPASS;
        if (desiredMode != mode) {
            mode = desiredMode;
            gpio_set_level((gpio_num_t)RELAY_K1_GPIO, mode == MODE_ACTIVE ? 1 : 0);
        }

        if (got == pdTRUE) {
            if (ev.level == 1) {
                // Rising edge: Jueclat is starting a new cycle.
                if (lastRiseUs > 0) {
                    double period = (double)(ev.t_us - lastRiseUs);
                    if (period > NATIVE_PERIOD_MIN_US && period < NATIVE_PERIOD_MAX_US) {
                        nativePeriodUs = period;
                    }
                }
                lastRiseUs = ev.t_us;

                if (mode == MODE_ACTIVE) {
                    if (targetAmps < MIN_CURRENT_A) {
                        dutyState = CP_STANDBY;
                    } else if (dutyState == CP_STANDBY && targetAmps < (MIN_CURRENT_A + HYSTERESIS_A)) {
                        dutyState = CP_STANDBY;
                    } else {
                        dutyState = CP_OSCILLATING;
                    }
                } else {
                    dutyState = CP_STANDBY;
                }

                clampedThisCycle = false;

                if (mode == MODE_ACTIVE && dutyState == CP_OSCILLATING && nativeHighKnown) {
                    float targetDutyPct = constrain(targetAmps * CP_DUTY_PCT_PER_AMP, CP_MIN_DUTY_PCT, CP_MAX_DUTY_PCT);
                    double nativeDutyPct = nativeHighUs * 100.0 / nativePeriodUs;
                    double effectiveDutyPct = min((double)targetDutyPct, nativeDutyPct);

                    if (nativeDutyPct - effectiveDutyPct >= MIN_CLAMP_MARGIN_PCT) {
                        double assertOffsetUs = nativePeriodUs * effectiveDutyPct / 100.0 - TX_DISPATCH_LATENCY_US;
                        double holdUs = nativeHighUs - assertOffsetUs;
                        arm_clamp(assertOffsetUs, holdUs);
                        clampedThisCycle = true;
                        appliedDutyPct = (float)effectiveDutyPct;
                    } else {
                        appliedDutyPct = (float)nativeDutyPct;
                    }
                } else if (mode == MODE_ACTIVE && dutyState == CP_OSCILLATING) {
                    // Native duty not learned yet - fail toward pass-through
                    // rather than guessing a clamp point.
                    appliedDutyPct = 0.0f;
                } else {
                    appliedDutyPct = nativeHighKnown ? (float)(nativeHighUs * 100.0 / nativePeriodUs) : 0.0f;
                }
            } else {
                // Falling edge. If we clamped this cycle, this transition is
                // our own MOSFET asserting, not the Jueclat's real falling
                // edge - don't use it to update the learned native duty.
                if (!clampedThisCycle && lastRiseUs > 0) {
                    double high = (double)(ev.t_us - lastRiseUs);
                    if (high > 0 && high < nativePeriodUs) {
                        nativeHighUs = high;
                        nativeHighKnown = true;
                    }
                }
            }
        }

        if (now - lastStatusPublishUs >= (int64_t)CP_STATUS_PUBLISH_MS * 1000) {
            lastStatusPublishUs = now;
            CpStatus status;
            status.mode = mode;
            status.duty_state = dutyState;
            status.connector_state = read_connector_state();
            status.native_duty_pct = nativeHighKnown ? (float)(nativeHighUs * 100.0 / nativePeriodUs) : 0.0f;
            status.applied_duty_pct = appliedDutyPct;
            shared_state_try_publish_cp_status(status);
        }

        esp_task_wdt_reset();
    }
}
