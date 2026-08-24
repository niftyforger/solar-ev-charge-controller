#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Pin assignments
//
// ESP32-S3: ADC1 = GPIO1-10 (ADC2 shares hardware with WiFi, avoid it).
// Strapping pins (0, 3, 45, 46) and USB-JTAG pins (19, 20) are avoided.
// Confirm against the actual bring-up wiring before first power-up.
// ---------------------------------------------------------------------------

// CP sense input (post opto-isolator, digital 0/3.3V square wave)
#define CP_PWM_SENSE_GPIO      5

// CP clamp drive output -> optocoupler -> MOSFET gate driver (RMT TX)
#define CLAMP_DRIVE_GPIO       6

// K1 relay coil driver transistor, active-high
#define RELAY_K1_GPIO          7

// Analog CP DC-level sense for connector state (A/B/C) discrimination.
// Must be ADC1 (GPIO1-10).
#define CP_STATE_ADC_GPIO      4

// Status LCD I2C bus (matches the bring-up example in examples/display/display.ino)
#define I2C_SDA_PIN             8
#define I2C_SCL_PIN             9
#define LCD_I2C_ADDR            0x27
#define LCD_COLS                20
#define LCD_ROWS                4
#define LCD_REFRESH_MS          1000

// ---------------------------------------------------------------------------
// CP electrical / protocol constants (IEC 61851-1 / SAE J1772 §6.4)
// ---------------------------------------------------------------------------
#define CP_MIN_DUTY_PCT         10.0f
#define CP_MAX_DUTY_PCT         96.0f
#define CP_DUTY_PCT_PER_AMP     (1.0f / 0.6f)   // duty% = amps / 0.6

#define MIN_CURRENT_A            6.0f
#define MAX_CURRENT_A            32.0f
#define MAINS_VOLTAGE_V          240.0f

// Native PWM sanity window (~1kHz nominal, generous guard band against glitches)
#define NATIVE_PERIOD_MIN_US     800.0
#define NATIVE_PERIOD_MAX_US     1300.0

// Only bother arming the clamp if it actually shortens the pulse by at least
// this many percentage points of duty - avoids relay/MOSFET/RMT churn for
// no meaningful effect.
#define MIN_CLAMP_MARGIN_PCT     0.5f

// Systematic compensation for the total latency between the rising-edge ISR
// arming the hardware timer and the RMT hardware actually asserting the
// clamp: cp_sense_isr() -> timer alarm -> assert_timer_isr() (genuine
// interrupt context, no FreeRTOS task dispatch - see cp_interceptor.cpp) ->
// raw RMT register write (rmt_ll_write_memory()/rmt_ll_tx_start()).
//
// History: 35.7us (calibrated against the superseded esp_timer/
// ESP_TIMER_TASK mechanism, never re-verified after that path was replaced)
// -> 10.7us (2026-08-24, first bench sample against the hardware-timer-
// group-ISR path: target 10.0% duty, ~1kHz native period, typical measured
// GPIO5-to-GPIO6 delta 75us against a 100us target, so old constant
// (35.7) was over-subtracting by 25us: 35.7 - 25 = 10.7).
//
// Recalibrated again 2026-08-24, same day, after the ISRs were rewritten to
// raw register access (hal/gpio_ll.h, hal/rmt_ll.h, hal/timer_ll.h) and
// ESP_INTR_FLAG_IRAM (see cp_hw_init()/assert_timer_isr() in
// cp_interceptor.cpp) - that rewrite removed the flash-resident driver call
// overhead the 10.7us figure was measured against, so the mean shifted:
// same 10.0% duty point now measured 95us against the still-100us target,
// 5us short. Corrected value = 10.7 - 5 = 5.7us.
//
// Still only single bench samples at one duty point each time - re-derive
// with more samples/duty points and a scope once real hardware (not the
// bench simulator) is available.
#define CLAMP_DISPATCH_LATENCY_US   5.7

// Hysteresis band for *dropping back* to STANDBY once already OSCILLATING:
// target must fall below MIN_CURRENT_A - HYSTERESIS_A, not just below
// MIN_CURRENT_A, so noise sitting right at the floor can't flap the clamp
// on/off every planning pass. Entering OSCILLATING from STANDBY uses the
// bare MIN_CURRENT_A floor with no margin - 10% duty is itself a fully
// valid signal, not a marginal one, so there's no reason to withhold it.
// (An earlier version put the margin on entry instead, which meant
// settling exactly at the 6.0A floor produced no clamp signal at all -
// caught bench-side 2026-08-24; see cp_interceptor.cpp.) Placeholder value
// - tune empirically.
#define HYSTERESIS_A             1.0f

// ---------------------------------------------------------------------------
// Connector-state (A/B/C) ADC thresholds
//
// Placeholder millivolt breakpoints for the CP_STATE_ADC_GPIO divider/peak
// detector. Exact resistor divider and comparator thresholds are an open
// question in CLAUDE.md - confirm against real CP voltages (12V/9V/6V) with
// a scope during bring-up and adjust these.
// ---------------------------------------------------------------------------
#define ADC_MV_STATE_A_MIN       2600   // ~12V CP -> highest divided level
#define ADC_MV_STATE_B_MIN       1900   // ~9V CP
#define ADC_MV_STATE_C_MIN       1200   // ~6V CP
// Anything below ADC_MV_STATE_C_MIN is treated as FAULT/disconnected.

// ---------------------------------------------------------------------------
// RMT (legacy driver/rmt.h - ESP-IDF 4.4 bundled with this platform version)
// ---------------------------------------------------------------------------
#define CP_RMT_TX_CHANNEL        RMT_CHANNEL_0
#define RMT_CLK_DIV              8       // 80MHz APB / 8 = 10MHz -> 0.1us/tick

// ---------------------------------------------------------------------------
// Core 1 CP task timing
// ---------------------------------------------------------------------------
// cp_interceptor_task's planning-loop cadence (vTaskDelay). Not in the
// timing-critical path - cp_sense_isr() applies the clamp plan directly on
// each rising edge, independent of this loop. See cp_interceptor.cpp.
#define CP_NOTIFY_WAIT_MS         100
#define CP_STALE_CHECK_MS         1000
#define CP_STATUS_PUBLISH_MS      200
#define CP_TASK_WDT_TIMEOUT_MS    3000

// Independent recovery watchdog for solar_control_task (Core 0), checked by
// Core 1 alongside its own CP_STALE_CHECK_MS cadence - see
// shared_state_heartbeat_solar_task()/shared_state_solar_task_heartbeat_age_ms()
// and the check in cp_interceptor_task(). Deliberately NOT the same
// esp_task_wdt_add() mechanism as CP_TASK_WDT_TIMEOUT_MS above: that's a
// single global ESP-IDF TWDT period shared by every registered task, and
// solar_control_task legitimately blocks far longer than 3s during an
// ArduinoOTA push (Core 0 blocks on flash writes for the whole transfer -
// see "OTA & simulation mode" in CLAUDE.md, which already treats pushes
// approaching STALE_DATA_TIMEOUT_MS/60s as unremarkable). Registering this
// task on the CP task's tight watchdog would abort every real OTA update;
// this separate, longer, Core-1-driven check exists so a genuinely wedged
// Core 0 (WiFi/lwIP/WebServer hang - the known failure mode this addresses)
// still self-recovers via esp_restart(), without weakening the CP task's
// own watchdog protection at all. Sized comfortably above any expected OTA
// duration for this firmware's image size on local WiFi.
#define SOLAR_TASK_HEARTBEAT_TIMEOUT_MS 120000UL

// ---------------------------------------------------------------------------
// Core 0 solar control loop
// ---------------------------------------------------------------------------
#define POLL_INTERVAL_MS          5000UL
#define STALE_DATA_TIMEOUT_MS     60000UL
#define STEP_MAX_A_PER_POLL       2.0f
#define SETTLE_MS                 15000UL
#define WIFI_RECONNECT_INTERVAL_MS 5000UL
#define WIFI_HOSTNAME              "geely-charger-controller"

// Solar simulation mode (night-time / no-sun bench testing): a WiFi control
// page toggles between real Modbus readings and this synthetic value. See
// CLAUDE.md "OTA & simulation mode".
#define SIM_DEFAULT_GRID_POWER_W  -1000.0f   // negative = exporting
#define SIM_HTTP_PORT              80

// ---------------------------------------------------------------------------
// Sungrow SH8.0RS Modbus TCP (via WiNet-S dongle)
// ---------------------------------------------------------------------------
#define MODBUS_TCP_PORT           502
#define MODBUS_UNIT_ID            1
#define REG_GRID_POWER            13009   // S32, W, raw register is positive = exporting;
                                            // modbus_read_grid_power_w() negates it so
                                            // software-facing values are negative = exporting
#define REG_GRID_POWER_COUNT      2
#define MODBUS_TIMEOUT_MS         2000

// ---------------------------------------------------------------------------
// FreeRTOS task priorities / core pinning
// ---------------------------------------------------------------------------
#define CORE_SOLAR                0
#define CORE_CP                   1
#define TASK_PRIO_CP              (configMAX_PRIORITIES - 1)
#define TASK_PRIO_SOLAR           2
#define TASK_PRIO_DISPLAY         1
