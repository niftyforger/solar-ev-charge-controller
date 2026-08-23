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
#define MAINS_VOLTAGE_V          230.0f

// Native PWM sanity window (~1kHz nominal, generous guard band against glitches)
#define NATIVE_PERIOD_MIN_US     800.0
#define NATIVE_PERIOD_MAX_US     1300.0

// Only bother arming the clamp if it actually shortens the pulse by at least
// this many percentage points of duty - avoids relay/MOSFET/RMT churn for
// no meaningful effect.
#define MIN_CLAMP_MARGIN_PCT     0.5f

// Systematic compensation for the software dispatch latency between the
// rising-edge ISR and the RMT hardware actually starting to output the
// scheduled waveform. Placeholder - characterize and tune during bench
// testing stage 1 (see CLAUDE.md open questions).
#define TX_DISPATCH_LATENCY_US   3.0

// Hysteresis band for re-entering OSCILLATING from STANDBY once surplus
// recovers above the minimum-current floor. Placeholder - tune empirically.
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
#define CP_EDGE_QUEUE_LEN         16
#define CP_NOTIFY_WAIT_MS         100    // also doubles as "no oscillation" poll rate
#define CP_STALE_CHECK_MS         1000
#define CP_STATUS_PUBLISH_MS      200
#define CP_TASK_WDT_TIMEOUT_MS    3000

// ---------------------------------------------------------------------------
// Core 0 solar control loop
// ---------------------------------------------------------------------------
#define POLL_INTERVAL_MS          5000UL
#define STALE_DATA_TIMEOUT_MS     60000UL
#define STEP_MAX_A_PER_POLL       2.0f
#define SETTLE_MS                 15000UL
#define WIFI_RECONNECT_INTERVAL_MS 5000UL
#define WIFI_HOSTNAME              "geely-charger-controller"

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
