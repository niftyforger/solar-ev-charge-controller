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

// CP connector-state sense, CONNECTED_IN (post opto, digital 0/3.3V).
// Per the CP interceptor schematic, this is no longer a raw analog read:
// an LM393 dual comparator (U2) on the sense divider does the level
// discrimination in hardware against two fixed references (REF_CONN
// ~2.5V, REF_EDGE ~0.45V - see CLAUDE.md "CP interception circuit"),
// and this GPIO just reads U2B's (CONNECTED_IN) digital output. No
// longer needs to be ADC1-capable, but GPIO4 is kept to match the
// physical tap already planned for it.
#define CP_CONNECTED_SENSE_GPIO 4

// CP disconnect relay drive -> optocoupler -> relay coil driver transistor.
// Drives a normally-closed (NC) relay in series on CP_LINE, downstream of
// the sense/clamp tap, between the tap and the vehicle connector - see
// CLAUDE.md "CP interception circuit". Energized (this GPIO driven high)
// opens the relay, isolating the vehicle's CP termination so the EVSE
// reads the line as unplugged (state A) and stops charging on its own.
// De-energized (GPIO low, including all power-loss/not-yet-initialized
// cases) is the NC/default/closed state - native pass-through - matching
// the same fail-safe direction as CLAMP_DRIVE/R3. GPIO7 was the original
// (since-abandoned) RELAY_K1_GPIO placeholder from early in the project;
// reused here for an unrelated purpose - see CLAUDE.md's K1 history note.
#define CP_DISCONNECT_RELAY_GPIO 7

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

// Minimum time the CP_STANDBY/CP_OSCILLATING duty state (and therefore the
// disconnect relay's position - see CP_DISCONNECT_RELAY_GPIO) must be held
// before it's allowed to flip again, on top of the amps hysteresis above.
// Two reasons this exists, independent of noise-driven flapping already
// handled by HYSTERESIS_A: a mechanical relay has finite cycle life, and
// every relay-open/relay-close cycle forces the EVSE and vehicle through
// a fresh unplug/replug handshake, which takes real time and shouldn't be
// re-triggered faster than it can complete. Placeholder value - tune once
// real handshake timing is observed on the bench.
#define RELAY_MIN_DWELL_MS       10000UL

// ---------------------------------------------------------------------------
// Connector-state discrimination
//
// Level discrimination now happens in hardware (U2, LM393 dual comparator -
// see CP_CONNECTED_SENSE_GPIO above and CLAUDE.md "CP interception
// circuit"), not via ADC breakpoints in firmware. What's left in software
// is combining CONNECTED_IN's level with EDGE_IN's (CP_PWM_SENSE_GPIO)
// recent activity/level - see read_connector_state() in cp_interceptor.cpp.
//
// With a single CONNECTED_IN threshold (REF_CONN ~2.5V, sitting between
// state A's ~12V and state B's ~9V) and a single EDGE_IN threshold
// (REF_EDGE ~0.45V, sitting between state D's ~3V and state E's 0V), the
// achievable resolution is four buckets, not the full six CP levels:
//   A          - CONNECTED_IN reads "not connected"
//   B          - connected, EDGE_IN steady-high, no recent toggling
//   C (incl. D)- connected, EDGE_IN toggling recently (oscillating)
//   FAULT (incl. E/F) - connected, EDGE_IN steady-low, no recent toggling
// C/D and E/F are each conflated - the peak-amplitude difference between
// them is exactly what a digital comparator discards. Accepted tradeoff:
// D (ventilated charging) is unused by this vehicle/install, and E/F both
// already map to "don't clamp" behaviourally, so the conflation doesn't
// affect control-loop safety, only status granularity.
//
// CP_ACTIVITY_TIMEOUT_US is how long since the last rising edge before
// read_connector_state() stops considering the line "currently
// oscillating". Sized a few native periods above NATIVE_PERIOD_MAX_US so
// normal jitter can't cause a false "not oscillating" read, while staying
// far shorter than CP_STATUS_PUBLISH_MS so a real stop is reflected
// promptly.
#define CP_ACTIVITY_TIMEOUT_US   3000

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
#define WIFI_HOSTNAME              "solar-ev-charger"

// ---------------------------------------------------------------------------
// BLE config server (WiFi SSID/password, inverter IP - see ble_config.cpp).
// WiFi/inverter-IP are provisioned entirely over BLE and stored in NVS;
// there is no compile-time fallback (see CLAUDE.md "BLE configuration").
// ---------------------------------------------------------------------------
#define BLE_DEVICE_NAME            "solar-ev-charger"

// How long after each boot BLE advertises, once the device has already been
// provisioned at least once (an unprovisioned device - no SSID yet stored in
// NVS - advertises indefinitely regardless of this value, since BLE is the
// only way in and a time limit there could brick a factory-fresh board out
// of range of its one configuration path). Placeholder - tune once real
// provisioning-workflow timing is observed on the bench.
#define BLE_ADVERTISE_WINDOW_MS    300000UL

// How often to nudge a connected-but-idle BLE client with a hint to send
// HELP, until they've sent anything at all - see ble_config.cpp. Repeating
// (rather than a single banner right on connect) also sidesteps a client
// not yet having subscribed to notifications at the instant of connection -
// an early nudge landing before that subscription is simply dropped, and a
// later one gets through.
#define BLE_IDLE_HELP_INTERVAL_MS  5000UL

// Solar simulation mode (night-time / no-sun bench testing): a WiFi control
// page toggles between real Modbus readings and this synthetic value. See
// CLAUDE.md "OTA & simulation mode".
#define SIM_DEFAULT_GRID_POWER_W  -1000.0f   // negative = exporting
#define SIM_HTTP_PORT              80

// ---------------------------------------------------------------------------
// Grid data source
// ---------------------------------------------------------------------------
// Which meter/inverter link supplies the grid-power figure is modular - see
// src/grid_data_source.h. Source-specific parameters (Modbus registers,
// port, unit ID) live with their implementation (e.g. grid_source_dtsu666.cpp),
// not here.

// ---------------------------------------------------------------------------
// FreeRTOS task priorities / core pinning
// ---------------------------------------------------------------------------
#define CORE_SOLAR                0
#define CORE_CP                   1
#define TASK_PRIO_CP              (configMAX_PRIORITIES - 1)
#define TASK_PRIO_SOLAR           2
#define TASK_PRIO_BLE             1
