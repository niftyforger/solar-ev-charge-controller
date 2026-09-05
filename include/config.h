#pragma once

#include <stdint.h>

// Pin assignments. ESP32-S3: ADC1 = GPIO1-10 (ADC2 shares hardware with WiFi, avoid it).
// Strapping pins (0, 3, 45, 46) and USB-JTAG pins (19, 20) are avoided.

// CP sense input (post opto-isolator, digital 0/3.3V square wave)
#define CP_PWM_SENSE_GPIO      5

// CP clamp drive output -> optocoupler -> MOSFET gate driver (RMT TX)
#define CLAMP_DRIVE_GPIO       6

// CP connector-state sense, CONNECTED_IN (post opto) - see CLAUDE.md "CP interception circuit"
#define CP_CONNECTED_SENSE_GPIO 4

// CP disconnect relay drive -> opto -> coil driver. NC relay in series on CP_LINE;
// energizing opens it (EVSE reads state A). De-energized/no-drive = closed = pass-through.
#define CP_DISCONNECT_RELAY_GPIO 7

// CP electrical / protocol constants (IEC 61851-1 / SAE J1772 §6.4)
#define CP_MIN_DUTY_PCT         10.0f
#define CP_MAX_DUTY_PCT         96.0f
#define CP_DUTY_PCT_PER_AMP     (1.0f / 0.6f)   // duty% = amps / 0.6

#define MIN_CURRENT_A            6.0f
#define MAX_CURRENT_A            32.0f

// Grid voltage is per-GridDataSource (see grid_data_source.h); this is the fixed value
// simulated sources report and grid_source_sungrow_winet.cpp's fallback before its
// first successful voltage-register read.
#define MAINS_VOLTAGE_FIXED_V    240.0f

// Native PWM sanity window (~1kHz nominal, generous guard band against glitches)
#define NATIVE_PERIOD_MIN_US     800.0
#define NATIVE_PERIOD_MAX_US     1300.0

// Only arm the clamp if it shortens the pulse by at least this much - avoids relay/MOSFET/RMT churn for no effect.
#define MIN_CLAMP_MARGIN_PCT     0.5f

// Compensates the rising-edge-ISR -> hardware-timer -> RMT-assert dispatch latency (see
// assert_timer_isr() in cp_interceptor.cpp). Bench-measured (5.7us, single sample at 10%
// duty); re-derive with a scope if the ISR path is ever rewritten.
#define CLAMP_DISPATCH_LATENCY_US   5.7

// Exit-from-OSCILLATING margin below MIN_CURRENT_A, to stop noise at the floor from
// flapping the clamp. Entry uses the bare floor, no margin (10% duty is a fully valid
// signal). Confirmed adequate against the real vehicle, 2026-09-04.
#define HYSTERESIS_A             1.0f

// Minimum hold time for the CP_STANDBY/CP_OSCILLATING duty state (and therefore the
// disconnect relay) before it can flip again, on top of HYSTERESIS_A - bounds relay
// cycle life and gives the EVSE/vehicle time to complete an unplug/replug handshake.
// Confirmed adequate against the real vehicle, 2026-09-04.
#define RELAY_MIN_DWELL_MS       10000UL

// Connector-state discrimination: one CONNECTED_IN threshold (REF_CONN, A vs B) and one
// EDGE_IN threshold (REF_EDGE, D vs E) give four buckets, not six - A; B; C incl. D
// (toggling); FAULT incl. E/F (steady-low). D is unused on this install and E/F both
// already mean "don't clamp", so the conflation costs status granularity only. See
// read_connector_state() in cp_interceptor.cpp and CLAUDE.md "CP interception circuit".
//
// How long since the last rising edge before read_connector_state() calls the line "not
// oscillating" - sized above NATIVE_PERIOD_MAX_US to tolerate jitter, well below
// CP_STATUS_PUBLISH_MS so a real stop shows up promptly.
#define CP_ACTIVITY_TIMEOUT_US   3000

// RMT (legacy driver/rmt.h - ESP-IDF 4.4 bundled with this platform version)
#define CP_RMT_TX_CHANNEL        RMT_CHANNEL_0
#define RMT_CLK_DIV              8       // 80MHz APB / 8 = 10MHz -> 0.1us/tick

// Core 1 CP task timing. cp_interceptor_task's planning loop is not on the timing-critical
// path - cp_sense_isr() applies the clamp plan directly on each rising edge.
#define CP_NOTIFY_WAIT_MS         100
#define CP_STALE_CHECK_MS         1000
#define CP_STATUS_PUBLISH_MS      200
#define CP_TASK_WDT_TIMEOUT_MS    3000

// Separate recovery watchdog for solar_control_task, checked by Core 1 (see
// shared_state_heartbeat_solar_task()). Kept independent of CP_TASK_WDT_TIMEOUT_MS
// because solar_control_task legitimately blocks much longer than 3s during an
// ArduinoOTA push - sized comfortably above any expected OTA duration.
#define SOLAR_TASK_HEARTBEAT_TIMEOUT_MS 120000UL

// Core 0 solar control loop
#define POLL_INTERVAL_MS          5000UL
#define STALE_DATA_TIMEOUT_MS     60000UL
#define SETTLE_MS                 15000UL
#define WIFI_RECONNECT_INTERVAL_MS 5000UL
#define WIFI_HOSTNAME              "solar-ev-charger"

// Server-side power-history rolling buffer for the control page's chart (see
// /api/history in solar_control.cpp) - fixed-size ring of time buckets, not raw
// per-poll samples, so 24h of history costs a few KB instead of needing PSRAM.
#define HISTORY_BUCKET_SPAN_S      300   // 5 minutes per bucket
#define HISTORY_BUCKET_CAPACITY    288   // 24h / 5min

// Persists the history buffer to LittleFS (see solar_control.cpp's history_load_from_fs()/
// history_save_to_fs()) so it survives reboots/reflashes. Bump HISTORY_FILE_VERSION whenever
// HistoryBucket's layout changes - a mismatched magic/version/size on load is treated as "no
// history" rather than misreading stale-format bytes.
#define HISTORY_FILE_PATH          "/history.bin"
#define HISTORY_FILE_MAGIC         0x48495354UL // "HIST"
#define HISTORY_FILE_VERSION       1

// BLE config server (WiFi SSID/password, inverter IP - see ble_config.cpp). Provisioned
// entirely over BLE into NVS; no compile-time fallback (see CLAUDE.md "BLE configuration").
#define BLE_DEVICE_NAME            "solar-ev-charger"

// How long BLE advertises after boot once provisioned at least once (an unprovisioned
// device advertises indefinitely regardless, since BLE is the only configuration path).
#define BLE_ADVERTISE_WINDOW_MS    300000UL

// How often to nudge a connected-but-idle BLE client to send HELP. Repeated rather than
// a single on-connect banner, since a client not yet subscribed to notifications would miss it.
#define BLE_IDLE_HELP_INTERVAL_MS  5000UL

// HTTP control page port. Bench/night-time testing uses the simulated grid sources
// selected from this same page rather than a separate sim mode - see CLAUDE.md "Solar data source".
#define SIM_HTTP_PORT              80

// Scheduled (fixed-current) charging. No RTC/timezone/DST logic on-device: synced via NTP
// to UTC only, schedule stored/evaluated in UTC, and the control page's JS converts
// to/from the viewer's local time at edit/display time - see solar_control.cpp's PAGE_HTML.
#define NTP_SERVER                 "pool.ntp.org"

// Below this, time(nullptr) is treated as "NTP hasn't synced yet" rather than real - a
// fixed point in the past (2023-11-14) so this constant never needs bumping.
#define NTP_MIN_VALID_EPOCH        1700000000UL

// Grid data source: which meter/inverter link supplies the grid-power figure is modular
// and selected at runtime from the HTTP control page (see grid_data_source.h,
// grid_data_source_registry.cpp, handle_set_source() in solar_control.cpp), persisted in
// its own NVS namespace ("gridsrc"). Source-specific parameters live with their
// implementation (e.g. grid_source_sungrow_winet.cpp), not here.

// FreeRTOS task priorities / core pinning
#define CORE_SOLAR                0
#define CORE_CP                   1
#define TASK_PRIO_CP              (configMAX_PRIORITIES - 1)
#define TASK_PRIO_SOLAR           2
#define TASK_PRIO_BLE             1
