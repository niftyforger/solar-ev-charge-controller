# Solar EV Charge Controller

See [README.md](README.md) for a project overview and architecture diagram. This file is the dense technical reference.

## Goal

Divert surplus solar into EV charging by dynamically limiting the charge current an existing "dumb" EVSE offers to the car, based on real-time solar export data from a Sungrow hybrid inverter.

## Control mechanism

- EVSE: single-phase 230V, up to 32A (7kW). No API/Modbus/WiFi control path — treated as fixed hardware.
- Only control point: the **Control Pilot (CP) line**, a ~1kHz PWM signal (IEC 61851-1) whose duty cycle sets the car's max current. `current_A = duty_% × 0.6` (10% = 6A min, 26.7% = 16A, 53.3% = 32A; valid range ~10-96% duty).
- Approach: clamp the CP line low partway through each high pulse, between the EVSE's controller and the vehicle connector, shortening duty below native when surplus is less than the charger would otherwise allow. A clip, not a regenerate — see "CP interception circuit". The topology makes lengthening the pulse physically impossible, so it can only ever clip down, never above native.
- **Duty <10% (excluding the reserved 5% comms marker) is not a valid "0A" signal** — undefined per spec, may fault some vehicles. Never drive below the 10% floor.
- Core 1 models this as `OSCILLATING` (clamp active per surplus) vs `STANDBY` (CP disconnect relay open, vehicle isolated), rather than letting the duty calculation go out-of-range below the 6A floor.
- `STANDBY`: the clamp can only shunt toward PE, never source voltage, so it can't hold a steady "connected, standby" level — attempting that would itself be the undefined signal above. So below-floor surplus opens a series relay on the vehicle side of `CP_LINE` instead, which the EVSE reads as a real unplug (state A) and handles via its own normal disconnect path (the earlier alternative — releasing to full native pass-through here — is in "Resolved" below). `MODE_BYPASS` (stale/unavailable data) still releases to native pass-through without opening the relay — a data fault is never treated as confident "no surplus."

## State detection & protocol integrity

- The EVSE determines connector state (A/B/C/D) from the CP line's DC voltage level, and separately verifies the vehicle via a diode-check on the **negative** half-cycle (a real CP diode only permits current on the positive half). Both depend on signal features our circuit must not disturb.
- The clamp only ever pulls the line low, only during the **high** portion of a cycle the EVSE is already driving, only once oscillating (state C) — guaranteed by topology (scheduled off the EVSE's own rising edge, not software gating). It never touches the negative half-cycle, the A/B handshake, or `STANDBY`.

## Solar data source

- Inverter: Sungrow SH8.0RS. Meter: DTSU666-20 via RS485 to the inverter (its Modbus RTU slave) — **don't poll it directly** (RTU is one-master-per-bus); read power/PV data via the inverter's own **Modbus TCP** interface instead (WiNet-S dongle, LAN port 502).
- Single-phase throughout (house + EVSE) — total active power at the meter is sufficient, no per-phase reconciliation needed.
- Registers (community-documented, e.g. `mkaiser/Sungrow-SHx-Inverter-Modbus-Home-Assistant`):
  - `13009` — total active power at the meter (grid import/export), S32/FC4/unit 1, 1W/count, spans regs 13009-13010. Primary control input. **Confirmed on the real WiNet-S (2026-08-24): word order is low-word-first** (`raw = (regs[1]<<16)|regs[0]`, opposite the community big-endian convention) **and native sign is positive=exporting** — `sungrow_winet_read_power_w` negates it so every other consumer sees the documented negative=exporting convention.
  - `5018` — Phase A voltage, U16, ×0.1V. Community-documented; unconfirmed against this specific hardware.
  - `13021` — battery power, a **standalone S16** register, not the low half of a `13021`-`13022` S32 pair (that assumption produced a garbage 58M W reading — see "Resolved"). Raw is already positive=charging/negative=discharging — the same convention the rest of the codebase (`outBatteryW`) uses — so it's passed through **unmodified**, unlike `13009` above which does need negating (fixed 2026-09-05, see "Resolved" — an earlier version of this doc had this backwards, claiming it needed negating "like the grid-power register," which made real charging read as "discharging" on the control page). Defensive ±20kW sanity bound (`SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W`). Ran ~10% below the Sungrow app's figure during one cloudy correlation window — plausibly just window skew, worth rechecking under steadier conditions.
  - PV/DC output registers (`5016`, `13033`) exist if surplus is ever computed as PV output minus house load instead of meter export.
- **Control loop is self-referential**: the EV's own draw is part of what `13009` measures, so raising target current reduces the next poll's measured export. `SETTLE_MS` bounds reaction rate so each change has time to show up before the loop reacts again — delayed closed-loop feedback, not just smoothing against cloud transients.
- **Grid source is a swappable interface** (`src/grid_data_source.h`'s `GridDataSource`: stable `id`, display `name`, `read_power_w`), selected at runtime from the HTTP control page, not compile-time. `grid_data_source_registry.cpp` lists every compiled-in source (`GRID_SOURCE_REGISTRY[]`, index 0 = `GRID_SOURCE_SUNGROW_WINET` = default/fallback for any unrecognized id). `solar_control.cpp` owns the active selection — a `solar_control_task`-local static (Core 0 only), persisted in its own NVS namespace (`"gridsrc"`, independent of BLE's `"netcfg"`) via `/api/set_source`. `GRID_SOURCE_SUNGROW_WINET` (`grid_source_sungrow_winet.cpp`) holds everything meter/inverter-specific — adding a real second source means adding a new `GridDataSource` and registry entry, nothing else.
- `GridDataSourceReadFn`: `bool(IPAddress host, float currentDrawW, float &outWatts, float &outVoltageV, float &outBatteryW)`. `currentDrawW` (the wattage currently commanded) feeds simulated sources so they can model the EV's own draw the way a real meter's reading always includes it (real sources ignore it). `outVoltageV`/`outBatteryW` let each source report its own grid voltage and battery power.
- **Home-battery discharge is excluded from EV-charging surplus** (2026-09-04): the meter sits downstream of the home battery, so it can't distinguish fresh-PV export from battery-discharge export — left unhandled, the loop would divert battery power into EV charging. `outBatteryW` (positive=charging, negative=discharging, 0=idle/no battery) lets `solar_control_task()` compute `effectiveGridPowerW = gridPowerW + fmaxf(0, -batteryPowerW)` before calling `compute_next_target_amps()`, which stays a pure function of "surplus" with no battery-specific knowledge. `status.grid_power_w`/the JSON `grid_w` still report the raw meter reading; the exclusion is surfaced separately as `battery_w`/`battery_state`/`surplus_excluded_w`.
- **All simulated sources are reactive** (`outWatts = currentDrawW + houseLoadW - pvCapacityW`), so reported export shrinks as target amps rises and the loop visibly converges instead of climbing to max (constant-value sources didn't do this — see "Resolved"). One file per scenario: `_no_sun` (0W PV, never charges), `_low` (2200W PV, nets just under the 6A/1,440W floor), `_moderate` (4000W PV, ~13.3A), `_high` (9000W PV, exceeds `MAX_CURRENT_A`), `_battery_discharge` (0W PV/800W load/7400W simulated discharge — `outWatts` alone looks like ~27A surplus, but `outBatteryW` correctly keeps target amps pinned near the floor). This is what "simulation mode" reduces to — picked from the same control-page dropdown as a real source.
- **Grid voltage is per-source**, not a global setting: `GRID_SOURCE_SUNGROW_WINET` reads register `5018`; a failed read falls back to the last good value (or `MAINS_VOLTAGE_FIXED_V`) since voltage only affects amps↔watts precision, never safety. Simulated sources report a fixed `MAINS_VOLTAGE_FIXED_V` (240V, `config.h`). `solar_control.cpp` caches the last reported value (`s_last_mains_voltage_v`) for status display only.

## Hardware: single ESP32-S3

Board: Freenove ESP32-S3-WROOM on its breakout board — its screw-terminal rails (3V3/5V/GND) power the whole CP interceptor circuit too, hence the shared-ground design (see "CP interception circuit"). Dual-core split:

- **Core 0**: WiFi + Modbus TCP poll loop; computes target current from surplus. Each settled poll, `compute_next_target_amps()` (`solar_control.cpp`) sets target amps to the exact value the surplus supports (bounded by `MIN_CURRENT_A`/`MAX_CURRENT_A`) — no per-step limit, since nothing in the CP protocol requires gradual changes; a cold start with abundant surplus reaches equilibrium in one settled poll. `SETTLE_MS` alone gates reaction rate. Below the 6A floor (no current yet to self-correct against), entry is decided directly against the floor's wattage each poll, no margin; exit uses `HYSTERESIS_A` to avoid flapping (mirrors `cp_interceptor.cpp`'s own thresholds). Also owns the BLE config server.
- **Core 1** (pinned, high priority): real-time CP interception.
  - Edge timestamping is a plain GPIO `ANYEDGE` interrupt (`esp_timer_get_time()` in the ISR) — not RMT RX, which is built for after-the-fact burst capture, not low-latency reaction to the current edge.
  - Pre-assert delay uses the legacy hardware timer group API (`driver/timer.h`): a free-running 1MHz timer armed directly from `cp_sense_isr()` (interrupt context, zero task hop). On fire, `assert_timer_isr()` (also interrupt context) asserts the clamp by writing the RMT item directly into hardware via register-level `rmt_ll_*` calls (`rmt_ll_tx_reset_pointer`/`rmt_ll_write_memory`/`rmt_ll_tx_start`) — mirroring what `rmt_write_items()` does, but IRAM/ISR-safe, since `rmt_write_items()` itself is flash-resident and can't be called from this ISR; RMT's idle-low guarantees release even if the task stalls, and release rides the EVSE's own falling edge.
  - `cp_interceptor_task` only plans, on its own cadence (`CP_NOTIFY_WAIT_MS`): floating-point duty%/offset math producing integer tick counts the ISR applies on the next rising edge, via Core-1-local `volatile`s (ticks written before dutyState so the ISR can never see `CP_OSCILLATING` paired with stale params). This is deliberately the only floating point in the path — **never in the ISR**, since the Xtensa FreeRTOS port saves/restores the FPU lazily per task and that isn't guaranteed safe on arbitrary interrupt entry/exit (confirmed via `portmacro.h`'s `coproc_area`; no `FPU_IN_ISR` Kconfig option exists to check instead).
  - Superseded three earlier approaches (kept here so they aren't retried): encoding the delay in the RMT item itself (re-arming every ~1ms caused significant jitter); a software `esp_timer` for the delay (real dispatch jitter — no ISR dispatch or core pinning in this build); routing the decision through a queue to the task instead of arming straight from the ISR (ordinary task-dispatch latency on top).
  - **Native duty is learned opportunistically, not measured live while clamping** — once clamping every cycle, the sense point can't distinguish the EVSE's real falling edge from the clamp's own assert transition, so the cached native-high-duration only updates on cycles the clamp didn't fire (`STANDBY`/`BYPASS`/target≥native). Clamping is skipped until native duty has been learned once (fails toward pass-through).
  - Reads target amps from Core 0 via a FreeRTOS queue/mutex, not a raw shared variable.
  - Toolchain: `platform = espressif32@6.10.0` → Arduino-ESP32 core 2.0.17/ESP-IDF 4.4, so RMT means the **legacy** `driver/rmt.h` API, not IDF5's `driver/rmt_tx.h`/`rmt_rx.h` channel API.

### GPIO constraints

- Avoid strapping pins for CP sense/drive lines.
- **Pin assignments (`include/config.h`, physically confirmed on the bench):**
  | Signal | GPIO | Notes |
  |---|---|---|
  | CP PWM sense / EDGE_IN (digital, post-opto) | 5 | `ANYEDGE` GPIO interrupt + timestamping |
  | Clamp drive (RMT TX → opto → MOSFET gate driver) | 6 | |
  | CP connector-state sense / CONNECTED_IN (digital, post-opto) | 4 | polled, no interrupt; level discrimination happens in hardware, not via ADC |
  | CP disconnect relay drive (opto → relay coil driver) | 7 | plain digital output, task-context only |

  GPIO 8/9 (formerly the status LCD's I2C bus) are unused now that the LCD has been removed.

## BLE configuration

- WiFi SSID/password, the inverter IP, and two independent passwords (BLE-config gate, HTTP control-page gate) have **no compile-time fallback** — `secrets.h` holds only `OTA_PASSWORD` (OTA flashing only). Everything else is provisioned at runtime over BLE (`src/ble_config.*`), persisted to NVS (`Preferences`, namespace `netcfg`), read via `shared_state`'s `RuntimeConfig`.
- **Owned by Core 0 only, never Core 1** — BLE isn't real-time, and a stuck/disconnected client must never affect the control loop or fail-safes. Uses `NimBLE-Arduino` 1.x (much lower footprint than the Arduino core's legacy `BLEDevice.h`; NimBLE 2.x targets IDF5, which would conflict with Core 1's legacy `driver/rmt.h`).
- `ble_config_init()` runs once from `setup()`, independent of WiFi/`solar_control_task` state, so BLE comes up even on a fully unprovisioned board. A separate `ble_periodic_task` re-evaluates the advertising window every ~1s.
- **Transport: Nordic UART Service (NUS)**, not a custom multi-characteristic GATT profile — ordinary BLE serial-terminal apps auto-detect NUS but can't drive a custom profile field-by-field (caught bench-side as a real connection failure against Serial Bluetooth Terminal). One RX (`WRITE`/`WRITE_NR`) + one TX (`NOTIFY`) characteristic (`6E400001`/`6E400002`/`6E400003-B5A3-F393-E0A9-E50E24DCCA9E`), newline-delimited text commands. `NimBLEDevice::setMTU(247)` requested up front so multi-line replies don't truncate before negotiation completes.
- **Command protocol** (`RxCallbacks::onWrite` assembles lines on `\n`, tolerating a trailing `\r`, dispatched via `process_line()`); `SET <FIELD> <value>` / `GET <FIELD>`, each password field named for what it gates:
  | Command | Auth required | Behavior |
  |---|---|---|
  | `AUTH <password>` | no | Compares to `RuntimeConfig.ble_password`. Unset (factory-fresh) authenticates unconditionally. Sets/clears a global authenticated flag (reset per connection) gating every `SET`/`COMMIT` below. |
  | `SET WIFI_SSID <ssid>` | yes | Stages a new SSID (not yet persisted). |
  | `SET WIFI_PASS <password>` | yes | Stages a new WiFi password. |
  | `SET INVERTER_IP <ip>` | yes | Stages a new inverter IP; rejects anything that doesn't parse via `IPAddress::fromString()`. |
  | `COMMIT` | yes | Atomically persists + publishes the three staged WiFi/inverter fields via `shared_state_set_wifi_config()` (bumps `generation`), so `solar_control_task` never observes a half-updated config. |
  | `SET BLE_PASS <password>` | yes | Sets the `AUTH` password. Applies immediately, doesn't bump `generation`. |
  | `SET WEB_PASS <password>` | yes | Sets the HTTP control-page password. Same immediate-apply pattern. |
  | `GET STATUS` | no | Reports SSID, staged inverter IP, live device IP, provisioned/auth flags, whether `BLE_PASS`/`WEB_PASS` have ever been set — **never echoes a password**. |
  | `HELP` | no | Lists the commands above. |
  | unrecognized `SET`/`GET` field, or unrecognized command | — | `ERR unknown field, try HELP` / `ERR unknown command, try HELP` |
- **Advertising window**: `BLE_ADVERTISE_WINDOW_MS` after boot, **except** indefinitely while never provisioned (BLE is the only config path). Re-evaluated ~1s by `ble_periodic_task`; doesn't drop an already-open connection.
- **Core 0 apply flow**: `connect_wifi()` skips `WiFi.begin()` entirely if unprovisioned/empty. `solar_control_task` tracks the last-applied `generation` and reconnects on change — only `COMMIT` bumps it, `SET BLE_PASS`/`SET WEB_PASS` never trigger a WiFi bounce. Unprovisioned/WiFi-less is already safe without new logic: Core 1's stale-data fail-safe already treats it as native pass-through.

## CP interception circuit

See `schematic/schematic.png` (source: `schematic/solar-ev-charge-controller/`) for the actual schematic — this section is the rationale behind it, not a restatement of every value; reference designators (`R5`, `U2`, `Q2`, etc.) match that drawing and were re-confirmed pin-by-pin after a later KiCad renumbering pass.

- **Reference**: CP (and PP) reference PE, not a floating signal ground or neutral. Confirm PE continuity before tapping in.
- **Isolation is signal-only, not power-domain**: `EDGE_IN`/`CONNECTED_IN`/`CLAMP_DRIVE` each cross into the ESP32's domain via their own optocoupler (`U3`/`U4`/`U1`, SFH617A-3). The sense divider, both comparators, and the clamp MOSFET (`Q2`) share `GND`/`+5V` with the ESP32 side — no separate PE-only rail. CP-side fault protection comes from the high-Z sense divider (`R5` 100kΩ) and `D3`/`D4` clamp diodes, not full galvanic isolation (deliberate — see [Open questions](#open-questions-for-implementation)).
- **Sensing**: `R5`(100k)/`R6`(33k) divider brings the CP swing down to a `SENSE` node, compared by an LM393 (`U2`) against a ladder (`R8`/`R9`/`R10`): REF_CONN≈2.5V (unit 1) → `CONNECTED_IN` via `U3`; REF_EDGE≈0.45V (unit 2) → `EDGE_IN` via `U4`. Two thresholds on one node give four buckets — A / B / C(+D) / FAULT(=E+F) — not full six-level discrimination; accepted since D (ventilated charging) is unused on this install and E/F both already mean "don't clamp."
- **Sensing values** (hand-verified 2026-08-30): REF_CONN 2.500V / REF_EDGE 0.450V at nominal, correctly bracketing their state boundaries after the 0.2481 divider ratio. At ±5% resistor tolerance, REF_CONN's worst case would overlap state-B's worst case by ~31mV, so `R5`/`R6`/`R8` are specified at 1% (others stay 5%) to close that gap. Confirmed landing correctly against the real EVSE in stage 2.
- **Clamping**: not a regenerator, and not a series break — `CP_LINE` runs continuous and uncut. `Q2` (2N7000) taps on through a series blocking diode (`D2`) and shunts toward `GND` when driven, only during the high portion of an in-progress cycle; `D2` isolates the negative half-cycle and `Q2`'s own body diode from the tap. `Q2`'s gate defaults off via a 10k pull-down (`R3`) — no relay in this leg (a weaker guarantee than a relay: a shorted `Q2`/opto could hold the line low continuously, but that's still non-hazardous — reads as a fault to the EVSE).
- **Disconnect relay (`K1`)**: NC, energize-to-open, in series on `CP_LINE` downstream of the sense/clamp tap. Own optocoupler + low-side driver (`Q1`, 2N7000) + flyback diode (`D1`). De-energized = closed = pass-through (same fail-safe direction as `R3`/`Q2`). Only `CP_STANDBY` energizes it, isolating the vehicle's CP termination — indistinguishable from a real unplug to the EVSE; reclosing renegotiates the handshake (A→B→C) like a fresh plug-in.
- **Inherent fail-safes**: no `CLAMP_DRIVE` (idle/unpowered/pre-firmware/watchdog reset) → `R3` holds `Q2` off → native pass-through, unconditionally. `Q2` stuck on/shorted → EVSE reads a fault/disconnect, stops offering charge. Clamp can only pull toward `GND`, never raise duty above native, regardless of firmware state. No relay-coil drive → NC stays closed → pass-through — losing the ESP32 must never leave the vehicle unable to charge.
- **Explicit fail-safes**: stale-data timeout (~60s, `STALE_DATA_TIMEOUT_MS`) → `MODE_BYPASS`, stops clamp scheduling, keeps the relay closed regardless of duty state — charging proceeds native/uncapped until the fault clears (deliberate: a data fault never triggers a disconnect). Hardware watchdog on the CP task forces reboot on a hang; RMT defaults idle-low on reset. Power loss → both optos stop driving, `R3` and the de-energized relay both fail to pass-through. Relay dwell-time debounce (`RELAY_MIN_DWELL_MS`, on top of the amps hysteresis) bounds relay cycling and gives the handshake time to complete before another flip.

## Firmware implementation

PlatformIO project, `env:freenove_esp32_s3_wroom`, framework `arduino`. Builds clean (`pio run`, zero warnings). Layout:

- `include/config.h` — pins, control-loop constants, thresholds. Anything still marked TBD here maps 1:1 to an item in "Open questions" below.
- `include/secrets.h` (gitignored) — just `OTA_PASSWORD`, used solely for OTA flashing. Auto-generated from `.env` by `tools/sync_secrets.py` (a PlatformIO pre-build step) — edit `.env`, not this file. WiFi credentials, the inverter IP, and the BLE-config/control-page passwords are never compiled in — see "BLE configuration" above.
- `src/shared_state.*` — the Core0↔Core1 bridge: an `xQueueOverwrite` queue (length 1) for target-amps, plus a mutex-protected status struct each direction. Core 1's side always uses zero-timeout takes so a contended mutex can never stall the real-time task. Also holds the BLE-provisioned `RuntimeConfig`.
- `src/modbus_tcp_client.*` — hand-rolled minimal Modbus TCP client: one persistent connection reused across polls, reconnecting only on failure (write error, timeout, malformed response) — never more than one open at a time. `setHost()` drops and reconnects on IP change. Generic, no register-map knowledge.
- `src/grid_data_source.h` — the `GridDataSource` interface. `src/grid_data_source_registry.cpp` — compiled-in source list + id lookup. `src/grid_source_sungrow_winet.cpp` — the real Sungrow WiNet-S/SH8.0RS implementation. `src/grid_source_simulated_reactive_{no_sun,low,moderate,high,battery_discharge}.cpp` — the reactive simulated family, one file per scenario, each also a minimal reference for adding a real second source. See "Solar data source" above.
- `src/solar_control.*` — Core 0: WiFi connect/reconnect (from `RuntimeConfig`), poll loop, control law, OTA, HTTP control page.
- `src/ble_config.*` — Core 0: BLE provisioning server (WiFi/inverter-IP config, BLE-config/control-page passwords, current-IP readback).
- `src/cp_interceptor.*` — Core 1: GPIO-interrupt edge capture, RMT clamp waveform, disconnect relay drive, `BYPASS`/`ACTIVE` × `OSCILLATING`/`STANDBY` state machine, task watchdog.

### OTA & the HTTP control page

- **OTA**: `ArduinoOTA` starts once WiFi connects, password-protected via `OTA_PASSWORD`. Core 0 blocks on flash writes, so the poll loop pauses; a push longer than `STALE_DATA_TIMEOUT_MS` (60s) triggers Core 1's stale-data bypass mid-update — expected, not a fault to avoid.
- **Bench/night-time testing**: no separate "simulation mode" — the reactive simulated sources are picked from the same control-page grid-source dropdown as any real source, exercising the full poll/control-law/target-amps path (though stale-data fail-safe testing still needs real hardware, since a simulated source always succeeds).
- **Control page**: `http://<device-ip>/`, HTTP Basic Auth (`admin` / `RuntimeConfig.web_password`, set via `SET WEB_PASS`, never `OTA_PASSWORD`). Refuses every request with 403 until a password has been committed at least once — no "open until first set" grace period, unlike WiFi, since this is network-reachable rather than a physical jumper. The interceptor's hardware topology already bounds what bad data here can do (clip down or fall back to native, never raise above native).

Bench-tested through all three stages — real EVSE + real vehicle confirmed working 2026-09-04. Real EVSE is a Geely-branded Jueclat EV Charger-Lite, model WB-AC230-7-N-C-W/S-EU2.0 (single-phase 230V, 7kW, matching the "no API/Modbus/WiFi control path" assumption above).

## Bench testing & validation plan

All four stages complete and passed clean.

0. **BLE provisioning — done.** Fresh-board bring-up: NVS erase, BLE advertises indefinitely while unprovisioned, `HELP`/`AUTH`/`SET`/`COMMIT` auth gating confirmed, `GET STATUS` never echoes a password, control page 403s before `WEB_PASS` is set, advertising window closes after first provision, OTA authenticates independently via `OTA_PASSWORD`.
1. **CP simulator alone — done.** A bench oscillator (native PWM stand-in) + switchable diode/resistor network (vehicle CP termination stand-in, A/B/C, with/without the state diode) validated sensing accuracy, clamp timing, negative-half-cycle/STANDBY/handshake isolation, and clean relay open/close.
2. **Real EVSE + CP simulator — done.** Comparator thresholds landed as expected; relay open/close mid-simulated-charge behaved as a normal unplug/replug — no fault code, current resumed ramping on reconnect.
3. **Real EVSE + real vehicle — done, 2026-09-04, full success.** EVSE: Geely-branded Jueclat EV Charger-Lite, model WB-AC230-7-N-C-W/S-EU2.0. Live CP duty-cycle changes accepted mid-charge with no state-B blip needed; fail-safe and relay behavior matched stages 1-2; no faults observed.

## Open questions for implementation

All previously open items are now resolved.

- Shared-ground/signal-only isolation (see "CP interception circuit" above) — **kept as final, 2026-09-04**, not a galvanically-isolated PE-side supply. Not worth the added cost/complexity given the existing protection (high-Z `R5` divider + `D3`/`D4` clamp diodes).
- Battery-power register (`13021`) exact scale — **closed out, 2026-09-04**. The ~10% discrepancy against the Sungrow app is attributed to correlation-window skew on a cloudy/fluctuating sampling day, not a real register/scale error.
- BLE range/reliability inside the metal EVSE enclosure — confirmed adequate, no external antenna needed.
- Resistor divider/comparator thresholds — confirmed against the real EVSE in stage 2; 1% tolerance on `R5`/`R6`/`R8` is sufficient.
- GPIO pin assignments — physically wired and confirmed correct in stages 0-2.
- Whether the vehicle accepts live CP duty-cycle changes mid-charge — **confirmed yes, 2026-09-04**, no state-B blip needed.
- Settling period (`SETTLE_MS`), hysteresis band (`HYSTERESIS_A`), relay dwell time (`RELAY_MIN_DWELL_MS`), and clamp dispatch-latency compensation (`CLAMP_DISPATCH_LATENCY_US`) — confirmed adequate at their current values against the real vehicle, 2026-09-04.
- Battery-power register (`13021`) mapping corrected, 2026-09-04 — it's a standalone S16, not the low half of a `13021`-`13022` S32 pair (that assumption gave an obviously-garbage 58M W reading). Confirmed via a standalone script against real battery discharge; ±20kW sanity bound kept regardless.
- Battery-power register (`13021`) sign convention corrected, 2026-09-05 — the 2026-09-04 fix above also negated the raw value on the (incorrect) assumption that it needed inverting "like the grid-power register." It didn't: raw `13021` is already positive=charging/negative=discharging, the same convention `outBatteryW` expects. The 2026-09-04 validation only ever exercised a real discharge event, which happened to look right under the wrong assumption too (both a correct pass-through and an incorrect double-negation of an always-negative-while-discharging raw value can appear to agree on that one direction); the inversion only became visible checking the charging direction. Caught 2026-09-05 via a live control-page pull (`grid_w:9495`, `battery_w:6400`, `battery_state:"discharging"`) during a confirmed real grid-scheduled battery charge — a 9.5kW grid import spike coinciding with EV charging is consistent with the battery genuinely charging, not discharging. Fixed by dropping the negation in `sungrow_winet_read_power_w()`.
- Relay-based CP disconnect/reconnect is clean and recoverable — confirmed via a manual CP-wire disconnect while charging and via bench validation of the actual `K1` circuit in stages 1-2.
- Both CP sense GPIOs' idle-level polarity was inverted from the original assumption (`CP_CONNECTED_SENSE_GPIO` LOW=state A; `CP_PWM_SENSE_GPIO` LOW=CP high) — this had caused clamping to land in the negative half-cycle; fixed along with a related bug where hold-duration double-counted `CLAMP_DISPATCH_LATENCY_US`.
- `CP_STANDBY` used to release the clamp to full native pass-through instead of stopping charging — backwards, since it happened exactly when surplus was lowest. Fixed by adding the disconnect relay (`CP_STANDBY` under `MODE_ACTIVE` now opens it; `MODE_BYPASS` is unaffected).
- `compute_next_target_amps()` used to ramp through fictitious sub-floor current, able to overshoot into a deficit at the 6A floor. Fixed by branching on whether currently charging: below-floor entry is decided directly against the floor's wattage each poll, no intermediate steps.
- All simulated grid sources used to be constant-value and never converged (didn't subtract the EV's own draw, so the loop always saw more headroom no matter how much current was already committed). Fixed by making sources reactive and removing the per-poll step cap entirely, relying on `SETTLE_MS` alone to gate reaction rate.
- Entry to charging used to take two settled polls to reach true equilibrium (always snapped to exactly `MIN_CURRENT_A` first). Fixed by unifying entry and steady-state correction into one formula.
- Replaced the single fixed-scenario reactive source with a family spanning no-sun through more-sun-than-usable (`_no_sun`/`_low`/`_moderate`/`_high`), and removed the four constant-value sources and the manual global-voltage setting (each source now reports its own voltage).
