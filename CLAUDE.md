# geely-charger-controller

## Goal

Divert surplus solar into EV charging by dynamically limiting the charge current an existing "dumb" EVSE offers to the car, based on real-time solar export data from a Sungrow hybrid inverter.

## Control mechanism

- EV charger is a Jueclat EV Charger-Lite (Model WB-AC230-7-N-C-W/S-EU2.0, 230V/7kW/32A single phase). No usable local API/Modbus/WiFi control path — treated as fixed, unmodifiable hardware.
- The only control point is the **Control Pilot (CP) line**: a ~1kHz PWM signal (IEC 61851-1) whose duty cycle tells the car its max allowed current. The car self-limits based on this signal.
- Approach: intercept the CP wire between the Jueclat's internal controller and the vehicle connector, and **actively clamp the line low** partway through each high pulse to shorten it to a lower duty cycle than the charger natively offers, when solar surplus is less than what the charger would otherwise allow. This is a clip, not a regenerate — see "CP interception circuit" below. Only ever clip current down, never lengthen the pulse beyond what the charger sends; the clamp topology makes lengthening physically impossible, not just software-disallowed.
- Duty cycle → current mapping (IEC 61851-1 / SAE J1772 §6.4): `current_A = duty_% x 0.6`, e.g. 10% = 6A (minimum), 26.7% = 16A, 53.3% = 32A. Valid range is ~10%-96% duty.
- **0% duty (or any duty <10%, excluding the reserved 5% digital-comms marker) is NOT a valid "0A" signal** — it's undefined behaviour per spec and some vehicles may fault on it rather than gracefully idle. To signal "no current available," stop the CP oscillation entirely and hold a steady DC level (the same state the EVSE uses for "connected, standby") instead of driving a sub-10% duty cycle.
- Core 1's CP task should implement this as an explicit two-state model — `OSCILLATING` (clamp active per current surplus) vs `STANDBY` (clamp fully released, no PWM touched) — rather than letting the duty-cycle calculation silently produce an out-of-range value when surplus drops below the 6A floor.
- **Resolved: what `STANDBY` actually does electrically.** The clamp can only shunt the line *toward* PE — it has no way to source a positive voltage — so it cannot itself produce the steady-DC "connected, standby" level described above (that's a level only the Jueclat's own driver can create). `STANDBY` in this implementation therefore means "clamp fully released, Jueclat's native/uncapped duty passes through" — i.e. when computed surplus can't even support the 6A floor, the firmware deliberately gives up active limiting for that period rather than risk an invalid sub-10% duty or a permanently-low line (which would read as CP state E/error, not a graceful standby, to the vehicle). This mirrors the stale-data bypass's philosophy: prefer a physically-safe, spec-valid pass-through over guessing at an unsupported signal.

## State detection & protocol integrity

- The Jueclat determines connector state (A/B/C/D) from the DC voltage level of the CP line, and separately verifies the vehicle is genuinely present via a diode-check on the **negative** half-cycle (a real vehicle's CP diode only permits current on the positive half; a missing/reversed diode indicates a fault). Both mechanisms depend on signal features our circuit must not disturb.
- Design requirement: the clamp only ever pulls the line low, only during the **positive (high) portion** of a cycle the Jueclat is already driving, and only once oscillation is already established (i.e., mid-charge, state C). It never touches the negative half-cycle, and it never asserts during the A/B handshake or in `STANDBY`. This is guaranteed by the clamp topology itself (see below), not by separate software gating — there's no code path that can clamp when it shouldn't, because the clamp driver is only ever scheduled relative to a rising edge the Jueclat itself produced.

## Solar data source

- Inverter: Sungrow SH8.0RS. Meter: DTSU666-20, wired via RS485 directly to the inverter as its smart meter.
- **Do not poll the DTSU666 directly** — it's already a Modbus RTU slave to the inverter, and RTU only supports one master per bus. Read meter/PV data back out of the **SH8.0RS's own Modbus TCP interface** instead (via WiNet-S dongle on the LAN, port 502).
- Installation is single-phase throughout (house + Jueclat both single-phase), so total active power at the meter is sufficient — no need to reconcile per-phase figures.
- Useful registers (community-documented, e.g. `mkaiser/Sungrow-SHx-Inverter-Modbus-Home-Assistant` on GitHub):
  - `13009` — total active power at the meter (grid import/export; negative = exporting). This is the primary control input.
  - PV/DC output registers (`5016`, `13033`) if computing surplus as PV output minus house load rather than using meter export directly
- `13009` is a 32-bit signed value spanning registers 13009-13010, S32/FC4/unit ID 1 as assumed, 1W/count. **Confirmed against the real WiNet-S during bring-up (2026-08-24): the word order is low-word-first (little-endian), not big-endian as originally assumed** — the community-convention guess was wrong on this one point. `modbus_tcp_client.cpp`'s `modbus_read_grid_power_w` composes `raw` as `(regs[1] << 16) | regs[0]` to match. This was caught because it produced tens-of-millions-of-watts readings on the status LCD (the sign-extension high word landing in the high position of the combined value) — if the register mapping is ever revisited, re-verify word order the same way: log `regs[0]`/`regs[1]` raw and sanity-check against a known real export/import figure.
- **Also confirmed during the same 2026-08-24 bring-up: the register's native sign polarity is positive = exporting / negative = importing — the opposite of the community-convention assumption this codebase was originally built against.** Caught by observing the status LCD label a "no sun, house-load-only" period, which must be pure grid import: the LCD showed `Export`, meaning the raw `13009` reading was positive during an import-only condition. `modbus_read_grid_power_w` now negates `raw` before returning it, so every other consumer in the codebase (control loop, shared state, LCD) keeps using the documented `negative = exporting` convention unchanged — only the raw-register parsing step in `modbus_tcp_client.cpp` knows about the hardware's true (inverted) polarity. If the register mapping is ever revisited, re-verify this the same way: compare the signed LCD label against a known ground-truth condition (e.g. no solar generation ⇒ must be importing), not just the magnitude.
- **Control loop is self-referential**: the EV's own draw is part of what `13009` measures, so raising target current will itself reduce measured export on the next poll. The control loop must account for this — bound the per-adjustment step size, and hold a settling period after each change before reacting to the next reading — rather than treating each poll as an independent, instantaneous surplus measurement. This is a stronger requirement than plain smoothing/hysteresis against cloud transients; it's closed-loop feedback with a delay, not just noisy input.

## Hardware: single ESP32-S3

Board: Freenove ESP32-S3-WROOM, mounted on the Freenove ESP32-S3 breakout board — its screw-terminal rails (3V3/5V/GND) supply the ESP32-side circuitry (optocoupler outputs, relay driver, LCD), and its prototyping area is the natural place to build that half of the CP interceptor. Dual-core split:

- **Core 0**: WiFi + Modbus TCP client polling the SH8.0RS every few seconds; computes target charge current from surplus power. Adjustments are step-size-limited and settle for a period after each change before the next adjustment is applied, to avoid chasing the EV's own draw (see "Solar data source" above) and to avoid hunting on cloud transients. Also owns the status LCD (see "Status display" below).
- **Core 1** (pinned, high priority): real-time CP interception using hardware peripherals — **RMT** to capture the Jueclat's native incoming PWM (rising edge, period, native duty), and to schedule the clamp driver's assert/release relative to that edge; the clamp is asserted at `target_duty% x period` after the detected rising edge and released at the Jueclat's own falling edge (by which point the line is already low from the source, so the release never fights the driver). Reads a "target amps" value set by Core 0 via a FreeRTOS queue/mutex (not a raw shared variable).
  - **Implemented split (deviation from "RMT does the capture" above):** edge *timestamping/triggering* is a plain GPIO `ANYEDGE` interrupt (`esp_timer_get_time()` in the ISR) handing off to a high-priority task via a queue — RMT RX's ring-buffer/task-wakeup pipeline isn't well suited to reacting to *the current* edge with low latency, it's built for after-the-fact burst capture. RMT is used for what it's actually good at: the TX channel generates the assert→release waveform in hardware once triggered (`idle_output_en` + `idle_level=LOW` guarantees release even if the task stalls), so only the trigger carries software jitter, not the pulse shape itself.
  - **Native duty is learned opportunistically, not measured live while clamping.** Once the clamp is asserting every cycle, the sense point (same tap as the clamp) can't distinguish the Jueclat's real falling edge from the clamp's own assert transition — so the falling-edge timestamp is only used to update the cached native-high-duration on cycles where the clamp did *not* fire that cycle (`STANDBY`, `BYPASS`, or `target_duty >= native_duty`). Clamping is skipped entirely until native duty has been learned at least once (fails toward pass-through, never toward a guessed setpoint).
  - Toolchain fact (confirmed by building): `platform = espressif32@6.10.0` resolves to Arduino-ESP32 core 2.0.17 / ESP-IDF 4.4, so RMT means the **legacy** `driver/rmt.h` API (`rmt_config`/`rmt_write_items`/`rmt_item32_t`), not the IDF5 `driver/rmt_tx.h`/`driver/rmt_rx.h` channel API.

### GPIO/ADC constraints

- Use **ADC1 pins only** for any analog CP sensing — ADC2 shares hardware with WiFi and gives unreliable readings while WiFi is active.
- Avoid strapping pins for CP sense/drive lines, and for the I2C bus used by the status display.
- **Pin assignments (in `include/config.h`, not yet physically confirmed on the bench):**
  | Signal | GPIO | Notes |
  |---|---|---|
  | CP PWM sense (digital, post-opto) | 5 | `ANYEDGE` GPIO interrupt + timestamping |
  | Clamp drive (RMT TX → opto → MOSFET gate driver) | 6 | |
  | K1 relay coil driver | 7 | active-high |
  | CP connector-state sense (analog) | 4 | ADC1 channel, for A/B/C discrimination |
  | I2C SDA / SCL (status LCD) | 8 / 9 | matches `examples/display/display.ino` |

## Status display

- Freenove I2C LCD 2004: 20x4 character LCD on a PCF8574-based I2C backpack (typical address `0x27` or `0x3F` — confirm with an I2C scan during bring-up). Implemented default is `0x27`; update `LCD_I2C_ADDR` in `include/config.h` if the scan disagrees.
- **Owned by Core 0 only, never Core 1.** I2C transactions are blocking and slow relative to CP timing; the display must never be touched from, or block, the real-time CP task. It's purely informational — a stuck, disconnected, or misbehaving LCD must never be able to affect the control loop or the fail-safes.
- Refresh at a modest rate (~1 Hz) — it's for human monitoring, not control, so there's no reason to load the I2C bus or Core 0's loop any harder than that.
- **Voltage**: the ESP32-S3's GPIOs are not 5V-tolerant. Simplest option is powering the backpack (and its I2C pull-ups) from the ESP32's 3.3V rail rather than 5V — the LCD backlight/contrast may be dimmer but no level shifter is needed. If full 5V contrast/brightness is wanted instead, add a level shifter (or at least re-reference the I2C pull-ups) rather than driving the ESP32's I2C pins directly from 5V-referenced signaling.
- Suggested content (4 lines, exact layout TBD during implementation):
  1. Grid export/import power (from register `13009`) — the raw surplus signal.
  2. Target current / duty cycle currently being commanded to the car.
  3. CP state: `OSCILLATING` / `STANDBY`, plus detected connector state (A/B/C).
  4. System status: WiFi/Modbus link health, time since last successful inverter poll, whether a fail-safe (stale-data or bypass) is currently active.

## CP interception circuit

- **Reference**: CP (and PP) are referenced to PE (Protective Earth), not to a floating signal ground or neutral. Confirm continuity from the connector's PE pin to the CP circuitry's reference before tapping in.
- **Isolation**: opto-isolate the PE-referenced CP domain from the ESP32's 3.3V logic domain — both the sensed CP signal going in (sense comparator output → optocoupler → ESP32 RMT/GPIO input) and the clamp-drive command going out (ESP32 GPIO → optocoupler → clamp driver transistor). PE and the ESP32's supply ground are not guaranteed to be the same reference or free of noise/fault coupling, and the CP line carries ±12V; isolation keeps a CP-side fault (or a wiring mistake during bring-up) from reaching the ESP32 and its downstream USB/logic connections.
- **Sensing**: resistor divider brings the +/-12V CP swing down to safe levels, into a comparator (or window comparator for full A/B/C/D state discrimination) feeding the optocoupler input, then ESP32 GPIO/RMT.
- **Clamping (generation)**: not a full waveform regenerator, and not a series break either — the CP wire from the Jueclat to the vehicle connector runs continuous and uncut. A MOSFET (e.g. BS170) shunts off a tap on that wire to pull it down to PE when driven via the opto-isolated control line from Core 1, only during the high portion of a cycle already in progress, per the RMT-scheduled assert/release described above. At all other times (negative half-cycle, STANDBY, A/B handshake) the switch is off and the Jueclat's native signal passes through completely untouched.
- **A relay (K1) gates the clamp leg, not the CP line itself**: K1's normally-open contact sits in series between the clamp MOSFET's drain and the CP tap, not in the CP wire's main path. De-energized (default, and the state on power-loss), K1 is open and the clamp circuitry has no electrical path to CP_LINE at all — not "fails to a safe state," but physically disconnected, even if the MOSFET has failed shorted. This is a stronger guarantee than gating on the MOSFET alone. Sensing stays passive and always-connected regardless of K1, since it's high-impedance and never sinks meaningful current.
- **Inherent fail-safe properties**:
  - K1 de-energized (idle, no supply, firmware not yet initialized, or watchdog-triggered) → clamp leg open → CP passes through at the Jueclat's native (uncapped) duty cycle unconditionally. Non-hazardous: charging continues at whatever the Jueclat would natively allow, just without solar-following.
  - Even with K1 energized, clamp driver stuck closed/shorted → CP line held low → Jueclat reads this as a state fault or vehicle disconnect and stops offering charge. Non-hazardous: charging stops rather than misbehaving.
  - Because the clamp can only pull the line toward PE, it cannot physically raise the effective duty cycle above native, regardless of firmware or relay state.
- **Explicit fail-safes**:
  - Stale-data timeout: if Core 0 hasn't successfully polled the inverter within ~60s, Core 1 de-energizes K1 and releases the clamp entirely, passing the Jueclat's native signal through unmodified — the same end state as the power-loss bypass below. (Accepted tradeoff: a fault in the data path means charging proceeds at the Jueclat's native/uncapped rate, potentially from grid, until the fault clears — deliberately chosen over pausing charging or guessing a setpoint.)
  - Hardware watchdog timer on the CP task, so a firmware hang forces reboot rather than freezing mid-cycle; K1's drive should default low on reset, not require an explicit "off" command.
  - Power-loss bypass: if the ESP32 loses power/crashes, K1 de-energizes on its own (no drive current), and since it only ever gated the clamp leg — never the CP wire itself — the Jueclat's original unmodified CP signal was never at risk of interruption in the first place.

## Firmware implementation

PlatformIO project, `env:freenove_esp32_s3_wroom`, framework `arduino`. Builds clean as of this writing (`pio run`, zero warnings). Layout:

- `include/config.h` — pins, control-loop constants, thresholds. Anything still marked TBD/placeholder here corresponds 1:1 to an item in "Open questions" below.
- `include/secrets.h` (gitignored, template in `secrets.example.h`) — WiFi credentials + inverter IP. Must be edited before flashing.
- `src/shared_state.*` — the Core0↔Core1 bridge: an `xQueueOverwrite` queue (length 1) for the target-amps command, plus a mutex-protected status struct in each direction. Core 1's side always uses zero-timeout mutex/queue takes so a contended mutex can never stall the real-time task.
- `src/modbus_tcp_client.*` — hand-rolled minimal Modbus TCP client (no external lib): short-lived connection per poll rather than a held-open socket, since the WiNet-S gateway's concurrent-connection support is unconfirmed.
- `src/solar_control.*` — Core 0: WiFi connect/reconnect, poll loop, the step-limited/settling-gated control law.
- `src/status_display.*` — Core 0: LCD refresh, its own task so a stuck I2C bus can't touch the control loop.
- `src/cp_interceptor.*` — Core 1: GPIO-interrupt edge capture, RMT-generated clamp waveform, the `BYPASS`/`ACTIVE` × `OSCILLATING`/`STANDBY` state machine, K1 drive, task watchdog.

Not yet bench-validated — matches stage 1 of the plan below (CP simulator, no Jueclat/vehicle) as the next step.

## Bench testing & validation plan

Stage the hardware bring-up so the real Jueclat and vehicle are never the first thing exercising a new circuit revision:

1. **CP simulator, no Jueclat/vehicle involved.** Build or source a simple bench CP simulator: an oscillator standing in for the Jueclat's native PWM, plus a switchable diode/resistor network standing in for the vehicle's state-dependent CP termination (A/B/C, with and without the state-detection diode). Validate sensing accuracy, clamp timing (assert point vs. target duty, release point vs. native falling edge), and that the negative half-cycle and STANDBY/handshake periods are never touched.
2. **Real Jueclat + CP simulator standing in for the vehicle.** Confirms the circuit behaves against the Jueclat's actual timing/tolerances (which may differ from the bench oscillator) without a vehicle attached, and that the Jueclat's own state machine and diode check still function normally through the interceptor.
3. **Real Jueclat + real vehicle, supervised.** Start with conservative (small) clipping targets, confirm the car actually reduces draw accordingly, watch for any fault/disconnect behavior, and only then test larger clip ranges. Deliberately trigger the stale-data and power-loss fail-safes (kill WiFi, pull ESP32 power) while charging is active and confirm the bypass actually engages, before ever leaving the system unattended.
- Use stage 3 to answer the open question below on whether the vehicle accepts live duty-cycle changes mid-charge.

## Open questions for implementation

Still genuinely unresolved (no amount of firmware work settles these — need the real hardware/vehicle):

- Whether this specific vehicle accepts live CP duty-cycle changes while charging is in progress, or requires a brief state transition (e.g. a State B blip) before it'll honor a new lower current — unknown, to be determined empirically in bench-testing stage 3. This affects how aggressively the control loop can adjust and whether an adjustment needs to be wrapped in a deliberate pause/resume.
- Final clamp switch component selection (MOSFET vs. bidirectional analog switch) and optocoupler selection for the CP front end.
- Exact resistor divider and comparator threshold values for state discrimination (A/B/C/D) on the sensing side — the ADC millivolt breakpoints in `config.h` are illustrative placeholders pending a scope on the real CP voltages.
- Whether 3.3V power is sufficient LCD contrast/brightness or a level shifter for 5V is worth adding.

Have an implemented default now, but still need bench confirmation/tuning:

- Step-size (`STEP_MAX_A_PER_POLL`), settling-period (`SETTLE_MS`), and hysteresis-band (`HYSTERESIS_A`) values for the control loop — placeholders in `config.h`, tune once real solar/EV behavior is observed.
- RMT trigger-dispatch-latency compensation (`TX_DISPATCH_LATENCY_US`) — placeholder, characterize with a scope in bench-testing stage 1.
- GPIO pin assignments and LCD I2C address — see the pin table above and `config.h`; chosen but not yet physically wired/confirmed.
