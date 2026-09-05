#include "grid_data_source.h"
#include "modbus_tcp_client.h"
#include "config.h"

// The WiNet-S dongle exposes Modbus TCP on the LAN; we poll that instead of the DTSU666-20
// meter directly, since it's already the inverter's own RTU slave (one bus master only).
// See CLAUDE.md "Solar data source".
#define SUNGROW_WINET_MODBUS_TCP_PORT     502
#define SUNGROW_WINET_MODBUS_UNIT_ID      1
// S32, W, spans two registers. Raw is positive=exporting; sungrow_winet_read_power_w()
// negates it to match the codebase's negative=exporting convention. Confirmed on real
// hardware: word order is low-word-first, not the big-endian community convention.
#define SUNGROW_WINET_REG_GRID_POWER      13009
#define SUNGROW_WINET_REG_GRID_POWER_COUNT 2
// "Phase A voltage" (community-documented, mkaiser/dnoegel Sungrow Modbus references) -
// unconfirmed against this specific hardware, like every register here.
#define SUNGROW_WINET_REG_GRID_VOLTAGE       5018    // U16, x0.1V
#define SUNGROW_WINET_REG_GRID_VOLTAGE_COUNT 1
#define SUNGROW_WINET_GRID_VOLTAGE_SCALE     0.1f
// Standalone S16 (not the low half of a 13021-13022 S32 pair - that assumption gave a
// garbage 58M W reading; corrected 2026-09-04 against real battery discharge, see
// CLAUDE.md "Resolved"). Raw is positive=discharging/negative=charging - opposite of the
// positive=charging/negative=discharging convention outBatteryW uses - so it's negated
// here, the same as SUNGROW_WINET_REG_GRID_POWER above. (A same-day 2026-09-05 change
// briefly dropped this negation based on an indirect inference from a grid-import spike;
// reverted later the same day after a directly Sungrow-app-confirmed discharge showed up
// as "charging" - the mirror image of the bug that change was meant to fix. Every direct
// app observation, on both 2026-09-04 and 2026-09-05, matches the negated reading.)
#define SUNGROW_WINET_REG_BATTERY_POWER      13021
#define SUNGROW_WINET_REG_BATTERY_POWER_COUNT 1
// Generous bound, well above any residential pack's real power - only guards against a
// mismapped/garbage register, not a precise spec limit.
#define SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W 20000.0f
#define SUNGROW_WINET_MODBUS_TIMEOUT_MS   2000

static bool sungrow_winet_read_power_w(IPAddress host, float /*currentDrawW*/,
                                         float &outWatts, float &outVoltageV,
                                         float &outBatteryW, bool &outBatteryDataValid) {
    // Persistent connection reused across polls - only ever runs serially on Core 0's
    // solar_control_task, so a function-local static is safe.
    static ModbusTcpClient client(IPAddress(0, 0, 0, 0), SUNGROW_WINET_MODBUS_TCP_PORT,
                                   SUNGROW_WINET_MODBUS_UNIT_ID, SUNGROW_WINET_MODBUS_TIMEOUT_MS);
    // Last successfully-read voltage, carried across polls - seeded with the same fixed
    // fallback the simulated sources use.
    static float lastGoodVoltageV = MAINS_VOLTAGE_FIXED_V;

    client.setHost(host);
    uint16_t regs[SUNGROW_WINET_REG_GRID_POWER_COUNT];
    if (!client.readInputRegisters(SUNGROW_WINET_REG_GRID_POWER, SUNGROW_WINET_REG_GRID_POWER_COUNT, regs)) {
        return false;
    }
    int32_t raw = ((int32_t)((uint32_t)regs[1] << 16 | regs[0]));
    outWatts = -(float)raw;
    Serial.printf("[modbus] regs[0]=0x%04X regs[1]=0x%04X raw=%ld grid_power_w=%.0f (%s)\n",
                  regs[0], regs[1], (long)raw, outWatts, outWatts <= 0 ? "exporting" : "importing");

    // Voltage is secondary - a miss only costs amps/watts precision, never safety (the
    // clamp can only ever clip current down), so it falls back to the last known-good value.
    uint16_t voltageReg[SUNGROW_WINET_REG_GRID_VOLTAGE_COUNT];
    if (client.readInputRegisters(SUNGROW_WINET_REG_GRID_VOLTAGE, SUNGROW_WINET_REG_GRID_VOLTAGE_COUNT, voltageReg)) {
        lastGoodVoltageV = (float)voltageReg[0] * SUNGROW_WINET_GRID_VOLTAGE_SCALE;
        Serial.printf("[modbus] voltageReg=0x%04X grid_voltage_v=%.1f\n", voltageReg[0], lastGoodVoltageV);
    }
    outVoltageV = lastGoodVoltageV;

    // Battery power is secondary too, same reasoning - a miss only means this poll can't
    // refresh the reading, never a safety issue on its own. Falls back to the last
    // known-good value for display continuity, but outBatteryDataValid (below) tracks
    // freshness explicitly so a stale/never-obtained reading can't silently masquerade as
    // a confirmed-idle battery to the discharge-exclusion logic in solar_control.cpp.
    static float lastGoodBatteryW = 0.0f;
    static bool batteryEverReadOk = false;
    static uint32_t lastBatteryReadOkMs = 0;
    uint16_t batteryRegs[SUNGROW_WINET_REG_BATTERY_POWER_COUNT];
    if (client.readInputRegisters(SUNGROW_WINET_REG_BATTERY_POWER, SUNGROW_WINET_REG_BATTERY_POWER_COUNT, batteryRegs)) {
        int16_t rawBattery = (int16_t)batteryRegs[0];
        float batteryReadingW = -(float)rawBattery;
        Serial.printf("[modbus] batteryRegs[0]=0x%04X raw=%d battery_power_w=%.0f (%s)\n",
                      batteryRegs[0], rawBattery, batteryReadingW,
                      batteryReadingW < 0 ? "discharging" : (batteryReadingW > 0 ? "charging" : "idle"));
        // Reject anything outside the sanity bound as a mismapped/garbage read rather than
        // trusting it - same fallback-to-last-known-good treatment as a failed read above.
        if (fabsf(batteryReadingW) <= SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W) {
            lastGoodBatteryW = batteryReadingW;
            batteryEverReadOk = true;
            lastBatteryReadOkMs = millis();
        } else {
            Serial.printf("[modbus] battery_power_w %.0f exceeds sanity bound (%.0f), rejecting\n",
                          batteryReadingW, SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W);
        }
    }
    outBatteryW = lastGoodBatteryW;
    outBatteryDataValid = batteryEverReadOk && (millis() - lastBatteryReadOkMs <= STALE_DATA_TIMEOUT_MS);
    return true;
}

const GridDataSource GRID_SOURCE_SUNGROW_WINET = {
    "sungrow_winet",
    "Sungrow WiNet-S (SH8.0RS)",
    sungrow_winet_read_power_w,
};
