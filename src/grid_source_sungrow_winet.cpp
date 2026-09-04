#include "grid_data_source.h"
#include "modbus_tcp_client.h"
#include "config.h"

// The Sungrow SH8.0RS inverter's WiNet-S dongle exposes a Modbus TCP
// interface on the LAN; that's what we actually poll. The real DTSU666-20
// meter is wired via RS485 directly to the inverter as its RTU slave, but we
// don't poll it directly - RTU only supports one bus master, and the
// inverter already is one - so this reads the same figure back out of the
// WiNet-S instead. See CLAUDE.md "Solar data source".
#define SUNGROW_WINET_MODBUS_TCP_PORT     502
#define SUNGROW_WINET_MODBUS_UNIT_ID      1
#define SUNGROW_WINET_REG_GRID_POWER      13009   // S32, W, raw register is positive = exporting;
                                                    // sungrow_winet_read_power_w() negates it so
                                                    // software-facing values are negative = exporting
#define SUNGROW_WINET_REG_GRID_POWER_COUNT 2
// Community-documented (mkaiser/Sungrow-SHx-Inverter-Modbus-Home-Assistant,
// also cross-referenced against dnoegel's Sungrow Modbus register gist) as
// "Phase A voltage" for SH-RS series inverters - same register block as the
// power reading above. Unconfirmed against this specific real WiNet-S/SH8.0RS
// pairing (like every register here - the community convention was already
// wrong twice for the power register's word order and polarity), so verify
// the logged raw value against a known real voltage if this ever looks off.
#define SUNGROW_WINET_REG_GRID_VOLTAGE       5018    // U16, x0.1V
#define SUNGROW_WINET_REG_GRID_VOLTAGE_COUNT 1
#define SUNGROW_WINET_GRID_VOLTAGE_SCALE     0.1f
// Battery power. The community-documented assumption (S32 spanning
// 13021-13022, mkaiser/Sungrow-SHx-Inverter-Modbus-Home-Assistant) produced
// an obviously-bogus 58,067,088 W reading in real-hardware testing
// (2026-09-04) - the same kind of wrong-versus-community-convention mismatch
// 13009 needed two rounds of correction for.
//
// Corrected the same day by bypassing the firmware and querying the
// WiNet-S's Modbus TCP interface directly with a standalone script while
// the Sungrow app showed ~535W of real battery discharge (cloudy day,
// battery covering most of the house load): sampling registers 13015-13040
// repeatedly showed 13021 alone tracking that discharge (~480-700W over the
// correlation window, right order of magnitude and right sign once negated)
// while 13022 stayed essentially uncorrelated - i.e. 13021 is a standalone
// S16 register, not the low half of an S32 pair with 13022. (13022 is some
// other, unidentified quantity - possibly a rated/config value rather than
// live telemetry, since it barely moved.) Wire polarity is positive =
// charging, same community convention as the grid-power register, and also
// inverted from reality here the same way - negate for the codebase's
// negative = discharging convention.
//
// Residual uncertainty: the corrected reading (~484W) was about 10% below
// the app's simultaneous 535W, plausibly just correlation-window skew on a
// cloudy/fluctuating day rather than a wrong scale factor - re-check against
// a steadier charge/discharge event if it ever looks consistently off by a
// similar margin. SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W below still
// guards against a mismapped/garbage read regardless.
#define SUNGROW_WINET_REG_BATTERY_POWER      13021   // S16, W, raw register is positive = charging;
                                                       // sungrow_winet_read_power_w() negates it, see
                                                       // comment above
#define SUNGROW_WINET_REG_BATTERY_POWER_COUNT 1
// Generous bound, well above any residential battery pack's real
// charge/discharge power - exists only to catch a mismapped/garbage
// register (see comment above), not as a precise spec limit.
#define SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W 20000.0f
#define SUNGROW_WINET_MODBUS_TIMEOUT_MS   2000

static bool sungrow_winet_read_power_w(IPAddress host, float /*currentDrawW*/,
                                         float &outWatts, float &outVoltageV,
                                         float &outBatteryW) {
    // Persistent connection reused across polls (only ever runs serially on
    // Core 0's solar_control_task, so a function-local static is safe) - see
    // CLAUDE.md "Firmware implementation".
    static ModbusTcpClient client(IPAddress(0, 0, 0, 0), SUNGROW_WINET_MODBUS_TCP_PORT,
                                   SUNGROW_WINET_MODBUS_UNIT_ID, SUNGROW_WINET_MODBUS_TIMEOUT_MS);
    // Last successfully-read voltage, carried across polls - seeded with the
    // same fixed fallback the simulated sources use, in case the voltage
    // register is never successfully read at all.
    static float lastGoodVoltageV = MAINS_VOLTAGE_FIXED_V;

    client.setHost(host);
    uint16_t regs[SUNGROW_WINET_REG_GRID_POWER_COUNT];
    if (!client.readInputRegisters(SUNGROW_WINET_REG_GRID_POWER, SUNGROW_WINET_REG_GRID_POWER_COUNT, regs)) {
        return false;
    }
    // Confirmed against the real WiNet-S during bring-up: this register pair
    // is low-word-first (little-endian word order), not big-endian as
    // originally assumed - see CLAUDE.md.
    int32_t raw = ((int32_t)((uint32_t)regs[1] << 16 | regs[0]));
    // Also confirmed during bring-up: the register's native polarity is
    // positive = exporting / negative = importing - opposite of what was
    // originally assumed. Negate here so the rest of the codebase's
    // negative = exporting convention (see CLAUDE.md) holds against the
    // real hardware.
    outWatts = -(float)raw;
    Serial.printf("[modbus] regs[0]=0x%04X regs[1]=0x%04X raw=%ld grid_power_w=%.0f (%s)\n",
                  regs[0], regs[1], (long)raw, outWatts, outWatts <= 0 ? "exporting" : "importing");

    // Voltage is a secondary reading here - a miss only makes the amps/watts
    // conversion slightly less precise, never unsafe (the clamp topology can
    // only ever clip current down, never raise it above native - see
    // CLAUDE.md "Inherent fail-safe properties"), so a failed read falls back
    // to the last known-good value instead of failing the whole poll.
    uint16_t voltageReg[SUNGROW_WINET_REG_GRID_VOLTAGE_COUNT];
    if (client.readInputRegisters(SUNGROW_WINET_REG_GRID_VOLTAGE, SUNGROW_WINET_REG_GRID_VOLTAGE_COUNT, voltageReg)) {
        lastGoodVoltageV = (float)voltageReg[0] * SUNGROW_WINET_GRID_VOLTAGE_SCALE;
        Serial.printf("[modbus] voltageReg=0x%04X grid_voltage_v=%.1f\n", voltageReg[0], lastGoodVoltageV);
    }
    outVoltageV = lastGoodVoltageV;

    // Battery power is a secondary reading here, same reasoning as voltage
    // above: a miss only means solar_control_task can't exclude battery
    // discharge from surplus this poll, never a safety issue (the clamp
    // topology can only ever clip current down - see CLAUDE.md "Inherent
    // fail-safe properties"), so a failed read falls back to the last
    // known-good value, or 0.0f (assume no battery activity - the safe
    // direction, since it makes the exclusion a no-op) if none yet.
    static float lastGoodBatteryW = 0.0f;
    uint16_t batteryRegs[SUNGROW_WINET_REG_BATTERY_POWER_COUNT];
    if (client.readInputRegisters(SUNGROW_WINET_REG_BATTERY_POWER, SUNGROW_WINET_REG_BATTERY_POWER_COUNT, batteryRegs)) {
        // Single S16 register, wire polarity positive=charging - negate for
        // the codebase's negative=discharging convention. See the comment on
        // SUNGROW_WINET_REG_BATTERY_POWER above for how this was corrected
        // from the original (wrong) S32 assumption.
        int16_t rawBattery = (int16_t)batteryRegs[0];
        float batteryReadingW = -(float)rawBattery;
        Serial.printf("[modbus] batteryRegs[0]=0x%04X raw=%d battery_power_w=%.0f (%s)\n",
                      batteryRegs[0], rawBattery, batteryReadingW,
                      batteryReadingW < 0 ? "discharging" : (batteryReadingW > 0 ? "charging" : "idle"));
        // Reject anything outside SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W as
        // a mismapped/garbage register read (see comment on
        // SUNGROW_WINET_REG_BATTERY_POWER) rather than trusting it - same
        // fallback-to-last-known-good treatment as a failed
        // readInputRegisters() call above.
        if (fabsf(batteryReadingW) <= SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W) {
            lastGoodBatteryW = batteryReadingW;
        } else {
            Serial.printf("[modbus] battery_power_w %.0f exceeds sanity bound (%.0f), rejecting - register mapping still unconfirmed, see CLAUDE.md\n",
                          batteryReadingW, SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W);
        }
    }
    outBatteryW = lastGoodBatteryW;
    return true;
}

const GridDataSource GRID_SOURCE_SUNGROW_WINET = {
    "sungrow_winet",
    "Sungrow WiNet-S (SH8.0RS)",
    sungrow_winet_read_power_w,
};
