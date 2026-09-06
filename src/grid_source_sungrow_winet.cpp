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
// Standalone register (not the low half of a 13021-13022 S32 pair - that assumption gave
// a garbage 58M W reading), and MAGNITUDE-ONLY on this firmware: it reads positive while
// charging and positive while discharging alike, so it carries no direction of its own.
// Direction comes from SUNGROW_WINET_REG_POWER_FLOW_STATUS below instead.
//
// Three 2026-09-04/05 attempts to fix this by picking a sign convention for 13021 alone
// (negate / don't negate / negate again) each looked right against whichever direction
// happened to be observed and wrong against the other - see CLAUDE.md "Resolved". No
// single-register convention can be right in both directions when the register has no
// sign to begin with. Matches mkaiser's Sungrow register notes ("old firmware (before
// october 2024): always positive battery power") and evcc's Sungrow template, which
// likewise derives the sign from the power-flow bits.
//
// Directly confirmed on this hardware 2026-09-06 with tools/probe_sungrow.py, in one
// capture spanning a live discharge -> charge transition: 13021 read +2654 W falling to 0
// while discharging, then +419 W rising to +1794 W while charging - positive throughout,
// never negative - while 13000 flipped 0x001D -> 0x001B (discharging bit 0x04 clearing,
// charging bit 0x02 setting) at exactly the changeover. Both directions, one continuous
// log; the missing half of every previous attempt.
#define SUNGROW_WINET_REG_BATTERY_POWER      13021
#define SUNGROW_WINET_REG_BATTERY_POWER_COUNT 1
// Power Flow Status bitfield - the direction source for 13021 above. Other bits (PV
// generating, import/export, load sign) exist but aren't needed here.
#define SUNGROW_WINET_REG_POWER_FLOW_STATUS       13000
#define SUNGROW_WINET_REG_POWER_FLOW_STATUS_COUNT 1
#define SUNGROW_WINET_FLOW_BIT_BATTERY_CHARGING    0x0002
#define SUNGROW_WINET_FLOW_BIT_BATTERY_DISCHARGING 0x0004
// With neither direction bit set, anything under this is taken at face value as an idle
// battery. Above it, a missing direction bit means the status register isn't describing
// real flow (it reads stuck at zero on some models - not this one, whose bits track
// correctly), so the reading is rejected rather than guessed at - outBatteryDataValid
// then goes false and the control law freezes.
#define SUNGROW_WINET_BATTERY_IDLE_EPSILON_W 100.0f
// Alternative not taken: registers 5213-5214 ("battery power, wide range", S32, low word
// first) ARE present on this inverter and ARE genuinely signed - the same 2026-09-06
// capture showed them at +2654 W discharging and -419/-1020/-1542/-1794 W charging, i.e.
// negative=charging, and magnitudes identical to 13021 throughout. Sungrow's own datasheet
// recommends them over 13021. Left unused because 13021+13000 is what got directly
// confirmed in both directions and needs no new word-order assumption, and because 5213
// is absent on some models ("illegal data address"). Worth switching to if 13000 ever
// proves unreliable; a cross-check between the two was considered and rejected, since the
// two reads are milliseconds apart on a value that can swing >600 W in 5 s, so honest
// skew would trip spurious freezes mid-charge.
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
    //
    // Takes the magnitude from 13021 and the direction from 13000's bits - see those
    // #defines above for why 13021's own sign can't be used. Both reads have to land for
    // the pair to mean anything, so they're gated together.
    static float lastGoodBatteryW = 0.0f;
    static bool batteryEverReadOk = false;
    static uint32_t lastBatteryReadOkMs = 0;
    uint16_t flowRegs[SUNGROW_WINET_REG_POWER_FLOW_STATUS_COUNT];
    uint16_t batteryRegs[SUNGROW_WINET_REG_BATTERY_POWER_COUNT];
    if (client.readInputRegisters(SUNGROW_WINET_REG_POWER_FLOW_STATUS, SUNGROW_WINET_REG_POWER_FLOW_STATUS_COUNT, flowRegs) &&
        client.readInputRegisters(SUNGROW_WINET_REG_BATTERY_POWER, SUNGROW_WINET_REG_BATTERY_POWER_COUNT, batteryRegs)) {
        uint16_t flowStatus = flowRegs[0];
        // fabsf() rather than a plain cast so this stays correct if the inverter is ever
        // updated to the firmware that reports 13021 signed: there the direction bit just
        // agrees with the sign the register already carried.
        float magnitudeW = fabsf((float)(int16_t)batteryRegs[0]);
        bool charging = (flowStatus & SUNGROW_WINET_FLOW_BIT_BATTERY_CHARGING) != 0;
        bool discharging = (flowStatus & SUNGROW_WINET_FLOW_BIT_BATTERY_DISCHARGING) != 0;

        float batteryReadingW = 0.0f;
        bool directionKnown = false;
        if (charging != discharging) {
            batteryReadingW = charging ? magnitudeW : -magnitudeW;
            directionKnown = true;
        } else if (!charging && magnitudeW <= SUNGROW_WINET_BATTERY_IDLE_EPSILON_W) {
            // Neither bit set and nothing meaningful flowing: a genuinely idle battery,
            // which is a trustworthy 0 W. (Both bits set falls through as unknown.)
            directionKnown = true;
        }

        Serial.printf("[modbus] flowStatus=0x%04X batteryRegs[0]=0x%04X magnitude=%.0f battery_power_w=%.0f (%s)\n",
                      flowStatus, batteryRegs[0], magnitudeW, batteryReadingW,
                      !directionKnown ? "direction unknown"
                                      : (batteryReadingW < 0 ? "discharging"
                                                             : (batteryReadingW > 0 ? "charging" : "idle")));
        // Reject an unknown direction or an out-of-bounds magnitude as a mismapped/garbage
        // read rather than trusting it - same fallback-to-last-known-good treatment as a
        // failed read above, and outBatteryDataValid ages out from here on its own.
        if (directionKnown && magnitudeW <= SUNGROW_WINET_BATTERY_POWER_SANITY_MAX_W) {
            lastGoodBatteryW = batteryReadingW;
            batteryEverReadOk = true;
            lastBatteryReadOkMs = millis();
        } else {
            Serial.printf("[modbus] battery reading rejected (%s)\n",
                          !directionKnown ? "no usable direction bits"
                                          : "magnitude exceeds sanity bound");
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
