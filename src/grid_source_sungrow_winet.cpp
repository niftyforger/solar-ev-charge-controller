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
#define SUNGROW_WINET_MODBUS_TIMEOUT_MS   2000

static bool sungrow_winet_read_power_w(IPAddress host, float /*currentDrawW*/,
                                         float &outWatts, float &outVoltageV) {
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
    return true;
}

const GridDataSource GRID_SOURCE_SUNGROW_WINET = {
    "sungrow_winet",
    "Sungrow WiNet-S (SH8.0RS)",
    sungrow_winet_read_power_w,
};
