#include "grid_data_source.h"
#include "modbus_tcp_client.h"

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
#define SUNGROW_WINET_MODBUS_TIMEOUT_MS   2000

static bool sungrow_winet_read_power_w(IPAddress host, float &outWatts) {
    ModbusTcpClient client(host, SUNGROW_WINET_MODBUS_TCP_PORT, SUNGROW_WINET_MODBUS_UNIT_ID,
                            SUNGROW_WINET_MODBUS_TIMEOUT_MS);
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
    return true;
}

const GridDataSource GRID_SOURCE_SUNGROW_WINET = {
    "Sungrow WiNet-S (SH8.0RS)",
    sungrow_winet_read_power_w,
};
