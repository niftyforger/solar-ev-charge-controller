#include "grid_data_source.h"
#include "modbus_tcp_client.h"

// DTSU666-20 meter, wired via RS485 directly to the Sungrow SH8.0RS as its
// smart meter. We don't poll it directly - it's already a Modbus RTU slave
// to the inverter, and RTU only supports one bus master - so this reads the
// figure back out of the SH8.0RS's own Modbus TCP interface (via the
// WiNet-S dongle on the LAN) instead. See CLAUDE.md "Solar data source".
#define DTSU666_MODBUS_TCP_PORT     502
#define DTSU666_MODBUS_UNIT_ID      1
#define DTSU666_REG_GRID_POWER      13009   // S32, W, raw register is positive = exporting;
                                              // dtsu666_read_power_w() negates it so
                                              // software-facing values are negative = exporting
#define DTSU666_REG_GRID_POWER_COUNT 2
#define DTSU666_MODBUS_TIMEOUT_MS   2000

static bool dtsu666_read_power_w(IPAddress host, float &outWatts) {
    ModbusTcpClient client(host, DTSU666_MODBUS_TCP_PORT, DTSU666_MODBUS_UNIT_ID,
                            DTSU666_MODBUS_TIMEOUT_MS);
    uint16_t regs[DTSU666_REG_GRID_POWER_COUNT];
    if (!client.readInputRegisters(DTSU666_REG_GRID_POWER, DTSU666_REG_GRID_POWER_COUNT, regs)) {
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

const GridDataSource GRID_SOURCE_DTSU666 = {
    "DTSU666 (via WiNet-S)",
    dtsu666_read_power_w,
};
