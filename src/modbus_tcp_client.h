#pragma once

#include <Arduino.h>
#include <IPAddress.h>

// Minimal, blocking Modbus TCP client. Opens a short-lived connection per
// request rather than holding one open - the WiNet-S dongle's Modbus TCP
// gateway is assumed to support only a small number of concurrent clients,
// so we don't want to permanently occupy a slot.
class ModbusTcpClient {
public:
    ModbusTcpClient(IPAddress ip, uint16_t port, uint8_t unitId, uint32_t timeoutMs);

    // Reads `count` input registers (function code 0x04) starting at
    // `startAddr` into `outRegs` (must hold `count` uint16_t). Returns true
    // on success.
    bool readInputRegisters(uint16_t startAddr, uint16_t count, uint16_t *outRegs);

private:
    IPAddress _ip;
    uint16_t _port;
    uint8_t _unitId;
    uint32_t _timeoutMs;
    uint16_t _transactionId;
};

// Convenience helper: reads REG_GRID_POWER (2 registers, S32, little-endian
// word order, 1W/count) and returns it as signed watts, negative = exporting
// (the raw register's own native polarity is the opposite - positive =
// exporting - this function negates it so callers see negative = exporting
// consistently; see CLAUDE.md).
bool modbus_read_grid_power_w(IPAddress ip, float &outWatts);
