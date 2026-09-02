#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiClient.h>

// Minimal, blocking Modbus TCP client. Holds one persistent connection open
// across calls and reuses it rather than reconnecting every request; any
// failure (write error, timeout, malformed/mismatched response) closes the
// socket so the next call starts from a clean reconnect. Never holds more
// than one connection open at a time.
class ModbusTcpClient {
public:
    ModbusTcpClient(IPAddress ip, uint16_t port, uint8_t unitId, uint32_t timeoutMs);

    // Reads `count` input registers (function code 0x04) starting at
    // `startAddr` into `outRegs` (must hold `count` uint16_t). Returns true
    // on success.
    bool readInputRegisters(uint16_t startAddr, uint16_t count, uint16_t *outRegs);

    // Repoints the client at a new host. If `ip` differs from the current
    // target, drops any open connection so the next call reconnects to the
    // new address.
    void setHost(IPAddress ip);

private:
    bool ensureConnected();

    IPAddress _ip;
    uint16_t _port;
    uint8_t _unitId;
    uint32_t _timeoutMs;
    uint16_t _transactionId;
    WiFiClient _client;
};
