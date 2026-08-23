#include "modbus_tcp_client.h"
#include "config.h"
#include <WiFi.h>

ModbusTcpClient::ModbusTcpClient(IPAddress ip, uint16_t port, uint8_t unitId, uint32_t timeoutMs)
    : _ip(ip), _port(port), _unitId(unitId), _timeoutMs(timeoutMs), _transactionId(0) {}

bool ModbusTcpClient::readInputRegisters(uint16_t startAddr, uint16_t count, uint16_t *outRegs) {
    if (count == 0 || count > 125) {
        return false;
    }

    WiFiClient client;
    client.setTimeout(_timeoutMs);
    if (!client.connect(_ip, _port, _timeoutMs)) {
        return false;
    }

    uint16_t txnId = ++_transactionId;

    uint8_t request[12];
    request[0] = (uint8_t)(txnId >> 8);
    request[1] = (uint8_t)(txnId & 0xFF);
    request[2] = 0x00; // protocol id hi
    request[3] = 0x00; // protocol id lo
    request[4] = 0x00; // length hi
    request[5] = 0x06; // length lo (unit id + PDU)
    request[6] = _unitId;
    request[7] = 0x04; // function code: read input registers
    request[8] = (uint8_t)(startAddr >> 8);
    request[9] = (uint8_t)(startAddr & 0xFF);
    request[10] = (uint8_t)(count >> 8);
    request[11] = (uint8_t)(count & 0xFF);

    size_t written = client.write(request, sizeof(request));
    if (written != sizeof(request)) {
        client.stop();
        return false;
    }

    // MBAP header (7) + function code (1) + byte count (1) + data
    uint8_t header[9];
    uint32_t start = millis();
    size_t headerRead = 0;
    while (headerRead < sizeof(header) && (millis() - start) < _timeoutMs) {
        if (client.available()) {
            int b = client.read();
            if (b < 0) {
                break;
            }
            header[headerRead++] = (uint8_t)b;
        } else {
            delay(1);
        }
    }
    if (headerRead != sizeof(header)) {
        client.stop();
        return false;
    }

    uint16_t respTxnId = ((uint16_t)header[0] << 8) | header[1];
    uint8_t respUnitId = header[6];
    uint8_t functionCode = header[7];
    uint8_t byteCount = header[8];

    if (respTxnId != txnId || respUnitId != _unitId) {
        client.stop();
        return false;
    }
    if (functionCode & 0x80) {
        // Exception response
        client.stop();
        return false;
    }
    if (functionCode != 0x04 || byteCount != count * 2) {
        client.stop();
        return false;
    }

    uint8_t data[250];
    size_t dataRead = 0;
    start = millis();
    while (dataRead < byteCount && (millis() - start) < _timeoutMs) {
        if (client.available()) {
            int b = client.read();
            if (b < 0) {
                break;
            }
            data[dataRead++] = (uint8_t)b;
        } else {
            delay(1);
        }
    }
    client.stop();
    if (dataRead != byteCount) {
        return false;
    }

    for (uint16_t i = 0; i < count; i++) {
        outRegs[i] = ((uint16_t)data[i * 2] << 8) | data[i * 2 + 1];
    }
    return true;
}

bool modbus_read_grid_power_w(IPAddress ip, float &outWatts) {
    ModbusTcpClient client(ip, MODBUS_TCP_PORT, MODBUS_UNIT_ID, MODBUS_TIMEOUT_MS);
    uint16_t regs[REG_GRID_POWER_COUNT];
    if (!client.readInputRegisters(REG_GRID_POWER, REG_GRID_POWER_COUNT, regs)) {
        return false;
    }
    // Confirmed against the real WiNet-S during bring-up: this register pair
    // is low-word-first (little-endian word order), not big-endian as
    // originally assumed - see CLAUDE.md.
    int32_t raw = ((int32_t)((uint32_t)regs[1] << 16 | regs[0]));
    outWatts = (float)raw;
    return true;
}
