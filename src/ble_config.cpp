#include "ble_config.h"
#include "config.h"
#include "shared_state.h"
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <WiFi.h>
#include <string.h>
#include <strings.h>

// Nordic UART Service (NUS) UUIDs - BLE serial-terminal apps auto-detect this pair and
// switch into terminal mode, instead of a raw per-characteristic GATT browser. RX is
// phone->device, TX is device->phone.
static const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHAR_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static const char *NVS_NAMESPACE = "netcfg";

// Gates every SET/COMMIT command behind RuntimeConfig.ble_password (never OTA_PASSWORD),
// sent first as "AUTH <password>". A single global flag is sufficient - single-operator
// device, not multi-tenant - and reset on every new connection.
static bool s_authenticated = false;

// Staged (not-yet-committed) config values. Pre-loaded from NVS at boot so
// a client that only changes one field and sends COMMIT doesn't
// accidentally blank out the others.
static char s_staged_ssid[33] = {0};
static char s_staged_password[64] = {0};
static char s_staged_inverter_ip[16] = {0};

static uint32_t s_boot_ms = 0;
static NimBLEAdvertising *s_advertising = nullptr;
static NimBLECharacteristic *s_txChar = nullptr;

// Tracks whether the current connection has ever sent a complete line, so
// ble_periodic_task can nudge a client that connected but doesn't know the
// protocol yet - there's no LCD or other feedback that BLE is even up.
static bool s_connected = false;
static bool s_hasInteracted = false;
static uint32_t s_lastNudgeMs = 0;

// Incoming-line assembly buffer for RxCallbacks::onWrite - a BLE write can
// deliver a partial line, a full line, or several lines at once depending
// on the terminal app's packetization, so bytes are accumulated here until
// a '\n' completes a command.
static char s_rxLineBuf[96];
static size_t s_rxLineLen = 0;

static void load_config_from_nvs() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, true);
    String ssid = prefs.getString("ssid", "");
    String password = prefs.getString("pass", "");
    String ip = prefs.getString("ip", "");
    String blePassword = prefs.getString("blepass", "");
    String webPassword = prefs.getString("webpass", "");
    prefs.end();

    RuntimeConfig cfg = {};
    snprintf(cfg.ssid, sizeof(cfg.ssid), "%s", ssid.c_str());
    snprintf(cfg.password, sizeof(cfg.password), "%s", password.c_str());
    snprintf(cfg.inverter_ip, sizeof(cfg.inverter_ip), "%s", ip.c_str());
    snprintf(cfg.ble_password, sizeof(cfg.ble_password), "%s", blePassword.c_str());
    snprintf(cfg.web_password, sizeof(cfg.web_password), "%s", webPassword.c_str());
    cfg.provisioned = ssid.length() > 0;
    shared_state_init_runtime_config(cfg);

    // Local staged mirrors for the WiFi/inverter-IP fields only - still
    // needed for the atomic COMMIT flow below. ble_password/web_password
    // have no staged mirror since SET BLE_PASS/SET WEB_PASS apply immediately.
    snprintf(s_staged_ssid, sizeof(s_staged_ssid), "%s", cfg.ssid);
    snprintf(s_staged_password, sizeof(s_staged_password), "%s", cfg.password);
    snprintf(s_staged_inverter_ip, sizeof(s_staged_inverter_ip), "%s", cfg.inverter_ip);
}

static void save_wifi_config_to_nvs(const char *ssid, const char *password, const char *ip) {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", password);
    prefs.putString("ip", ip);
    prefs.end();
}

static void save_ble_password_to_nvs(const char *password) {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putString("blepass", password);
    prefs.end();
}

static void save_web_password_to_nvs(const char *password) {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putString("webpass", password);
    prefs.end();
}

// An unprovisioned device (no SSID ever committed) advertises indefinitely,
// regardless of BLE_ADVERTISE_WINDOW_MS - BLE is the only way to configure
// WiFi at all, so a time limit here could strand a factory-fresh board out
// of range of its one configuration path. Once provisioned, advertising is
// only open for BLE_ADVERTISE_WINDOW_MS after each boot.
static bool should_be_advertising() {
    RuntimeConfig cfg = shared_state_get_runtime_config();
    if (!cfg.provisioned) {
        return true;
    }
    return (millis() - s_boot_ms) < BLE_ADVERTISE_WINDOW_MS;
}

static void update_advertising() {
    if (s_advertising == nullptr) {
        return;
    }
    bool want = should_be_advertising();
    bool isAdvertising = s_advertising->isAdvertising();
    if (want && !isAdvertising) {
        s_advertising->start();
    } else if (!want && isAdvertising) {
        s_advertising->stop();
    }
}

static void send_line(const char *text) {
    if (s_txChar == nullptr) {
        return;
    }
    std::string out(text);
    out += "\r\n";
    s_txChar->setValue(out);
    s_txChar->notify();
}

// If no BLE password has ever been set (RuntimeConfig.ble_password empty),
// authenticate unconditionally - mirrors the same "wide open until first
// provisioned" bootstrap this project already uses for WiFi (see
// should_be_advertising() above), since there is otherwise no way to set
// that first password at all. Once one exists, an exact match is required.
static void handle_auth(const char *arg) {
    RuntimeConfig cfg = shared_state_get_runtime_config();
    if (cfg.ble_password[0] == '\0') {
        s_authenticated = true;
    } else {
        s_authenticated = (strcmp(arg, cfg.ble_password) == 0);
    }
    send_line(s_authenticated ? "OK" : "ERR bad password");
}

static void handle_set_wifi_ssid(const char *arg) {
    if (!s_authenticated) {
        send_line("ERR not authenticated");
        return;
    }
    if (arg[0] == '\0' || strlen(arg) > sizeof(s_staged_ssid) - 1) {
        send_line("ERR invalid ssid");
        return;
    }
    snprintf(s_staged_ssid, sizeof(s_staged_ssid), "%s", arg);
    send_line("OK");
}

static void handle_set_wifi_pass(const char *arg) {
    if (!s_authenticated) {
        send_line("ERR not authenticated");
        return;
    }
    if (strlen(arg) > sizeof(s_staged_password) - 1) {
        send_line("ERR too long");
        return;
    }
    snprintf(s_staged_password, sizeof(s_staged_password), "%s", arg);
    send_line("OK");
}

static void handle_set_inverter_ip(const char *arg) {
    if (!s_authenticated) {
        send_line("ERR not authenticated");
        return;
    }
    IPAddress ip;
    if (strlen(arg) > sizeof(s_staged_inverter_ip) - 1 || !ip.fromString(arg)) {
        send_line("ERR invalid ip"); // reject anything that doesn't parse as IPv4
        return;
    }
    snprintf(s_staged_inverter_ip, sizeof(s_staged_inverter_ip), "%s", arg);
    send_line("OK");
}

// The one atomic "apply" step: persists the currently staged values to NVS
// and publishes them into shared_state's RuntimeConfig together, so
// solar_control_task never observes a half-updated config (e.g. a new SSID
// paired with the old password) even if a client sent fields one at a time
// before committing.
static void handle_commit() {
    if (!s_authenticated) {
        send_line("ERR not authenticated");
        return;
    }
    if (s_staged_ssid[0] == '\0') {
        send_line("ERR no ssid staged");
        return;
    }
    save_wifi_config_to_nvs(s_staged_ssid, s_staged_password, s_staged_inverter_ip);
    shared_state_set_wifi_config(s_staged_ssid, s_staged_password, s_staged_inverter_ip);
    send_line("OK");
}

// SET BLE_PASS/SET WEB_PASS apply immediately (their own NVS write +
// shared_state update) rather than staging behind COMMIT - each is an
// independent single value, not part of the WiFi atomic-apply group above.
static void handle_set_ble_pass(const char *arg) {
    if (!s_authenticated) {
        send_line("ERR not authenticated");
        return;
    }
    if (strlen(arg) > 63) {
        send_line("ERR too long");
        return;
    }
    save_ble_password_to_nvs(arg);
    shared_state_set_ble_password(arg);
    send_line("OK");
}

static void handle_set_web_pass(const char *arg) {
    if (!s_authenticated) {
        send_line("ERR not authenticated");
        return;
    }
    if (strlen(arg) > 63) {
        send_line("ERR too long");
        return;
    }
    save_web_password_to_nvs(arg);
    shared_state_set_web_password(arg);
    send_line("OK");
}

// Never includes the staged WiFi password or either BLE/web password - STATUS needs no
// auth (nothing it reports is secret), so this is the one place a leak could sneak in.
//
// Sent as ONE multi-line notify() rather than one call per line: separate notify()s
// outrace the actual radio transmission (BLE only sends at its connection interval), so
// the terminal app can render lines interleaved/garbled (bench-observed) - a single
// notify() with embedded "\r\n"s avoids that and fits in one packet under the 247-byte MTU.
static void handle_status() {
    RuntimeConfig cfg = shared_state_get_runtime_config();
    String deviceIp = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("0.0.0.0");

    char buf[192];
    snprintf(buf, sizeof(buf),
        "ssid=%s\r\n"
        "inverter_ip=%s\r\n"
        "device_ip=%s\r\n"
        "provisioned=%d auth=%d\r\n"
        "blepass_set=%d webpass_set=%d",
        s_staged_ssid, s_staged_inverter_ip, deviceIp.c_str(),
        cfg.provisioned ? 1 : 0, s_authenticated ? 1 : 0,
        cfg.ble_password[0] != '\0' ? 1 : 0, cfg.web_password[0] != '\0' ? 1 : 0);
    send_line(buf);
}

// See handle_status()'s comment above for why this is one send_line() call
// with embedded "\r\n"s rather than several separate calls.
static void handle_help() {
    send_line(
        "Commands:\r\n"
        "  AUTH <pw>          (pw = whatever SET BLE_PASS last set)\r\n"
        "  SET BLE_PASS <pw>  (sets the AUTH password)\r\n"
        "  SET WIFI_SSID <ssid>\r\n"
        "  SET WIFI_PASS <pw>\r\n"
        "  SET INVERTER_IP <ip>\r\n"
        "  COMMIT\r\n"
        "  SET WEB_PASS <pw>\r\n"
        "  GET STATUS\r\n"
        "  HELP");
}

// Splits "line" on the first space into a leading token and the remainder
// (remainder is "" if there's no space), skipping any extra leading spaces
// on the remainder. Used twice per line: once for <command> [rest], and
// again inside SET/GET to split [field] [value] out of that rest.
static char *split_first_token(char *line, char **rest_out) {
    char *token = line;
    while (*token == ' ') {
        token++;
    }
    char *rest = strchr(token, ' ');
    if (rest != nullptr) {
        *rest = '\0';
        rest++;
        while (*rest == ' ') {
            rest++;
        }
    } else {
        rest = token + strlen(token);
    }
    *rest_out = rest;
    return token;
}

static void process_line(char *line) {
    s_hasInteracted = true;

    char *rest = nullptr;
    char *cmd = split_first_token(line, &rest);

    if (cmd[0] == '\0') {
        return; // blank line, ignore
    } else if (strcasecmp(cmd, "AUTH") == 0) {
        handle_auth(rest);
    } else if (strcasecmp(cmd, "COMMIT") == 0) {
        handle_commit();
    } else if (strcasecmp(cmd, "HELP") == 0) {
        handle_help();
    } else if (strcasecmp(cmd, "SET") == 0) {
        char *value = nullptr;
        char *field = split_first_token(rest, &value);
        if (strcasecmp(field, "WIFI_SSID") == 0) {
            handle_set_wifi_ssid(value);
        } else if (strcasecmp(field, "WIFI_PASS") == 0) {
            handle_set_wifi_pass(value);
        } else if (strcasecmp(field, "INVERTER_IP") == 0) {
            handle_set_inverter_ip(value);
        } else if (strcasecmp(field, "BLE_PASS") == 0) {
            handle_set_ble_pass(value);
        } else if (strcasecmp(field, "WEB_PASS") == 0) {
            handle_set_web_pass(value);
        } else {
            send_line("ERR unknown field, try HELP");
        }
    } else if (strcasecmp(cmd, "GET") == 0) {
        if (strcasecmp(rest, "STATUS") == 0) {
            handle_status();
        } else {
            send_line("ERR unknown field, try HELP");
        }
    } else {
        send_line("ERR unknown command, try HELP");
    }
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pServer) override {
        (void)pServer;
        s_connected = true;
        s_hasInteracted = false;
        s_lastNudgeMs = millis();
    }
    void onDisconnect(NimBLEServer *pServer) override {
        (void)pServer;
        s_authenticated = false;
        s_connected = false;
        s_hasInteracted = false;
        s_rxLineLen = 0; // discard any partial line from the dropped connection
        update_advertising();
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        for (char c : value) {
            if (c == '\n') {
                s_rxLineBuf[s_rxLineLen] = '\0';
                if (s_rxLineLen > 0 && s_rxLineBuf[s_rxLineLen - 1] == '\r') {
                    s_rxLineBuf[s_rxLineLen - 1] = '\0';
                }
                process_line(s_rxLineBuf);
                s_rxLineLen = 0;
            } else if (s_rxLineLen < sizeof(s_rxLineBuf) - 1) {
                s_rxLineBuf[s_rxLineLen++] = c;
            } else {
                s_rxLineLen = 0; // line too long: drop it rather than process garbage
            }
        }
    }
};

// Re-evaluates the advertising window and nudges an idle connected client
// on a ~1s cadence. Not timing-critical - Core 1's real-time path never
// touches this task or anything it owns.
static void ble_periodic_task(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
        update_advertising();

        if (s_connected && !s_hasInteracted &&
            (millis() - s_lastNudgeMs) >= BLE_IDLE_HELP_INTERVAL_MS) {
            s_lastNudgeMs = millis();
            send_line("Connected. Send 'HELP' for instructions.");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ble_config_init() {
    load_config_from_nvs();
    s_boot_ms = millis();

    NimBLEDevice::init(BLE_DEVICE_NAME);
    // Request a larger MTU up front so STATUS's multi-field reply lines
    // don't risk truncating against the default 23-byte (20-byte payload)
    // ATT MTU before negotiation completes.
    NimBLEDevice::setMTU(247);

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService *service = server->createService(SERVICE_UUID);

    NimBLECharacteristic *rxChar = service->createCharacteristic(
        CHAR_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxChar->setCallbacks(new RxCallbacks());

    s_txChar = service->createCharacteristic(CHAR_TX_UUID, NIMBLE_PROPERTY::NOTIFY);

    service->start();

    s_advertising = NimBLEDevice::getAdvertising();
    s_advertising->addServiceUUID(SERVICE_UUID);
    update_advertising();

    xTaskCreatePinnedToCore(ble_periodic_task, "ble_periodic", 4096, nullptr,
                             TASK_PRIO_BLE, nullptr, CORE_SOLAR);
}
