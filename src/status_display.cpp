#include "status_display.h"
#include "config.h"
#include "shared_state.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <string.h>

static const char *connector_state_str(ConnectorState s) {
    switch (s) {
        case CONN_STATE_A: return "A";
        case CONN_STATE_B: return "B";
        case CONN_STATE_C: return "C";
        default: return "-";
    }
}

static void pad20(char *buf) {
    size_t len = strlen(buf);
    for (size_t i = len; i < LCD_COLS; i++) {
        buf[i] = ' ';
    }
    buf[LCD_COLS] = '\0';
}

void status_display_task(void *pvParameters) {
    (void)pvParameters;

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
    lcd.init();
    lcd.backlight();

    char line[LCD_COLS + 1];

    for (;;) {
        SolarStatus solar = shared_state_get_solar_status_blocking();
        CpStatus cp = shared_state_get_cp_status_blocking();
        uint32_t nowMs = millis();
        uint32_t ageS = (nowMs - solar.last_poll_success_ms) / 1000;

        lcd.setCursor(0, 0);
        if (solar.grid_power_w <= 0) {
            snprintf(line, sizeof(line), "Export %5.0fW", -solar.grid_power_w);
        } else {
            snprintf(line, sizeof(line), "Import %5.0fW", solar.grid_power_w);
        }
        pad20(line);
        lcd.print(line);

        lcd.setCursor(0, 1);
        if (cp.mode == MODE_ACTIVE && cp.duty_state == CP_OSCILLATING) {
            float amps = cp.applied_duty_pct / CP_DUTY_PCT_PER_AMP;
            snprintf(line, sizeof(line), "Tgt %4.1fA Duty%4.1f%%", amps, cp.applied_duty_pct);
        } else {
            snprintf(line, sizeof(line), "Tgt --.-A Duty --.-%%");
        }
        pad20(line);
        lcd.print(line);

        lcd.setCursor(0, 2);
        const char *cpStateStr = (cp.mode == MODE_BYPASS) ? "BYPASS"
                                  : (cp.duty_state == CP_OSCILLATING) ? "OSCILLATING" : "STANDBY";
        snprintf(line, sizeof(line), "CP:%s Conn:%s", cpStateStr, connector_state_str(cp.connector_state));
        pad20(line);
        lcd.print(line);

        lcd.setCursor(0, 3);
        snprintf(line, sizeof(line), "WiFi:%s %s Age:%lus",
                 solar.wifi_connected ? "OK" : "--",
                 solar.simulated ? "SIM" : (solar.modbus_ok ? "MB:OK" : "MB:--"),
                 (unsigned long)ageS);
        pad20(line);
        lcd.print(line);

        vTaskDelay(pdMS_TO_TICKS(LCD_REFRESH_MS));
    }
}
