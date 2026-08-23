#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Example code that works for my LCD

#define SDA_PIN 8
#define SCL_PIN 9

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Hello!");
  lcd.setCursor(0, 1);
  lcd.print("ESP32-S3");
}

void loop() {
}