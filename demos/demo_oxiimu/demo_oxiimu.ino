#include <Arduino.h>
#include "ImuFall.h"
#include "PulseOx.h"
#include <Wire.h>
#include <rgb_lcd.h>

ImuFall::Config imuCfg;
PulseOx::Config oxCfg;

ImuFall imu(imuCfg);
PulseOx ox(oxCfg);
rgb_lcd lcd;

static const uint32_t IMU_PERIOD_MS    = 10;
static const uint32_t OX_PERIOD_MS     = 10;
static const uint32_t SERIAL_PERIOD_MS = 200;
static const uint32_t LCD_PERIOD_MS    = 500;
static const uint32_t FALL_HOLD_MS     = 2000;

uint32_t lastImuMs    = 0;
uint32_t lastOxMs     = 0;
uint32_t lastSerialMs = 0;
uint32_t lastLcdMs    = 0;
uint32_t fallShownAt  = 0;
bool     fallScreenUp = false;

String lcdField(String s, uint8_t w) {
  while (s.length() < w) s += ' ';
  if (s.length() > w) s = s.substring(0, w);
  return s;
}

void showFallScreen() {
  lcd.setRGB(255, 0, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("** FALL DETECT **");
  lcd.setCursor(0, 1);
  lcd.print("Check on patient");
  fallScreenUp = true;
  fallShownAt  = millis();
}

void showNormalScreen() {
  lcd.setRGB(0, 255, 0);
  lcd.clear();
  fallScreenUp = false;
}

void updateNormalLcd() {
  String irStr  = String(ox.ir());
  String bpmStr = (ox.fingerPresent() && ox.bpmSmoothed() > 0)
                  ? String((int)round(ox.bpmSmoothed()))
                  : String("--");

  String row0 = "IR:" + lcdField(irStr, 6) + " BPM:" + lcdField(bpmStr, 3);
  lcd.setCursor(0, 0);
  lcd.print(lcdField(row0, 16));

  String spo2Str;
  if (!ox.fingerPresent()) {
    spo2Str = "No finger       ";
    lcd.setRGB(0, 121, 255);
  } else if (ox.spo2Smoothed() > 0) {
    lcd.setRGB(0, 255, 0);
    spo2Str = "SpO2:" + lcdField(String((int)round(ox.spo2Smoothed())), 3) + "%       ";
    
  } else {
     lcd.setRGB(0, 255, 0);
    spo2Str = "SpO2: --        ";
  }
  lcd.setCursor(0, 1);
  lcd.print(lcdField(spo2Str, 16));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Initialising  ");
  lcd.setCursor(0, 1);
  lcd.print("  Please wait.. ");

  Serial.println("IMU fall + MAX30102 HR/SpO2");

  if (!imu.begin()) {
    Serial.println("IMU init failed (WHO_AM_I mismatch / wiring).");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("IMU FAILED");
    lcd.setCursor(0, 1); lcd.print("Check wiring");
    while (1) delay(200);
  }

  if (!ox.begin()) {
    Serial.println("MAX30102 init failed (I2C / wiring).");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("OX FAILED");
    lcd.setCursor(0, 1); lcd.print("Check wiring");
    while (1) delay(200);
  }

  lcd.clear();
  uint32_t now = millis();
  lastImuMs = lastOxMs = lastSerialMs = lastLcdMs = now;
}

void loop() {
  uint32_t now = millis();

  if (now - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = now;
    imu.update(now);
  }

  if (now - lastOxMs >= OX_PERIOD_MS) {
    lastOxMs = now;
    ox.update(now);
  }

  if (imu.fallLatched()) {
    if (!fallScreenUp) showFallScreen();
  } else if (fallScreenUp && (now - fallShownAt >= FALL_HOLD_MS)) {
    showNormalScreen();
  }

  if (!fallScreenUp && now - lastLcdMs >= LCD_PERIOD_MS) {
    lastLcdMs = now;
    updateNormalLcd();
  }

  if (now - lastSerialMs >= SERIAL_PERIOD_MS) {
    lastSerialMs = now;

    // IMU debug — state names printed explicitly for clarity
    Serial.print("IMU |A|=");
    Serial.print(imu.accelMagnitudeG(), 3);
    Serial.print(" state=");
    switch (imu.state()) {
      case ImuFall::MONITORING:      Serial.print("0");      break;
      case ImuFall::FREEFALL_SEEN:   Serial.print("1");   break;
      case ImuFall::FALL_CONFIRMED:  Serial.print("2");  break;
      case ImuFall::STILL_CONFIRMED: Serial.print("3"); break;
    }
    if (imu.fallLatched()) Serial.print(" *** FALL ***");

    Serial.print(" || ");

    if (!ox.fingerPresent()) {
      Serial.print("No finger IR=");
      Serial.print(ox.ir());
    } else {
      Serial.print("IR=");    Serial.print(ox.ir());
      Serial.print(" RED=");  Serial.print(ox.red());
      Serial.print(" BPM=");  Serial.print(ox.bpmInstant(),  1);
      Serial.print(" BPMs="); Serial.print(ox.bpmSmoothed(), 1);
      Serial.print(" SpO2=");
      if (ox.spo2Smoothed() > 0.0f) Serial.print(ox.spo2Smoothed(), 1);
      else                           Serial.print("...");
      Serial.print("%");
    }
    Serial.println();
  }
}
