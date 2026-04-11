/*******************************************************
  combined_all.ino — All Four Sensors
  Board: Arduino Nano ESP32

  Sensors:
    ICM-20689  IMU          (SPI)     -> ImuFall.h/.cpp
    MAX30102   HR + SpO2    (I2C)     -> PulseOx.h/.cpp
    GSR                     (Analog A0)
    DS18B20    Temperature  (OneWire GPIO4)

  Serial Monitor output every 500ms (115200 baud):
    Finger | IR | BPM | SpO2 | IMU | GSR | Temp

  Libraries (install via Library Manager):
    SparkFun MAX3010x Pulse and Proximity Sensor Library
    OneWire
    DallasTemperature
********************************************************/

#include <Arduino.h>
#include "ImuFall.h"
#include "PulseOx.h"
#include <OneWire.h>
#include <DallasTemperature.h>


//  Module instances

// IMU 
ImuFall::Config imuCfg;
// imuCfg.pinSck     = 13;
// imuCfg.pinMiso    = 12;
// imuCfg.pinMosi    = 11;
// imuCfg.pinCs      = 10;
// imuCfg.fallOutPin = 7;
ImuFall imu(imuCfg);

// PulseOx — all defaults are sensible, tweak in Config if needed
PulseOx::Config oxCfg;
PulseOx ox(oxCfg);


//  GSR
const int   GSR_PIN          = A0;
const int   GSR_CALIBRATION  = 2476;  // recalibrate: read raw with no contact
const int   GSR_NUM_SAMPLES  = 4;
const int   GSR_SAMPLE_DELAY = 2;
const float GSR_DENOM_MIN    = 5.0f;
float       lastGsrVal       = NAN;

float gsrRead() {
  long sum = 0;
  for (int i = 0; i < GSR_NUM_SAMPLES; i++) {
    sum += analogRead(GSR_PIN);
    delay(GSR_SAMPLE_DELAY);
  }
  float raw   = (float)sum / GSR_NUM_SAMPLES;
  float denom = (float)GSR_CALIBRATION - raw;
  if (denom < GSR_DENOM_MIN) return NAN;
  float R_ohm = ((1024.0f + 2.0f * raw) * 10000.0f) / denom;
  return (R_ohm > 0.0f) ? (1000000.0f / R_ohm) : NAN;
}


//  DS18B20
#define ONE_WIRE_BUS 2
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
bool  tempRequested = false;
float lastTempC     = NAN;


//  Timing
const uint32_t IMU_PERIOD_MS    = 10;
const uint32_t GSR_PERIOD_MS    = 50;
const uint32_t TEMP_PERIOD_MS   = 1000;
const uint32_t SERIAL_PERIOD_MS = 500;

uint32_t lastImuMs    = 0;
uint32_t lastGsrMs    = 0;
uint32_t lastTempMs   = 0;
uint32_t lastSerialMs = 0;


//  Setup
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("=== Combined All Sensors ===");

  if (!imu.begin()) {
    Serial.println("FATAL: IMU init failed. Check SPI wiring + WHO_AM_I.");
    while (1) delay(200);
  }
  Serial.println("IMU        OK");

  if (!ox.begin()) {
    Serial.println("FATAL: MAX30102 not found. Check I2C wiring.");
    while (1) delay(200);
  }
  Serial.println("MAX30102   OK");

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  if (tempSensor.getDeviceCount() == 0)
    Serial.println("WARNING:   No DS18B20 found. Check wiring.");
  else
    Serial.println("DS18B20    OK");

  Serial.println("GSR        OK (analog)");
  Serial.println("============================");

  tempSensor.requestTemperatures();
  tempRequested = true;

  uint32_t now = millis();
  lastImuMs = lastGsrMs = lastTempMs = lastSerialMs = now;
}


//  Loop
void loop() {
  uint32_t now = millis();

  // IMU (~100Hz)
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = now;
    imu.update(now);
  }

  // PulseOx — called every loop, sensor buffers internally
  ox.update(now);

  // GSR (20Hz)
  if (now - lastGsrMs >= GSR_PERIOD_MS) {
    lastGsrMs  = now;
    lastGsrVal = gsrRead();
  }

  // DS18B20 (1Hz, non-blocking)
  if (now - lastTempMs >= TEMP_PERIOD_MS) {
    lastTempMs = now;
    if (tempRequested) {
      float t   = tempSensor.getTempCByIndex(0);
      lastTempC = (t == DEVICE_DISCONNECTED_C) ? NAN : t;
    }
    tempSensor.requestTemperatures();
    tempRequested = true;
  }

  // Serial Monitor output
  if (now - lastSerialMs >= SERIAL_PERIOD_MS) {
    lastSerialMs = now;

    // Finger + HR + SpO2
    Serial.print("Finger: ");
    Serial.print(ox.fingerPresent() ? "YES" : "NO ");

    Serial.print("  |  IR: ");
    Serial.print(ox.ir());

    Serial.print("  |  BPM: ");
    if (ox.fingerPresent() && ox.bpm() > 0.0f)
      Serial.print(ox.bpm(), 1);
    else
      Serial.print("--  ");

    Serial.print("  |  SpO2: ");
    if (ox.fingerPresent() && ox.spo2() > 0.0f) {
      Serial.print(ox.spo2(), 1);
      Serial.print("%");
    } else {
      Serial.print("-- ");
    }

    // IMU
    Serial.print("  |  IMU |A|: ");
    Serial.print(imu.accelMagnitudeG(), 2);
    Serial.print("g  state: ");
    switch (imu.state()) {
      case ImuFall::MONITORING:     Serial.print("OK       "); break;
      case ImuFall::FREEFALL_SEEN:  Serial.print("FREEFALL "); break;
      case ImuFall::FALL_CONFIRMED: Serial.print("*** FALL ***"); break;
    }

    // GSR
    Serial.print("  |  GSR: ");
    if (isnan(lastGsrVal)) Serial.print("--     ");
    else { Serial.print(lastGsrVal, 2); Serial.print("uS"); }

    // Temp
    Serial.print("  |  Temp: ");
    if (isnan(lastTempC)) Serial.print("--");
    else { Serial.print(lastTempC, 1); Serial.print("C"); }

    Serial.println();
  }
}
