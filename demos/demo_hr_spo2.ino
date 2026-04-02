/*******************************************************
  HR + SpO2 Sketch — MAX30102
  Board: Arduino Nano ESP32

  BPM method : v2 reliable (refractory period, DC removal,
               rolling average, no outlier rejection)
  SpO2 method: original RMS ratio method from PulseOx.cpp
    

  Serial Monitor output (115200 baud) every 500ms:
    Finger: YES/NO
    IR    : raw IR value
    BPM   : rolling average (shows -- until 3 beats in)
    SpO2  : smoothed % (shows -- until window fills)

  Wiring:
    MAX30102 SDA -> A4
    MAX30102 SCL -> A5
    MAX30102 VIN -> 3.3V (external rail)
    MAX30102 GND -> GND

  Libraries:
    SparkFun MAX3010x Pulse and Proximity Sensor Library
********************************************************/

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <math.h>

MAX30105 sensor;

//BPM config (v2 method)
const uint32_t FINGER_THRESHOLD  = 15000;
const uint8_t  RATE_SIZE         = 8;
const uint16_t MIN_BPM           = 40;
const uint16_t MAX_BPM           = 150;
const uint32_t REFRACTORY_MS     = 400;
const float    DC_ALPHA          = 0.95f;

//SpO2 config (original PulseOx method) 
const float    SPO2_BETA         = 0.02f;   // DC tracking speed
const uint16_t SPO2_WINDOW       = 100;     // samples before SpO2 valid
const float    SPO2_MIN_AC_RMS   = 150.0f;  // minimum AC signal strength
const float    SPO2_ALPHA        = 0.10f;   // smoothing factor
const float    SPO2_MIN          = 70.0f;
const float    SPO2_MAX          = 100.0f;

// Timing 
const uint32_t SERIAL_INTERVAL_MS = 500;

// BPM state
float    dcLevel          = 0.0f;
float    rates[RATE_SIZE] = {0};
uint8_t  rateSpot         = 0;
uint8_t  rateCount        = 0;
float    bpmAvg           = 0.0f;
uint32_t lastBeatMs       = 0;

//  SpO2 state 
// Fixed arrays sized to SPO2_WINDOW
static float irDet[100]  = {0};
static float redDet[100] = {0};
uint16_t detIndex   = 0;
uint16_t detCount   = 0;
float    meanIR     = 0.0f;
float    meanRed    = 0.0f;
float    spo2Val    = 0.0f;
float    spo2Smooth = 0.0f;

// Timing state 
uint32_t lastSerialMs = 0;

//SpO2 helpers 
float rmsOfBuffer(const float* buf, uint16_t n) {
  if (n == 0) return 0.0f;
  double s = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    double v = buf[i];
    s += v * v;
  }
  return (float)sqrt(s / (double)n);
}

float spo2FromR(float R) {
  return (-45.060f * R * R) + (30.354f * R) + 97.845f;
}

// Reset all state on finger removal 
void resetAll() {
  dcLevel   = 0.0f;
  rateCount = 0;
  rateSpot  = 0;
  bpmAvg    = 0.0f;
  lastBeatMs = 0;

  meanIR     = 0.0f;
  meanRed    = 0.0f;
  spo2Val    = 0.0f;
  spo2Smooth = 0.0f;
  detIndex   = 0;
  detCount   = 0;
  for (uint16_t i = 0; i < SPO2_WINDOW; i++) {
    irDet[i]  = 0.0f;
    redDet[i] = 0.0f;
  }
}

//Setup 
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("=== HR + SpO2 Monitor ===");

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("FATAL: MAX30102 not found - check wiring");
    while (1) delay(200);
  }

  sensor.setup(
    0x1F,   // LED brightness
    4,      // 4x hardware averaging
    2,      // ledMode 2 = Red + IR (required for SpO2)
    400,    // 400 samples/sec
    215,    // pulse width
    4096    // ADC range
  );

  lastSerialMs = millis();
  Serial.println("Waiting for finger...");
}

//Loop
void loop() {
  uint32_t now   = millis();
  long irValue   = sensor.getIR();
  long redValue  = sensor.getRed();
  bool finger    = (irValue >= FINGER_THRESHOLD);

  if (!finger) {
    resetAll();

    if (now - lastSerialMs >= SERIAL_INTERVAL_MS) {
      lastSerialMs = now;
      Serial.println("Finger: NO  | IR: --  | BPM: --  | SpO2: --");
    }
    return;
  }

  // BPM (v2 method)
  dcLevel = DC_ALPHA * dcLevel + (1.0f - DC_ALPHA) * (float)irValue;

  if (checkForBeat(irValue)) {
    uint32_t ibiMs   = now - lastBeatMs;
    bool tooSoon     = (lastBeatMs != 0) && (ibiMs < REFRACTORY_MS);
    bool plausible   = (lastBeatMs == 0) ||
                       (ibiMs >= (60000u / MAX_BPM) &&
                        ibiMs <= (60000u / MIN_BPM));

    if (!tooSoon && plausible && lastBeatMs != 0) {
      float bpmInst       = 60000.0f / (float)ibiMs;
      rates[rateSpot]     = bpmInst;
      rateSpot            = (rateSpot + 1) % RATE_SIZE;
      if (rateCount < RATE_SIZE) rateCount++;

      float sum = 0;
      for (uint8_t i = 0; i < rateCount; i++) sum += rates[i];
      bpmAvg = sum / (float)rateCount;
    }

    if (!tooSoon) lastBeatMs = now;
  } 

  // SpO2  
  meanIR  = (1.0f - SPO2_BETA) * meanIR  + SPO2_BETA * (float)irValue;
  meanRed = (1.0f - SPO2_BETA) * meanRed + SPO2_BETA * (float)redValue;

  irDet[detIndex]  = (float)irValue  - meanIR;
  redDet[detIndex] = (float)redValue - meanRed;
  detIndex = (detIndex + 1) % SPO2_WINDOW;
  if (detCount < SPO2_WINDOW) detCount++;

  float acIR  = rmsOfBuffer(irDet,  detCount);
  float acRed = rmsOfBuffer(redDet, detCount);

  bool spo2Valid = (detCount >= SPO2_WINDOW) &&
                   (meanIR  > 1.0f) && (meanRed > 1.0f) &&
                   (acIR  > SPO2_MIN_AC_RMS) &&
                   (acRed > SPO2_MIN_AC_RMS);

  if (spo2Valid) {
    float R = (acRed / meanRed) / (acIR / meanIR);
    float s = spo2FromR(R);
    s = constrain(s, SPO2_MIN, SPO2_MAX);
    spo2Val = s;
    if (spo2Smooth == 0.0f) spo2Smooth = spo2Val;
    spo2Smooth = (1.0f - SPO2_ALPHA) * spo2Smooth + SPO2_ALPHA * spo2Val;
  }

  // Serial Monitor output 
  if (now - lastSerialMs >= SERIAL_INTERVAL_MS) {
    lastSerialMs = now;

    Serial.print("Finger: YES");

    Serial.print("  |  IR: ");
    Serial.print(irValue);

    Serial.print("  |  BPM: ");
    if (rateCount >= 3) Serial.print(bpmAvg, 1);
    else                Serial.print("--");

    Serial.print("  |  SpO2: ");
    if (spo2Smooth > 0.0f) { Serial.print(spo2Smooth, 1); Serial.print("%"); }
    else                     Serial.print("--");

    Serial.println();
  }
}

