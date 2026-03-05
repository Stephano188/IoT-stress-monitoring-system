#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <math.h>
#include <rgb_lcd.h>

MAX30105 particleSensor;
rgb_lcd lcd;

//Tunables 
static const uint8_t IBI_BUF_SIZE = 8;
static const uint32_t FINGER_IR_THRESHOLD = 50000;
static const uint16_t MIN_BPM = 40;
static const uint16_t MAX_BPM = 200;
static const uint16_t MIN_IBI_MS = 60000 / MAX_BPM;
static const uint16_t MAX_IBI_MS = 60000 / MIN_BPM;
static const float OUTLIER_RATIO = 0.20f;

//State (BPM) 
uint16_t ibiBuf[IBI_BUF_SIZE] = {0};
uint8_t  ibiIndex = 0;
uint8_t  ibiCount = 0;

uint32_t lastBeatMs = 0;
float bpmInstant = 0.0f;
float bpmSmoothed = 0.0f;

//SpO2 
static const uint16_t SPO2_WINDOW = 100;
static const float MIN_AC_RMS = 150.0f;
static const float SPO2_ALPHA = 0.10f;
static const float SPO2_MIN = 70.0f;
static const float SPO2_MAX = 100.0f;

float meanIR = 0.0f, meanRed = 0.0f;
float spo2 = 0.0f;
float spo2Smoothed = 0.0f;

float irDet[SPO2_WINDOW];
float redDet[SPO2_WINDOW];
uint16_t detIndex = 0;
uint16_t detCount = 0;

//LCD update timing
unsigned long lastLcdMs = 0;
static const unsigned long LCD_PERIOD_MS = 250;

//Helpers
static uint16_t medianOfBuffer(const uint16_t* buf, uint8_t n) {
  uint16_t tmp[IBI_BUF_SIZE];
  for (uint8_t i = 0; i < n; i++) tmp[i] = buf[i];

  for (uint8_t i = 1; i < n; i++) {
    uint16_t key = tmp[i];
    int8_t j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

static float bpmFromIbi(uint16_t ibiMs) {
  if (ibiMs == 0) return 0.0f;
  return 60000.0f / (float)ibiMs;
}

static bool fingerPresent(long irValue) {
  return irValue >= (long)FINGER_IR_THRESHOLD;
}

static bool ibiPlausible(uint32_t ibiMs) {
  return (ibiMs >= MIN_IBI_MS && ibiMs <= MAX_IBI_MS);
}

static void resetMeasurement() {
  ibiIndex = 0;
  ibiCount = 0;
  for (uint8_t i = 0; i < IBI_BUF_SIZE; i++) ibiBuf[i] = 0;
  lastBeatMs = 0;
  bpmInstant = 0.0f;
  bpmSmoothed = 0.0f;

  meanIR = meanRed = 0.0f;
  spo2 = 0.0f;
  spo2Smoothed = 0.0f;
  detIndex = 0;
  detCount = 0;
  for (uint16_t i = 0; i < SPO2_WINDOW; i++) {
    irDet[i] = 0.0f;
    redDet[i] = 0.0f;
  }
}

static void addIbi(uint16_t ibiMs) {
  ibiBuf[ibiIndex] = ibiMs;
  ibiIndex = (ibiIndex + 1) % IBI_BUF_SIZE;
  if (ibiCount < IBI_BUF_SIZE) ibiCount++;
}

static float computeSmoothedBpm() {
  if (ibiCount < 3) return 0.0f;
  uint16_t med = medianOfBuffer(ibiBuf, ibiCount);
  return bpmFromIbi(med);
}

static float rmsOfBuffer(const float* buf, uint16_t n) {
  if (n == 0) return 0.0f;
  double s = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    double v = buf[i];
    s += v * v;
  }
  return (float)sqrt(s / (double)n);
}

static float spo2FromR(float R) {
  return (-45.060f * R * R) + (30.354f * R) + 94.845f;
}

// LCD helper to avoid leftover characters
static void lcdPrintPadded(const char* s) {
  lcd.print(s);
  int len = strlen(s);
  for (int i = len; i < 16; i++) lcd.print(' ');
}

void setup() {
  // LCD init
  Wire.begin();
  lcd.begin(16, 2);
  lcd.setRGB(255, 255, 255);
  lcd.clear();
  lcdPrintPadded("Initializing...");
  lcd.setCursor(0, 1);
  lcdPrintPadded("MAX30102 + LCD");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    lcd.clear();
    lcdPrintPadded("MAX30102 FAIL");
    lcd.setCursor(0, 1);
    lcdPrintPadded("Check wiring");
    while (1) {}
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeIR(0x24);
  particleSensor.setPulseAmplitudeRed(0x12);
  particleSensor.setPulseAmplitudeGreen(0);

  resetMeasurement();

  delay(800);
  lcd.clear();
}

void loop() {
  long irValue  = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  bool finger = fingerPresent(irValue);

  if (!finger) {
    resetMeasurement();

    // Update LCD (not every loop)
    if (millis() - lastLcdMs >= LCD_PERIOD_MS) {
      lastLcdMs = millis();
      lcd.setRGB(255, 0, 0);
      lcd.setCursor(0, 0);
      lcdPrintPadded("No finger");
      lcd.setCursor(0, 1);
      char line2[17];
      snprintf(line2, sizeof(line2), "IR:%ld", irValue);
      lcdPrintPadded(line2);
    }

    delay(50);
    return;
  }

  // Beat detection (unchanged logic)
  if (checkForBeat(irValue)) {
    uint32_t now = millis();

    if (lastBeatMs != 0) {
      uint32_t ibiMs = now - lastBeatMs;

      if (ibiPlausible(ibiMs)) {
        float candidateBpm = bpmFromIbi((uint16_t)ibiMs);

        if (bpmSmoothed > 0.0f) {
          float diff = fabs(candidateBpm - bpmSmoothed) / bpmSmoothed;
          if (diff > OUTLIER_RATIO) {
            lastBeatMs = now;
            return;
          }
        }

        bpmInstant = candidateBpm;
        addIbi((uint16_t)ibiMs);
        bpmSmoothed = computeSmoothedBpm();
      }
    }
    lastBeatMs = now;
  }

  // SpO2 estimation (unchanged maths)
  const float beta = 0.02f;
  meanIR  = (1.0f - beta) * meanIR  + beta * (float)irValue;
  meanRed = (1.0f - beta) * meanRed + beta * (float)redValue;

  float irD  = (float)irValue  - meanIR;
  float redD = (float)redValue - meanRed;

  irDet[detIndex]  = irD;
  redDet[detIndex] = redD;

  detIndex = (detIndex + 1) % SPO2_WINDOW;
  if (detCount < SPO2_WINDOW) detCount++;

  float acIR  = rmsOfBuffer(irDet, detCount);
  float acRed = rmsOfBuffer(redDet, detCount);

  bool spo2Valid = (detCount >= SPO2_WINDOW) &&
                   (meanIR > 1.0f) && (meanRed > 1.0f) &&
                   (acIR > MIN_AC_RMS) && (acRed > MIN_AC_RMS);

  if (spo2Valid) {
    float R = (acRed / meanRed) / (acIR / meanIR);
    float s = spo2FromR(R);

    if (s < SPO2_MIN) s = SPO2_MIN;
    if (s > SPO2_MAX) s = SPO2_MAX;

    spo2 = s;
    if (spo2Smoothed == 0.0f) spo2Smoothed = spo2;
    spo2Smoothed = (1.0f - SPO2_ALPHA) * spo2Smoothed + SPO2_ALPHA * spo2;
  }
//serial output
Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", RED=");
  Serial.print(redValue);
  Serial.print(", BPM=");
  Serial.print(bpmInstant, 1);
  Serial.print(", Smoothed BPM=");
  Serial.print(bpmSmoothed, 1);
  Serial.print(", SpO2=");
  if (spo2Smoothed > 0.0f) Serial.print(spo2Smoothed, 1);
  else Serial.print("...");
  Serial.println("%");

  delay(10);

  // LCD output
  if (millis() - lastLcdMs >= LCD_PERIOD_MS) {
    lastLcdMs = millis();

    // backlight color: green if both readings are valid
    if (bpmSmoothed > 0.0f && spo2Smoothed > 0.0f) lcd.setRGB(0, 255, 0);
    else lcd.setRGB(255, 140, 0);

    // Line 1: HR and SpO2
    lcd.setCursor(0, 0);
    char line1[17];
    int hr = (bpmSmoothed > 0.0f) ? (int)(bpmSmoothed + 0.5f) : 0;
    int sp = (spo2Smoothed > 0.0f) ? (int)(spo2Smoothed + 0.5f) : 0;

    if (hr > 0 && sp > 0)
      snprintf(line1, sizeof(line1), "HR:%3d  o2:%3d", hr, sp);
    else if (hr > 0)
      snprintf(line1, sizeof(line1), "HR:%3d  o2:---", hr);
    else
      snprintf(line1, sizeof(line1), "HR:---  o2:---");

    lcdPrintPadded(line1);

    // Line 2: Display IR 
    lcd.setCursor(0, 1);
    char line2[17];
    // If we want both IR and RED, use this (may truncate):
    // snprintf(line2, sizeof(line2), "IR:%ld R:%ld", irValue, redValue);

    // Safer: just IR
    snprintf(line2, sizeof(line2), "IR:%ld", irValue);
    lcdPrintPadded(line2);
  }

  delay(10);
}

