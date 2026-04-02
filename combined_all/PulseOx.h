#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

class PulseOx {
public:

  struct Config {
    // Finger detection
    uint32_t fingerIrThreshold = 50000;

    // BPM (v2 — refractory + DC removal + rolling average)
    uint8_t  rateBufSize    = 8;       // rolling average window
    uint16_t minBpm         = 40;
    uint16_t maxBpm         = 150;
    uint32_t refractoryMs   = 400;     // min ms between accepted beats
    float    dcAlpha        = 0.95f;   // DC removal filter coefficient

    // SpO2 (original RMS ratio method)
    uint16_t spo2Window     = 100;     // samples before SpO2 is valid
    float    spo2Beta       = 0.02f;   // DC tracking speed
    float    minAcRms       = 150.0f;  // min AC signal strength to trust R
    float    spo2Alpha      = 0.10f;   // smoothing factor on output
    float    spo2Min        = 70.0f;
    float    spo2Max        = 100.0f;

    // Sensor hardware
    uint8_t  ledBrightness  = 0x1F;
    uint8_t  sampleAvg      = 4;
    uint16_t sampleRate     = 400;
    uint16_t pulseWidth     = 215;
    uint16_t adcRange       = 4096;
  };

  explicit PulseOx(const Config& cfg);

  // Call once in setup(). Pass Wire or Wire1 if needed.
  bool begin(TwoWire& w = Wire);

  // Call every loop — non-blocking
  void update(uint32_t nowMs);

  //Getters
  bool    fingerPresent()  const { return finger; }
  long    ir()             const { return irValue; }
  long    red()            const { return redValue; }

  // BPM — returns 0 until 3+ beats buffered
  float   bpm()            const { return bpmAvg; }

  // SpO2 — returns 0 until window fills (~2s)
  float   spo2()           const { return spo2Smooth; }

private:
  Config   cfg;
  MAX30105 sensor;

  // Raw sensor values
  long irValue  = 0;
  long redValue = 0;
  bool finger   = false;

  // BPM state
  static constexpr uint8_t MAX_RATE = 16;  // hard cap on buffer
  float    rates[MAX_RATE] = {0};
  uint8_t  rateSpot        = 0;
  uint8_t  rateCount       = 0;
  float    bpmAvg          = 0.0f;
  float    dcLevel         = 0.0f;
  uint32_t lastBeatMs      = 0;

  // SpO2 state
  static constexpr uint16_t MAX_SPO2_WIN = 150;
  float    irDet[MAX_SPO2_WIN]  = {0};
  float    redDet[MAX_SPO2_WIN] = {0};
  uint16_t detIndex    = 0;
  uint16_t detCount    = 0;
  float    meanIR      = 0.0f;
  float    meanRed     = 0.0f;
  float    spo2Smooth  = 0.0f;

  //Helpers
  void  resetMeasurement();
  void  updateBpm(uint32_t nowMs);
  void  updateSpo2();

  static float rmsOfBuffer(const float* buf, uint16_t n);
  static float spo2FromR(float R);
};
