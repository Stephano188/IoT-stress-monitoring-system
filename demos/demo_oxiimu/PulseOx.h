#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

class PulseOx {
public:
  struct Config {
    uint32_t fingerIrThreshold = 15000;

    // BPM (v2 — refractory period + DC removal + rolling average)
    uint8_t  rateBufSize  = 10;       // rolling average window
    uint16_t minBpm       = 40;
    uint16_t maxBpm       = 150;
    uint32_t refractoryMs = 400;      // min ms between accepted beats
    float    dcAlpha      = 0.95f;    // DC removal filter coefficient

    // SpO2 (original RMS ratio method with improved tuning)
    uint16_t spo2Window   = 50;       // samples before SpO2 is valid
    float    spo2Beta     = 0.05f;    // DC tracking speed (faster convergence)
    float    minAcRms     = 80.0f;    // min AC signal strength
    float    spo2Alpha    = 0.10f;    // output smoothing factor
    float    spo2Min      = 70.0f;
    float    spo2Max      = 100.0f;

    // Sensor hardware — balanced LED amplitudes for accurate R ratio
    uint8_t ampIR  = 0x2A;
    uint8_t ampRed = 0x24;
  };

  explicit PulseOx(const Config& cfg);

  bool begin(TwoWire& w = Wire);

  // Call every loop — non-blocking
  void update(uint32_t nowMs);

  // ── Getters — identical names to original so .ino needs no changes ──
  bool  fingerPresent() const { return finger;     }
  long  ir()            const { return irValue;    }
  long  red()           const { return redValue;   }

  // BPM — instant = most recent single beat, smoothed = rolling average
  float bpmInstant()    const { return bpmInst;    }
  float bpmSmoothed()   const { return bpmAvg;     }

  // SpO2 — instant = current window value, smoothed = EMA of above
  float spo2Instant()   const { return spo2Val;    }
  float spo2Smoothed()  const { return spo2Smooth; }

private:
  Config   cfg;
  MAX30105 sensor;

  long irValue  = 0;
  long redValue = 0;
  bool finger   = false;

  // BPM state 
  static constexpr uint8_t MAX_RATE = 16;
  float    rates[MAX_RATE] = {0};
  uint8_t  rateSpot        = 0;
  uint8_t  rateCount       = 0;
  float    bpmInst         = 0.0f;
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
  float    spo2Val     = 0.0f;
  float    spo2Smooth  = 0.0f;

  //  Helpers 
  void  resetMeasurement();
  void  updateBpm(uint32_t nowMs);
  void  updateSpo2();

  static float   rmsOfBuffer(const float* buf, uint16_t n);
  static float   spo2FromR(float R);
};
