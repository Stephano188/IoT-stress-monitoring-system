#include "PulseOx.h"
#include <math.h>

PulseOx::PulseOx(const Config& c) : cfg(c) {}

// begin 
bool PulseOx::begin(TwoWire& w) {
  w.begin();

  if (!sensor.begin(w, I2C_SPEED_FAST)) return false;

  // Clamp buffer size to hard cap
  if (cfg.rateBufSize  > MAX_RATE)     cfg.rateBufSize  = MAX_RATE;
  if (cfg.spo2Window   > MAX_SPO2_WIN) cfg.spo2Window   = MAX_SPO2_WIN;

  sensor.setup(
    cfg.ledBrightness,
    cfg.sampleAvg,
    2,               // ledMode 2 = Red + IR (required for SpO2)
    cfg.sampleRate,
    cfg.pulseWidth,
    cfg.adcRange
  );

  resetMeasurement();
  return true;
}

// resetMeasurement 
void PulseOx::resetMeasurement() {
  // BPM
  rateCount  = 0;
  rateSpot   = 0;
  bpmAvg     = 0.0f;
  dcLevel    = 0.0f;
  lastBeatMs = 0;
  for (uint8_t i = 0; i < MAX_RATE; i++) rates[i] = 0.0f;

  // SpO2
  meanIR     = 0.0f;
  meanRed    = 0.0f;
  spo2Smooth = 0.0f;
  detIndex   = 0;
  detCount   = 0;
  for (uint16_t i = 0; i < MAX_SPO2_WIN; i++) {
    irDet[i]  = 0.0f;
    redDet[i] = 0.0f;
  }
}

// update
void PulseOx::update(uint32_t nowMs) {
  irValue  = sensor.getIR();
  redValue = sensor.getRed();
  finger   = (irValue >= (long)cfg.fingerIrThreshold);

  if (!finger) {
    resetMeasurement();
    return;
  }

  updateBpm(nowMs);
  updateSpo2();
}

// updateBpm (v2 method)
void PulseOx::updateBpm(uint32_t nowMs) {
  // DC removal — track slow baseline, subtract to get AC pulse
  dcLevel = cfg.dcAlpha * dcLevel +
            (1.0f - cfg.dcAlpha) * (float)irValue;

  if (!checkForBeat(irValue)) return;

  uint32_t ibiMs  = nowMs - lastBeatMs;
  bool tooSoon    = (lastBeatMs != 0) && (ibiMs < cfg.refractoryMs);
  bool plausible  = (lastBeatMs == 0) ||
                    (ibiMs >= (60000u / cfg.maxBpm) &&
                     ibiMs <= (60000u / cfg.minBpm));

  if (!tooSoon && plausible && lastBeatMs != 0) {
    float bpmInst       = 60000.0f / (float)ibiMs;
    rates[rateSpot]     = bpmInst;
    rateSpot            = (rateSpot + 1) % cfg.rateBufSize;
    if (rateCount < cfg.rateBufSize) rateCount++;

    float sum = 0;
    for (uint8_t i = 0; i < rateCount; i++) sum += rates[i];
    bpmAvg = sum / (float)rateCount;
  }

  if (!tooSoon) lastBeatMs = nowMs;
}

// updateSpo2 (original RMS ratio method) 
void PulseOx::updateSpo2() {
  // Track DC level of both channels
  meanIR  = (1.0f - cfg.spo2Beta) * meanIR  + cfg.spo2Beta * (float)irValue;
  meanRed = (1.0f - cfg.spo2Beta) * meanRed + cfg.spo2Beta * (float)redValue;

  // Store AC component (DC removed)
  irDet[detIndex]  = (float)irValue  - meanIR;
  redDet[detIndex] = (float)redValue - meanRed;
  detIndex = (detIndex + 1) % cfg.spo2Window;
  if (detCount < cfg.spo2Window) detCount++;

  float acIR  = rmsOfBuffer(irDet,  detCount);
  float acRed = rmsOfBuffer(redDet, detCount);

  bool valid = (detCount  >= cfg.spo2Window) &&
               (meanIR    >  1.0f) &&
               (meanRed   >  1.0f) &&
               (acIR      >  cfg.minAcRms) &&
               (acRed     >  cfg.minAcRms);

  if (!valid) return;

  float R = (acRed / meanRed) / (acIR / meanIR);
  float s = spo2FromR(R);

  s = constrain(s, cfg.spo2Min, cfg.spo2Max);

  if (spo2Smooth == 0.0f) spo2Smooth = s;
  spo2Smooth = (1.0f - cfg.spo2Alpha) * spo2Smooth + cfg.spo2Alpha * s;
}

// rmsOfBuffer 
float PulseOx::rmsOfBuffer(const float* buf, uint16_t n) {
  if (n == 0) return 0.0f;
  double s = 0.0;
  for (uint16_t i = 0; i < n; i++) { double v = buf[i]; s += v * v; }
  return (float)sqrt(s / (double)n);
}

// spo2FromR — empirical calibration curve
float PulseOx::spo2FromR(float R) {
  return (-45.060f * R * R) + (30.354f * R) + 97.845f;
}
