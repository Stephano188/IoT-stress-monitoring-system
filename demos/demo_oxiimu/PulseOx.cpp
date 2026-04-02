#include "PulseOx.h"
#include <math.h>

PulseOx::PulseOx(const Config& c) : cfg(c) {}

// begin
bool PulseOx::begin(TwoWire& w) {
  w.begin();

  if (!sensor.begin(w, I2C_SPEED_FAST)) return false;

  if (cfg.rateBufSize  > MAX_RATE)     cfg.rateBufSize  = MAX_RATE;
  if (cfg.spo2Window   > MAX_SPO2_WIN) cfg.spo2Window   = MAX_SPO2_WIN;

  // Balanced LED amplitudes — keeps Red close to IR so R ratio
  // is not artificially skewed, which was pushing SpO2 readings low
  sensor.setup(
    cfg.ampIR,   // brightness (used as a starting point; setup() accepts it)
    4,           // 4x hardware averaging for a cleaner signal
    2,           // ledMode 2 = Red + IR (required for SpO2)
    400,         // 400 samples/sec
    215,         // pulse width 215us
    4096         // ADC range
  );
  sensor.setPulseAmplitudeIR(cfg.ampIR);
  sensor.setPulseAmplitudeRed(cfg.ampRed);
  sensor.setPulseAmplitudeGreen(0);

  resetMeasurement();
  return true;
}

// resetMeasurement 
void PulseOx::resetMeasurement() {
  rateCount  = 0;
  rateSpot   = 0;
  bpmInst    = 0.0f;
  bpmAvg     = 0.0f;
  dcLevel    = 0.0f;
  lastBeatMs = 0;
  for (uint8_t i = 0; i < MAX_RATE; i++) rates[i] = 0.0f;

  meanIR     = 0.0f;
  meanRed    = 0.0f;
  spo2Val    = 0.0f;
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

// updateBpm — v2 method 
// Refractory period prevents double-counting rising + falling
// edge of the same pulse (which caused 2x BPM readings).
// DC removal centres the waveform so the SparkFun peak detector
// works on the AC component only, reducing false triggers.
void PulseOx::updateBpm(uint32_t nowMs) {
  // Track and subtract the slow DC baseline
  dcLevel = cfg.dcAlpha * dcLevel +
            (1.0f - cfg.dcAlpha) * (float)irValue;

  if (!checkForBeat(irValue)) return;

  uint32_t ibiMs = nowMs - lastBeatMs;

  // Refractory gate — ignore anything within 400ms of the last beat
  bool tooSoon   = (lastBeatMs != 0) && (ibiMs < cfg.refractoryMs);

  // Physiological plausibility gate
  bool plausible = (lastBeatMs == 0) ||
                   (ibiMs >= (60000u / cfg.maxBpm) &&
                    ibiMs <= (60000u / cfg.minBpm));

  if (!tooSoon && plausible && lastBeatMs != 0) {
    bpmInst             = 60000.0f / (float)ibiMs;
    rates[rateSpot]     = bpmInst;
    rateSpot            = (rateSpot + 1) % cfg.rateBufSize;
    if (rateCount < cfg.rateBufSize) rateCount++;

    // Rolling average (bpmSmoothed via bpmAvg)
    float sum = 0;
    for (uint8_t i = 0; i < rateCount; i++) sum += rates[i];
    bpmAvg = sum / (float)rateCount;
  }

  if (!tooSoon) lastBeatMs = nowMs;
}

// updateSpo2 — original RMS ratio method
void PulseOx::updateSpo2() {
  // Track DC level of both channels (faster beta = quicker convergence)
  meanIR  = (1.0f - cfg.spo2Beta) * meanIR  + cfg.spo2Beta * (float)irValue;
  meanRed = (1.0f - cfg.spo2Beta) * meanRed + cfg.spo2Beta * (float)redValue;

  // Store AC-only component
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

  // Clamp R to the range the empirical formula is valid for
  if (R < 0.4f) R = 0.4f;
  if (R > 1.0f) R = 1.0f;

  float s = spo2FromR(R);
  s = constrain(s, cfg.spo2Min, cfg.spo2Max);

  spo2Val = s;
  if (spo2Smooth == 0.0f) spo2Smooth = spo2Val;
  spo2Smooth = (1.0f - cfg.spo2Alpha) * spo2Smooth + cfg.spo2Alpha * spo2Val;
}

// rmsOfBuffer 
float PulseOx::rmsOfBuffer(const float* buf, uint16_t n) {
  if (n == 0) return 0.0f;
  double s = 0.0;
  for (uint16_t i = 0; i < n; i++) { double v = buf[i]; s += v * v; }
  return (float)sqrt(s / (double)n);
}

//spo2FromR — empirical calibration curve
float PulseOx::spo2FromR(float R) {
  return (-45.060f * R * R) + (30.354f * R) + 97.845f;
}
