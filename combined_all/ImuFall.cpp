#include <Arduino.h>
#include "ImuFall.h"
#include <math.h>

ImuFall::ImuFall(const Config& c)
: cfg(c),
  spiSettings(cfg.spiHz, MSBFIRST, SPI_MODE0) {}

uint8_t ImuFall::spiRead8(uint8_t reg) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(cfg.pinCs, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(cfg.pinCs, HIGH);
  SPI.endTransaction();
  return val;
}

void ImuFall::spiWrite8(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(cfg.pinCs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cfg.pinCs, HIGH);
  SPI.endTransaction();
}

void ImuFall::spiReadBytes(uint8_t startReg, uint8_t* buf, size_t n) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(cfg.pinCs, LOW);
  SPI.transfer(startReg | 0x80);
  for (size_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(cfg.pinCs, HIGH);
  SPI.endTransaction();
}

int16_t ImuFall::toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

void ImuFall::readIMU() {
  uint8_t data[14];
  spiReadBytes(REG_ACCEL_XOUT_H, data, 14);

  int16_t axRaw = toInt16(data[0],  data[1]);
  int16_t ayRaw = toInt16(data[2],  data[3]);
  int16_t azRaw = toInt16(data[4],  data[5]);
  int16_t gxRaw = toInt16(data[8],  data[9]);
  int16_t gyRaw = toInt16(data[10], data[11]);
  int16_t gzRaw = toInt16(data[12], data[13]);

  const float accelLSBperG  = 2048.0f;
  const float gyroLSBperDPS = 16.4f;

  ax_g = axRaw / accelLSBperG;
  ay_g = ayRaw / accelLSBperG;
  az_g = azRaw / accelLSBperG;

  gx_dps = gxRaw / gyroLSBperDPS;
  gy_dps = gyRaw / gyroLSBperDPS;
  gz_dps = gzRaw / gyroLSBperDPS;
}

float ImuFall::accelMagnitudeG() const {
  return sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
}

// FIX 3: stillAccelBand raised to 0.45f and stillGyroDps raised to 40.0f
// in Config so the sensor orientation after a fall (not necessarily flat)
// and normal IMU noise no longer continuously resets the stillness timer.
bool ImuFall::isStill() const {
  float amag = accelMagnitudeG();
  bool accelStill = fabs(amag - 1.0f) < cfg.stillAccelBand;
  bool gyroStill  = (fabs(gx_dps) < cfg.stillGyroDps) &&
                    (fabs(gy_dps) < cfg.stillGyroDps) &&
                    (fabs(gz_dps) < cfg.stillGyroDps);
  return accelStill && gyroStill;
}

bool ImuFall::begin() {
  pinMode(cfg.fallOutPin, OUTPUT);
  digitalWrite(cfg.fallOutPin, LOW);

  pinMode(cfg.pinCs, OUTPUT);
  digitalWrite(cfg.pinCs, HIGH);

  SPI.begin(cfg.pinSck, cfg.pinMiso, cfg.pinMosi, cfg.pinCs);
  delay(50);

  uint8_t who = spiRead8(REG_WHO_AM_I);
  if (who != 0x98) return false;

  spiWrite8(REG_PWR_MGMT_1, 0x01);
  spiWrite8(REG_PWR_MGMT_2, 0x00);
  spiWrite8(REG_GYRO_CONFIG,  0x18);
  spiWrite8(REG_ACCEL_CONFIG, 0x18);

  delay(50);
  fallState    = MONITORING;
  t_freefall   = 0;
  t_impact     = 0;
  t_stillStart = 0;
  t_latch      = 0;
  return true;
}

void ImuFall::update(uint32_t nowMs) {
  readIMU();
  float amag = accelMagnitudeG();

  switch (fallState) {

    case MONITORING:
      if (amag < cfg.freefallThrG) {
        t_freefall = nowMs;
        fallState  = FREEFALL_SEEN;
      }
      break;

    case FREEFALL_SEEN:
      if (amag > cfg.impactThrG) {
        t_impact     = nowMs;
        t_stillStart = 0;         // ensure clean start for stillness timer
        fallState    = FALL_CONFIRMED;
      } else if (nowMs - t_freefall > cfg.impactWindowMs) {
        fallState = MONITORING;
      }
      break;

    case FALL_CONFIRMED:
      // FIX 3: Wait a short period (stillnessWaitMs = 500ms) for the impact
      // vibration to die down, then start actively checking for stillness.
      // The old value (2000ms) meant the person had to be perfectly motionless
      // for 4 full seconds — combined wait + duration.
      if (nowMs - t_impact < cfg.stillnessWaitMs) {
        break;
      }

      if (isStill()) {
        if (t_stillStart == 0) {
          t_stillStart = nowMs;   // first still sample — start the timer
        }
        if (nowMs - t_stillStart >= cfg.stillnessDurMs) {
          // Still long enough — confirmed real fall
          fallState = STILL_CONFIRMED;
          t_latch   = nowMs;
          digitalWrite(cfg.fallOutPin, HIGH);
        }
      } else {
        // Movement detected — reset stillness streak
        t_stillStart = 0;

        // FIX 3: Use dedicated abandonMs instead of the convoluted
        if (nowMs - t_impact > cfg.abandonMs) {
          fallState    = MONITORING;
          t_stillStart = 0;
        }
      }
      break;

    case STILL_CONFIRMED:
      if (nowMs - t_latch > cfg.fallLatchMs) {
        digitalWrite(cfg.fallOutPin, LOW);
        fallState    = MONITORING;
        t_stillStart = 0;
      }
      break;
  }
}
