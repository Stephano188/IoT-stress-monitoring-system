#pragma once
#include <Arduino.h>
#include <SPI.h>

class ImuFall {
public:
  enum FallState {
    MONITORING,       // normal operation
    FREEFALL_SEEN,    // low-g detected, waiting for impact
    FALL_CONFIRMED,   // impact detected, now checking for stillness
    STILL_CONFIRMED   // person has been still after impact — real fall
  };

  struct Config {
    int pinSck     = 13;
    int pinMiso    = 12;
    int pinMosi    = 11;
    int pinCs      = 10;
    int fallOutPin = 7;

    // Fall thresholds
    float freefallThrG = 0.50f;
    float impactThrG   = 2.50f;

    // Timing
    uint32_t impactWindowMs  = 500;   // max time between freefall and impact

    // FIX: Reduced wait before checking stillness (was 2000ms — too long,
    // caused the window to expire before stillness was ever evaluated)
    uint32_t stillnessWaitMs = 500;

    // FIX: Reduced required stillness duration (was 2000ms — combined with
    // the wait above meant 4s total motionless, unrealistic after a fall)
    uint32_t stillnessDurMs  = 1500;

    uint32_t fallLatchMs     = 3000; // how long STILL_CONFIRMED is held

    // FIX: Relaxed stillness detection thresholds.
    // stillAccelBand was 0.20f — too tight. A fallen sensor at any angle other
    // than flat won't read exactly 1g, so this kept resetting t_stillStart.
    // Raised to 0.45f to tolerate arbitrary sensor orientation after a fall.
    float stillAccelBand = 0.45f;

    // stillGyroDps was 15.0f — at 10ms polling, IMU noise was breaching this
    // continuously and resetting the stillness timer. Raised to 40.0f.
    float stillGyroDps   = 40.0f;

    // Abandon FALL_CONFIRMED if no stillness seen after this long (ms)
    uint32_t abandonMs = 8000;

    uint32_t spiHz = 1000000;
  };

  explicit ImuFall(const Config& cfg);

  bool begin();
  void update(uint32_t nowMs);

  float ax() const { return ax_g; }
  float ay() const { return ay_g; }
  float az() const { return az_g; }
  float gx() const { return gx_dps; }
  float gy() const { return gy_dps; }
  float gz() const { return gz_dps; }

  float     accelMagnitudeG() const;
  FallState state()           const { return fallState; }
  bool      fallLatched()     const { return fallState == STILL_CONFIRMED; }

private:
  Config cfg;

  static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
  static constexpr uint8_t REG_PWR_MGMT_2   = 0x6C;
  static constexpr uint8_t REG_WHO_AM_I      = 0x75;
  static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
  static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
  static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;

  SPISettings spiSettings;

  float ax_g = 0, ay_g = 0, az_g = 0;
  float gx_dps = 0, gy_dps = 0, gz_dps = 0;

  FallState fallState   = MONITORING;
  uint32_t t_freefall   = 0;
  uint32_t t_impact     = 0;
  uint32_t t_stillStart = 0;
  uint32_t t_latch      = 0;

  uint8_t  spiRead8(uint8_t reg);
  void     spiWrite8(uint8_t reg, uint8_t val);
  void     spiReadBytes(uint8_t startReg, uint8_t* buf, size_t n);
  static int16_t toInt16(uint8_t hi, uint8_t lo);

  void readIMU();
  bool isStill() const;
};
