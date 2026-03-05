#include <SPI.h>
#include <math.h>

//SPI Pins 
static const int PIN_SCK  = 13;
static const int PIN_MISO = 12;
static const int PIN_MOSI = 11;
static const int PIN_CS   = 10;

//Output pin for fall event 
static const int FALL_OUT_PIN = 7; 

//ICM-20689 registers
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_PWR_MGMT_2   = 0x6C;
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// SPI settings 
SPISettings imuSPI(1000000, MSBFIRST, SPI_MODE0);

// IMU scaled values 
float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;

// Fall detection tunables 
const float FREEFALL_THR_G = 0.50f;   // low threshold (g) - start point
const float IMPACT_THR_G   = 2.50f;   // high threshold (g) - start point
const uint32_t IMPACT_WINDOW_MS = 500; // must see impact within 0.5s after free-fall
const uint32_t FALL_LATCH_MS    = 2000; // keep output HIGH for 2 seconds

// Fall detector state 
enum FallState { MONITORING, FREEFALL_SEEN, FALL_CONFIRMED };
FallState fallState = MONITORING;
uint32_t t_freefall = 0;
uint32_t t_fall_latch = 0;

// SPI helpers
uint8_t spiRead8(uint8_t reg) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void spiWrite8(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void spiReadBytes(uint8_t startReg, uint8_t *buf, size_t n) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(startReg | 0x80);
  for (size_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

static int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

// Read IMU (accel in g, gyro in dps)
void readIMU() {
  uint8_t data[14];
  spiReadBytes(REG_ACCEL_XOUT_H, data, 14);

  int16_t axRaw = toInt16(data[0],  data[1]);
  int16_t ayRaw = toInt16(data[2],  data[3]);
  int16_t azRaw = toInt16(data[4],  data[5]);
  int16_t gxRaw = toInt16(data[8],  data[9]);
  int16_t gyRaw = toInt16(data[10], data[11]);
  int16_t gzRaw = toInt16(data[12], data[13]);

  // Set ±16g and ±2000 dps:
  const float accelLSBperG  = 2048.0f;
  const float gyroLSBperDPS = 16.4f;

  ax_g = axRaw / accelLSBperG;
  ay_g = ayRaw / accelLSBperG;
  az_g = azRaw / accelLSBperG;

  gx_dps = gxRaw / gyroLSBperDPS;
  gy_dps = gyRaw / gyroLSBperDPS;
  gz_dps = gzRaw / gyroLSBperDPS;
}

float accelMagnitudeG() {
  return sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
}

// Setup 
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(FALL_OUT_PIN, OUTPUT);
  digitalWrite(FALL_OUT_PIN, LOW);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  delay(50); // IMPORTANT: lets IMU finish powering up 

  Serial.println("ICM-20689 SPI fall test");

  uint8_t who = spiRead8(REG_WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  if (who != 0x98) {
    Serial.println("IMU not detected!");
    while (1) delay(100);
  }

  // Wake up + enable sensors
  spiWrite8(REG_PWR_MGMT_1, 0x01);
  spiWrite8(REG_PWR_MGMT_2, 0x00);

  // Ranges
  spiWrite8(REG_GYRO_CONFIG, 0x18);   // ±2000 dps
  spiWrite8(REG_ACCEL_CONFIG, 0x18);  // ±16g

  delay(50);
}

// ---------- Loop ----------
void loop() {
  readIMU();
  float amag = accelMagnitudeG();
  uint32_t now = millis();

  // Threshold-based fall detection (free-fall to impact) ---
  switch (fallState) {
    case MONITORING:
      if (amag < FREEFALL_THR_G) {
        t_freefall = now;
        fallState = FREEFALL_SEEN;
      }
      break;

    case FREEFALL_SEEN:
      if (amag > IMPACT_THR_G) {
        // Fall confirmed
        fallState = FALL_CONFIRMED;
        t_fall_latch = now;
        digitalWrite(FALL_OUT_PIN, HIGH);
      } else if (now - t_freefall > IMPACT_WINDOW_MS) {
        // No impact in time == false alarm
        fallState = MONITORING;
      }
      break;

    case FALL_CONFIRMED:
      // Keep pin HIGH for a bit, then reset
      if (now - t_fall_latch > FALL_LATCH_MS) {
        digitalWrite(FALL_OUT_PIN, LOW);
        fallState = MONITORING;
      }
      break;
  }

  // --- Debug print ---
  Serial.print("A[g]: ");
  Serial.print(ax_g, 3); Serial.print(", ");
  Serial.print(ay_g, 3); Serial.print(", ");
  Serial.print(az_g, 3);

  Serial.print(" | |A|=");
  Serial.print(amag, 3);

  Serial.print(" | state=");
  Serial.print((int)fallState);

  if (fallState == FALL_CONFIRMED) Serial.print("  FALL!");
  Serial.println();

  delay(10); // ~100 Hz (better chance to catch short free-fall dip)
}