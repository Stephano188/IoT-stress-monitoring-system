/*******************************************************
   Grove GSR (Galvanic Skin Response) Sensor Test
   Board: Arduino Nano ESP32
   Sensor: Seeed Studio 101020052

   Wiring:
     GSR VCC  -> 3.3V(NOT 5v)
     GSR GND  -> GND
     GSR SIG  -> A0  (GPIO 4 on Nano ESP32)

   This sketch:
     - Reads raw analog GSR values
     - Averages readings to reduce noise
     - Prints value to Serial Monitor
********************************************************/

// Analog pin for the GSR signal
const int GSR_PIN = A0;   // GPIO 4 on Nano ESP32

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Grove GSR Sensor Test (Arduino Nano ESP32)");
}

void loop() {

  // ---- Read and Smooth ----
  int numReadings = 20;      // number of samples for noise reduction
  long total = 0;

  for (int i = 0; i < numReadings; i++) {
    total += analogRead(GSR_PIN);
    delay(5);
  }

  int gsrValue = total / numReadings;

  // ---- Print Values ----
  Serial.print("GSR Raw Value: ");
  Serial.println(gsrValue);

  // A short delay before next reading
  delay(100);
}
