/*******************************************************
   DS18B20 Temperature Sensor Test
   Board: Arduino Nano ESP32

   Wiring:
     DS18B20 VDD -> 3.3V
     DS18B20 GND -> GND
     DS18B20 DQ  -> GPIO 4
     4.7kΩ resistor between DQ and 3.3V

   Libraries required:
     OneWire
     DallasTemperature
********************************************************/

#include <OneWire.h>
#include <DallasTemperature.h>

// GPIO used for OneWire bus
#define ONE_WIRE_BUS 4

// Setup OneWire and temperature objects
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing DS18B20...");
  sensors.begin();

  int deviceCount = sensors.getDeviceCount();
  Serial.print("Devices found: ");
  Serial.println(deviceCount);

  if (deviceCount == 0) {
    Serial.println("WARNING No DS18B20 detected. Check wiring!");
  } else {
    Serial.println("DS18B20 detected!");
  }
}

void loop() {
  sensors.requestTemperatures();   // Ask sensor for temperature

  float tempC = sensors.getTempCByIndex(0);
  float tempF = sensors.toFahrenheit(tempC);

  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("! Sensor disconnected!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print(" °C  |  ");
    Serial.print(tempF);
    Serial.println(" °F");
  }

  delay(1000); // update once per second
}
