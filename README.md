# IoT-stress-monitoring-system
Physiological health monitoring device using an Arduino Nano ESP32 and multiple biosensors.

This project explores the development of an **IoT health monitoring prototype** using an **Arduino Nano ESP32** and multiple physiological sensors.

The system collects real-time data from several sensors and processes it on-device to monitor physiological indicators and detect events such as falls.

## Features
- Heart rate and SpO₂ monitoring (MAX30102)
- Motion sensing and fall detection using a 6-DOF IMU
- Skin conductance monitoring (GSR)
- Body temperature sensing (DS18B20)
- LCD display for device status and sensor readings

Sensor data is sampled continuously and processed using basic filtering and threshold-based logic to identify significant events such as falls or abnormal readings.

## Hardware

Main components used in the prototype:
- Arduino Nano ESP32
- MAX30102 pulse oximeter
- 6-DOF IMU motion sensor
- Grove GSR sensor
- DS18B20 temperature sensor
- RGB LCD Shield (16x2)
- External 5V power supply


## Disclaimer

This project is a **research prototype developed for academic purposes** and is **not intended for medical diagnosis or clinical use**.
