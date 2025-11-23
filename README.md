# Power Detector

Firmware for ESP32-C3 SuperMini development board that sends data when powered on via ESP-NOW to the other board. Primarily used as a power detector.

## Setup

- Copy `secrets.h.example` to `secrets.h`
- Configure WiFi credentials in `secrets.h`
- Figure out MAC address of the primary board which will read ESP-NOW packets and update `masterAddress` in `main.cpp`.
