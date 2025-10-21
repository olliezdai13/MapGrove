# MapGrove
A wearable tracker for Mangrove reforestation, replacing clipboard or phone based logging with a basic ML system


Project: Wearable Data Logger for Mangrove Planting MRV
 Version: 1.0 (Hand-off Version)
 Date: 21-10-2025

Description:
 This firmware is for an Arduino Nano-based wearable device that logs high-frequency
 motion and GPS data to an SD card. It is designed to capture the "signature" of
 a specific motion for Monitoring, Reporting, and Verification (MRV) purposes.

Features:
 - Logs Accelerometer & Gyroscope data from an MPU-6050.
 - Logs GPS location and satellite data from a standard GPS module.
 - Uses the memory-efficient SdFat library for reliable logging.
 - Implements a "bare-metal" MPU-6050 communication method to save SRAM.
 - Provides real-time event feedback via an LED for high-G events.
 - Includes a manual "abort" button to flag data entries.

Hardware & Wiring:
 - SD Card (SPI): MOSI-11, MISO-12, SCK-13, CS-10
 - MPU-6050 (I2C): SDA-A4, SCL-A5
 - GPS Module (Software Serial): GPS TX -> Nano Pin D4, GPS RX -> Nano Pin D5
 - LED: Pin D3 -> Resistor (220-330 Ohm) -> LED -> GND
 - Button: Pin D2 -> Button -> GND (using internal pull-up)


0: Default. 1: High-G event detected. 2: Manually aborted event.
