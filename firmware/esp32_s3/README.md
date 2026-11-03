# EARS-RL ESP32-S3 Real-Time Actuator Firmware Skeleton

This directory contains standalone embedded firmware for the ESP32-S3 microcontroller.
Built with ESP-IDF v5.x / C++.

## Architecture & Responsibilities

The ESP32-S3 handles hard real-time deterministic tasks that cannot run deterministically under standard Linux (Raspberry Pi 5):
1. **Pulse Generation**: High-frequency STEP/DIR pulses ($\ge 10\text{ kHz}$) using ESP32-S3 dedicated hardware timers / RMT / MCPWM.
2. **Sensor Acquisition**: SPI / I2C / UART acquisition of angular encoders and 6-DOF IMU.
3. **Hardware Watchdog**: Automatic motor driver disable if high-level policy heartbeat is lost ($> 500\text{ ms}$).
4. **Physical E-Stop Monitoring**: GPIO interrupt line monitoring hardwired emergency stop.
5. **Telemetry Stream**: Serial packet streaming with sequence IDs and monotonic timestamps.

## Separation Notice

This firmware is built independently using ESP-IDF (`idf.py build`) and is NOT packaged into the Python wheel.
Python communication is mediated via the high-level `ActuatorInterface` over UART.
