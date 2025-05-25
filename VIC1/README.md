
# RTLS_JARM VIC1

## Project Overview

This project is for the development of a Return To Launch Site (RTLS) rocket, designed to autonomously return to its launch location after ascent. The system is built on an STM32 Nucleo F411RE board using the Arduino framework and integrates multiple sensors for navigation, control, and environmental monitoring.

## Features

- **Sensor Integration**: Reads data from temperature, humidity, GPS, IMU, and magnetometer sensors.
- **Kalman Filtering**: Filters sensor readings to improve state estimation (orientation and altitude).
- **PID Motor Control**: Controls four motors for flight stabilization and trajectory correction.
- **Flight Modes**:
  - **IDLE**: Waits for launch conditions and high-precision GPS readings.
  - **UP**: Detects ascent and transitions to correction mode after apogee.
  - **CORRECTION**: Actively stabilizes and orients the rocket for return.
  - **DESCENT**: Controls descent and manages parachute deployment if needed.
- **Failure Handling**: Deploys parachute if mission parameters are not met.

## Future Updates

- Implement the full scheduling system for sensor readings and physics updates.
- Complete Kalman filter and PID controller integration.
- Add advanced failure detection and recovery logic.
- Improve sensor calibration, especially for magnetometer orientation.
- Enhance data logging and telemetry for post-flight analysis.

## Expectations

- The RTLS rocket will autonomously detect launch, stabilize itself, and attempt to return to the launch site.
- The system will handle failures gracefully, deploying a parachute if safe return is not possible.
- Ongoing development will focus on reliability, precision, and robust handling of real-world flight conditions.

---

Let me know if you want to add more technical details or specific instructions for contributors!

## Libraries Used

This project leverages several open-source libraries via PlatformIO to enable sensor integration, control, and scheduling:

- **TaskScheduler**: Task scheduling for periodic sensor readings and control updates
- **AHT20**: Temperature and humidity sensor driver
- **XGZP6897D**: Barometric pressure sensor driver
- **XENSIV 3D Magnetic Sensor TLx493D**: Magnetometer sensor driver
- **ICM42688**: IMU (Inertial Measurement Unit) sensor driver
- **UbxGps**: GPS module communication and parsing
- **RC_ESC**: Electronic Speed Controller (ESC) interface for motor control
- **PID**: Proportional-Integral-Derivative controller implementation for flight stabilization
- **SimpleKalmanFilter**: Kalman filter for sensor data fusion and noise reduction

All libraries are managed through the `platformio.ini` file for easy setup and reproducibility.
