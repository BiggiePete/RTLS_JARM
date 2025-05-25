
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

### Detailed TODOs

- Set up scheduling system for getting sensor readings
- Set up Kalman filtering system to filter readings, and provide better idea of state in terms of orientation and altitude
- Set up PID controllers for the 4 motors
- Set up scheduling system to update the "physics" of the rocket at either a set rate, or when data is available
- Get temperature data
- Get humidity data
- Get GPS data
- Get IMU data
- Get Magnetometer data
- Determine rotational offset of Magnetometer on the board in reference to the north direction
- Set control flow for rocket in mode: IDLE, UP, CORRECTION, DESCENT
- In IDLE mode, listen for rocket to be above 500ft & take GPS readings at high precision when the button is pressed
- In UP mode, wait until altitude has descended over 3 datapoints before switching to CORRECTION
- In CORRECTION mode, immediately get IMU readings at a good pace, and make all corrections necessary until the drone is within 10% of level flight, and within 10° of facing north, then switch to DESCENT mode
- In DESCENT mode, use all measurement data to determine the PID levels necessary to control the rocket movements on the way down
- If in DESCENT mode & below X ft, and the distance to the landing site is > X ft + OFFSET ft, deploy parachute, as the mission has become a failure
- If in CORRECTION mode, and unable to make correction in under 15s, fire parachute method, as mission is a failure

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
