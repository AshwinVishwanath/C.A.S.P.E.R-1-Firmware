#  Extended Kalman Filter (EKF) for Real-Time Position and Orientation Tracking

## Overview
This project implements a real-time position and orientation tracking system using an Extended Kalman Filter (EKF) for aerospace applications. The system fuses data from multiple sensors, including an Adafruit BNO055 IMU and an Adafruit BMP388 barometric pressure sensor, to estimate 3D position, velocity, and orientation. This system is designed to run on a Teensy 4.1 microcontroller, leveraging its high-speed processing capabilities.

The project aims to achieve precise real-time tracking for aerospace applications, such as rocketry, where reliable position and orientation data are critical for flight stability, control, and telemetry.

## Key Features
- **Sensor Fusion:** Combines data from IMU (BNO055) and barometric pressure (BMP388) sensors.
- **Extended Kalman Filter (EKF):** Tracks 6 states (x, y, z, vx, vy, vz) to estimate position and velocity in 3D space.
- **Real-Time Processing:** Runs at a 1 kHz update rate on the Teensy 4.1, allowing for high-speed flight dynamics tracking.
- **Data Filtering:** Employs sensor offset calibration and low-pass filtering to improve measurement accuracy.
- **Calibration:** Automatically calibrates IMU and barometric sensors before operation.
- **Camera Trigger:** Automatically activates a connected camera shortly after power up.
- **Orientation Estimation:** Integrates gyroscope rates to provide roll, pitch and yaw angles.

---

## Repository Structure
```
├── platformio.ini           # PlatformIO build configuration
├── include/                 # Header files
│   ├── BNO_LUT.h            # Calibration offsets for BNO055
│   ├── camera_trigger.h     # Camera trigger interface
│   ├── ekf_sensor_fusion.h  # EKF state and update functions
│   ├── orientation_estimation.h # Gyro integration helpers
│   └── sensor_setup.h       # Sensor initialization and utilities
├── src/                     # Source files
│   ├── camera_trigger.cpp
│   ├── ekf_sensor_fusion.cpp
│   ├── orientation_estimation.cpp
│   ├── sensor_setup.cpp
│   └── main.cpp             # Main flight and state machine logic
└── C.A.S.P.E.R/             # Reserved directory for flight data
```

---

## How It Works
1. **Initialization:** The sensors (BNO055 and BMP388) are initialized and calibrated to measure baseline altitude, gyroscope, and accelerometer offsets.
2. **Main Loop:**
    - IMU and barometric pressure data are collected at 1 kHz.
    - Acceleration data from the IMU is transformed from the body frame to the NED (North-East-Down) frame.
    - The EKF predict step uses the acceleration data to propagate position and velocity estimates.
    - The EKF update step uses the relative altitude measured from the barometric sensor to correct the z-axis position.
3. **Data Output:** Outputs telemetry data to the serial monitor, which includes altitude, velocity (vx, vy, vz), and orientation (yaw, pitch, roll).

---

## Sensors Used
- **Adafruit BNO055**: Provides 3-axis accelerometer, gyroscope, and magnetometer data.
- **Adafruit BMP388**: Measures barometric pressure to calculate relative altitude.

---

## Core Algorithms
### 1. **Extended Kalman Filter (EKF)**
The EKF tracks the 6 states of the system:
- Position: (x, y, z)
- Velocity: (vx, vy, vz)

#### **Predict Step**
1. Update the position using the current velocity and time step (dt).
2. Update the velocity using the acceleration data from the IMU.
3. Propagate the covariance matrix using the state transition model.

#### **Update Step**
1. Use the measured altitude from the BMP388 to correct the z-axis position.
2. Calculate the Kalman Gain using the current state covariance.
3. Update the state estimate and reduce the state covariance.

---

### 2. **Sensor Calibration**
- **IMU Calibration:** Measures accelerometer, gyroscope, and magnetometer offsets over a stationary period.
- **Barometer Calibration:** Averages initial pressure readings to set a baseline altitude.

---

## Usage Instructions
### Prerequisites
- **Hardware:**
  - Teensy 4.1
  - Adafruit BNO055 IMU
  - Adafruit BMP388 Barometric Sensor
- **Software:**
  - Arduino IDE with Teensyduino plugin installed.
  - Required Libraries: Adafruit_Sensor, Adafruit_BNO055, Adafruit_BMP3XX

### Setup
1. **Wiring:**
   - BNO055 connected to I2C (SDA, SCL) pins.
   - BMP388 connected to I2C (SDA1, SCL1) pins.
2. **Load Code:**
   - Bring all project files together into one Arduino sketch.
   - Upload this combined sketch to the Teensy 4.1.
---

### Operation
1. Upon startup, the system will calibrate sensors.
2. If calibration succeeds, telemetry data will be printed to the serial monitor in the following format:
   ```
   > altitude:100.23,vx:0.15,vy:0.02,vz:-0.03,yaw:25.2,pitch:-3.1,roll:0.5
   ```
3. The system updates at 1 kHz, and the telemetry can be used for real-time tracking, logging, and visualization.

---

## Example Serial Output
```
> altitude:50.235,vx:0.12,vy:0.03,vz:-0.02,yaw:32.5,pitch:-1.2,roll:0.8
> altitude:50.236,vx:0.10,vy:0.02,vz:-0.03,yaw:32.6,pitch:-1.3,roll:0.7
> altitude:50.237,vx:0.08,vy:0.01,vz:-0.04,yaw:32.7,pitch:-1.4,roll:0.6
```

---

## To-Do / Future Improvements
- **Equations of Motion:** Implement motion equations for more advanced trajectory prediction.
- **Error Handling:** Add error reporting for sensor failures.
- **Telemetry Enhancements:** Log data to an SD card for post-flight analysis.
- **Data Visualization:** Add support for real-time visualization of the position and orientation.

---
## Contributions
Contributions are welcome! To contribute, please fork the repo, make your changes, and submit a pull request. Feel free to open issues for bug reports or feature requests.

---

## Contact
For any inquiries, contact avishwanath1@sheffield.ac.uk or open an issue in the repository.

