# C.A.S.P.E.R-1 Firmware

Firmware for the C.A.S.P.E.R-1 flight computer running on a Teensy 4.1. The
code fuses measurements from a BMX160 IMU, a BNO055 IMU/magnetometer, and a
BMP388 barometer using an Extended Kalman Filter (EKF) to estimate vehicle
position, velocity, and orientation in real time.

## Hardware
- Teensy 4.1 (Arduino framework via PlatformIO)
- DFRobot BMX160 (accelerometer + gyroscope)
- Adafruit BNO055 (accelerometer + gyroscope + magnetometer)
- Adafruit BMP388 (barometer for altitude)

## Features
- Dual-IMU fusion: BMX160 and BNO055 data are aligned to the body frame and
  fused before entering the EKF.
- EKF state estimation: tracks 15 states (position, velocity, quaternion
  attitude, gyro bias, accel bias) with barometer and magnetometer updates.
- 200 Hz control loop: deterministic 5 ms loop handles sensor reads, state
  propagation, corrections, and telemetry.
- Ready-to-plot telemetry: serial output prefixed with `>` containing roll,
  pitch, yaw, position, and velocity for live plotting or logging.
- Configurable noise and frame alignment: default noise values and sensor
  rotation matrices are defined in `src/main.cpp` and can be tuned without
  changing the EKF implementation.

## Repository Layout
```
├── platformio.ini          # PlatformIO target for Teensy 4.1
├── include/                # Project headers
│   ├── dual_imu_ekf.h      # EKF interfaces and data structures
│   ├── orientation_estimation.h
│   └── sensor_setup.h      # Sensor initialisation and read helpers
├── src/                    # Firmware sources
│   ├── dual_imu_ekf.cpp    # EKF predict/update and helper math
│   ├── orientation_estimation.cpp
│   ├── sensor_setup.cpp    # IMU + barometer bring-up and sampling
│   └── main.cpp            # 200 Hz loop, fusion pipeline, telemetry output
└── C.A.S.P.E.R/            # Reserved for recorded flight data/logs
```

## Build and Upload
1. Install [PlatformIO CLI](https://platformio.org/install/cli).
2. From the repository root, build the firmware:
   ```bash
   pio run
   ```
3. Connect the Teensy 4.1 and upload:
   ```bash
   pio run -t upload
   ```
4. Open a serial monitor (adjust `monitor_port` in `platformio.ini`):
   ```bash
   pio device monitor -b 115200 --port <PORT>
   ```

## Runtime Behaviour
- On startup, sensors are initialised and the EKF state/covariance are seeded
  using the defaults in `main.cpp`.
- Each 5 ms loop:
  1. BMX160 and BNO055 accelerometer/gyro data are read and rotated into the
     body frame.
  2. Measurements are fused into a single IMU sample and passed to
     `EkfPredict`.
  3. Barometer altitude and body-frame magnetometer data are used for update
     steps when available.
  4. Telemetry is emitted over serial for plotting/logging, e.g.:
     ```
     >roll:-0.0123,pitch:0.0345,yaw:0.6543,p_x:0.0000,p_y:0.0000,p_z:0.0000,v_x:0.0000,v_y:0.0000,v_z:0.0000
     ```
- Optional helpers in `sensor_setup.cpp` support manual BMP388 calibration
  and simple relative-altitude readings if you need pre-flight checks.

## Future Work
- Tune noise parameters using flight-test data.
- Persist logs to SD for post-flight analysis.
- Add fault detection and richer telemetry framing for ground-station use.

## Contact
For questions, open an issue or contact avishwanath1@sheffield.ac.uk.
