#pragma once

#include <Arduino.h>

// Forward declarations
bool isBMX160Ready();

// One-time sensor init
void setupSensors();

// Optional manual baro calibration (blocking, 1000 samples)
float manualCalibrateBMP388();

// Simple relative altitude helper (optional)
float getRelativeAltitude();

// Live sensor read functions for EKF
bool ReadBmxAccelGyro(float accel_bmx[3], float gyro_bmx_deg[3]);
bool ReadBnoAccelGyroMag(float accel_bno[3],
                         float gyro_bno_rad[3],
                         float mag_bno[3]);
bool ReadBmpAltitude(float &altitude_m);
