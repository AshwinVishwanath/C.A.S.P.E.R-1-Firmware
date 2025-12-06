#include <Arduino.h>
#include <Wire.h>
#include "ins_ekf.h"
#include "sensor_setup.h"

// Core loop at 200 Hz (5 ms)
static const uint32_t LOOP_PERIOD_US = 5000;
static const float DT_SEC = 0.005f;
static const float DEG_TO_RAD = 0.017453292519943295f;

INSEKF12 ekf12;
INSEKF15 ekf15;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  setupSensors();

  float p0[3] = {0.0f, 0.0f, 0.0f};
  float v0[3] = {0.0f, 0.0f, 0.0f};
  float q0[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float ba0[3] = {0.0f, 0.0f, 0.0f};
  float bg0[3] = {0.0f, 0.0f, 0.0f};

  ekf12.initialize(p0, v0, q0, ba0, bg0);
  ekf12.setDt(DT_SEC);
  ekf12.setProcessNoiseFromSensors();
  ekf12.setMeasurementNoiseFromSensors();

  ekf15.initialize(p0, v0, q0, ba0, bg0);
  ekf15.setDt(DT_SEC);
  ekf15.setProcessNoiseFromSensors();
  ekf15.setMeasurementNoiseFromSensors();

  Serial.println("INS EKF initialized (12- and 15-state).");
}

void loop() {
  static uint32_t lastUpdate = micros();
  uint32_t now = micros();
  uint32_t elapsed = now - lastUpdate;
  if (elapsed < LOOP_PERIOD_US) {
    return;
  }
  lastUpdate = now;

  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  float gx = 0.0f, gy = 0.0f, gz = 0.0f;
  float mx = 0.0f, my = 0.0f, mz = 0.0f;

  bool bmxOk = getBMXSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
  if (!bmxOk) {
    // If BMX is not ready yet, skip this iteration.
    return;
  }

  float gyro_rad[3] = {gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD};
  float accel_mps2[3] = {ax, ay, az};

  ekf12.predict(accel_mps2, gyro_rad);
  ekf15.predict(accel_mps2, gyro_rad);

  float relAlt = getRelativeAltitude();
  ekf12.updateBaro(relAlt);
  ekf15.updateBaro(relAlt);

  static uint32_t logCounter = 0;
  if ((logCounter++ % 20U) == 0U) { // 10 Hz logging
    const float *p = ekf15.position();
    const float *v = ekf15.velocity();
    const float *q = ekf15.quaternion();
    Serial.print("EKF15 p[m]: ");
    Serial.print(p[0], 3); Serial.print(", ");
    Serial.print(p[1], 3); Serial.print(", ");
    Serial.print(p[2], 3); Serial.print(" | v[m/s]: ");
    Serial.print(v[0], 3); Serial.print(", ");
    Serial.print(v[1], 3); Serial.print(", ");
    Serial.print(v[2], 3); Serial.print(" | q: ");
    Serial.print(q[0], 4); Serial.print(", ");
    Serial.print(q[1], 4); Serial.print(", ");
    Serial.print(q[2], 4); Serial.print(", ");
    Serial.print(q[3], 4); Serial.print(" | alt: ");
    Serial.println(relAlt, 3);
  }
}

