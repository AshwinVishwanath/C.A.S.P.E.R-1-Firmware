#include <Arduino.h>
#include <math.h>

#include "ins_ekf.h"
#include "sensor_setup.h"

// Core loop at 200 Hz (5 ms)
namespace {
constexpr uint32_t LOOP_PERIOD_US = 5000U;
constexpr float DT_SEC = 0.005f;
constexpr uint8_t PLOT_PRECISION = 4U;

void quaternionToEuler(const float *q, float &roll, float &pitch, float &yaw) {
  const float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  const float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
  roll = atan2f(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
  if (fabsf(sinp) >= 1.0f) {
    pitch = copysignf(PI / 2.0f, sinp);
  } else {
    pitch = asinf(sinp);
  }

  const float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
  const float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
  yaw = atan2f(siny_cosp, cosy_cosp);
}

void emitPlotLine(const INSEKF15 &ekf) {
  const float *position = ekf.position();
  const float *velocity = ekf.velocity();
  const float *quat = ekf.quaternion();

  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  quaternionToEuler(quat, roll, pitch, yaw);

  Serial.print('>');
  Serial.print("roll:");
  Serial.print(roll, PLOT_PRECISION);
  Serial.print(",pitch:");
  Serial.print(pitch, PLOT_PRECISION);
  Serial.print(",yaw:");
  Serial.print(yaw, PLOT_PRECISION);
  Serial.print(",p_x:");
  Serial.print(position[0], PLOT_PRECISION);
  Serial.print(",p_y:");
  Serial.print(position[1], PLOT_PRECISION);
  Serial.print(",p_z:");
  Serial.print(position[2], PLOT_PRECISION);
  Serial.print(",v_x:");
  Serial.print(velocity[0], PLOT_PRECISION);
  Serial.print(",v_y:");
  Serial.print(velocity[1], PLOT_PRECISION);
  Serial.print(",v_z:");
  Serial.print(velocity[2], PLOT_PRECISION);
  Serial.print("\r\n");
}

void logEkfStatus(const INSEKF15 &ekf, float relAlt) {
  const float *p = ekf.position();
  const float *v = ekf.velocity();
  const float *q = ekf.quaternion();
  Serial.print("EKF15 p[m]: ");
  Serial.print(p[0], 3);
  Serial.print(", ");
  Serial.print(p[1], 3);
  Serial.print(", ");
  Serial.print(p[2], 3);
  Serial.print(" | v[m/s]: ");
  Serial.print(v[0], 3);
  Serial.print(", ");
  Serial.print(v[1], 3);
  Serial.print(", ");
  Serial.print(v[2], 3);
  Serial.print(" | q: ");
  Serial.print(q[0], 4);
  Serial.print(", ");
  Serial.print(q[1], 4);
  Serial.print(", ");
  Serial.print(q[2], 4);
  Serial.print(", ");
  Serial.print(q[3], 4);
  Serial.print(" | alt: ");
  Serial.println(relAlt, 3);
}
} // namespace

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

  ekf15.initialize(p0, v0, q0, ba0, bg0);
  ekf15.setDt(DT_SEC);
  ekf15.setProcessNoiseFromSensors();
  ekf15.setMeasurementNoiseFromSensors();

  Serial.println("INS EKF initialized (15-state).");
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

  float gyro_rad[3] = {static_cast<float>(gx * RAD_TO_DEG),
                       static_cast<float>(gy * RAD_TO_DEG),
                       static_cast<float>(gz * RAD_TO_DEG)};
  float accel_mps2[3] = {ax, ay, az};

  ekf15.predict(accel_mps2, gyro_rad);

  float relAlt = getRelativeAltitude();
  ekf15.updateBaro(relAlt);

  emitPlotLine(ekf15);

  static uint32_t logCounter = 0;
  if ((logCounter++ % 20U) == 0U) { // 10 Hz logging
    logEkfStatus(ekf15, relAlt);
  }
}

