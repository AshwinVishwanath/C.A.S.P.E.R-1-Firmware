#include <Arduino.h>
#include <cstddef>
#include <math.h>

#include "dual_imu_ekf.h"
#include "sensor_setup.h"

using namespace dual_imu_ekf;

namespace {
constexpr uint32_t LOOP_PERIOD_US = 5000U; // 200 Hz
constexpr float DEG_TO_RAD_F = static_cast<float>(M_PI / 180.0);
constexpr uint8_t PLOT_PRECISION = 4U;

EkfState g_state;
float g_P[N_STATE][N_STATE];
EkfConfig g_config = {};

void FillDefaultConfig() {
  // Noise placeholders; tune using sensor characterisation.
  const float sigma_gyro_bmx[3] = {0.02f, 0.02f, 0.02f};
  const float sigma_gyro_bno[3] = {0.01f, 0.01f, 0.01f};
  const float sigma_accel_bmx[3] = {0.25f, 0.25f, 0.25f};
  const float sigma_accel_bno[3] = {0.15f, 0.15f, 0.15f};

  for (std::size_t i = 0U; i < 3U; ++i) {
    g_config.imuNoise.sigma_gyro_bmx[i] = sigma_gyro_bmx[i];
    g_config.imuNoise.sigma_gyro_bno[i] = sigma_gyro_bno[i];
    g_config.imuNoise.sigma_accel_bmx[i] = sigma_accel_bmx[i];
    g_config.imuNoise.sigma_accel_bno[i] = sigma_accel_bno[i];
  }

  g_config.sigma_baro_alt = 3.0f;
  g_config.sigma_mag[0] = 0.8f;
  g_config.sigma_mag[1] = 0.8f;
  g_config.sigma_mag[2] = 0.8f;
  g_config.sigma_bias_gyro = 1e-4f;
  g_config.sigma_bias_accel = 1e-3f;

  // Reference magnetic field (approximate placeholder).
  g_config.B_N[0] = 0.2f;
  g_config.B_N[1] = 0.0f;
  g_config.B_N[2] = 0.5f;

  // BMX frame is rotated -90 deg about Z to align with BNO/body frame (Y-up).
  g_config.R_B_body_from_BMX[0][0] = 0.0f;
  g_config.R_B_body_from_BMX[0][1] = -1.0f;
  g_config.R_B_body_from_BMX[0][2] = 0.0f;
  g_config.R_B_body_from_BMX[1][0] = 1.0f;
  g_config.R_B_body_from_BMX[1][1] = 0.0f;
  g_config.R_B_body_from_BMX[1][2] = 0.0f;
  g_config.R_B_body_from_BMX[2][0] = 0.0f;
  g_config.R_B_body_from_BMX[2][1] = 0.0f;
  g_config.R_B_body_from_BMX[2][2] = 1.0f;

  // BNO frame defines body frame.
  g_config.R_B_body_from_BNO[0][0] = 1.0f;
  g_config.R_B_body_from_BNO[0][1] = 0.0f;
  g_config.R_B_body_from_BNO[0][2] = 0.0f;
  g_config.R_B_body_from_BNO[1][0] = 0.0f;
  g_config.R_B_body_from_BNO[1][1] = 1.0f;
  g_config.R_B_body_from_BNO[1][2] = 0.0f;
  g_config.R_B_body_from_BNO[2][0] = 0.0f;
  g_config.R_B_body_from_BNO[2][1] = 0.0f;
  g_config.R_B_body_from_BNO[2][2] = 1.0f;

  // Initial covariance diagonal.
  const float initP[N_STATE] = {1.0f, 1.0f, 1.0f,   // position
                                0.5f, 0.5f, 0.5f,   // velocity
                                1e-2f, 1e-2f, 1e-2f, 1e-2f, // quaternion
                                1e-3f, 1e-3f, 1e-3f,       // gyro bias
                                1e-2f, 1e-2f, 1e-2f};      // accel bias
  for (std::size_t i = 0U; i < N_STATE; ++i) {
    g_config.initial_covariance[i] = initP[i];
  }
}

void emitPlotLine(const EkfOutput &out) {
  Serial.print('>');
  Serial.print("roll:");
  Serial.print(out.rollY, PLOT_PRECISION);
  Serial.print(",pitch:");
  Serial.print(out.pitchX, PLOT_PRECISION);
  Serial.print(",yaw:");
  Serial.print(out.yawZ, PLOT_PRECISION);
  Serial.print(",p_x:");
  Serial.print(out.p[0], PLOT_PRECISION);
  Serial.print(",p_y:");
  Serial.print(out.p[1], PLOT_PRECISION);
  Serial.print(",p_z:");
  Serial.print(out.p[2], PLOT_PRECISION);
  Serial.print(",v_x:");
  Serial.print(out.v[0], PLOT_PRECISION);
  Serial.print(",v_y:");
  Serial.print(out.v[1], PLOT_PRECISION);
  Serial.print(",v_z:");
  Serial.print(out.v[2], PLOT_PRECISION);
  Serial.print("\r\n");
}

void logEkfStatus(const EkfOutput &out) {
  Serial.print("EKF p[m]: ");
  Serial.print(out.p[0], 3);
  Serial.print(", ");
  Serial.print(out.p[1], 3);
  Serial.print(", ");
  Serial.print(out.p[2], 3);
  Serial.print(" | v[m/s]: ");
  Serial.print(out.v[0], 3);
  Serial.print(", ");
  Serial.print(out.v[1], 3);
  Serial.print(", ");
  Serial.print(out.v[2], 3);
  Serial.print(" | q: ");
  Serial.print(out.q[0], 4);
  Serial.print(", ");
  Serial.print(out.q[1], 4);
  Serial.print(", ");
  Serial.print(out.q[2], 4);
  Serial.print(", ");
  Serial.print(out.q[3], 4);
  Serial.print(" | rpy: ");
  Serial.print(out.rollY, 3);
  Serial.print(", ");
  Serial.print(out.pitchX, 3);
  Serial.print(", ");
  Serial.println(out.yawZ, 3);
}
} // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  setupSensors();
  FillDefaultConfig();
  EkfInitialize(g_state, g_P, g_config);
  Serial.println("Dual-IMU EKF initialized.");
}

void loop() {
  static uint32_t lastUpdate = micros();
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastUpdate;
  if (elapsed < LOOP_PERIOD_US) {
    return;
  }
  lastUpdate = now;

  const float dt_sec = static_cast<float>(elapsed) / 1.0e6f;

  float accel_bmx[3];
  float gyro_bmx_deg[3];
  if (!ReadBmxAccelGyro(accel_bmx, gyro_bmx_deg)) {
    return;
  }

  float accel_bno[3];
  float gyro_bno[3];
  float mag_bno[3];
  if (!ReadBnoAccelGyroMag(accel_bno, gyro_bno, mag_bno)) {
    return;
  }

  float accel_bmx_body[3];
  float gyro_bmx_body_rad[3];
  float gyro_bmx_rad_temp[3];
  gyro_bmx_rad_temp[0] = gyro_bmx_deg[0] * DEG_TO_RAD_F;
  gyro_bmx_rad_temp[1] = gyro_bmx_deg[1] * DEG_TO_RAD_F;
  gyro_bmx_rad_temp[2] = gyro_bmx_deg[2] * DEG_TO_RAD_F;
  TransformToBody(g_config.R_B_body_from_BMX, accel_bmx, accel_bmx_body);
  TransformToBody(g_config.R_B_body_from_BMX, gyro_bmx_rad_temp,
                  gyro_bmx_body_rad);

  float accel_bno_body[3];
  float gyro_bno_body[3];
  TransformToBody(g_config.R_B_body_from_BNO, accel_bno, accel_bno_body);
  TransformToBody(g_config.R_B_body_from_BNO, gyro_bno, gyro_bno_body);

  ImuFusedMeasurement fusedImu;
  FuseImuMeasurements(g_config.imuNoise, accel_bmx_body, gyro_bmx_body_rad,
                      accel_bno_body, gyro_bno_body, fusedImu);

  EkfPredict(g_state, g_P, g_config, fusedImu, dt_sec);

  float altitude_m = 0.0f;
  if (ReadBmpAltitude(altitude_m)) {
    BaroMeasurement baro{altitude_m};
    EkfUpdateBaro(g_state, g_P, g_config, baro);
  }

  MagMeasurement magMeas;
  TransformToBody(g_config.R_B_body_from_BNO, mag_bno, magMeas.mag_body);
  EkfUpdateMag(g_state, g_P, g_config, magMeas);

  EkfOutput out;
  EkfGetOutput(g_state, out);
  emitPlotLine(out);

  static uint32_t logCounter = 0;
  if ((logCounter++ % 20U) == 0U) {
    logEkfStatus(out);
  }
}

