#pragma once

#include <cstddef>

namespace dual_imu_ekf {

// State dimension: position (3) + velocity (3) + quaternion (4) + gyro bias (3)
// + accel bias (3) = 16.
constexpr std::size_t N_STATE = 16U;

struct EkfState {
  float p[3];  // position in nav frame (m)
  float v[3];  // velocity in nav frame (m/s)
  float q[4];  // quaternion body->nav (w, x, y, z)
  float bg[3]; // gyro bias (rad/s)
  float ba[3]; // accel bias (m/s^2)
};

struct EkfOutput {
  float p[3];
  float v[3];
  float q[4];
  float rollY;
  float pitchX;
  float yawZ;
  float bg[3];
  float ba[3];
};

struct ImuFusedMeasurement {
  float accel_body[3];
  float gyro_body[3];
};

struct BaroMeasurement {
  float altitude_m;
};

struct MagMeasurement {
  float mag_body[3];
};

struct ImuNoiseConfig {
  float sigma_gyro_bmx[3];
  float sigma_gyro_bno[3];
  float sigma_accel_bmx[3];
  float sigma_accel_bno[3];
};

struct EkfConfig {
  ImuNoiseConfig imuNoise;
  float sigma_baro_alt;           // std dev of altitude measurement (m)
  float sigma_mag[3];             // mag measurement std in body units
  float sigma_bias_gyro;          // gyro bias random walk (rad/s)
  float sigma_bias_accel;         // accel bias random walk (m/s^2)
  float B_N[3];                   // reference magnetic field in nav frame
  float initial_covariance[N_STATE]; // diagonal entries for initial P
  float R_B_body_from_BMX[3][3];  // rotation BMX -> body
  float R_B_body_from_BNO[3][3];  // rotation BNO -> body (likely identity)
};

void EkfInitialize(EkfState &state, float P[N_STATE][N_STATE],
                   const EkfConfig &config);

void EkfPredict(EkfState &state, float P[N_STATE][N_STATE],
                const EkfConfig &config, const ImuFusedMeasurement &imu,
                float dt);

void EkfUpdateBaro(EkfState &state, float P[N_STATE][N_STATE],
                   const EkfConfig &config, const BaroMeasurement &baro);

void EkfUpdateMag(EkfState &state, float P[N_STATE][N_STATE],
                  const EkfConfig &config, const MagMeasurement &mag);

void EkfGetOutput(const EkfState &state, EkfOutput &out);

// Utility helpers for building fused IMU measurements.
void FuseImuMeasurements(const ImuNoiseConfig &noise,
                         const float accel_bmx_body[3],
                         const float gyro_bmx_body_rad[3],
                         const float accel_bno_body[3],
                         const float gyro_bno_body[3],
                         ImuFusedMeasurement &out);

void TransformToBody(const float R[3][3], const float v_in[3], float v_out[3]);

} // namespace dual_imu_ekf

