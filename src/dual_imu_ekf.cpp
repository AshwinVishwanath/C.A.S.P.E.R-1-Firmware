#include "dual_imu_ekf.h"

#include <cmath>

namespace dual_imu_ekf {
namespace {
constexpr float GRAVITY = 9.80665f;

static void ZeroMatrix(float *M, std::size_t rows, std::size_t cols) {
  for (std::size_t r = 0U; r < rows; ++r) {
    for (std::size_t c = 0U; c < cols; ++c) {
      M[r * cols + c] = 0.0f;
    }
  }
}

static void IdentityMatrix(float *M, std::size_t rows, std::size_t cols) {
  ZeroMatrix(M, rows, cols);
  for (std::size_t i = 0U; i < rows && i < cols; ++i) {
    M[i * cols + i] = 1.0f;
  }
}

static void MatMul(const float *A, const float *B, float *C, std::size_t m,
                   std::size_t n, std::size_t p) {
  for (std::size_t i = 0U; i < m; ++i) {
    for (std::size_t j = 0U; j < p; ++j) {
      float sum = 0.0f;
      for (std::size_t k = 0U; k < n; ++k) {
        sum += A[i * n + k] * B[k * p + j];
      }
      C[i * p + j] = sum;
    }
  }
}

static void MatAdd(const float *A, const float *B, float *C, std::size_t rows,
                   std::size_t cols) {
  for (std::size_t r = 0U; r < rows; ++r) {
    for (std::size_t c = 0U; c < cols; ++c) {
      C[r * cols + c] = A[r * cols + c] + B[r * cols + c];
    }
  }
}

static void MatTranspose(const float *A, float *AT, std::size_t rows,
                         std::size_t cols) {
  for (std::size_t r = 0U; r < rows; ++r) {
    for (std::size_t c = 0U; c < cols; ++c) {
      AT[c * rows + r] = A[r * cols + c];
    }
  }
}

static void NormalizeQuat(float q[4]) {
  const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (n2 <= 0.0f) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }
  const float inv = 1.0f / std::sqrt(n2);
  q[0] *= inv;
  q[1] *= inv;
  q[2] *= inv;
  q[3] *= inv;
}

static void QuatToRotmBN(const float q[4], float R[9]) {
  // q = [w, x, y, z]
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];

  const float ww = w * w;
  const float xx = x * x;
  const float yy = y * y;
  const float zz = z * z;

  R[0] = ww + xx - yy - zz;
  R[1] = 2.0f * (x * y - w * z);
  R[2] = 2.0f * (x * z + w * y);

  R[3] = 2.0f * (x * y + w * z);
  R[4] = ww - xx + yy - zz;
  R[5] = 2.0f * (y * z - w * x);

  R[6] = 2.0f * (x * z - w * y);
  R[7] = 2.0f * (y * z + w * x);
  R[8] = ww - xx - yy + zz;
}

static void QuatFromOmega(const float q_in[4], const float omega[3], float dt,
                          float q_out[4]) {
  const float p = omega[0];
  const float q = omega[1];
  const float r = omega[2];

  const float half_dt = 0.5f * dt;
  const float dq[4] = {1.0f, p * half_dt, q * half_dt, r * half_dt};

  // quaternion multiplication dq * q_in
  q_out[0] = dq[0] * q_in[0] - dq[1] * q_in[1] - dq[2] * q_in[2] -
             dq[3] * q_in[3];
  q_out[1] = dq[0] * q_in[1] + dq[1] * q_in[0] + dq[2] * q_in[3] -
             dq[3] * q_in[2];
  q_out[2] = dq[0] * q_in[2] - dq[1] * q_in[3] + dq[2] * q_in[0] +
             dq[3] * q_in[1];
  q_out[3] = dq[0] * q_in[3] + dq[1] * q_in[2] - dq[2] * q_in[1] +
             dq[3] * q_in[0];
  NormalizeQuat(q_out);
}

static void ApplyDeltaQuat(float q[4], const float dq_vec[4]) {
  float q_new[4];
  // dq_vec is additive correction; treat as quaternion then renormalize.
  q_new[0] = q[0] + dq_vec[0];
  q_new[1] = q[1] + dq_vec[1];
  q_new[2] = q[2] + dq_vec[2];
  q_new[3] = q[3] + dq_vec[3];
  NormalizeQuat(q_new);
  q[0] = q_new[0];
  q[1] = q_new[1];
  q[2] = q_new[2];
  q[3] = q_new[3];
}

static void JosephUpdate(float *P, std::size_t dim, const float *K,
                         const float *H, const float *R, std::size_t meas_dim) {
  float I_KH[N_STATE * N_STATE];
  IdentityMatrix(I_KH, dim, dim);

  // I - K*H
  float KH[N_STATE * N_STATE];
  ZeroMatrix(KH, dim, dim);
  for (std::size_t r = 0U; r < dim; ++r) {
    for (std::size_t c = 0U; c < dim; ++c) {
      float sum = 0.0f;
      for (std::size_t k = 0U; k < meas_dim; ++k) {
        sum += K[r * meas_dim + k] * H[k * dim + c];
      }
      KH[r * dim + c] = sum;
    }
  }

  for (std::size_t i = 0U; i < dim; ++i) {
    for (std::size_t j = 0U; j < dim; ++j) {
      I_KH[i * dim + j] -= KH[i * dim + j];
    }
  }

  float temp[N_STATE * N_STATE];
  MatMul(I_KH, P, temp, dim, dim, dim);
  float I_KH_T[N_STATE * N_STATE];
  MatTranspose(I_KH, I_KH_T, dim, dim);

  float temp2[N_STATE * N_STATE];
  MatMul(temp, I_KH_T, temp2, dim, dim, dim);

  float KRK[N_STATE * N_STATE];
  ZeroMatrix(KRK, dim, dim);
  for (std::size_t r = 0U; r < dim; ++r) {
    for (std::size_t c = 0U; c < dim; ++c) {
      float sum = 0.0f;
      for (std::size_t k = 0U; k < meas_dim; ++k) {
        for (std::size_t j = 0U; j < meas_dim; ++j) {
          sum += K[r * meas_dim + k] * R[k * meas_dim + j] *
                 K[c * meas_dim + j];
        }
      }
      KRK[r * dim + c] = sum;
    }
  }

  MatAdd(temp2, KRK, P, dim, dim);
}

static void BuildProcessNoise(const EkfConfig &config, float dt, float Q[N_STATE][N_STATE]) {
  ZeroMatrix(&Q[0][0], N_STATE, N_STATE);

  float wg[3];
  float wa[3];
  for (std::size_t i = 0U; i < 3U; ++i) {
    const float wg_bmx = 1.0f / (config.imuNoise.sigma_gyro_bmx[i] *
                                 config.imuNoise.sigma_gyro_bmx[i]);
    const float wg_bno = 1.0f / (config.imuNoise.sigma_gyro_bno[i] *
                                 config.imuNoise.sigma_gyro_bno[i]);
    const float wa_bmx = 1.0f / (config.imuNoise.sigma_accel_bmx[i] *
                                 config.imuNoise.sigma_accel_bmx[i]);
    const float wa_bno = 1.0f / (config.imuNoise.sigma_accel_bno[i] *
                                 config.imuNoise.sigma_accel_bno[i]);

    wg[i] = 1.0f / (wg_bmx + wg_bno);
    wa[i] = 1.0f / (wa_bmx + wa_bno);
  }

  const float sigma_a_nav[3] = {wa[0], wa[1], wa[2]};
  for (std::size_t i = 0U; i < 3U; ++i) {
    const float pos_var = 0.25f * dt * dt * dt * dt * sigma_a_nav[i];
    const float vel_var = dt * dt * sigma_a_nav[i];
    Q[i][i] = pos_var;
    Q[3 + i][3 + i] = vel_var;
  }

  float avg_wg = (wg[0] + wg[1] + wg[2]) / 3.0f;
  const float quat_var = 0.25f * dt * dt * avg_wg;
  for (std::size_t i = 0U; i < 4U; ++i) {
    Q[6 + i][6 + i] = quat_var;
  }

  const float bg_var = config.sigma_bias_gyro * config.sigma_bias_gyro;
  const float ba_var = config.sigma_bias_accel * config.sigma_bias_accel;
  for (std::size_t i = 0U; i < 3U; ++i) {
    Q[10 + i][10 + i] = bg_var;
    Q[13 + i][13 + i] = ba_var;
  }
}

static void PropagateCovariance(float P[N_STATE][N_STATE], const float Fc[N_STATE][N_STATE],
                                const float Q[N_STATE][N_STATE], float dt) {
  float Fk[N_STATE][N_STATE];
  ZeroMatrix(&Fk[0][0], N_STATE, N_STATE);
  for (std::size_t i = 0U; i < N_STATE; ++i) {
    Fk[i][i] = 1.0f + Fc[i][i] * dt;
    for (std::size_t j = 0U; j < N_STATE; ++j) {
      if (i != j) {
        Fk[i][j] += Fc[i][j] * dt;
      }
    }
  }

  float FP[N_STATE][N_STATE];
  MatMul(&Fk[0][0], &P[0][0], &FP[0][0], N_STATE, N_STATE, N_STATE);

  float FkT[N_STATE][N_STATE];
  MatTranspose(&Fk[0][0], &FkT[0][0], N_STATE, N_STATE);

  float FPFt[N_STATE][N_STATE];
  MatMul(&FP[0][0], &FkT[0][0], &FPFt[0][0], N_STATE, N_STATE, N_STATE);

  MatAdd(&FPFt[0][0], &Q[0][0], &P[0][0], N_STATE, N_STATE);
}

static void ComputeEulerYUp(const float q[4], float &rollY, float &pitchX,
                            float &yawZ) {
  // Convert quaternion to rotation matrix and then to Euler using Z-Y-X order.
  float R[9];
  QuatToRotmBN(q, R);

  // R maps body -> nav. In Y-up, roll about Y corresponds to pitch in matrix.
  pitchX = std::atan2(R[7], R[8]);
  float sy = -R[6];
  float cy = std::sqrt(R[7] * R[7] + R[8] * R[8]);
  rollY = std::atan2(R[6], cy);
  yawZ = std::atan2(R[3], R[0]);
}

static void SmallAngleSkew(const float v[3], float S[9]) {
  S[0] = 0.0f;
  S[1] = -v[2];
  S[2] = v[1];
  S[3] = v[2];
  S[4] = 0.0f;
  S[5] = -v[0];
  S[6] = -v[1];
  S[7] = v[0];
  S[8] = 0.0f;
}

} // namespace

void TransformToBody(const float R[3][3], const float v_in[3], float v_out[3]) {
  for (std::size_t i = 0U; i < 3U; ++i) {
    v_out[i] = R[i][0] * v_in[0] + R[i][1] * v_in[1] + R[i][2] * v_in[2];
  }
}

void FuseImuMeasurements(const ImuNoiseConfig &noise, const float accel_bmx_body[3],
                         const float gyro_bmx_body_rad[3], const float accel_bno_body[3],
                         const float gyro_bno_body[3], ImuFusedMeasurement &out) {
  float wg_bmx[3];
  float wg_bno[3];
  float wa_bmx[3];
  float wa_bno[3];
  for (std::size_t i = 0U; i < 3U; ++i) {
    wg_bmx[i] = 1.0f /
                (noise.sigma_gyro_bmx[i] * noise.sigma_gyro_bmx[i]);
    wg_bno[i] = 1.0f /
                (noise.sigma_gyro_bno[i] * noise.sigma_gyro_bno[i]);
    wa_bmx[i] = 1.0f /
                (noise.sigma_accel_bmx[i] * noise.sigma_accel_bmx[i]);
    wa_bno[i] = 1.0f /
                (noise.sigma_accel_bno[i] * noise.sigma_accel_bno[i]);

    const float sum_wg = wg_bmx[i] + wg_bno[i];
    const float sum_wa = wa_bmx[i] + wa_bno[i];

    const float W_gyro_bmx = wg_bmx[i] / sum_wg;
    const float W_gyro_bno = wg_bno[i] / sum_wg;
    const float W_accel_bmx = wa_bmx[i] / sum_wa;
    const float W_accel_bno = wa_bno[i] / sum_wa;

    out.gyro_body[i] =
        W_gyro_bmx * gyro_bmx_body_rad[i] + W_gyro_bno * gyro_bno_body[i];
    out.accel_body[i] =
        W_accel_bmx * accel_bmx_body[i] + W_accel_bno * accel_bno_body[i];
  }
}

void EkfInitialize(EkfState &state, float P[N_STATE][N_STATE],
                   const EkfConfig &config) {
  for (std::size_t i = 0U; i < 3U; ++i) {
    state.p[i] = 0.0f;
    state.v[i] = 0.0f;
    state.bg[i] = 0.0f;
    state.ba[i] = 0.0f;
  }
  state.q[0] = 1.0f;
  state.q[1] = 0.0f;
  state.q[2] = 0.0f;
  state.q[3] = 0.0f;

  ZeroMatrix(&P[0][0], N_STATE, N_STATE);
  for (std::size_t i = 0U; i < N_STATE; ++i) {
    P[i][i] = config.initial_covariance[i];
  }
}

void EkfPredict(EkfState &state, float P[N_STATE][N_STATE],
                const EkfConfig &config, const ImuFusedMeasurement &imu,
                float dt) {
  // Bias corrected rates and specific force in body frame.
  float omega_body[3];
  float f_body[3];
  for (std::size_t i = 0U; i < 3U; ++i) {
    omega_body[i] = imu.gyro_body[i] - state.bg[i];
    f_body[i] = imu.accel_body[i] - state.ba[i];
  }

  // Quaternion propagation.
  float q_next[4];
  QuatFromOmega(state.q, omega_body, dt, q_next);

  // Rotation body -> nav.
  float R_BN[9];
  QuatToRotmBN(q_next, R_BN);

  // Acceleration in nav frame with gravity.
  float f_nav[3];
  f_nav[0] = R_BN[0] * f_body[0] + R_BN[1] * f_body[1] + R_BN[2] * f_body[2];
  f_nav[1] = R_BN[3] * f_body[0] + R_BN[4] * f_body[1] + R_BN[5] * f_body[2] -
             GRAVITY;
  f_nav[2] = R_BN[6] * f_body[0] + R_BN[7] * f_body[1] + R_BN[8] * f_body[2];

  for (std::size_t i = 0U; i < 3U; ++i) {
    state.v[i] += f_nav[i] * dt;
    state.p[i] += state.v[i] * dt + 0.5f * f_nav[i] * dt * dt;
  }

  state.q[0] = q_next[0];
  state.q[1] = q_next[1];
  state.q[2] = q_next[2];
  state.q[3] = q_next[3];

  // Linearised dynamics for covariance.
  float Fc[N_STATE][N_STATE];
  ZeroMatrix(&Fc[0][0], N_STATE, N_STATE);
  // Position dot depends on velocity.
  Fc[0][3] = 1.0f;
  Fc[1][4] = 1.0f;
  Fc[2][5] = 1.0f;

  // Velocity sensitivity to specific force and quaternion (approximate).
  float R_nb[9];
  // R_NB is transpose of R_BN
  R_nb[0] = R_BN[0];
  R_nb[1] = R_BN[3];
  R_nb[2] = R_BN[6];
  R_nb[3] = R_BN[1];
  R_nb[4] = R_BN[4];
  R_nb[5] = R_BN[7];
  R_nb[6] = R_BN[2];
  R_nb[7] = R_BN[5];
  R_nb[8] = R_BN[8];

  float skew_f[9];
  SmallAngleSkew(f_body, skew_f);
  // quaternion indices start at 6.
  for (std::size_t r = 0U; r < 3U; ++r) {
    for (std::size_t c = 0U; c < 3U; ++c) {
      Fc[3 + r][6 + c + 1U] = -R_BN[r * 3U + c];
    }
    Fc[3 + r][13 + r] = -R_BN[r * 3U + 0U] * 1.0f; // accel bias effect
  }
  // gyro bias affects quaternion derivative.
  Fc[6][10] = -0.5f;
  Fc[7][11] = -0.5f;
  Fc[8][12] = -0.5f;

  float Q[N_STATE][N_STATE];
  BuildProcessNoise(config, dt, Q);
  PropagateCovariance(P, Fc, Q, dt);
}

void EkfUpdateBaro(EkfState &state, float P[N_STATE][N_STATE],
                   const EkfConfig &config, const BaroMeasurement &baro) {
  float Hfull[1 * N_STATE] = {};
  Hfull[1] = 1.0f; // p_y index

  float z_pred = state.p[1];
  float innov = baro.altitude_m - z_pred;

  float PHt[N_STATE];
  for (std::size_t r = 0U; r < N_STATE; ++r) {
    PHt[r] = P[r][1];
  }
  float S = P[1][1] + config.sigma_baro_alt * config.sigma_baro_alt;
  float K[N_STATE];
  for (std::size_t i = 0U; i < N_STATE; ++i) {
    K[i] = PHt[i] / S;
  }

  // State update
  for (std::size_t i = 0U; i < 3U; ++i) {
    state.p[i] += K[i] * innov;
    state.v[i] += K[3 + i] * innov;
  }
  for (std::size_t i = 0U; i < 4U; ++i) {
    state.q[i] += K[6 + i] * innov;
  }
  for (std::size_t i = 0U; i < 3U; ++i) {
    state.bg[i] += K[10 + i] * innov;
    state.ba[i] += K[13 + i] * innov;
  }
  NormalizeQuat(state.q);

  // Covariance update (Joseph form)
  float Kmat[N_STATE * 1U];
  for (std::size_t i = 0U; i < N_STATE; ++i) {
    Kmat[i] = K[i];
  }
  float R = config.sigma_baro_alt * config.sigma_baro_alt;
  JosephUpdate(&P[0][0], N_STATE, Kmat, Hfull, &R, 1U);
}

void EkfUpdateMag(EkfState &state, float P[N_STATE][N_STATE],
                  const EkfConfig &config, const MagMeasurement &mag) {
  float R_BN[9];
  QuatToRotmBN(state.q, R_BN);
  float R_NB[9];
  MatTranspose(R_BN, R_NB, 3U, 3U);

  float B_pred[3];
  for (std::size_t i = 0U; i < 3U; ++i) {
    B_pred[i] = R_NB[i * 3U + 0U] * config.B_N[0] +
                R_NB[i * 3U + 1U] * config.B_N[1] +
                R_NB[i * 3U + 2U] * config.B_N[2];
  }

  float innov[3];
  for (std::size_t i = 0U; i < 3U; ++i) {
    innov[i] = mag.mag_body[i] - B_pred[i];
  }

  // Numerical Jacobian w.r.t quaternion elements.
  float H[3][N_STATE];
  ZeroMatrix(&H[0][0], 3U, N_STATE);
  const float eps = 1e-3f;
  for (std::size_t qidx = 0U; qidx < 4U; ++qidx) {
    float q_pert[4] = {state.q[0], state.q[1], state.q[2], state.q[3]};
    q_pert[qidx] += eps;
    NormalizeQuat(q_pert);
    float R_BN_eps[9];
    QuatToRotmBN(q_pert, R_BN_eps);
    float R_NB_eps[9];
    MatTranspose(R_BN_eps, R_NB_eps, 3U, 3U);

    float B_eps[3];
    for (std::size_t i = 0U; i < 3U; ++i) {
      B_eps[i] = R_NB_eps[i * 3U + 0U] * config.B_N[0] +
                 R_NB_eps[i * 3U + 1U] * config.B_N[1] +
                 R_NB_eps[i * 3U + 2U] * config.B_N[2];
      const float deriv = (B_eps[i] - B_pred[i]) / eps;
      H[i][6U + qidx] = deriv;
    }
  }

  // Compute S = HPH^T + R
  float HT[N_STATE][3];
  MatTranspose(&H[0][0], &HT[0][0], 3U, N_STATE);

  float PHt[N_STATE][3];
  MatMul(&P[0][0], &HT[0][0], &PHt[0][0], N_STATE, N_STATE, 3U);

  float S[3][3];
  MatMul(&H[0][0], &PHt[0][0], &S[0][0], 3U, N_STATE, 3U);
  for (std::size_t i = 0U; i < 3U; ++i) {
    S[i][i] += config.sigma_mag[i] * config.sigma_mag[i];
  }

  // Inverse of 3x3 S using adjugate method.
  const float det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) -
                    S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                    S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
  if (std::fabs(det) < 1e-6f) {
    return;
  }
  float invDet = 1.0f / det;
  float S_inv[3][3];
  S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * invDet;
  S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * invDet;
  S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * invDet;
  S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * invDet;
  S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * invDet;
  S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * invDet;
  S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * invDet;
  S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * invDet;
  S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * invDet;

  // K = P H^T S^-1
  float K[N_STATE][3];
  float temp[N_STATE][3];
  MatMul(&P[0][0], &HT[0][0], &temp[0][0], N_STATE, N_STATE, 3U);
  MatMul(&temp[0][0], &S_inv[0][0], &K[0][0], N_STATE, 3U, 3U);

  // State update
  float dx[N_STATE];
  for (std::size_t i = 0U; i < N_STATE; ++i) {
    dx[i] = K[i][0] * innov[0] + K[i][1] * innov[1] + K[i][2] * innov[2];
  }

  for (std::size_t i = 0U; i < 3U; ++i) {
    state.p[i] += dx[i];
    state.v[i] += dx[3 + i];
  }
  ApplyDeltaQuat(state.q, &dx[6]);
  for (std::size_t i = 0U; i < 3U; ++i) {
    state.bg[i] += dx[10 + i];
    state.ba[i] += dx[13 + i];
  }
  NormalizeQuat(state.q);

  // Covariance update
  float Rmat[9] = {};
  Rmat[0] = config.sigma_mag[0] * config.sigma_mag[0];
  Rmat[4] = config.sigma_mag[1] * config.sigma_mag[1];
  Rmat[8] = config.sigma_mag[2] * config.sigma_mag[2];
  JosephUpdate(&P[0][0], N_STATE, &K[0][0], &H[0][0], Rmat, 3U);
}

void EkfGetOutput(const EkfState &state, EkfOutput &out) {
  for (std::size_t i = 0U; i < 3U; ++i) {
    out.p[i] = state.p[i];
    out.v[i] = state.v[i];
    out.bg[i] = state.bg[i];
    out.ba[i] = state.ba[i];
  }
  for (std::size_t i = 0U; i < 4U; ++i) {
    out.q[i] = state.q[i];
  }
  ComputeEulerYUp(state.q, out.rollY, out.pitchX, out.yawZ);
}

} // namespace dual_imu_ekf

