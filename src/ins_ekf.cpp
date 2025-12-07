#include "ins_ekf.h"

#include <cstddef>
#include <math.h>

static const float GRAVITY = 9.80665f;

static void zeroMatrix(float *M, int rows, int cols) {
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      M[r * cols + c] = 0.0f;
    }
  }
}

static void copyMatrix(const float *A, float *B, int rows, int cols) {
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      B[r * cols + c] = A[r * cols + c];
    }
  }
}

static void matMul(const float *A, const float *B, float *C, int m, int n,
                   int p) {
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < p; ++j) {
      float sum = 0.0f;
      for (int k = 0; k < n; ++k) {
        sum += A[i * n + k] * B[k * p + j];
      }
      C[i * p + j] = sum;
    }
  }
}

static void matAdd(const float *A, const float *B, float *C, int rows,
                   int cols) {
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      C[r * cols + c] = A[r * cols + c] + B[r * cols + c];
    }
  }
}

static void matTrans(const float *A, float *AT, int rows, int cols) {
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      AT[c * rows + r] = A[r * cols + c];
    }
  }
}

static void skew(const float v[3], float S[9]) {
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

static void quatNormalize(float q[4]) {
  float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                     q[3] * q[3]);
  if (norm > 0.0f) {
    float inv = 1.0f / norm;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
  }
}

static void quatToRot(const float q[4], float R[9]) {
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

static void quatIntegrate(const float q[4], const float w_b[3], float dt,
                          float q_out[4]) {
  // omega in rad/s
  float dq[4];
  dq[0] = 1.0f;
  dq[1] = 0.5f * w_b[0] * dt;
  dq[2] = 0.5f * w_b[1] * dt;
  dq[3] = 0.5f * w_b[2] * dt;
  quatNormalize(dq);

  // q_out = dq ⊗ q
  q_out[0] = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
  q_out[1] = dq[0] * q[1] + dq[1] * q[0] + dq[2] * q[3] - dq[3] * q[2];
  q_out[2] = dq[0] * q[2] - dq[1] * q[3] + dq[2] * q[0] + dq[3] * q[1];
  q_out[3] = dq[0] * q[3] + dq[1] * q[2] - dq[2] * q[1] + dq[3] * q[0];
  quatNormalize(q_out);
}

static void applyDeltaTheta(float q[4], const float dtheta[3]) {
  float dq[4];
  dq[0] = 1.0f;
  dq[1] = 0.5f * dtheta[0];
  dq[2] = 0.5f * dtheta[1];
  dq[3] = 0.5f * dtheta[2];
  quatNormalize(dq);

  float q_new[4];
  q_new[0] = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
  q_new[1] = dq[0] * q[1] + dq[1] * q[0] + dq[2] * q[3] - dq[3] * q[2];
  q_new[2] = dq[0] * q[2] - dq[1] * q[3] + dq[2] * q[0] + dq[3] * q[1];
  q_new[3] = dq[0] * q[3] + dq[1] * q[2] - dq[2] * q[1] + dq[3] * q[0];

  q[0] = q_new[0];
  q[1] = q_new[1];
  q[2] = q_new[2];
  q[3] = q_new[3];
  quatNormalize(q);
}

static void addVector(float *dst, const float *src, int n) {
  for (int i = 0; i < n; ++i) {
    dst[i] += src[i];
  }
}

template <size_t N>
void resetCovarianceMatrix(float (&P)[N][N]) {
  zeroMatrix(&P[0][0], static_cast<int>(N), static_cast<int>(N));
  for (size_t i = 0U; i < N; ++i) {
    P[i][i] = 1e-3f;
  }
}

template <size_t N>
void propagateCovarianceMatrix(float (&P)[N][N], const float (&Q)[N][N],
                               const float (&Fc)[N][N], float dt) {
  float Fk[N][N];
  zeroMatrix(&Fk[0][0], static_cast<int>(N), static_cast<int>(N));
  for (size_t i = 0U; i < N; ++i) {
    Fk[i][i] = 1.0f + Fc[i][i] * dt;
    for (size_t j = 0U; j < N; ++j) {
      if (i != j) {
        Fk[i][j] += Fc[i][j] * dt;
      }
    }
  }

  float FP[N][N];
  matMul(&Fk[0][0], &P[0][0], &FP[0][0], static_cast<int>(N),
         static_cast<int>(N), static_cast<int>(N));

  float FkT[N][N];
  matTrans(&Fk[0][0], &FkT[0][0], static_cast<int>(N), static_cast<int>(N));

  float FPFt[N][N];
  matMul(&FP[0][0], &FkT[0][0], &FPFt[0][0], static_cast<int>(N),
         static_cast<int>(N), static_cast<int>(N));

  matAdd(&FPFt[0][0], &Q[0][0], &P[0][0], static_cast<int>(N),
         static_cast<int>(N));
}

template <size_t N>
void applyBaroUpdate(float (&P)[N][N], const float (&R)[1][1], float innovation,
                     float (&dxOut)[N]) {
  float S = P[1][1] + R[0][0];
  float gainVec[N];
  for (size_t i = 0U; i < N; ++i) {
    gainVec[i] = P[i][1] / S;
    dxOut[i] = gainVec[i] * innovation;
  }

  float KH[N][N] = {};
  for (size_t r = 0U; r < N; ++r) {
    KH[r][1] = gainVec[r];
  }

  float IminusKH[N][N];
  for (size_t r = 0U; r < N; ++r) {
    for (size_t c = 0U; c < N; ++c) {
      float val = (r == c) ? 1.0f : 0.0f;
      IminusKH[r][c] = val - KH[r][c];
    }
  }

  float temp[N][N];
  matMul(&IminusKH[0][0], &P[0][0], &temp[0][0], static_cast<int>(N),
         static_cast<int>(N), static_cast<int>(N));
  copyMatrix(&temp[0][0], &P[0][0], static_cast<int>(N), static_cast<int>(N));
}

// ----------------------------------------------------------
// INSEKF15 implementation
// ----------------------------------------------------------

INSEKF15::INSEKF15() : dt(0.005f) {
  float p0[3] = {0.0f, 0.0f, 0.0f};
  float v0[3] = {0.0f, 0.0f, 0.0f};
  float q0[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float ba0[3] = {0.0f, 0.0f, 0.0f};
  float bg0[3] = {0.0f, 0.0f, 0.0f};
  initialize(p0, v0, q0, ba0, bg0);
  setProcessNoiseFromSensors();
  setMeasurementNoiseFromSensors();
}

void INSEKF15::initialize(const float *p0, const float *v0, const float *q0,
                          const float *ba0, const float *bg0) {
  for (int i = 0; i < 3; ++i) {
    p_nom[i] = p0[i];
    v_nom[i] = v0[i];
    b_a_nom[i] = ba0[i];
    b_g_nom[i] = bg0[i];
  }
  for (int i = 0; i < 4; ++i) {
    q_nom[i] = q0[i];
  }
  quatNormalize(q_nom);
  resetCovariance();
}

void INSEKF15::setDt(float dtSeconds) { dt = dtSeconds; }

void INSEKF15::resetCovariance() {
  resetCovarianceMatrix(P);
}

void INSEKF15::setProcessNoiseFromSensors() {
  zeroMatrix(&Q[0][0], N, N);
  const float Sa = 4.75e-4f;
  const float Sg = 5.60e-4f;
  const float AllanA = 8.71e-2f;
  const float AllanG = 2.08e-1f;

  const float sigma_v2 = Sa * dt;
  const float sigma_p2 = (1.0f / 3.0f) * Sa * dt * dt * dt;
  const float sigma_theta2 = Sg * dt;
  const float Sb_a = AllanA * AllanA;
  const float Sb_g = AllanG * AllanG;
  const float q_ba = Sb_a * dt;
  const float q_bg = Sb_g * dt;

  for (int i = 0; i < 3; ++i) {
    Q[i][i] = sigma_p2;
    Q[3 + i][3 + i] = sigma_v2;
    Q[6 + i][6 + i] = sigma_theta2;
    Q[9 + i][9 + i] = q_ba;
    Q[12 + i][12 + i] = q_bg;
  }
}

void INSEKF15::setMeasurementNoiseFromSensors() {
  const float sigma_baro = 3.0f;
  R[0][0] = sigma_baro * sigma_baro;
}

void INSEKF15::buildFc(const float *a_corr, const float *w_corr,
                       float Fc[N][N]) const {
  zeroMatrix(&Fc[0][0], N, N);
  float Rbn[9];
  quatToRot(q_nom, Rbn);

  float skew_a[9];
  skew(a_corr, skew_a);
  float skew_w[9];
  skew(w_corr, skew_w);

  for (int i = 0; i < 3; ++i) {
    Fc[i][3 + i] = 1.0f;
  }

  float tmp[9];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float sum = 0.0f;
      for (int k = 0; k < 3; ++k) {
        sum += Rbn[r * 3 + k] * skew_a[k * 3 + c];
      }
      tmp[r * 3 + c] = -sum;
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Fc[3 + r][6 + c] = tmp[r * 3 + c];
    }
  }

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Fc[3 + r][9 + c] = -Rbn[r * 3 + c];
    }
  }

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Fc[6 + r][6 + c] = -skew_w[r * 3 + c];
    }
  }

  for (int i = 0; i < 3; ++i) {
    Fc[6 + i][12 + i] = -1.0f;
  }
}

void INSEKF15::propagateCovariance(const float (&Fc)[N][N]) {
  propagateCovarianceMatrix(P, Q, Fc, dt);
}

void INSEKF15::injectErrorState(const float *dx) {
  addVector(p_nom, dx, 3);
  addVector(v_nom, dx + 3, 3);
  applyDeltaTheta(q_nom, dx + 6);
  addVector(b_a_nom, dx + 9, 3);
  addVector(b_g_nom, dx + 12, 3);
}

void INSEKF15::predict(const float *a_m, const float *w_m) {
  float a_corr[3];
  float w_corr[3];
  for (int i = 0; i < 3; ++i) {
    a_corr[i] = a_m[i] - b_a_nom[i];
    w_corr[i] = w_m[i] - b_g_nom[i];
  }

  float Rbn[9];
  quatToRot(q_nom, Rbn);

  float acc_nav[3];
  for (int r = 0; r < 3; ++r) {
    acc_nav[r] = Rbn[r * 3 + 0] * a_corr[0] + Rbn[r * 3 + 1] * a_corr[1] +
                 Rbn[r * 3 + 2] * a_corr[2];
  }
  acc_nav[1] -= GRAVITY;

  for (int i = 0; i < 3; ++i) {
    p_nom[i] += v_nom[i] * dt;
    v_nom[i] += acc_nav[i] * dt;
  }
  quatIntegrate(q_nom, w_corr, dt, q_nom);

  float Fc[N][N];
  buildFc(a_corr, w_corr, Fc);
  propagateCovariance(Fc);
}

void INSEKF15::updateBaro(float z_baro) {
  float innovation = z_baro - p_nom[1];
  float dx[N];
  applyBaroUpdate(P, R, innovation, dx);
  injectErrorState(dx);
}

