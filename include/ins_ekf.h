#ifndef INS_EKF_H
#define INS_EKF_H

#include <Arduino.h>

/*
 * INS Error-State EKF Summary
 *
 * Nominal states (shared by 12- and 15-state variants):
 *   p_nom ∈ R^3 : position in navigation frame (Y-up)
 *   v_nom ∈ R^3 : velocity in navigation frame
 *   q_nom ∈ R^4 : unit quaternion (body → navigation)
 *   b_a_nom ∈ R^3 : accelerometer bias (body)
 *   b_g_nom ∈ R^3 : gyro bias (body, only used by 15-state numerics)
 *
 * Error-state vectors:
 *   12-state δx12 = [δp(3); δv(3); δθ(3); δb_a(3)]
 *   15-state δx15 = [δp(3); δv(3); δθ(3); δb_a(3); δb_g(3)]
 *
 * Nominal dynamics:
 *   ṗ = v
 *   v̇ = R_bn(q) · (a_m - b_a) + g_n        (g_n = [0, -g, 0] for Y-up)
 *   q̇ = 0.5 · Ω(ω_m - b_g) · q              (Ω is quaternion rate matrix)
 *   ḃ_a = 0 + noise_ba
 *   ḃ_g = 0 + noise_bg
 *
 * Error dynamics Jacobian Fc (major blocks):
 *   δṗ = δv
 *   δv̇ = -R_bn · skew(â) · δθ - R_bn · δb_a
 *   δθ̇ = -skew(ω̂) · δθ - δb_g (15-state only)
 *   δḃ_a = noise; δḃ_g = noise
 *
 * Measurement model (baro altitude):
 *   z_baro = e_h^T · p + v_b , with e_h = [0 1 0]^T (Y-up altitude)
 *   Linearized: z ≈ h(p_nom) + e_h^T δp + v_b → H = [e_h^T 0 0 0 0]
 *
 * Noise design:
 *   Process noise Q derived from BMX160 averages (200 Hz):
 *     accel PSD S_a ≈ 4.75e-4 (m/s^2)^2/Hz → σ_v^2 ≈ S_a·dt
 *     gyro PSD  S_g ≈ 5.60e-4 (rad/s)^2/Hz → σ_θ^2 ≈ S_g·dt
 *     accel bias RW via Allan A_a ≈ 8.71e-2 → S_ba ≈ A_a^2/1s, Q_ba ≈ S_ba·dt
 *     gyro bias RW  via Allan A_g ≈ 2.08e-1 → S_bg ≈ A_g^2/1s, Q_bg ≈ S_bg·dt
 *     Position noise uses simple integrated white accel model σ_p^2 ≈ (1/3) S_a·dt^3
 *   Measurement noise R from BMP388 bench noise: choose σ_baro ≈ 3 m → R = σ_baro^2
 */

class INSEKF12 {
public:
  static const int N = 12;

  INSEKF12();

  void initialize(const float *p0, const float *v0, const float *q0,
                  const float *ba0, const float *bg0);
  void setDt(float dtSeconds);
  void setProcessNoiseFromSensors();
  void setMeasurementNoiseFromSensors();
  void predict(const float *a_m, const float *w_m);
  void updateBaro(float z_baro);

  const float *position() const { return p_nom; }
  const float *velocity() const { return v_nom; }
  const float *quaternion() const { return q_nom; }
  const float *accelBias() const { return b_a_nom; }
  const float *gyroBias() const { return b_g_nom; }

private:
  float dt;
  float p_nom[3];
  float v_nom[3];
  float q_nom[4];
  float b_a_nom[3];
  float b_g_nom[3];

  float P[N][N];
  float Q[N][N];
  float R[1][1];

  void resetCovariance();
  void buildFc(const float *a_corr, const float *w_corr, float Fc[N][N]) const;
  void propagateCovariance(const float (&Fc)[N][N]);
  void injectErrorState(const float *dx);
};

class INSEKF15 {
public:
  static const int N = 15;

  INSEKF15();

  void initialize(const float *p0, const float *v0, const float *q0,
                  const float *ba0, const float *bg0);
  void setDt(float dtSeconds);
  void setProcessNoiseFromSensors();
  void setMeasurementNoiseFromSensors();
  void predict(const float *a_m, const float *w_m);
  void updateBaro(float z_baro);

  const float *position() const { return p_nom; }
  const float *velocity() const { return v_nom; }
  const float *quaternion() const { return q_nom; }
  const float *accelBias() const { return b_a_nom; }
  const float *gyroBias() const { return b_g_nom; }

private:
  float dt;
  float p_nom[3];
  float v_nom[3];
  float q_nom[4];
  float b_a_nom[3];
  float b_g_nom[3];

  float P[N][N];
  float Q[N][N];
  float R[1][1];

  void resetCovariance();
  void buildFc(const float *a_corr, const float *w_corr, float Fc[N][N]) const;
  void propagateCovariance(const float (&Fc)[N][N]);
  void injectErrorState(const float *dx);
};

#endif // INS_EKF_H

