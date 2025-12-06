#include <Arduino.h>
#include <ArduinoEigen.h>
#include <math.h>

#include "sensor_setup.h"

static const int n_x = 6;
static const int n_z = 1;

static Eigen::VectorXf x_mean;
static Eigen::MatrixXf P;
static Eigen::MatrixXf Q;
static Eigen::MatrixXf R;

Eigen::MatrixXf lastKalmanGain = Eigen::MatrixXf::Zero(n_x, 1);

static bool initialized = false;

void ekfInit(float x, float y, float z, float vx, float vy, float vz) {
    x_mean = Eigen::VectorXf(n_x);
    P      = Eigen::MatrixXf(n_x, n_x);
    Q      = Eigen::MatrixXf(n_x, n_x);
    R      = Eigen::MatrixXf(n_z, n_z);

    x_mean << x, y, z, vx, vy, vz;
    P = Eigen::MatrixXf::Identity(n_x, n_x) * 0.1f;

    Q.setZero();
    Q(0,0) = 0.02f;
    Q(1,1) = 0.04f;
    Q(2,2) = 0.03f;
    Q(3,3) = 0.4f;
    Q(4,4) = 0.36f;
    Q(5,5) = 0.32f;

    R(0,0) = 0.3f;

    initialized = true;
}

void ekfPredict(float ax, float ay, float az, float dt) {
    if (!initialized) return;

    float x  = x_mean(0);
    float y  = x_mean(1);
    float z  = x_mean(2);
    float vx = x_mean(3);
    float vy = x_mean(4);
    float vz = x_mean(5);

    auto f = [&](float vx_, float vy_, float vz_, float ax_, float ay_, float az_) {
        Eigen::VectorXf dx(n_x);
        dx << vx_, vy_, vz_, ax_, ay_, az_;
        return dx;
    };

    Eigen::VectorXf k1 = f(vx, vy, vz, ax, ay, az);
    Eigen::VectorXf k2 = f(
        vx + 0.5f * k1(3) * dt,
        vy + 0.5f * k1(4) * dt,
        vz + 0.5f * k1(5) * dt,
        ax, ay, az
    );
    Eigen::VectorXf k3 = f(
        vx + 0.5f * k2(3) * dt,
        vy + 0.5f * k2(4) * dt,
        vz + 0.5f * k2(5) * dt,
        ax, ay, az
    );
    Eigen::VectorXf k4 = f(
        vx + k3(3) * dt,
        vy + k3(4) * dt,
        vz + k3(5) * dt,
        ax, ay, az
    );

    Eigen::VectorXf dx_total = (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);

    x  += dx_total(0);
    y  += dx_total(1);
    z  += dx_total(2);
    vx += dx_total(3);
    vy += dx_total(4);
    vz += dx_total(5);

    Eigen::VectorXf x_pred(n_x);
    x_pred << x, y, z, vx, vy, vz;

    Eigen::MatrixXf F_mat = Eigen::MatrixXf::Identity(n_x, n_x);
    F_mat(0,3) = dt;
    F_mat(1,4) = dt;
    F_mat(2,5) = dt;

    P = F_mat * P * F_mat.transpose() + Q;

    x_mean = x_pred;
}

void ekfUpdateBaro(float alt_meas) {
    if (!initialized) return;

    Eigen::VectorXf z(n_z);
    z(0) = alt_meas;

    Eigen::VectorXf h(n_z);
    h(0) = x_mean(1);

    Eigen::VectorXf y_resid = z - h;

    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_z, n_x);
    H(0, 1) = 1.0f;

    Eigen::MatrixXf S = H * P * H.transpose() + R;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    x_mean += K * y_resid;

    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(n_x, n_x);
    P = (I - K * H) * P;

    lastKalmanGain = K;
}

void ekfUpdate(float ax_meas, float ay_meas, float az_meas, float alt_meas) {
    ekfUpdateBaro(alt_meas);
}

void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz) {
    x = x_mean(0);
    y = x_mean(1);
    z = x_mean(2);
    vx = x_mean(3);
    vy = x_mean(4);
    vz = x_mean(5);
}

