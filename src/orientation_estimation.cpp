#include "orientation_estimation.h"

namespace {
constexpr float FILTER_ALPHA = 0.7f;
constexpr float RAD_TO_DEG_SQUARED = RAD_TO_DEG * RAD_TO_DEG;

float integratedRoll = 0.0f;
float integratedPitch = 0.0f;
float integratedYaw = 0.0f;

float filteredGx = 0.0f;
float filteredGy = 0.0f;
float filteredGz = 0.0f;

float wrapAngle(float angle) {
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}
}

void resetIntegratedAngles() {
    integratedRoll = 0.0f;
    integratedPitch = 0.0f;
    integratedYaw = 0.0f;

    filteredGx = 0.0f;
    filteredGy = 0.0f;
    filteredGz = 0.0f;
}

void updateIntegratedAngles(float gx, float gy, float gz, float dt) {
    const float filterComplement = 1.0f - FILTER_ALPHA;
    filteredGx = FILTER_ALPHA * gx + filterComplement * filteredGx;
    filteredGy = FILTER_ALPHA * gy + filterComplement * filteredGy;
    filteredGz = FILTER_ALPHA * gz + filterComplement * filteredGz;

    integratedRoll = wrapAngle(integratedRoll + filteredGy * RAD_TO_DEG_SQUARED * dt);
    integratedPitch = wrapAngle(integratedPitch + filteredGx * RAD_TO_DEG_SQUARED * dt);
    integratedYaw = wrapAngle(integratedYaw + filteredGz * RAD_TO_DEG_SQUARED * dt);
}

void getIntegratedAngles(float &roll, float &pitch, float &yaw) {
    roll = integratedRoll;
    pitch = integratedPitch;
    yaw = integratedYaw;
}

