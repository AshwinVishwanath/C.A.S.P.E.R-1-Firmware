#include <Arduino.h>
#include <Wire.h>
#include "sensor_setup.h"
#include "ekf_sensor_fusion.h"
#include "orientation_estimation.h"
#include "datalogging.h"

const uint32_t LOOP_PERIOD_US = 5000; // 200 Hz core loop

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  setupSensors();
  float initialAltitude = getRelativeAltitude();
  ekfInit(0.0f, initialAltitude, 0.0f, 0.0f, 0.0f, 0.0f);
  resetIntegratedAngles();
#if ENABLE_DATALOGGING
  setupFiles();
  Summarylog("Core debug loop initialized with EKF and datalogging enabled.");
#endif
}

void loop() {
  static uint32_t lastUpdate = micros();
  uint32_t now = micros();
  uint32_t elapsed = now - lastUpdate;

  if (elapsed < LOOP_PERIOD_US) {
    return;
  }

  lastUpdate = now;
  float dt = elapsed / 1e6f;

  float bnoAx, bnoAy, bnoAz, bnoGx, bnoGy, bnoGz, bnoMx, bnoMy, bnoMz;
  getSensorData(bnoAx, bnoAy, bnoAz, bnoGx, bnoGy, bnoGz, bnoMx, bnoMy, bnoMz);

  float bmxAx = 0.0f, bmxAy = 0.0f, bmxAz = 0.0f;
  float bmxGx = 0.0f, bmxGy = 0.0f, bmxGz = 0.0f;
  float bmxMx = 0.0f, bmxMy = 0.0f, bmxMz = 0.0f;
  bool bmxOk = getBMXSensorData(bmxAx, bmxAy, bmxAz, bmxGx, bmxGy, bmxGz, bmxMx, bmxMy, bmxMz);

  float ax = bmxOk ? bmxAx : bnoAx;
  float ay = bmxOk ? bmxAy : bnoAy;
  float az = bmxOk ? bmxAz : bnoAz;
  float gx = bmxOk ? bmxGx : bnoGx;
  float gy = bmxOk ? bmxGy : bnoGy;
  float gz = bmxOk ? bmxGz : bnoGz;
  float mx = bmxOk ? bmxMx : bnoMx;
  float my = bmxOk ? bmxMy : bnoMy;
  float mz = bmxOk ? bmxMz : bnoMz;
  float relAlt = getRelativeAltitude();

  ekfPredict(ax, ay, az, dt);
  ekfUpdateBaro(relAlt);

  updateIntegratedAngles(gx, gy, gz, dt);

  float x, y, z, vx, vy, vz;
  ekfGetState(x, y, z, vx, vy, vz);

#if ENABLE_DATALOGGING
  logSensorData();
#endif

  float roll, pitch, yaw;
  getIntegratedAngles(roll, pitch, yaw);

  Serial.print(">alt_fused:");
  Serial.print(y, 3);
  Serial.print(",vel_fused:");
  Serial.print(vy, 3);
  Serial.print(",alt_raw:");
  Serial.print(relAlt, 3);
  Serial.print(",ax:");
  Serial.print(ax, 3);
  Serial.print(",ay:");
  Serial.print(ay, 3);
  Serial.print(",az:");
  Serial.print(az, 3);
  Serial.print(",roll:");
  Serial.print(roll, 3);
  Serial.print(",pitch:");
  Serial.print(pitch, 3);
  Serial.print(",yaw:");
  Serial.print(yaw, 3);
  Serial.print(",bno_ax:");
  Serial.print(bnoAx, 3);
  Serial.print(",bno_ay:");
  Serial.print(bnoAy, 3);
  Serial.print(",bno_az:");
  Serial.print(bnoAz, 3);
  Serial.print(",bno_gx:");
  Serial.print(bnoGx, 3);
  Serial.print(",bno_gy:");
  Serial.print(bnoGy, 3);
  Serial.print(",bno_gz:");
  Serial.print(bnoGz, 3);
  Serial.print(",bno_mx:");
  Serial.print(bnoMx, 3);
  Serial.print(",bno_my:");
  Serial.print(bnoMy, 3);
  Serial.print(",bno_mz:");
  Serial.print(bnoMz, 3);

  if (bmxOk) {
    Serial.print(",bmx_ax:");
    Serial.print(bmxAx, 3);
    Serial.print(",bmx_ay:");
    Serial.print(bmxAy, 3);
    Serial.print(",bmx_az:");
    Serial.print(bmxAz, 3);
    Serial.print(",bmx_gx:");
    Serial.print(bmxGx, 3);
    Serial.print(",bmx_gy:");
    Serial.print(bmxGy, 3);
    Serial.print(",bmx_gz:");
    Serial.print(bmxGz, 3);
    Serial.print(",bmx_mx:");
    Serial.print(bmxMx, 3);
    Serial.print(",bmx_my:");
    Serial.print(bmxMy, 3);
    Serial.print(",bmx_mz:");
    Serial.print(bmxMz, 3);
  }
  Serial.print("\r\n");
}
