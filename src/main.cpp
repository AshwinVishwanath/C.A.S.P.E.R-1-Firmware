#include <Arduino.h>
#include <Wire.h>
#include "sensor_setup.h"
// #include "ekf_sensor_fusion.h"
// #include "orientation_estimation.h"
// #include "datalogging.h"

#include <vector>
#include <numeric>
#include <cmath>

const float COLLECTION_DURATION_SECONDS = 10.0f; // exposed collection duration

const uint32_t LOOP_PERIOD_US = 5000; // 200 Hz core loop

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  setupSensors();
  Serial.println("Beginning sensor noise characterization run...");
  Serial.print("Configured collection duration (s): ");
  Serial.println(COLLECTION_DURATION_SECONDS, 3);
}

void loop() {
  static uint32_t lastUpdate = micros();
  uint32_t now = micros();
  uint32_t elapsed = now - lastUpdate;

  if (elapsed < LOOP_PERIOD_US) {
    return;
  }

  lastUpdate = now;

  static bool initializedBuffers = false;
  static uint32_t startMillis = millis();
  static std::vector<float> bnoAccelX, bnoAccelY, bnoAccelZ;
  static std::vector<float> bnoGyroX, bnoGyroY, bnoGyroZ;
  static std::vector<float> bnoMagX, bnoMagY, bnoMagZ;
  static std::vector<float> bmxAccelX, bmxAccelY, bmxAccelZ;
  static std::vector<float> bmxGyroX, bmxGyroY, bmxGyroZ;
  static std::vector<float> bmxMagX, bmxMagY, bmxMagZ;
  static std::vector<float> baroAltitude;

  if (!initializedBuffers) {
    bnoAccelX.reserve(2000);
    bnoAccelY.reserve(2000);
    bnoAccelZ.reserve(2000);
    bnoGyroX.reserve(2000);
    bnoGyroY.reserve(2000);
    bnoGyroZ.reserve(2000);
    bnoMagX.reserve(2000);
    bnoMagY.reserve(2000);
    bnoMagZ.reserve(2000);
    bmxAccelX.reserve(2000);
    bmxAccelY.reserve(2000);
    bmxAccelZ.reserve(2000);
    bmxGyroX.reserve(2000);
    bmxGyroY.reserve(2000);
    bmxGyroZ.reserve(2000);
    bmxMagX.reserve(2000);
    bmxMagY.reserve(2000);
    bmxMagZ.reserve(2000);
    baroAltitude.reserve(2000);
    initializedBuffers = true;
  }

  float bnoAx, bnoAy, bnoAz, bnoGx, bnoGy, bnoGz, bnoMx, bnoMy, bnoMz;
  getSensorData(bnoAx, bnoAy, bnoAz, bnoGx, bnoGy, bnoGz, bnoMx, bnoMy, bnoMz);

  float bmxAx = 0.0f, bmxAy = 0.0f, bmxAz = 0.0f;
  float bmxGx = 0.0f, bmxGy = 0.0f, bmxGz = 0.0f;
  float bmxMx = 0.0f, bmxMy = 0.0f, bmxMz = 0.0f;
  bool bmxOk = getBMXSensorData(bmxAx, bmxAy, bmxAz, bmxGx, bmxGy, bmxGz, bmxMx, bmxMy, bmxMz);

  float relAlt = getRelativeAltitude();

  bnoAccelX.push_back(bnoAx);
  bnoAccelY.push_back(bnoAy);
  bnoAccelZ.push_back(bnoAz);
  bnoGyroX.push_back(bnoGx);
  bnoGyroY.push_back(bnoGy);
  bnoGyroZ.push_back(bnoGz);
  bnoMagX.push_back(bnoMx);
  bnoMagY.push_back(bnoMy);
  bnoMagZ.push_back(bnoMz);
  baroAltitude.push_back(relAlt);

  if (bmxOk) {
    bmxAccelX.push_back(bmxAx);
    bmxAccelY.push_back(bmxAy);
    bmxAccelZ.push_back(bmxAz);
    bmxGyroX.push_back(bmxGx);
    bmxGyroY.push_back(bmxGy);
    bmxGyroZ.push_back(bmxGz);
    bmxMagX.push_back(bmxMx);
    bmxMagY.push_back(bmxMy);
    bmxMagZ.push_back(bmxMz);
  }

  float elapsedSeconds = (millis() - startMillis) / 1000.0f;
  if (elapsedSeconds < COLLECTION_DURATION_SECONDS) {
    return;
  }

  Serial.println("Collection complete. Computing statistics...");
  const float sampleRateHz = 1e6f / static_cast<float>(LOOP_PERIOD_US);

  auto printStats = [&](const char *label, const std::vector<float> &data) {
    if (data.empty()) {
      Serial.print(label);
      Serial.println(": no data recorded");
      return;
    }
    const size_t n = data.size();
    float mean = std::accumulate(data.begin(), data.end(), 0.0f) / n;
    float variance = 0.0f;
    for (float v : data) {
      float diff = v - mean;
      variance += diff * diff;
    }
    variance /= (n > 1 ? (n - 1) : 1);
    float stddev = sqrtf(variance);

    float allanNumerator = 0.0f;
    if (n > 1) {
      for (size_t i = 0; i + 1 < n; ++i) {
        float delta = data[i + 1] - data[i];
        allanNumerator += delta * delta;
      }
      allanNumerator /= (2.0f * (n - 1));
    }
    float allanDeviation = sqrtf(allanNumerator);

    float psd = variance / sampleRateHz;

    Serial.print(label);
    Serial.print(",mean:");
    Serial.print(mean, 6);
    Serial.print(",std:");
    Serial.print(stddev, 6);
    Serial.print(",allan:");
    Serial.print(allanDeviation, 6);
    Serial.print(",psd:");
    Serial.println(psd, 6);
  };

  printStats("BNO055_AX", bnoAccelX);
  printStats("BNO055_AY", bnoAccelY);
  printStats("BNO055_AZ", bnoAccelZ);
  printStats("BNO055_GX", bnoGyroX);
  printStats("BNO055_GY", bnoGyroY);
  printStats("BNO055_GZ", bnoGyroZ);
  printStats("BNO055_MX", bnoMagX);
  printStats("BNO055_MY", bnoMagY);
  printStats("BNO055_MZ", bnoMagZ);

  printStats("BMX160_AX", bmxAccelX);
  printStats("BMX160_AY", bmxAccelY);
  printStats("BMX160_AZ", bmxAccelZ);
  printStats("BMX160_GX", bmxGyroX);
  printStats("BMX160_GY", bmxGyroY);
  printStats("BMX160_GZ", bmxGyroZ);
  printStats("BMX160_MX", bmxMagX);
  printStats("BMX160_MY", bmxMagY);
  printStats("BMX160_MZ", bmxMagZ);

  printStats("BARO_ALT", baroAltitude);

  Serial.println("Noise characterization complete. Please capture this log to a text file on the host PC.");

  while (true) {
    delay(1000);
  }
}

