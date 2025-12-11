#include "sensor_setup.h"
#include "BNO_LUT.h"               // optional: for calibration offsets, if you use them
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <DFRobot_BMX160.h>
#include <Wire.h>

// -----------------------------------------------------------------------------
// I2C buses
//   - BNO055 on default Wire
//   - BMX160 + BMP388 on Wire1 (pins 16=SCL1, 17=SDA1 on Teensy 4.1)
// -----------------------------------------------------------------------------
Adafruit_BNO055 bno(55, 0x28);
Adafruit_BMP3XX bmp;
DFRobot_BMX160 bmx160(&Wire1);

// -----------------------------------------------------------------------------
// Globals for baro baseline
// -----------------------------------------------------------------------------
static float baselineAltitude = 0.0f;
static float altitudeBias     = 0.0f;
static bool  bmxInitialized   = false;

// -----------------------------------------------------------------------------
// Public helper: BMX init status
// -----------------------------------------------------------------------------
bool isBMX160Ready()
{
  return bmxInitialized;
}

// -----------------------------------------------------------------------------
// Sensor setup
// -----------------------------------------------------------------------------
void setupSensors()
{
  // Start both I2C buses
  Wire.begin();              // default I2C bus for BNO055
  Wire1.begin();             // Wire1 for BMX160 + BMP388

  // ---------------- BNO055 ----------------
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed!");
    while (1) {
      // Hard fail – cannot proceed without main IMU
    }
  }
  Serial.println("BNO055 initialized successfully.");
  bno.setExtCrystalUse(true);

  // If you have calibration offsets:
  // bno.setSensorOffsets(BNO_CALIBRATION_OFFSETS);
  // Serial.println("BNO055 calibration offsets applied.");

  // ---------------- BMX160 ----------------
  if (!bmx160.begin()) {
    Serial.println("BMX160 initialization failed!");
    bmxInitialized = false;
  } else {
    bmxInitialized = true;
    Serial.println("BMX160 initialized successfully.");

    // Optional: configure ranges/ODR if needed.
    // Check the DFRobot_BMX160 docs for exact API; examples:
    // bmx160.setGyroRange(eGyroRange_2000DPS);
    // bmx160.setAccelRange(eAccelRange_16G);
    // bmx160.setOutputDataRate(eGyroOdr_200Hz, eAccelOdr_200Hz);
  }

  // ---------------- BMP388 ----------------
  if (!bmp.begin_I2C(0x76, &Wire1)) {
    Serial.println("BMP388 initialization failed!");
    while (1) {
      // Hard fail – baro is critical for altitude
    }
  }
  Serial.println("BMP388 initialized successfully.");

  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // If you want a manual long calibration, uncomment:
  // altitudeBias = manualCalibrateBMP388();
  // Serial.print("Manual calibration complete. Altitude bias: ");
  // Serial.println(altitudeBias, 3);

  // Take initial baseline altitude (relative reference)
  if (bmp.performReading()) {
    float altNow = bmp.readAltitude(1013.25f);  // sea-level pressure in hPa
    baselineAltitude = altNow - altitudeBias;
    Serial.print("Baseline altitude set to: ");
    Serial.println(baselineAltitude, 3);
  } else {
    Serial.println("Failed to read initial BMP388 altitude.");
    baselineAltitude = 0.0f;
  }
}

// -----------------------------------------------------------------------------
// Optional blocking baro calibration: averages N samples to estimate bias
// -----------------------------------------------------------------------------
float manualCalibrateBMP388()
{
  Serial.println("Starting manual calibration for BMP388...");

  const int numSamples = 1000;
  float totalAltitude  = 0.0f;

  for (int i = 0; i < numSamples; i++) {
    if (bmp.performReading()) {
      float currentAltitude = bmp.readAltitude(1013.25f);
      totalAltitude += currentAltitude;
    } else {
      Serial.println("Failed to read BMP388 altitude during calibration.");
    }

    if ((i % 100) == 0) {
      float progress = (static_cast<float>(i) / static_cast<float>(numSamples)) * 100.0f;
      Serial.print("Calibration progress: ");
      Serial.print(progress, 1);
      Serial.println("%");
    }
  }

  float bias = totalAltitude / static_cast<float>(numSamples);
  Serial.print("BMP388 Calibration Complete. Altitude Bias: ");
  Serial.println(bias, 3);

  return bias;
}

// -----------------------------------------------------------------------------
// Simple relative altitude helper (optional use)
// -----------------------------------------------------------------------------
float getRelativeAltitude()
{
  if (!bmp.performReading()) {
    Serial.println("BMP388 failed to read altitude.");
    return 0.0f;
  }

  float currentAltitude   = bmp.readAltitude(1013.25f);
  float correctedAltitude = currentAltitude - altitudeBias;
  return correctedAltitude - baselineAltitude;
}

// -----------------------------------------------------------------------------
// Live BMX160 accel + gyro (deg/s) via I2C on Wire1
//   accel_bmx: [ax, ay, az] (sensor units, typically m/s^2 or LSB scaled)
//   gyro_bmx_deg: [gx, gy, gz] in deg/s (as provided by DFRobot library)
// -----------------------------------------------------------------------------
bool ReadBmxAccelGyro(float accel_bmx[3], float gyro_bmx_deg[3])
{
  if (!bmxInitialized) {
    return false;
  }

  sBmx160SensorData_t magData;
  sBmx160SensorData_t gyroData;
  sBmx160SensorData_t accelData;

  // DFRobot_BMX160 fills all three structs in one call
  bmx160.getAllData(&magData, &gyroData, &accelData);

  accel_bmx[0] = accelData.x;
  accel_bmx[1] = accelData.y;
  accel_bmx[2] = accelData.z;

  gyro_bmx_deg[0] = gyroData.x;
  gyro_bmx_deg[1] = gyroData.y;
  gyro_bmx_deg[2] = gyroData.z;

  return true;
}

// -----------------------------------------------------------------------------
// Live BNO055 accel + gyro + mag via Adafruit_BNO055
//   accel_bno: [ax, ay, az] (m/s^2)
//   gyro_bno_rad: [gx, gy, gz] (rad/s)
//   mag_bno: [mx, my, mz] (mag units, e.g. µT)
// -----------------------------------------------------------------------------
bool ReadBnoAccelGyroMag(float accel_bno[3],
                         float gyro_bno_rad[3],
                         float mag_bno[3])
{
  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t magEvent;

  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent,  Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magEvent,   Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Accel in m/s^2
  accel_bno[0] = accelEvent.acceleration.x;
  accel_bno[1] = accelEvent.acceleration.y;
  accel_bno[2] = accelEvent.acceleration.z;

  // Gyro in rad/s
  gyro_bno_rad[0] = gyroEvent.gyro.x;
  gyro_bno_rad[1] = gyroEvent.gyro.y;
  gyro_bno_rad[2] = gyroEvent.gyro.z;

  // Mag in whatever units BNO uses (consistent across axes)
  mag_bno[0] = magEvent.magnetic.x;
  mag_bno[1] = magEvent.magnetic.y;
  mag_bno[2] = magEvent.magnetic.z;

  return true;
}

// -----------------------------------------------------------------------------
// Live BMP388 relative altitude (meters)
//   altitude_m = (currentAltitude - altitudeBias) - baselineAltitude
// -----------------------------------------------------------------------------
bool ReadBmpAltitude(float &altitude_m)
{
  if (!bmp.performReading()) {
    return false;
  }

  float currentAltitude   = bmp.readAltitude(1013.25f);
  float correctedAltitude = currentAltitude - altitudeBias;
  altitude_m = correctedAltitude - baselineAltitude;

  return true;
}
