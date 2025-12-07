#include "sensor_setup.h"
#include "BNO_LUT.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <DFRobot_BMX160.h>
#include <Wire.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;
DFRobot_BMX160 bmx160(&Wire1);

float baselineAltitude = 0.0f;
float altitudeBias = 0.0f;
bool bmxInitialized = false;

void setupSensors() {
  Wire1.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed!");
    while (1);
  }
  Serial.println("BNO055 initialized successfully.");
  bno.setExtCrystalUse(true);
  // bno.setSensorOffsets(BNO_CALIBRATION_OFFSETS);
  // Serial.println("BNO055 calibration offsets applied.");

  if (!bmx160.begin()) {
    Serial.println("BMX160 initialization failed!");
  } else {
    bmxInitialized = true;
    Serial.println("BMX160 initialized successfully.");
  }

  if (!bmp.begin_I2C(0x76, &Wire1)) {
    Serial.println("BMP388 initialization failed!");
    while (1);
  }
  Serial.println("BMP388 initialized successfully.");
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // altitudeBias = manualCalibrateBMP388();
  // Serial.print("Manual calibration complete. Altitude bias: ");
  // Serial.println(altitudeBias, 3);

  if (bmp.performReading()) {
    baselineAltitude = bmp.readAltitude(1013.25) - altitudeBias;
  } else {
    Serial.println("Failed to read initial BMP388 altitude.");
  }
}

float manualCalibrateBMP388() {
  Serial.println("Starting manual calibration for BMP388...");

  const int numSamples = 1000;
  float totalAltitude = 0.0f;
  for (int i = 0; i < numSamples; i++) {
    if (bmp.performReading()) {
      float currentAltitude = bmp.readAltitude(1013.25);
      totalAltitude += currentAltitude;
    } else {
      Serial.println("Failed to read BMP388 altitude during calibration.");
    }
    if (i % 100 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print((float)i / numSamples * 100, 1);
      Serial.println("%");
    }
  }
  float bias = totalAltitude / numSamples;
  Serial.print("BMP388 Calibration Complete. Altitude Bias: ");
  Serial.println(bias, 3);

  return bias;
}

float getRelativeAltitude() {
  if (bmp.performReading()) {
    float currentAltitude = bmp.readAltitude(1013.25);
    float correctedAltitude = currentAltitude - altitudeBias;
    return correctedAltitude - baselineAltitude;
  } else {
    Serial.println("BMP388 failed to read altitude.");
    return 0.0f;
  }
}

void getCorrectedIMUData(float &yaw, float &pitch, float &roll, 
                         float &ax_ned, float &ay_ned, float &az_ned, 
                         float &mx, float &my, float &mz) {
  sensors_event_t accelEvent, gyroEvent, magEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  yaw = gyroEvent.gyro.z;
  pitch = gyroEvent.gyro.y;
  roll = gyroEvent.gyro.x;
  ax_ned = accelEvent.acceleration.x;
  ay_ned = accelEvent.acceleration.y;
  az_ned = accelEvent.acceleration.z;
  mx = magEvent.magnetic.x;
  my = magEvent.magnetic.y;
  mz = magEvent.magnetic.z;
}

void getSensorData(float &ax, float &ay, float &az,
                   float &gx, float &gy, float &gz,
                   float &mx, float &my, float &mz) {
  sensors_event_t accelEvent, gyroEvent, magEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  ax = accelEvent.acceleration.x;
  ay = accelEvent.acceleration.y;
  az = accelEvent.acceleration.z;
  gx = gyroEvent.gyro.x;
  gy = gyroEvent.gyro.y;
  gz = gyroEvent.gyro.z;
  mx = magEvent.magnetic.x;
  my = magEvent.magnetic.y;
  mz = magEvent.magnetic.z;
}

bool getBMXSensorData(float &ax, float &ay, float &az,
                      float &gx, float &gy, float &gz,
                      float &mx, float &my, float &mz) {
  if (!bmxInitialized) {
    return false;
  }

  sBmx160SensorData_t magData, gyroData, accelData;
  bmx160.getAllData(&magData, &gyroData, &accelData);

  ax = accelData.x;
  ay = accelData.y;
  az = accelData.z;
  gx = gyroData.x;
  gy = gyroData.y;
  gz = gyroData.z;
  mx = magData.x;
  my = magData.y;
  mz = magData.z;

  return true;
}

bool isBMX160Ready() {
  return bmxInitialized;
}

