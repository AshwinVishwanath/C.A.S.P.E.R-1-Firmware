#include "ekf_sensor_fusion.h"
#include "orientation_estimation.h"
#include "sensor_setup.h"
#include <SD.h>
#include <SPI.h>
#include <Arduino.h>

File rawDataFile;
File filteredDataFile;
File systemLogFile;

String rawDataBuffer       = "";
String filteredDataBuffer  = "";
String systemLogBuffer     = "";

int rawDataCount           = 0;
int filteredDataCount      = 0;
int systemLogCount         = 0;

const int RAW_BUFFER_THRESHOLD      = 100;
const int FILTERED_BUFFER_THRESHOLD = 100;
const int SYSTEM_LOG_BUFFER_THRESHOLD = 1;

void flushBuffer(File &file, String &buffer, int &count)
{
  if (file) {
    file.print(buffer);
    file.flush();
  }
  buffer = "";
  count  = 0;
}

void flushSystemLog()
{
  flushBuffer(systemLogFile, systemLogBuffer, systemLogCount);
}

void Summarylog(const String &msg)
{
  unsigned long timestamp = micros();
  String line = String(timestamp) + ": " + msg;
  systemLogBuffer += line + "\n";
  systemLogCount++;

  if (systemLogCount >= SYSTEM_LOG_BUFFER_THRESHOLD) {
    flushSystemLog();
  }
}

void logData(File &file, String &buffer, int &count, 
             const String &data, int threshold)
{
  buffer += data + "\n";
  count++;
  if (count >= threshold) {
    flushBuffer(file, buffer, count);
  }
}

void setupFiles()
{
  bool sdOk = false;
#ifdef BUILTIN_SDCARD
  sdOk = SD.begin(BUILTIN_SDCARD);
#else
  sdOk = SD.begin();
#endif

  if (!sdOk) {
    Serial.println("SD initialization failed!");
    Summarylog("ERROR: SD initialization failed!");
  } else {
    Serial.println("SD initialized!");
    Summarylog("SD initialized!");
  }

  rawDataFile = SD.open("RawDataFile.txt", FILE_WRITE);
  filteredDataFile = SD.open("FilteredDataFile.txt", FILE_WRITE);
  systemLogFile = SD.open("SystemLogFile.txt", FILE_WRITE);

  if (!rawDataFile || !filteredDataFile || !systemLogFile) {
    Summarylog("ERROR: Failed to open one or more log files!");
  } 
  else 
  {
    Summarylog("Successfully opened raw, filtered, and system log files.");
  }

  if (rawDataFile) {
    rawDataFile.println("timestamp,bno_ax,bno_ay,bno_az,bno_gx,bno_gy,bno_gz,bno_mx,bno_my,bno_mz,bmx_ax,bmx_ay,bmx_az,bmx_gx,bmx_gy,bmx_gz,bmx_mx,bmx_my,bmx_mz,baroAlt,rawPressure,rawTemperature,bnoRoll,bnoPitch,bnoYaw,kalmanHex");
    rawDataFile.flush();
  }
  if (filteredDataFile) {
    filteredDataFile.println("timestamp,x,Altitude,z,vx,vy,vz,roll,pitch,yaw");
    filteredDataFile.flush();
  }

}

void logRawData()
{
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
  float bmxAx, bmxAy, bmxAz, bmxGx, bmxGy, bmxGz, bmxMx, bmxMy, bmxMz;
  bool bmxOk = getBMXSensorData(bmxAx, bmxAy, bmxAz, bmxGx, bmxGy, bmxGz, bmxMx, bmxMy, bmxMz);

  bmp.performReading();
  float baroAlt = getRelativeAltitude();
  float rawPressure = bmp.pressure;
  float rawTemperature = bmp.temperature;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float bnoPitch = euler.x();
  float bnoRoll = euler.y();
  float bnoYaw = euler.z();

  auto serializeMatrixHex = [](const Eigen::MatrixXf &mat) -> String {
    String out = "";
    for (int i = 0; i < mat.rows(); ++i) {
      for (int j = 0; j < mat.cols(); ++j) {
        float val = mat(i, j);
        byte *bytes = (byte *)&val;
        for (int b = 0; b < 4; ++b) {
          if (bytes[b] < 16) out += "0";
          out += String(bytes[b], HEX);
        }
      }
    }
    return out;
  };

  String kalmanHex = serializeMatrixHex(lastKalmanGain);

  auto formatBmx = [&](float value) -> String {
    return bmxOk ? String(value, 3) : String("nan");
  };

  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                formatBmx(bmxAx) + "," + formatBmx(bmxAy) + "," + formatBmx(bmxAz) + "," +
                formatBmx(bmxGx) + "," + formatBmx(bmxGy) + "," + formatBmx(bmxGz) + "," +
                formatBmx(bmxMx) + "," + formatBmx(bmxMy) + "," + formatBmx(bmxMz) + "," +
                String(baroAlt, 3) + "," +
                String(rawPressure, 2) + "," + String(rawTemperature, 2) + "," +
                String(bnoRoll, 2) + "," + String(bnoPitch, 2) + "," + String(bnoYaw, 2) + "," +
                kalmanHex;

  logData(rawDataFile, rawDataBuffer, rawDataCount, line, RAW_BUFFER_THRESHOLD);
}

void logFilteredData()
{
  float x, y, z, vx, vy, vz;
  ekfGetState(x, y, z, vx, vy, vz);

  float roll, pitch, yaw;
  getIntegratedAngles(roll, pitch, yaw);

  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                String(roll, 3) + "," + String(pitch, 3) + "," + String(yaw, 3);

  logData(filteredDataFile, filteredDataBuffer, filteredDataCount, 
          line, FILTERED_BUFFER_THRESHOLD);
}

void logSensorData()
{
  logRawData();
  logFilteredData();
}

void flushAllBuffers()
{
  flushBuffer(rawDataFile,       rawDataBuffer,       rawDataCount);
  flushBuffer(filteredDataFile,  filteredDataBuffer,  filteredDataCount);
  flushSystemLog();
}

