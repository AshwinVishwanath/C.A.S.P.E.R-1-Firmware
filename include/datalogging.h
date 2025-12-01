#ifndef DATALOGGING_H
#define DATALOGGING_H

#include <Arduino.h>

#ifndef ENABLE_DATALOGGING
#define ENABLE_DATALOGGING 0
#endif

// Initializes the SD card and opens files for logging raw and filtered sensor data.
void setupFiles();

// Logs both raw sensor data (IMU and barometric altitude) and filtered sensor data (EKF and VQF outputs).
void logSensorData();

// Flushes any remaining buffered data to the SD files.
void flushAllBuffers();

// Log a message to the system log with timestamp
void Summarylog(const String &msg);

#endif // DATALOGGING_H
