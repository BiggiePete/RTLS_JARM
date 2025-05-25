#ifndef B69A16C7_21F5_4EE1_9780_168F9E3F6E54
#define B69A16C7_21F5_4EE1_9780_168F9E3F6E54

#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include <Wire.h>
#include <Arduino.h>
#include <AHT20.h>
#include <UbxGps.h>
#include <Tlv493d.h>

Tlv493d mag = Tlv493d();
AHT20 aht20;

using dataTaskData_t = struct dataTaskData
{
  float temperature;
  float humidity;
  float gpsLatitude;
  float gpsLongitude;
  float gpsAltitude;
  float combinedAltitude;
  float magHeading;
  float magDiffFromNorth;
  float imuX;
  float imuY;
  float imuZ;
  float magX;
  float magY;
  float magZ;
};

dataTaskData_t sensorData;

// Forced to create prototype for the callback function
// because the TaskScheduler library requires it
void getDataCallback();

Task getDataTask(200, TASK_FOREVER, &getDataCallback);
void getDataCallback()
{

  if (getDataTask.isFirstIteration())
  {
    Serial.println("Initializing all sensors...");
    // TODO setup all necessary systems
    aht20.begin();
    mag.begin();
  }

  sensorData.temperature = aht20.getTemperature();
  sensorData.humidity = aht20.getHumidity();

  // TODO get GPS data
  // TODO get IMU data
  // TODO get Magnetometer data
  // TODO determine rotational offset of Magnetometer on the board in reference to the north direction
}
#endif /* B69A16C7_21F5_4EE1_9780_168F9E3F6E54 */
