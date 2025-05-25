#ifndef B69A16C7_21F5_4EE1_9780_168F9E3F6E54
#define B69A16C7_21F5_4EE1_9780_168F9E3F6E54

#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include <Wire.h>
#include <Arduino.h>
#include <AHT20.h>
#include <UbxGps.h>
#include <Tlv493d.h>
#include "ICM42688.h"
#include <XGZP6897D.h>
#include <cmath>
#include "UbxGpsNavPvt.h"

UbxGpsNavPvt<HardwareSerial> gps(Serial2);

#define K 64              // based on the docs (scaling factor for the pressure range)
#define NORTH_OFFSET 0.0f // TODO determine the offset of the magnetometer on the board in reference to the north direction

Tlv493d mag = Tlv493d();
AHT20 aht20;
ICM42688 IMU(Wire, 0x68);
XGZP6897D pressure(K);

using dataTaskData_t = struct dataTaskData
{
  float temperature;
  float pressureTemperature;
  float humidity;
  float gpsLatitude;
  float gpsLongitude;
  float gpsAltitude;
  float combinedAltitude;
  float magHeading;
  float magDiffFromNorth;
  float pressure;
  float pressureAltitude;
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
  float magX;
  float magY;
  float magZ;
};

volatile dataTaskData_t sensorData;

// Forced to create prototype for the callback function
// because the TaskScheduler library requires it
void getDataCallback();

Task getDataTask(200, TASK_FOREVER, &getDataCallback);
void getDataCallback()
{

  if (getDataTask.isFirstIteration())
  {
    Serial.println("Initializing all sensors...");
    aht20.begin();

    mag.begin();
    mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
    mag.disableTemp();

    IMU.begin();
    // set scale and ODR
    IMU.setAccelFS(ICM42688::AccelFS::gpm16);
    IMU.setGyroFS(ICM42688::GyroFS::dps2000);
    IMU.setAccelODR(ICM42688::odr12_5);
    IMU.setGyroODR(ICM42688::odr12_5);

    while (!pressure.begin())
    {
      Serial.println("Pressure sensor not found, retrying...");
      delay(500);
    }
    gps.begin(9600);
  }

  sensorData.temperature = aht20.getTemperature();
  sensorData.humidity = aht20.getHumidity();

  mag.updateData(); // we should be delaying between every reading according to the docs & examples, but if the task is running slow enough, we should be able to survive

  sensorData.magX = mag.getX();
  sensorData.magY = mag.getY();
  sensorData.magZ = mag.getZ();

  sensorData.magHeading = mag.getPolar();
  sensorData.magDiffFromNorth = sensorData.magHeading - NORTH_OFFSET;

  if (sensorData.magDiffFromNorth < 0)
  {
    sensorData.magDiffFromNorth += 360;
  }
  else if (sensorData.magDiffFromNorth > 360)
  {
    sensorData.magDiffFromNorth -= 360;
  }
  // read IMU
  IMU.getAGT();

  // set corresponding sensor data
  sensorData.accX = IMU.accX();
  sensorData.accY = IMU.accY();
  sensorData.accZ = IMU.accZ();

  sensorData.gyrX = IMU.gyrX();
  sensorData.gyrY = IMU.gyrY();
  sensorData.gyrZ = IMU.gyrZ();

  // read pressure
  float pTemperature = 0;
  float pPressure = 0;
  pressure.readSensor(pTemperature, pPressure);

  sensorData.pressureTemperature = pTemperature;
  sensorData.pressure = pPressure;

  // Calculate altitude from pressure using the barometric formula
  // likely to be less accurate than the GPS altitude, but we can use it to refine our GPS altitude
  sensorData.pressureAltitude = (1 - pow(pPressure / 101325, 0.190284)) * 288.15 / 0.0065;

  // consider if it is necessary to move the GPS data to another task, as the GPS readings, will likely be slow, and block this task for longer than necessary
  if (gps.ready())
  {
    sensorData.gpsLongitude = gps.lon / 10000000.0;
    sensorData.gpsLatitude = gps.lat / 10000000.0;
    sensorData.gpsAltitude = gps.height / 1000.0;
  }

  // for now, debug print all of the sensor data
  Serial.printf("Temperature: %.2f C\n", sensorData.temperature);
  Serial.printf("Humidity: %.2f %%\n", sensorData.humidity);
  Serial.printf("Pressure: %.2f hPa\n", sensorData.pressure);
  Serial.printf("Pressure Temperature: %.2f C\n", sensorData.pressureTemperature);
  Serial.printf("Pressure Altitude: %.2f m\n", sensorData.pressureAltitude);
  Serial.printf("GPS Latitude: %.7f\n", sensorData.gpsLatitude);
  Serial.printf("GPS Longitude: %.7f\n", sensorData.gpsLongitude);
  Serial.printf("GPS Altitude: %.2f m\n", sensorData.gpsAltitude);
  Serial.printf("Magnetometer X: %.2f\n", sensorData.magX);
  Serial.printf("Magnetometer Y: %.2f\n", sensorData.magY);
  Serial.printf("Magnetometer Z: %.2f\n", sensorData.magZ);
  Serial.printf("Magnetometer Heading: %.2f\n", sensorData.magHeading);
  Serial.printf("Magnetometer Diff from North: %.2f\n", sensorData.magDiffFromNorth);
  Serial.printf("Accelerometer X: %.2f\n", sensorData.accX);
  Serial.printf("Accelerometer Y: %.2f\n", sensorData.accY);
  Serial.printf("Accelerometer Z: %.2f\n", sensorData.accZ);
  Serial.printf("Gyroscope X: %.2f\n", sensorData.gyrX);
  Serial.printf("Gyroscope Y: %.2f\n", sensorData.gyrY);
  Serial.printf("Gyroscope Z: %.2f\n", sensorData.gyrZ);
  Serial.println();
}

#endif /* B69A16C7_21F5_4EE1_9780_168F9E3F6E54 */
