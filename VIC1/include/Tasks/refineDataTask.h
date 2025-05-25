#ifndef C9FA84E4_6F50_43C2_938C_FDEA633670A9
#define C9FA84E4_6F50_43C2_938C_FDEA633670A9

#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include "SimpleKalmanFilter.h"
#include "getDataTask.h"

void refineDataCallback();
// TODO determine the best interval for this task
Task refineDataTask(200, TASK_FOREVER, &refineDataCallback);

typedef struct refinedData
{
  float temperature;
  float humidity;
  float latitude;
  float longitude;
  float altitude;
  float heading;

} refinedData_t;

volatile refinedData_t refinedData;

SimpleKalmanFilter filterLatitude(0.00005, 0.00001, 0.01);
SimpleKalmanFilter filterLongitude(0.00005, 0.00001, 0.01);
SimpleKalmanFilter filterAltitude(5.0, 1.0, 0.1);
SimpleKalmanFilter filterHeading(0.1, 0.1, 0.1);
SimpleKalmanFilter filterTemperature(0.1, 0.1, 0.1);

void refineDataCallback()
{
  if (refineDataTask.isFirstIteration())
  {
    Serial.println("RefineDataTask started");
    Serial.println("Expect intitial measurements to be innaccurate");
    // TODO initialize kalman filter
    // TODO initialize all necessary variables
  }
  // for our first attempt at this, lets just update all of our filters with the latest data, and we will act entirely on the GPS results from the kalman filter

  // this updates all of the refined data instances with the latest data from the kalman filter
  refinedData.latitude = filterLatitude.updateEstimate(sensorData.gpsLatitude);

  refinedData.longitude = filterLongitude.updateEstimate(sensorData.gpsLongitude);

  refinedData.altitude = filterAltitude.updateEstimate(sensorData.gpsAltitude);

  refinedData.heading = filterHeading.updateEstimate(sensorData.magHeading);

  refinedData.temperature = filterTemperature.updateEstimate(sensorData.temperature);

  // TODO implement kalman filtering for all datapoints
  // TODO provide a struct with final output data which can be updated all the time
  // TODO
}

#endif /* C9FA84E4_6F50_43C2_938C_FDEA633670A9 */
