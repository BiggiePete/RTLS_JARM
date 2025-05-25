#ifndef C9FA84E4_6F50_43C2_938C_FDEA633670A9
#define C9FA84E4_6F50_43C2_938C_FDEA633670A9

#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include "SimpleKalmanFilter.h"
#include "getDataTask.h"
#include "motorDrivesTask.h"
#include "PID_v1.h"

void refineDataCallback();
// TODO determine the best interval for this task
Task refineDataTask(200, TASK_FOREVER, &refineDataCallback);

typedef struct refinedData_t
{
  float temperature;
  float humidity;
  float latitude;
  float longitude;
  float altitude;
  float heading;
  float pitch;
  float roll;

} refinedData_t;

volatile refinedData_t refinedData;

SimpleKalmanFilter filterLatitude(0.00005, 0.00001, 0.01);
SimpleKalmanFilter filterLongitude(0.00005, 0.00001, 0.01);
SimpleKalmanFilter filterAltitude(5.0, 1.0, 0.1);
SimpleKalmanFilter filterHeading(0.1, 0.1, 0.1);
SimpleKalmanFilter filterTemperature(0.1, 0.1, 0.1);
SimpleKalmanFilter filterPitch(0.1, 0.1, 0.1);
SimpleKalmanFilter filterRoll(0.1, 0.1, 0.1);

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

  refinedData.pitch = filterPitch.updateEstimate(sensorData.gyrX); // TODO find out which axis is which, and use the correct one

  refinedData.roll = filterRoll.updateEstimate(sensorData.gyrY); // TODO find out which axis is which, and use the correct one

  refinedData.temperature = filterTemperature.updateEstimate(sensorData.temperature);

  // TODO update motor drive percentages based on the refined data and the necessary motion to correct ourselves
  // TODO split up 100% of the drive into several parts, and make "factors" for each component
  // For instance, lets increase the power to the rear motordrives, and decrease the power to the front motors, to make the drone tilt forward

  // TODO there might be better ways to go about this, given thar we could lean more on the PID, and simply feed in logic about what the current state is
  // this section is based on the belief that our heading is near to or directly facing north
  // if the pitch is positive, we increase rear motors, and decrease front motors
  // if the pitch is negative, we increase front motors, and decrease rear motors

  motorDriveData.motorFL_PERCENT = 0.3f * refinedData.pitch * -1.0f;
  motorDriveData.motorFR_PERCENT = 0.3f * refinedData.pitch * -1.0f;
  motorDriveData.motorBL_PERCENT = 0.3f * refinedData.pitch;
  motorDriveData.motorBR_PERCENT = 0.3f * refinedData.pitch;

  // if the roll is positive, we decrease left motors, and increase right motors
  // if the roll is negative, we increase right motors, and decrease left motors
  motorDriveData.motorFL_PERCENT -= 0.3f * refinedData.roll;
  motorDriveData.motorFR_PERCENT += 0.3f * refinedData.roll;
  motorDriveData.motorBL_PERCENT -= 0.3f * refinedData.roll;
  motorDriveData.motorBR_PERCENT += 0.3f * refinedData.roll;

  // if heading points left of north we increase front left and back right ,and decrease front right and back left
  // if heading points right of north we increase front right and back left, and decrease front left and back right
  if (refinedData.heading < 90)
  {
    motorDriveData.motorFL_PERCENT += 0.3f * refinedData.heading / 180.0f;
    motorDriveData.motorFR_PERCENT -= 0.3f * refinedData.heading / 180.0f;
    motorDriveData.motorBL_PERCENT -= 0.3f * refinedData.heading / 180.0f;
    motorDriveData.motorBR_PERCENT += 0.3f * refinedData.heading / 180.0f;
  }
  else
  {
    motorDriveData.motorFL_PERCENT -= 0.3f * (360 - refinedData.heading) / 180.0f;
    motorDriveData.motorFR_PERCENT += 0.3f * (360 - refinedData.heading) / 180.0f;
    motorDriveData.motorBL_PERCENT += 0.3f * (360 - refinedData.heading) / 180.0f;
    motorDriveData.motorBR_PERCENT -= 0.3f * (360 - refinedData.heading) / 180.0f;
  }
}

#endif /* C9FA84E4_6F50_43C2_938C_FDEA633670A9 */
