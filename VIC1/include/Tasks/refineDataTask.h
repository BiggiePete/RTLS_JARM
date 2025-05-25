#ifndef C9FA84E4_6F50_43C2_938C_FDEA633670A9
#define C9FA84E4_6F50_43C2_938C_FDEA633670A9

#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include "SimpleKalmanFilter.h"

void refineDataCallback();
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

void refineDataCallback()
{
  // TODO implement kalman filtering for all datapoints
  // TODO provide a struct with final output data which can be updated all the time
  // TODO
}

#endif /* C9FA84E4_6F50_43C2_938C_FDEA633670A9 */
