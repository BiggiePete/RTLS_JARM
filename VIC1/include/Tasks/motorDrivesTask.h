#ifndef B010FF50_A56C_4662_A550_96F225830788
#define B010FF50_A56C_4662_A550_96F225830788
#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include "ESC.h"
typedef struct motorDriveData
{
  float motorFL;
  float motorFR;
  float motorRL;
  float motorRR;
} motorDriveData_t;

volatile motorDriveData_t motorDriveData;

void motorDriveCallback();

#define SPEED_MIN (1000) // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000) // Set the Minimum Speed in microseconds

ESC motorFL(PA11, SPEED_MIN, SPEED_MAX, SPEED_MIN);
ESC motorFR(PA10, SPEED_MIN, SPEED_MAX, SPEED_MIN);
ESC motorRL(PA9, SPEED_MIN, SPEED_MAX, SPEED_MIN);
ESC motorRR(PA8, SPEED_MIN, SPEED_MAX, SPEED_MIN);

Task motorDriveTask(125, TASK_FOREVER, &motorDriveCallback);
void motorDriveCallback()
{

  if (motorDriveTask.isFirstIteration())
  {
    // setup motors, by arming them
    motorFL.arm();
    motorFR.arm();
    motorRL.arm();
    motorRR.arm();
  }
  // TODO this function will take into account the refined data output, and find the correct PID values for the motors
  // TODO this function will apply the PID values to the motors
}

#endif /* B010FF50_A56C_4662_A550_96F225830788 */
