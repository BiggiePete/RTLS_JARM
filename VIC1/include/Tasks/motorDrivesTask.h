#ifndef B010FF50_A56C_4662_A550_96F225830788
#define B010FF50_A56C_4662_A550_96F225830788
#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include "refineDataTask.h"
#include "ESC.h"
#include "pins.h"
#include "states.h"

extern State_e currentState; // get state from main.cpp

typedef struct motorDriveData_t
{
  float motorFL_PERCENT; // 0-1 of drive factor
  float motorFR_PERCENT;
  float motorBL_PERCENT;
  float motorBR_PERCENT;

  float motorFL_PWM; // SPEED_MIN-SPEED_MAX
  float motorFR_PWM;
  float motorBL_PWM;
  float motorBR_PWM;
} motorDriveData_t;

volatile motorDriveData_t motorDriveData;

void motorDriveCallback();

#define SPEED_MIN (1000) // TODO Set the Minimum Speed in microseconds
#define SPEED_MAX (2000) // TODO Set the Minimum Speed in microseconds

ESC motorFL(MOTOR_FL_PIN, SPEED_MIN, SPEED_MAX, SPEED_MIN);
ESC motorFR(MOTOR_FR_PIN, SPEED_MIN, SPEED_MAX, SPEED_MIN);
ESC motorBL(MOTOR_BL_PIN, SPEED_MIN, SPEED_MAX, SPEED_MIN);
ESC motorBR(MOTOR_BR_PIN, SPEED_MIN, SPEED_MAX, SPEED_MIN);

Task motorDriveTask(125, TASK_FOREVER, &motorDriveCallback);
void motorDriveCallback()
{

  if (motorDriveTask.isFirstIteration())
  {
    // setup motors, by arming them
    motorFL.arm();
    motorFR.arm();
    motorBL.arm();
    motorBR.arm();
  }
  // TODO this function will take into account the refined data output, and find the correct PID values for the motors
  // TODO this function will apply the PID values to the motors
}

#endif /* B010FF50_A56C_4662_A550_96F225830788 */
