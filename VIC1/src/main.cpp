#include <Arduino.h>
#include <TaskScheduler.h>
#include <Wire.h>
#include "states.h"
#include "Tasks/getDataTask.h"
#include "Tasks/refineDataTask.h"
#include "Tasks/motorDrivesTask.h"
#include "pins.h"

Scheduler runner;

State_e currentState = State_e::IDLE;

void setup()
{
  Serial.setTx(TX_PIN);
  Serial.setRx(RX_PIN);
  Serial.begin(115200);
  Serial.println("Hello World!");
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();

  runner.init();
  runner.addTask(getDataTask);
  runner.addTask(refineDataTask);
  runner.addTask(motorDriveTask);

  runner.enableAll();

  // TODO setup all necessary systems
}

void loop()
{
  runner.execute(); // this will run all tasks that are ready to run
  switch (currentState)
  {
  case State_e::IDLE:
    // TODO handle IDLE state
    break;
  case State_e::UP:
    // TODO handle UP state
    break;
  case State_e::CORRECTION:
    // TODO handle CORRECTION state
    break;
  case State_e::DESCENT:
    // TODO handle DESCENT state
    break;
  case State_e::LANDED:
    // TODO handle LANDED state
    break;
  case State_e::FAILED:
    // TODO handle FAILED state
    break;
  }

  // TODO use tasking system to run all necessary tasks
}