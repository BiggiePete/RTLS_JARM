#include <Arduino.h>
#include <TaskScheduler.h>
#include <Wire.h>
#include "states.h"

Scheduler runner;

State_e currentState = State_e::IDLE;

void setup()
{
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