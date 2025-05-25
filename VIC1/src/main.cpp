#include <Arduino.h>

// TODO set up scheduling system for getting sensor readings
// TODO set up kalman filtering system to filter readings, and provide better idea of state in terms of orientation and altitude
// TODO set up PID controllers for the 4 motors
// TODO set up scheduling system to update the "physics" of the rocket at either a set rate, or when data is availiable
// TODO get temperature data
// TODO get humidity data
// TODO get GPS data
// TODO get IMU data
// TODO get Magnetometer data
// TODO determine rotational offset of Magnetometer on the board in reference to the north direction
// TODO set control flow for rocket in mode, IDLE, UP, CORRECTION, DESCENT
// TODO in IDLE mode, listen for rocket to be above 500ft & take GPS readings at high precision when the button is pressed
// TODO in UP mode, wait until altitude has descended over 3 datapoints before switching to CORRECTION
// TODO in CORRECTION mode, immediately get IMU readings at a good pace, and make all corrections necessary until the drone is within 10% of level flight, and within 10deg of facing north, then switch to DESCENT mode
// TODO in DESCENT mode, use all measurment data to determine the PID levels necessary to control the rocket movements on the way down
// TODO if in DESCENT mode & below Xft, and the distance to the landing site is > Xft + OFFSETft, deploy parachute, as the mission has become a failure
// TODO if in CORRECTION mode, and unable to make correction in under 15s, fire parachute method, as mission is a failure

void setup()
{
  // TODO setup all necessary systems
}

void loop()
{
  // TODO use tasking system to run all necessary tasks
}