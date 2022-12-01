// File:          Epuck_Move_To_Point.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// and/or to add some other includes
#include "Epuck_Move_To_Point.h"

#include "webots/Compass.hpp"

using namespace webots;

#define TIME_STEP 64

// 50% speed, therefore angularspeed will be 3.14 rad/s
// Tangential speed = angular speed * wheel radius
// Tangential speed = 3.14 rad/s * 2.05 cm = 0.06437 m/s
#define TANGENTIAL_SPEED 0.06437

// Spd of robot spinning in place in rotations per second
// axel length = distance between wheels == 0.052 m
// rotational speed = tangential speed / (pi * axel length)
// rotational speed = 0.06437m/s / (pi * 0.052m) = 0.3940309
#define ROBOT_ROTATIONAL_SPEED 0.394030911

// Speed of robot spinning in place in degrees per second
// angular speed in degrees = rotational speed * 360
// angular speed in degrees = 0.3940309 * 306
#define ROBOT_ROTATIONAL_SPEED_DEG = 141.851127971

#define DEGREE_ERROR            2
#define GPS_SAMPLING_PERIOD     1 // in ms
#define COMPASS_SAMPLING_PERIOD 1 // in ms

int main(int argc, char** argv) {
  // create the Robot instance.
  Robot* robot = new Robot();

  // get the time step of the current world.
  // int timeStep = (int) robot->getBasicTimeStep();

  // init sensors
  GPS* locator    = robot->getGPS("gps");
  Compass* needle = robot->getCompass("compass");

  locator->enable(GPS_SAMPLING_PERIOD);
  needle->enable(COMPASS_SAMPLING_PERIOD);

  const double* curCoord = nullptr;
  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    curCoord = locator->getValues();
    // std::printf("%f, %f", curCoord[0], curCoord[1]);
    std::cout << curCoord[0] << "," << curCoord[1] << std::endl;

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
