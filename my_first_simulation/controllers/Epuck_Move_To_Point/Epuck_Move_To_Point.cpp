// File:          Epuck_Move_To_Point.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// and/or to add some other includes
#include <stdio.h>
#include <stdlib.h>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Robot.hpp>

using namespace webots;

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
#define ROBOT_ANGULAR_SPEED_DEG = 141.851127971

int main(int argc, char** argv) {
  // create the Robot instance.
  Robot* robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int) robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
