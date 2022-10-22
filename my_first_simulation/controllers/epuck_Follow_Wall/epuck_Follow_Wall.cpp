// File:          epuck_Follow_Wall.cpp
// Date:
// Description:
// Author:       William Daniel Hiromoto
// Modifications:

#include <cstdio>
#define TIME_STEP 64
#define MAX_SPEED 6.28

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {

  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // initalize devices
  DistanceSensor *ps[8];
  char psNames[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

  // map devices
  for (int i = 0; i < 8; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    // time step can be gathered from the world or from the defined macro
    ps[i]->enable(TIME_STEP);
  }

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");

  // unlock the motors to control by velocity
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // std::cout << "Test" << std::endl;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    double psValues[8];
    for (int i = 0; i < 8; i++)
      psValues[i] = ps[i]->getValue();

    // detect obstacles
    // std::cout << psValues[1] << std::endl;
    // The higher the value of the distance sensor, the closer the robot is
    bool right_obstacle = psValues[1] > 80.0 || psValues[2] > 80.0;
    bool left_obstacle = psValues[5] > 80.0 || psValues[6] > 80.0;
    // see if there is a wall in front
    bool front_wall = psValues[7] > 80.0 || psValues[0] > 80.0;

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    double leftSpeed = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;

    if (left_obstacle) {
      // turn right
      leftSpeed = 0.5 * MAX_SPEED;
      rightSpeed = -0.5 * MAX_SPEED;
    } else if (front_wall) {
      // turn left
      leftSpeed = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
    }
    // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
