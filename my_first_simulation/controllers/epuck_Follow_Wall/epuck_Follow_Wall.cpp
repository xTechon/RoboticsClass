// File:          epuck_Follow_Wall.cpp
// Date:
// Description:
// Author:       William Daniel Hiromoto
// Modifications:

#include <cstdio>
#define TIME_STEP 64
#define MAX_SPEED 6.28

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Pen.hpp>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {

  std::cout << "STARTING PROGRAM" << std::endl;
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // initalize devices
  std::cout << "INIT DEVICES" << std::endl;
  DistanceSensor *ps[8];
  char psNames[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

  // map devices
  for (int i = 0; i < 8; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    // time step can be gathered from the world or from the defined macro
    ps[i]->enable(TIME_STEP);
  }

  // Webots pen
  // Pen *pencil = robot->getPen("pencil");
  // pencil->write(true);

  // Webots Camera
  Camera *cam = robot->getCamera("camera");
  cam->enable(TIME_STEP);

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");

  // unlock the motors to control by velocity
  std::cout << "UNLOCKING MOTORS" << std::endl;
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  bool right_distance = false;
  bool left_distance = false;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  std::cout << "STARTING RUN";
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    double psValues[8];
    for (int i = 0; i < 8; i++)
      psValues[i] = ps[i]->getValue();

    // detect obstacles
    // std::cout << psValues[1] << std::endl;
    // The higher the value of the distance sensor, the closer the robot is
    right_distance =
        (psValues[1] < 60.0 || psValues[2] < 60.0) && !left_distance;
    left_distance =
        (psValues[1] > 80.0 || psValues[2] > 80.0) && !right_distance;
    // see if there is a wall in front
    bool front_wall = psValues[7] > 80.0 || psValues[0] > 80.0;

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    double leftSpeed = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;

    if (front_wall) {
      // turn left
      leftSpeed = -0.8 * MAX_SPEED;
      rightSpeed = 0.8 * MAX_SPEED;
      std::cout << "TURNNING LEFT" << std::endl;
    } else if (right_distance) {
      // adjust toward wall
      leftSpeed = 1 * MAX_SPEED;
      rightSpeed = 0 * MAX_SPEED;
      right_distance = false;
      std::cout << "TURNING TOWARD WALL" << std::endl;
    } else if (left_distance) {
      leftSpeed = 0.3 * MAX_SPEED;
      rightSpeed = 0.6 * MAX_SPEED;
      left_distance = false;
      std::cout << "TURNING AWAY FROM WALL" << std::endl;
    } else {
      std::cout << "GO STRAIGHT" << std::endl;
    }
    // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
