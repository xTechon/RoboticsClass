// File:          epuck_blob_recognition.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <cstddef>
#include <iostream>
#include <ostream>
#include <vector>
#define TIME_STEP 64
#define MAX_SPEED 6.28

#include <cstdio>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();

  // Webots Camera
  Camera *cam = robot->getCamera("camera");
  cam->enable(TIME_STEP);

  // Add and enable color recognition camera
  Camera *colorCam = robot->getCamera("colorCamera");
  colorCam->recognitionEnable(TIME_STEP);
  colorCam->enableRecognitionSegmentation();
  colorCam->enable(TIME_STEP);

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");

  // unlock the motors to control by velocity
  std::cout << "UNLOCKING MOTORS" << std::endl;
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // wait for objects to appear in vision
  robot->step(TIME_STEP * 4);

  // Learn the data structures
  int numObj = colorCam->getRecognitionNumberOfObjects();
  auto objects = colorCam->getRecognitionObjects();
  std::cout << "Numb Obj = " << numObj << std::endl;
  // std::cout << colors.colors[0] << std::endl;
  //  colors array follows: red, green, blue; each value is a double btwn 0-1

  bool red = false;
  double leftSpeed = 0.5 * MAX_SPEED;
  double rightSpeed = 0.5 * MAX_SPEED;
  //   Main loop:
  //   - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    //  Read the sensors:
    objects = colorCam->getRecognitionObjects();
    numObj = colorCam->getRecognitionNumberOfObjects();
    if (numObj > 0) {
      for (int i = 0; i < numObj; i++) {
        if (objects[i].colors[0] == 1) {
          std::cout << "RED DETECTED, OBJ " << i << std::endl;
          red = true;
        } else {
          std::cout << "No Red on obj " << i << std::endl;
        }
      }
    } else {
      std::cout << "no OBJ in view" << std::endl;
    }
    if (red == true) {
      leftSpeed = 0.0;
      rightSpeed = 0.0;
      red = false;
    } else {
      leftSpeed = 1 * MAX_SPEED;
      rightSpeed = 0.7 * MAX_SPEED;
    }

    // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  };

  delete robot;
  return 0;
}
