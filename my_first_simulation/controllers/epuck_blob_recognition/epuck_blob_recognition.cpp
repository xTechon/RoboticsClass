// File:          epuck_blob_recognition.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <iostream>
#define TIME_STEP 64
#define MAX_SPEED 6.28

#include <cstdio>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Webots Camera
  Camera *cam = robot->getCamera("camera");
  cam->enable(TIME_STEP);
  // Add and enable color recognition camera
  Camera *colorCam = robot->getCamera("colorCamera");
  colorCam->recognitionEnable(TIME_STEP);
  colorCam->enableRecognitionSegmentation();
  colorCam->enable(TIME_STEP);
  // std::cout << colorCam->getName() << std::endl;

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
  webots::CameraRecognitionObject colors = colorCam->getRecognitionObjects()[0];
  std::cout << colors.colors[0] << std::endl;
  // colors array follows: red, green, blue; each value is a double btwn 0-1
  //   Main loop:
  //   - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    //  Read the sensors:
    //  Enter here functions to read sensor data, like:
    //   double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
