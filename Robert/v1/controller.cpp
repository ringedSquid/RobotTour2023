#include <Arduino.h>
#include <AccelStepper.h>
#include "controller.h"

controller::controller
(
  double iWheelRadius, double iTrackWidth,
  AccelStepper *iStepperL, AccelStepper *iStepperR,
  uint32_t iStepsPerRev
) 
{
  wheelRadius = iWheelRadius;
  trackWidth = iTrackWidth;
  stepperL = iStepperL;
  stepperR = iStepperR;
  stepsPerRev = iStepsPerRev;
}

void controller::init() {
  maxVx = 0;
  maxAx = 0;
  maxAngVx = 0;
  oldus = micros();
  STATE = 0;
}

void controller::update() {
  switch (STATE) {
    case 0:
      break;
    case 1:
      stepperL->run();
      stepperR->run();
      if (!(stepperL->isRunning()) && !(stepperR->isRunning())) {
        STATE = 0;
      }
      break;
    default:
      STATE = 0;
      break;
  }
}

long controller::mmToSteps(double mm) {
  return (long)((mm/(TWO_PI*wheelRadius)) * stepsPerRev);
}

double controller::stepsTomm(long steps) {
  return (steps/stepsPerRev)*TWO_PI*wheelRadius;
}

void controller::setMaxVx(double newVx) {
  maxVx = newVx;
}

void controller::setMaxAx(double newAx) {
  maxAx = newAx;
}

void controller::setMaxAngVx(double newAngVx) {
  maxAngVx = newAngVx;
}

void controller::moveX(double dist) {
  //set accel and vel
  stepperL->setAcceleration(mmToSteps(maxAx));
  stepperR->setAcceleration(mmToSteps(maxAx));
  stepperL->setMaxSpeed(mmToSteps(maxVx));
  stepperR->setMaxSpeed(mmToSteps(maxVx));
  stepperL->setSpeed(mmToSteps(maxVx));
  stepperR->setSpeed(mmToSteps(maxVx));
  //set wheel positions
  stepperL->move(mmToSteps(dist));
  stepperR->move(mmToSteps(dist));
  STATE = 1;
}


void controller::turn(double theta) {
  //set accel and vel
  stepperL->setAcceleration(mmToSteps(maxAx));
  stepperR->setAcceleration(mmToSteps(maxAx));
  stepperL->setMaxSpeed(mmToSteps(maxAngVx));
  stepperR->setMaxSpeed(mmToSteps(maxAngVx));
  stepperL->setSpeed(mmToSteps(maxAngVx));
  stepperR->setSpeed(mmToSteps(maxAngVx));
  //set wheel positions
  stepperL->move(mmToSteps(0.5*trackWidth*theta));
  stepperR->move(mmToSteps(-0.5*trackWidth*theta));
  STATE = 1;
}

double controller::getMaxVx() {
  return maxVx;
}

double controller::getMaxAx() {
  return maxAx;
}

double controller::getMaxAngVx() {
  return maxAngVx;
}

int controller::getState() {
  return STATE;
}
