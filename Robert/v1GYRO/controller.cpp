#include <mutex>

#include <Arduino.h>
#include <AccelStepper.h>
#include "controller.h"
#include <BMI160Gen.h>

controller::controller
(
  double iWheelRadius, double iTrackWidth,
  AccelStepper *iStepperL, AccelStepper *iStepperR,
  uint32_t iStepsPerRev, uint32_t iTurnInterval,
  uint32_t iIntervalIMUus, std::mutex *iSteppersEngaged_mtx,
  void (*iEngageSteppers)(void * parameter),
  TaskHandle_t *iEngageSteppersHandle
) 
{
  wheelRadius = iWheelRadius;
  trackWidth = iTrackWidth;
  stepperL = iStepperL;
  stepperR = iStepperR;
  stepsPerRev = iStepsPerRev;
  turnInterval = iTurnInterval;
  intervalIMUus = iIntervalIMUus;
  steppersEngaged_mtx = iSteppersEngaged_mtx;
  engageSteppers = iEngageSteppers;
  engageSteppersHandle = iEngageSteppersHandle;
}

void controller::init(double iTheta) {
  init();
  theta = iTheta;
  targetTheta = theta;
}

void controller::init() {
  //init steppers
  steppersEngaged_mtx->lock();
  stepperR->setPinsInverted(true);
  stepperL->setMinPulseWidth(2);
  stepperR->setMinPulseWidth(2);
  steppersEngaged_mtx->unlock();
  //init values
  maxVx = 0;
  maxAx = 0;
  maxAngVx = 0;
  oldus = micros();
  oldIMUus = micros();
  STATE = 0;
  theta = 0;
  targetTheta = 0;
}

void controller::update() {
  updateTheta();
  double deltaTheta = targetTheta - theta;
  switch (STATE) {
    case 0:
      break;
      
    case 1:
      //stepperL->run();
      //stepperR->run();
      if (steppersEngaged_mtx->try_lock()) {
        STATE = 0;
        steppersEngaged_mtx->unlock();
      }
      break;
      
    //deciding turn
    case 2:
      if (deltaTheta > PI) {
         deltaTheta -= TWO_PI;
      }
      else if (deltaTheta < -PI) {
        deltaTheta += TWO_PI;
      }
      //check if theta within desired range
      if ((abs(deltaTheta) > 0.000174) && steppersEngaged_mtx->try_lock()) {
        stepperL->move(mmToSteps(-0.5*trackWidth*deltaTheta));
        stepperR->move(mmToSteps(0.5*trackWidth*deltaTheta));
        steppersEngaged_mtx->unlock();
        xTaskCreate(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle);
        STATE = 3;
      }
      else {
        STATE = 0;
      }
      break;
        
    //actually turning
    case 3:
      if (steppersEngaged_mtx->try_lock()) {
          //Serial.println(stepperL->isRunning());
          //Serial.println(stepperR->isRunning());
          if (!stepperL->isRunning() && !stepperR->isRunning()) {
            STATE = 2;
          }
          //Serial.println("Done with movement");
          steppersEngaged_mtx->unlock();
      }
      break;
      
    default:
      STATE = 0;
      break;
  }
}

void controller::updateTheta() {
  if (micros() - oldIMUus > intervalIMUus) {
    double angVel = ((BMI160.getRotationZ() * 250.0) / 32768.0) * PI/180;
    double interval = micros() - oldIMUus;
    theta += angVel*(interval/pow(10, 6));
    while (theta > PI) {
      theta -= TWO_PI;
    }
    while (theta < -PI) {
      theta += TWO_PI;
    }
    oldIMUus = micros();
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
  steppersEngaged_mtx->lock();
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
  steppersEngaged_mtx->unlock();
  xTaskCreate(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle);
  STATE = 1;
}


void controller::setTheta(double newTheta) {
  //set accel and vel
  steppersEngaged_mtx->lock();
  stepperL->setAcceleration(mmToSteps(maxAx));
  stepperR->setAcceleration(mmToSteps(maxAx));
  stepperL->setMaxSpeed(mmToSteps(maxAngVx));
  stepperR->setMaxSpeed(mmToSteps(maxAngVx));
  stepperL->setSpeed(mmToSteps(maxAngVx));
  stepperR->setSpeed(mmToSteps(maxAngVx));
  //set wheel positions
  targetTheta = newTheta;
  STATE = 2;
  steppersEngaged_mtx->unlock();
  xTaskCreate(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle);
  oldus = micros();
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

double controller::getTheta() {
  return theta;
}

void controller::initTheta(double newTheta) {
  //theta = newTheta;
  return;
}

int controller::getState() {
  return STATE;
}
