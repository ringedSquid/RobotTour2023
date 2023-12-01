#include "controller.h"

#include <PID_v1.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

Controller::Controller(
    double iWheelRadius, double iTrackWidth,
    DCMotor *iMotorL, DCMotor *iMotorR,
    double iMaxAngVel,
    Odometry *iOdometry,
    uint32_t iIntervalus,
    double iThetaKp, double iThetaKi, double iThetaKd
    double iThetaF
    ) 
{
  wheelRadius = iWheelRadius;
  trackWidth = iTrackWidth;
  odometry = iOdometry;
  motorL = iMotorL;
  motorR = iMotorR;
  intervalus = iIntervalus;
  thetaKp = iThetaKp;
  thetaKi = iThetaKi;
  thetaKd = iThetaKd;
  thetaF = iThetaF
  maxAngVel = iMaxAngVel;

}
      
void Controller::init() {
  
  motorL->init();
  motorR->init();

  currentTheta = odometry->getTheta();
  
  targetVx = 0;
  targetAngVel = 0;
  targetTheta = 0;

  thetaPID->SetOutputLimits(-maxAngVel, maxAngVel);
  
  oldus = micros();
  
  disable();
  
}

void Controller::update() {
  motorL->update();
  motorR->update();
  odometry->update();
  thetaPID->Compute();
  currentTheta = odometry->getTheta(); 
  if (enabled) {
    motorL->setRPS(computeLRPS());
    motorR->setRPS(computeRRPS());
    Serial.printf("Current t %f, Target T %f, target omega %f\n", currentTheta, targetTheta, targetAngVel);
  } 
}
  
void Controller::enable() {
  motorL->enable();
  motorR->enable();
  thetaPID->SetMode(AUTOMATIC);
  enabled = true;
}

void Controller::disable() {
  motorL->disable();
  motorR->disable();
  thetaPID->SetMode(MANUAL);
  enabled = false;
  
}

void Controller::setTargetTheta(double newTheta) {
  targetTheta = newTheta;
}

double Controller::computeRRPS() {
  return (targetVx/wheelRadius)+(trackWidth/2)*(targetAngVel/wheelRadius);
}
double Controller::computeLRPS() {
  return (targetVx/wheelRadius)-(trackWidth/2)*(targetAngVel/wheelRadius);
}
