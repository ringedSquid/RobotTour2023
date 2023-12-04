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
    double iThetaKp, double iThetaKi, double iThetaKd,
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
  thetaF = iThetaF;
  thetaPID = new PID(&currentTheta, &targetAngVel, &targetTheta,
                    thetaKp, thetaKi, thetaKd,
                    DIRECT);
  maxAngVel = iMaxAngVel;

}
      
void Controller::init() {
  
  thetaPID->SetOutputLimits(-maxAngVel, maxAngVel);
  
  motorL->init();
  motorR->init();
  
  targetVx = 0;
  targetAngVel = 0;
  targetTheta = 0;

  oldus = micros();
  
  disable();
  
}

void Controller::update() {
  motorL->update();
  motorR->update();
  odometry->update();
  currentTheta = odometry->getTheta();
  thetaPID->Compute();
  if (enabled) {
    motorL->setRPS(computeLRPS());
    motorR->setRPS(computeRRPS());
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

void Controller::setTargetVx(double newVx) {
  targetVx = newVx;
}

double Controller::computeRRPS() {
  return (targetVx/wheelRadius)+(trackWidth/2)*(targetAngVel/wheelRadius);
}
double Controller::computeLRPS() {
  return (targetVx/wheelRadius)-(trackWidth/2)*(targetAngVel/wheelRadius);
}
