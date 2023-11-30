#include "controller.h"

#include <PID_v1.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

Controller::Controller(
    double iWheelRadius, double iTrackWidth,
    DCMotor *iMotorL, DCMotor *iMotorR,
    double iMaxAngVel,
    uint32_t iintervalus,
    double iThetaKp, double iThetaKi, double iThetaKd
    ) 
{
  wheelRadius = iWheelRadius;
  trackWidth = iTrackWidth;
  odometry = iOdometry;
  intervalus = iIntervalus;
  thetaKp = iThetaKp;
  thetaKi = iThetaKi;
  thetaKd = iThetaKd;
  maxAngVel = iMaxAngVel;
  thetaPID = new PID(&odometry->getTheta, &targetAngVel, &targetTheta,
                    thetaKp, thetaKi, thetaKd,
                    DIRECT);

}
      
void Controller::init() {
  disable();
  
  targetVx = 0;
  targetAngVel = 0;
  targetTheta = 0;

  thetaPID->SetOutputLimits(-maxAngVel, maxAngVel);
  
  oldus = micros();
}

void Controller::update() {
  
}
  
void Controller::enable() {
  motorL.enable();
  motorR.enable();
  thetaPID->SetMode(AUTOMATIC);
  enabled = true;
}

void Controller::disable() {
  motorL.disable();
  motorR.disable();
  thetaPID->SetMode(MANUAL);
  enabled = false;
  
}

double Controller::computeRRPS() {
  return (targetLinVelx/wheelRadius)-(trackWidth/2)*(targetAngVel/wheelRadius);
}
double Controller::computeLRPS() {
  return (targetVx/wheelRadius)+(trackWidth/2)*(targetAngVel/wheelRadius);
}
