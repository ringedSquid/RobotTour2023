#include <Arduino.h>
#include <PID_v1.h>
#include "robot.h"

Robot::Robot(DCMotor *iMotorL, DCMotor *iMotorR, 
             float iWheelDia, float iTrackWidth, int iMaxRPM) {
  MotorL = iMotorL;
  MotorR = iMotorR;
  wheelDia = iWheelDia;
  trackWidth = iTrackWidth;
  maxRPM = iMaxRPM;
  robotKModel = new Kinematics(maxRPM, wheelDia, 0, trackWidth, 16); 
  
  
  
              
}
