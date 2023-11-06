#ifndef robot_h
#define robot_h

#include <Arduino.h>
#include <PID_v1.h>
#include "DCMotor.h"
#include "kinematics.h"

class Robot {
  private:
    float wheelDia, trackWidth; // in m
    int maxRPM;
    double posInterval; // in ms
    Kinematics *robotKModel; 
    
  public:
    DCMotor *MotorL, *MotorR;
    Robot(DCMotor *iMotorL, DCMotor *iMotorR, 
          float iWheelDia, float iTrackWidth, int iMaxRPM
          );
    double robotX, robotY, robotHeading; // in m, radians
    double robotVX, robotVH; // in m/s
    
    void init();
    //void setHeading(double angle);
    void setSpeed(double speed);
    void setAVel(double AVel);
    void stateUpdate();
    
};

#endif 
