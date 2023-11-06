#ifndef purepu
#define robot_h

#include <Arduino.h>
#include "DCMotor.h";
#include <PID_v1.h>

class Robot {
  private:
    
  public:
    DCMotor MotorL, MotorR;
    Robot()
    void init();
};

#endif
