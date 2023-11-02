#ifndef DCMotor_h
#define DCMotor_h

#include <Arduino.h>
#include <PID_v1.h>

class DCMotor {
  private:
    int ticks, oldticks;
    byte encoderP1, encoderP2, motorP1, motorP2;
    double targetRPM, currentRPM, motorOut, kp, ki, kd, oldt, updateInt;
    PID *rpmPID;
    static void updateTicks();
    
  public:
    DCMotor(byte c1, byte c2, byte m1, byte m2, 
            double ikp, double iki, double ikd,
            double updateInt);
    void init();
    void enable();
    void disable();
    void enableEncoder();
    void disableEncoder();
    void motorUpdate();
    void setRPM(double rpm);
    double getRPM();
};


#endif
