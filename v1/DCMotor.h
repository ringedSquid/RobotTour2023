#ifndef DCMotor_h
#define DCMotor_h

#include <Arduino.h>
#include <PID_v1.h>

class DCMotor {
  private:
    volatile int ticks, oldticks;
    byte motorP1, motorP2, pchannel1, pchannel2;
    double currentRPM, targetRPM, motorOut, kp, ki, kd, oldt, updateInt;
    PID *rpmPID;
    
  public:
    byte encoderP1, encoderP2;
    
    DCMotor(byte c1, byte c2, byte m1, byte m2, 
            byte ipchannel1, byte ipchannel2,
            double ikp, double iki, double ikd,
            double updateInt);
    void init();
    void enable();
    void disable();
    void updateTicks();
    void motorUpdate();
    void setRPM(double rpm);
    double getRPM();
};

#endif
