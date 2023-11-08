#ifndef DCMotor_h
#define DCMotor_h

#include <Arduino.h>
#include <PID_v1.h>

class DCMotor {
  private:
    byte motorP1, motorP2;
    byte pchannel1, pchannel2;
    volatile long ticks;
    long oldticks;
    unsigned long oldt;
    long tpr;
    double currentAv;
    double targetAv;
    double updateInt;
    double motorOut, kp, ki, kd;
    PID *rpmPID;
    double toRadPerSec;
    
  public:
    byte encoderP1, encoderP2;
    DCMotor(byte c1, byte c2, byte m1, byte m2, 
            byte ipchannel1, byte ipchannel2,
            double ikp, double iki, double ikd,
            double updateInt, unsigned long iTpr);
    void init();
    void enable();
    void disable();
    void updateTicks();
    void motorUpdate();
    void computeAv();
    void setTPMi(double tpmi);//Ticks per micro
    void setRPS(double rps); //rad per sec
    void setRPM(double rpm);
    double getTPMi();
    double getRPS();
    double getRPM();
    long getTicks();
    
    
};

#endif
