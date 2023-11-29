#ifndef DCMotor_h
#define DCMotor_h

#include <Arduino.h>
#include <PID_v1.h>
#include <Filters.h>

class DCMotor {
  private:
    //Hardware
    uint8_t encoderP1;
    uint8_t encoderP2;
    uint8_t motorP1;
    uint8_t motorP2;

    uint16_t TPR;

    //Conversions
    double toRPS;
    double toRPM;

    //Difference vars
    long oldTicks;
    volatile long ticks;

    uint32_t oldus;
    uint32_t intervalus;

    //Speed control/PID
    PID *tpusPID;

    //Speed lowpass Filter
    FilterOnePole *lowPassTPus;
    
    double currentTPus;
    double targetTPus;

    double Kp;
    double Ki;
    double Kd;

    double filterFreq;
    
    double pidOut;

    double motorPWM;
    uint8_t PWMChannel1;
    uint8_t PWMChannel2;

    //Feedforward
    double F;

    double getFeedForward();

    //
    bool isEnabled;
    
  public:
    DCMotor(uint8_t iEncoderP1, uint8_t iEncoderP2, 
            uint8_t  iMotorP1, uint8_t iMotorP2,
            uint8_t iPWMChannel1, uint8_t iPWMChannel2,
            uint32_t intervalus, double iTPR,
            double iKp, double iKi, double iKd,
            double ifilterFreq,
            double iF);

    void init();
    void setTPus(double newTPus);
    void setRPS(double newRPS);
    void setRPM(double newRPM);
    void enable();
    void disable();
    void update();
    void tickEncoderA();
    void tickEncoderB();
    void computeTPus();
    
    double getTPus();
    double getRPS();
    double getRPM();


    long getTicks();
   
};


#endif
