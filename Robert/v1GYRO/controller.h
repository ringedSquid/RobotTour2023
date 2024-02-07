#ifndef controller_h
#define controller_h

#include <Arduino.h>
#include <AccelStepper.h>
#include <BMI160Gen.h>

class controller {
  private:
    //int mm
    double wheelRadius;
    double trackWidth;

    //All in mm/s
    double maxVx;
    double maxAx;

    //Rad/s
    double maxAngVx;
    
    uint32_t oldus;
    uint32_t oldIMUus;
    uint32_t intervalIMUus;
    
    volatile double theta;
    double targetTheta;
    
    AccelStepper *stepperL;
    AccelStepper *stepperR;
    uint32_t stepsPerRev;

    uint32_t turnInterval;

    //0 is idle
    //1 is moving
    int STATE;

  public:
    controller(
      double iWheelRadius, double iTrackWidth,
      AccelStepper *iStepperL, AccelStepper *iStepperR,
      uint32_t iStepsPerRev, uint32_t iTurnInterval,
      uint32_t iIntervalIMUussd
    );

    void init(double iTheta);
    void init();
    void update();

    void setMaxVx(double newVx);
    void setMaxAx(double newAx);
    void setMaxAngVx(double newAngVx);

    //Motion profiled
    void moveX(double dist);
    void setTheta(double newTheta);
    void updateTheta();
    void initTheta(double newTheta);

    double getMaxVx();
    double getMaxAx();
    double getMaxAngVx();

    double getTheta();
    double getTargetTheta();

    int getState();

    long mmToSteps(double mm);
    double stepsTomm(long steps);
};

#endif
