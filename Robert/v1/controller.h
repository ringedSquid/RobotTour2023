#ifndef controller_h
#define controller_h

#include <Arduino.h>
#include <AccelStepper.h>


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
    
    AccelStepper *stepperL;
    AccelStepper *stepperR;
    uint32_t stepsPerRev;

    //Converts a step to mm traveled
    long mmToSteps(double mm);
    double stepsTomm(long steps);

    //0 is idle
    //1 is moving
    int STATE;

  public:
    controller(
      double iWheelRadius, double iTrackWidth,
      AccelStepper *iStepperL, AccelStepper *iStepperR,
      uint32_t iStepsPerRev
    );

    void init();
    void update();

    void setMaxVx(double newVx);
    void setMaxAx(double newAx);
    void setMaxAngVx(double newAngVx);

    //Motion profiled
    void moveX(double dist);
    void turn(double theta);

    double getMaxVx();
    double getMaxAx();
    double getMaxAngVx();
    
};

#endif
