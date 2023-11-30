#ifndef controller_h
#define controller_h

#include "DCMotor.h"

#include <PID_v1.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

class Controller {
  private:
    //int mm
    double wheelRadius;
    double trackWidth;
    
    double targetTheta;
    double targetVx;

    //PID out
    double targetAngVel;

    double maxAngVel;
    
    double thetaKp;
    double thetaKi;
    double thetaKd;

    uint32_t intervalus;
    uint32_t oldus;
    
    PID *thetaPID;
    Odometry *odometry;

    DCMotor *motorL;
    DCMotor *motorR;

    double computeRRPS();
    double computeLRPS();

    bool enabled;

  public:
    Controller(
      double iWheelRadius, double iTrackWidth,
      DCMotor *iMotorL, DCMotor *iMotorR,
      double iMaxAngeVel,
      Odometry *iOdometry,
      uint32_t iIntervalus,
      double iThetaKp, double iThetaKi, double iThetaKd
    );

    void init();
    void update();
    void enable();
    void disable();
     
    double setTargetTheta(double newTheta);

}
   
