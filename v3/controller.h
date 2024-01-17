#ifndef controller_h
#define controller_h

#include "DCMotor.h"
#include "odometry.h"
#include "PID_Ballsack.h"
#include <Filters.h>

#include <ArduinoEigenDense.h>
using namespace Eigen;

class Controller {
  private:
    //int mm
    double wheelRadius;
    double trackWidth;
    
    double absTargetTheta;
    double relTargetTheta;
    double targetVx;

    double currentTheta;

    //PID_Ballsack out
    PID_Ballsack *thetaPID;
    
    double targetAngVel;
    double maxAngVel;
    
    double thetaKp;
    double thetaKi;
    double thetaKd;

    double thetaF;

    uint32_t intervalus;
    uint32_t oldus;

    double oldError;
    
    Odometry *odometry;

    DCMotor *motorL;
    DCMotor *motorR;

    double computeRRPS();
    double computeLRPS();

    bool enabled;
    FilterOnePole *lowPassVx;

  public:
    Controller(
      double iWheelRadius, double iTrackWidth,
      DCMotor *iMotorL, DCMotor *iMotorR,
      double iMaxAngeVel,
      Odometry *iOdometry,
      uint32_t iIntervalus,
      double iThetaKp, double iThetaKi, double iThetaKd,
      double iThetaF
    );

    void init();
    void update();
    void enable();
    void disable();

    void setTargetVx(double newVx);
    void setTargetTheta(double newTheta);
    void resetMotorIntegral();
    void updateTargetTheta(double newTheta);

};

#endif  
