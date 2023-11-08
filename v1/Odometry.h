#ifndef Odometry_h
#define Odometry_h

#include "DCMotor.h"
#include <ArduinoEigenDense.h>

using namespace Eigen;

class Odometry {
  private:
    //Hardware
    unsigned long tps; //ticks per rev
    double trackWidth, wheelRadius; //in mm
    DCMotor *motorL_p, *motorR_p;

    unsigned long oldTicksL, oldTicksR;

    //[x, y, theta]
    Vector3d robotState;

    

    double toRadPerSec; //ticks/microsecond to rad/s conversion factor
    

  public:
    Odometry(DCMotor *iMotorL_p, DCMotor *iMotorR_p, 
             unsigned long tps, 
             double trackWidth, wheelRadius);

    void setState(Vector3d newState);
    void setXY(double newX, double newY);
    void setX(double newX);
    void setY(double newY);
    void setThetha(double newTheta);

    Vector3d getState();
    double getX();
    double getY();
    double getTheta();

    void computeNewPos();
    void computeNewAngularVel();
    void computeNewState();
}







#endif
