#ifndef odometry_h
#define odometry_h

#include "DCMotor.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

//Using XYZ coordinate system SCS 

class Odometry {
  private:
    DCMotor *motorL;
    DCMotor *motorR;

    //in mm
    double trackWidth; 
    double wheelRadius;

    //Calculated values
    Vector3d pose;
    double linVelx;
    double angVel;
    double theta;

    double oldLinVelx;
    double oldAngVel;

    uint32_t oldus;
    uint32_t intervalus;
    
  public:
    Odometry(DCMotor *iMotorL, DCMotor *iMotorR,
             double iTrackWidth, double iWheelRadius,
             uint32_t iintervalus);
    void init(Vector3d iPose, double iTheta);
    void update();
    void setPose(Vector3d newPose);
    void setX(double newX);
    void setY(double newY);
    void setZ(double newZ);
    void setXYTheta(Vector3d newXYTheta);

    double getLinVelx();
    double getAngVel();
    
    double getX();
    double getY();
    double getZ();

    Vector3d getPose();
    Vector3d getXYTheta();
    
};

#endif
