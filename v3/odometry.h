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
    Vector2d pose;
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
    void init(Vector2d iPose, double iTheta);
    void update();
    void setPose(Vector2d newPose);
    void setX(double newX);
    void setY(double newY);
    void setTheta(double newTheta);
    void setXYTheta(Vector3d newXYTheta);

    double getLinVelx();
    double getAngVel();
    
    double getX();
    double getY();
    double getTheta();

    Vector2d getPose();
    Vector3d getXYTheta();

};

#endif
