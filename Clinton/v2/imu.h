#ifndef imu_h
#define imu_h

#include "REG.h"
#include "IMUsdk.h"

#include <Filters.h>

#include <ArduinoEigenDense.h>
using namespace Eigen;

//Uses Serial2

class IMU {
  private:
    Vector3d orient;
    //Vector3d aVel;
    Vector3d pose;
    Vector3d linVel;

    Vector3d orientOffset;

    FilterOnePole *velFilterX;
    FilterOnePole *velFilterY;
    FilterOnePole *velFilterZ;
    
    FilterOnePole *poseFilterX;
    FilterOnePole *poseFilterY;
    FilterOnePole *poseFilterZ;

    uint32_t intervalus;
    uint32_t linAccOldus;
    uint32_t linVelOldus;

    Vector3d oldLinAcc;
    
    
  public:
    IMU(uint32_t iintervalus, 
    void init(Vector3d iPose, Vector3d iOrient);
    void update();
    void computelinVel();
    void computePose();

    //Calculated values
    Vector3d getPose();
    Vecotr3d getLinVel();
    Vector3d getXYTheta();
    
    //Measured values
    Vector3d getLinAcc();
    Vector3d getLinAccRaw();
    Vector3d getOrient();
    Vector3d getOrientRaw();

    void setOrientOffset();
    void setPose(Vector3d newPose);
    void setX(double newX);
    void setY(double newY);
    void setZ(double newZ);
    void setXYTheta(Vector3d newXYTheta);
    
}


#endif
