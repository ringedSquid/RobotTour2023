#ifndef robot_h
#define robot_h

#include "DCMotor.h"
#include "odometry.h"
#include "purepursuit.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

class Robot {
  private:
    DCMotor *motorL;
    DCMotor *motorR;
    Odometry *odo;
    PurePursuitController *ppc;

    //Fused with IMU and odo
    Vector2d pose;
    double theta;

    //Speed targets
    double targetVx;

    //bools
    bool nearTarget;
    void isNearTarget();
        
    Vector2d endPoint;

    double minEndDist;
    
  public:
    Robot(DCMotor *iMotorL, DCMotor *iMotorR,
          Odometry *iOdo, 
          PurePursuitController *ippc);

    void init();
    void init(Vector2d iPose, double iTheta, Vector2d path[], uint8_t path_size, double iMinEndDist);
    void start();
    void stop();
    void update();
    void followPath();
    void setTargetVx(double newVx);

    bool getNearTarget();
    
    double getTargetVx();
    
};


#endif
