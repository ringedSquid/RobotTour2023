#ifndef LebronJames_h
#define LebronJames_h

#include "odometry.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

//CAUSE HE FINDS DA POINTS!
//Basically just finds the next point in the path
class LebronJames {
  private:
    Odometry *odo;
    
    double lookAhead;
    double poseDeviation;

    double sgn(double x);
    double getDist(Vector3d p1, Vector3d p2);
  public:
    LebronJames(Odometry *odo, double iLookAhead, double iPoseDeviation);
    //In format (x, y, theta)
    Vector3d findPoint(Vector3d p1, Vector3d p2);
    
    bool atGoal(Vector3d p1);
};

#endif
