#ifndef SimplePursuit_h
#define SimplePursuit_h

#include "odometry.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

//Finds angle for robot to point int
//Finds speed robot needs to go to
// -> Motion profiling

class SimplePursuit {
  private:
    Vector2d *path;
    Odometry *odometry;

    byte pathLength;
    byte prevPointIndex;
    byte goalPointIndex;

    //Radius to check if on point
    double checkrs;
    //Distance before  acc/decell
    double trafDist;

    double getDist(Vector2d p1, Vector2d p2);

    //TrafDist
    //distace from the "traffic light" before you start slowing down/speeding up
  public:
    SimplePursuit(Vector2d *iPath, byte iPathLength, 
                  Odometry *iOdometry, double iCheckrs,
                  double iTrafDist); 
    void init();
    
    boolean nextPoint();
    boolean atPoint();

    double getDistToGoalPoint();

    double getVx(double rTime, double rDist);
    double getTheta();

    Vector2d getGoalPoint();
};


#endif
