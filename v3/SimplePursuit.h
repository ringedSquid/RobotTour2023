#ifndef SimplePursuit_h
#define SimplePursuit_h

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

    double getDist(Vector2d p1, Vector2d p2);
    
  public:
    SimplePursuit(Vector2d *iPath, byte iPathLength, 
                  Odometry *iOdometry, double iCheckrs); 
    void init();
    
    boolean nextPoint();
    boolean atPoint();

    double getDistToPoint();

    double getVx();
    double getTheta();

    Vector2d getGoalPoint();
}


#endif
