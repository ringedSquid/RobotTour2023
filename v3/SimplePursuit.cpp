#include "SimplePursuit.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

SimplePursuit::SimplePursuit(Vector2d *iPath, byte iPathLength, 
                             Odometry *iOdometry, double iCheckrs) 
{
  path = iPath;
  pathLength = iPathLength;
  odometry = iOdometry;
  checkrs = iCheckrs;
}

double SimplePursuit::getDist(Vector2d p1, Vector2d p2) {
  return sqrt(pow((p2(0)-p1(0)), 2) + pow((p2(1)-p1(1)), 2));
}

void SimplePursuit::init() {
  prevPointIndex = 0;
  goalPointIndex = 1;
}

//selects next point in path
boolean SimplePursuit::nextPoint() {
  if (goalPointIndex + 1 >= pathLength) {
    return false;
  }
  prevPointIndex++;
  goalPointIndex++;
  return true;
}

boolean SimplePursuit::atPoint() {
  return (getDistToPoint() <= checkrs);
}

double SimplePursuit::getDistToGoalPoint() {
  return (getDist(odo.getPose(), getGoalPoint()));
}

Vector2d SimplePursuit::getGoalPoint() {
  return path[goalPointIndex];
}

double SimplePursuit::getVx() {
   
}

double SimplePursuit::getTheta() {
  return
}

class SimplePursuit {
  private:
    Vector2d *path;
    Odometry *odometry;

    byte pathLength;
    byte prevPointIndex;
    byte goalPointIndex; 

    double getDist(Vector2d p1, Vector2d p2);
    
  public:
    SimplePursuit(Vector2d *iPath, byte iPathLength, 
                  Odometry *iOdometry, double iCheckrs); 
    void init();
    
    boolean nextPoint();
    boolean atPoint();

    double getDistToGoalPoint();

    double getVx();
    double getTheta();

    Vector2d getGoalPoint();
}


#endif
