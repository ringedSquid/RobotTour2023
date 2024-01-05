#include "SimplePursuit.h"

#include "odometry.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

SimplePursuit::SimplePursuit(Vector2d *iPath, byte iPathLength, 
                             Odometry *iOdometry, double iCheckrs,
                             double iTrafDist) 
{
  path = iPath;
  pathLength = iPathLength;
  odometry = iOdometry;
  checkrs = iCheckrs;
  trafDist = iTrafDist;
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
  return (getDistToGoalPoint() <= checkrs);
}

double SimplePursuit::getDistToGoalPoint() {
  return (getDist(odometry->getPose(), getGoalPoint()));
}

Vector2d SimplePursuit::getGoalPoint() {
  return path[goalPointIndex];
}

double SimplePursuit::getVx(double rTime, double rDist) {
  double totalDist = getDist(path[prevPointIndex], path[goalPointIndex]);
  double vAvg = rDist/rTime;
  return vAvg*totalDist/(totalDist - trafDist);
}

double SimplePursuit::getTheta() {
  Vector2d pose = odometry->getPose();
  Vector2d point = getGoalPoint();
  double dx = point(0)-pose(0);
  double dy = point(1)-pose(1);
  if (dx == 0) {
    if (dy >= 0) {
      return PI/2; 
    }
    return -PI/2;
  }
  return atan2(dy/dx);
}
