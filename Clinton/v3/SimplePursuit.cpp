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
  pathDist = 0;
  for (int i=1; i<pathLength; i++) {
    pathDist += getDist(path[i-1], path[i]);    
  }
  //pathDist -= getDist(path[prevPointIndex], getGoalPoint());
}

//selects next point in path
boolean SimplePursuit::nextPoint() {
  if (goalPointIndex + 1 >= pathLength) {
    return false;
  }
  prevPointIndex++;
  goalPointIndex++;
  //For use of constant v avg
  //pathDist -= getDist(path[prevPointIndex], getGoalPoint());
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

Vector2d SimplePursuit::getEndPoint() {
  return path[pathLength-1];
}

int SimplePursuit::getPathIndexCount() {
  return pathLength;
}


double SimplePursuit::getVx(double rTime, double rDist) {
  /*
  double totalDist = getDist(path[prevPointIndex], getGoalPoint());
  double vAvg = rDist/rTime;
  double distFromPoint = getDist(odometry->getPose(), getGoalPoint());
  */
  return pathDist/rTime;
  //Serial.printf("RD: %f, vAvg: %f\n", rDist, vAvg);
  /*
  currentVx = vAvg*totalDist/(totalDist - trafDist);
  if ((totalDist - distFromPoint) < trafDist) {
    return (currentVx/trafDist)*(totalDist - distFromPoint);
  }
  else if (distFromPoint > (totalDist - trafDist)) {
    return currentVx - (currentVx/trafDist)*(trafDist - distFromPoint);
  }
  else {
    return currentVx;
  }
  */
  //return vAvg;
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
  return atan2(dy, dx);
}

double SimplePursuit::getPathDist() {
  return pathDist;
}

boolean SimplePursuit::atTerminal() {
  return (goalPointIndex >= pathLength-1);
}
