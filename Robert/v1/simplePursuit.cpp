#include <Arduino.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "simplePursuit.h"

simplePursuit::simplePursuit
(
  Vector2d *iPath, uint8_t iPathSize,
  double iTargetTime, uint32_t iTurnInterval
)
{
  path = iPath;
  pathSize = iPathSize;
  targetTime = iTargetTime;
  turnInterval = iTurnInterval;
}

double simplePursuit::getDist(Vector2d p1, Vector2d p2) {
  return sqrt(pow((p2(0)-p1(0)), 2) + pow((p2(1)-p1(1)), 2));
}

void simplePursuit::init() {
  prevPointIndex = 0;
  currentGoalPointIndex = 1;

  //Calculate pathTotalDist and avgVx
  pathTotalDist = 0;
  avgVx = 0;
  for (uint8_t i=1; i<pathSize; i++) {
    pathTotalDist += getDist(path[i], path[i-1]);
  }
  avgVx = pathTotalDist/(targetTime - turnInterval/pow(10,6)*pathSize);
}

uint8_t simplePursuit::getPathIndexCount() {
  return currentGoalPointIndex;
}

void simplePursuit::nextPoint() {
  if (currentGoalPointIndex + 1 < pathSize) {
    prevPointIndex++;
    currentGoalPointIndex++;
  }
}

boolean simplePursuit::atLastPoint() {
  return (currentGoalPointIndex == pathSize-1);
}

double simplePursuit::getCurrentGoalPointDist() {
  return getDist(path[currentGoalPointIndex], path[prevPointIndex]);
}

double simplePursuit::getTheta() {
  return atan2(
    path[currentGoalPointIndex](1)-path[prevPointIndex](1), 
    path[currentGoalPointIndex](0)-path[prevPointIndex](0)
    );
}

double simplePursuit::getAvgVx() {
  return avgVx;
}
