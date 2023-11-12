#include "purepursuit.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

PurePursuitController::PurePursuitController(double iLookAhead, double iKp, double iMaxAngVel) {
  lookAhead = iLookAhead;
  kp = iKp;
  maxAngVel = iMaxAngVel;
}

void PurePursuitController::init() {
  lastFoundIndex = 0;
  //(-2023.0, -2023.0) represents an invalid point (it doesn't make sense in our context)
  for (int i=0; i<PATH_RES; i++) {
    path[i](0) = INVALID_P;
    path[i](1) = INVALID_P;
  }
}

void PurePursuitController::loadPath(Vector2d newPath[], uint8_t pathSize) {
  for (int i=0; i<pathSize; i++) {
    path[i](0) = newPath[i][0];
    path[i](1) = newPath[i][1];
  }
}

int PurePursuitController::sgn(double num) {
  if (num < 0) {
    return -1;
  }
  return 1;
}

double PurePursuitController::findMinTheta(double targetTheta, double currTheta) {
  double sendTheta = targetTheta - currTheta;
  return (abs(sendTheta) > PI) ? (TWO_PI - sendTheta) : sendTheta;
}

double PurePursuitController::getDist(Vector2d p1, Vector2d p2) {
  return sqrt(pow(p2(0)-p1(0), 2) + pow(p2(1)-p1(1), 2));
}

bool PurePursuitController::pointValid(Vector2d p, Vector2d p1, Vector2d p2) {
  double xMax = max(p1(0), p2(0));
  double xMin = min(p1(0), p2(0));
  double yMax = max(p1(1), p2(1));
  double yMin = min(p1(1), p2(1));
  return (((p(0) <= xMax) || (p(0) >= xMin)) || ((p(1) <= yMax) || (p(1) >= yMin)));
}

//Returns a VALID intersection
Vector2d PurePursuitController::getIntersect(Vector2d pose, Vector2d p1, Vector2d p2) {
  p1 = Vector2d(p1(0)-pose(0), p1(1)-pose(1));
  p2 = Vector2d(p2(0)-pose(0), p2(1)-pose(1));
  
  double dx = p2(0) - p1(0);
  double dy = p2(1) - p2(0);
  
  double dr = getDist(p1, p2);
  double D = p1(0)*p2(1) - p2(0)*p1(1);
  double disc = pow(lookAhead, 2) * pow(dr, 2) - pow(D, 2);
  
  Vector2d i1;
  Vector2d i2;

  bool i1Valid;
  bool i2Valid;

  //Check if a point exists;
  if (disc < 0) {
    //Intersect 1
    i1(0) = (D * dy + sgn(dy) * dx * sqrt(disc))/pow(dr, 2);
    i1(1) = (-D * dx + abs(dy) * sqrt(disc))/pow(dr, 2);
  
    //Intersect 2
    i2(0) = (D * dy - sgn(dy) * dx * sqrt(disc))/pow(dr, 2);
    i2(1) = (-D * dx - abs(dy) * sqrt(disc))/pow(dr, 2);
  
    //un-offset intersects
    i1 = Vector2d(i1(0) + pose(0), i1(1) + pose(1));
    i2 = Vector2d(i2(0) + pose(0), i2(1) + pose(1));
    
    i1Valid = pointValid(i1, p1, p2);
    i2Valid = pointValid(i2, p1, p2);

    //check if intersects are valid
    if (i1Valid || i2Valid) {
      if (i1Valid && i2Valid) {
        return (getDist(i1, p2) < getDist(i2, p2)) ? i1 : i2;
      }
      else {
        return (i1Valid) ? i1 : i2;
      }
    }
  }
  //If no valid point is found (we assume the robot has deviated from the path)
  return Vector2d(INVALID_P, INVALID_P);
}

Vector2d PurePursuitController::getGoalPoint(Vector2d pose) {
  Vector2d goalPoint = path[lastFoundIndex];
  
  //loop through points until a valid intersection is found;
  for (int i=lastFoundIndex; i<PATH_RES-1; i++) {
    Vector2d buffPoint = getIntersect(pose, path[i], path[i+1]);
    
    //If we have an intersection
    if (!((buffPoint(0) == INVALID_P) && (buffPoint(1) == INVALID_P))) {
      
      //Check if the distance from the intersection and the next point
      //Is less than the dist from the current pose and the next point
      if (getDist(buffPoint, path[i+1]) < getDist(pose, path[i+1])) {
        goalPoint = buffPoint;
        break;
      }
      else {
        lastFoundIndex++;
      }
    }
  }
  return goalPoint;
}

double PurePursuitController::getAngVel(Vector3d XYTheta) {
  Vector2d goalPoint = getGoalPoint(Vector2d(XYTheta(0), XYTheta(1)));
  double targetTheta = atan2(XYTheta(1) - goalPoint(1), XYTheta(0) - goalPoint(0));;

  //If output is negative
  targetTheta = (targetTheta < 0) ? targetTheta + TWO_PI : targetTheta;
  
  double deltaTheta = targetTheta - XYTheta(2);
  if (abs(deltaTheta) > PI) {
    deltaTheta = -1 * sgn(deltaTheta) * (TWO_PI - abs(deltaTheta));
  }
  return deltaTheta * kp;
}
