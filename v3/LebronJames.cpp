#include "odometry.h"
#include "LebronJames.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

LebronJames::LebronJames(Odometry *iOdo, double iLookAhead, double iPoseDeviation) {
  odo = iOdo;
  lookAhead = iLookAhead;
  poseDeviation = iPoseDeviation;
}

Vector3d LebronJames::findPoint(Vector3d p1, Vector3d p2) {
  double dx = p2(0) - p1(0);
  double dy = p2(1) - p1(1);
  double dr = sqrt(dx*dx + dy*dy);
  double D = p1(0)*p2(1)-p2(0)*p1(1);
  
  double disc = lookAhead*lookAhead*dr*dr - D*D;

  if (D < 0) {
    return p2;
  }

  double x1 = (D*dy + sgn(dy)*dx*sqrt(D))/(dr*dr);
  double x2 = (D*dy - sgn(dy)*dx*sqrt(D))/(dr*dr);
  double y1 = (-D*dx + abs(dy)*sqrt(D))/(dr*dr);
  double y2 = (-D*dx - abs(dy)*sqrt(D))/(dr*dr);

  if (D == 0) {
    return Vector3d(x1, y2, p1(2));
  }

  double dist1 = getDist(Vector3d(x1, y1, 0), p2);
  double dist2 = getDist(Vector3d(x2, y2, 0), p2);

  if (dist1 < dist2) {
    return Vector3d(x1, y1, p2(2));
  }

  return Vector3d(x2, y2, p2(2));
}

double LebronJames::sgn(double x) {
  return (x < 0) ? -1 : 1;
}

double LebronJames::getDist(Vector3d p1, Vector3d p2) {
  return sqrt(pow(p2(0)-p1(0), 2) + pow(p2(1)-p1(1), 2));
}
    
bool LebronJames::atGoal(Vector3d p1) {
  return (getDist(p1, odo->getXYTheta()) <= poseDeviation);
}
