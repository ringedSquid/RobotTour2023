/*
 * Based off of this tutorial
 * https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit#line-circle-intersection-with-bounds
 */

#ifndef purepursuit_h
#define purepursuit_h

#define PATH_RES 100
#define INVALID_P -2023.0
#define FAKE_INF 69420.0

#include <ArduinoEigenDense.h>
using namespace Eigen;

class PurePursuitController {
  private:
    Vector2d path[PATH_RES];
    int lastFoundIndex;
    
    double lookAhead;
    double kp;

    int sgn(double num);
    double findMinTheta(double targetTheta, double currTheta);  
    double getDist(Vector2d p1, Vector2d p2);
      
    bool pointValid(Vector2d p, Vector2d p1, Vector2d p2);
    Vector2d getIntersect(Vector2d pose, Vector2d p1, Vector2d p2);
    Vector2d getGoalPoint(Vector2d pose);

    //rad/s
    double maxAngVel;
    
  public:
    PurePursuitController(double iLookAhead, double iKp, double iMaxAngVel);

    void init();
    void loadPath(Vector2d newPath[], uint8_t pathSize);
    double getAngVel(Vector3d XYTheta);
};




#endif
