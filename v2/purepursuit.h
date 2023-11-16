/*
 * Based off of this tutorial
 * https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit#line-circle-intersection-with-bounds
 */

#ifndef purepursuit_h
#define purepursuit_h

#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "CONFIG.h"

class PurePursuitController {
  private:
    Vector2d path[PATH_RES];
    int lastFoundIndex;
    
    double lookAhead;
    double kp;

    int sgn(double num);
    double findMinTheta(double targetTheta, double currTheta);  
      
    bool pointValid(Vector2d p, Vector2d p1, Vector2d p2);
    Vector2d getIntersect(Vector2d pose, Vector2d p1, Vector2d p2);
    Vector2d getGoalPoint(Vector2d pose);

    //rad/s
    double maxAngVel;

    //stuff
    double targetAngVel;

    //timing
    uint32_t oldus;
    uint32_t intervalus;
    
  public:
    PurePursuitController(double iLookAhead, double iKp, double iMaxAngVel, uint32_t iintervalus);

    void init();
    void loadPath(Vector2d newPath[], uint8_t pathSize);
    void update(Vector3d XYTheta);
    void computeAngVel(Vector3d XYTheta);

    double getDist(Vector2d p1, Vector2d p2);
    double getTargetAngVel();

    void setKp(double newKp);
};




#endif
