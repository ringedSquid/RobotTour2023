#ifndef simplePursuit_h
#define simplePursuit_h

#include <Arduino.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

//Gets points and tells distances/required headings

class simplePursuit {
  private:
    Vector2d *path;

    uint8_t pathSize;
    uint8_t prevPointIndex;
    uint8_t currentGoalPointIndex;

    double centerToDowel; 
    double pathTotalDist; 
    double targetTime;

    //Calculated at init()
    double avgVx;
    //Time alloted for each turn, used to calculate avgVx

    //in us
    uint32_t turnInterval;
    double limitVx;
   
    double getDist(Vector2d p1, Vector2d p2);
    
  public:
    simplePursuit(double iLimitVx, double iCenterToDowel);
    
    void init(Vector2d *iPath, uint8_t iPathSize, double iTargetTime);

    //get index of path
    uint8_t getPathIndexCount();

    //True if not the end of the path, false if it is
    void nextPoint();
    boolean atLastPoint();

    //Distance needed to be traveled from point a to b
    double getCurrentGoalPointDist();
    //Requried heading to go from point a to b
    double getTheta();

    //Average speed needed to complete track on time
    double getAvgVx(uint32_t elapsedTime);
};

#endif
