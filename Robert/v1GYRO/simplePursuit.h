#ifndef simplePursuit_h
#define simplePursuit_h

#include <Arduino.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

//Gets points and tells distances/required headings

class simplePursuit {
  private:
    double centerToDowel;
    double finalOffset;
    
    Vector2d *path;

    uint8_t pathSize;
    uint8_t prevPointIndex;
    uint8_t currentGoalPointIndex;

    double pathTotalDist; 
    double targetTime;

    //Calculated at init()
    double avgVx;
    //Time alloted for each turn, used to calculate avgVx

    //in us
    uint32_t turnInterval;
   
    double getDist(Vector2d p1, Vector2d p2);
    
  public:
    simplePursuit(double iCenterToDowel, uint32_t iTurnInterval);
    
    void init(Vector2d *iPath, uint8_t iPathSize, double iTargetTime, double iFinalOffset);

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
    double getAvgVx();
};

#endif
