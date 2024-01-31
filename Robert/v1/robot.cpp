#include "simplePursuit.h"
#include "controller.h"

robot::robot
( 
  simplePursuit *iSimplePursuit, controller *iController,
  double iMaxAx, double iMaxAngVx,
  double iCenterToDowel
)
{
  robotSimplePursuit = iSimplePursuit;
  robotController = iController;
  maxAx = iMaxAx;
  maxAngVx = iMaxAngVx;
  centerToDowel = iCenterToDowel;
}

void robot::init() {
  simplePursuit->init();
  controller->init();
  controller->setMaxAx(maxAx);
  controller->setMaxAngVx(maxAngVx);
  STATE = 0;
}

void robot::update() {
  switch (STATE) {
    case 0:
      
    case 1:
      double dist = robotSimplePursuit->getCurrentGoalPointDist();
      if (robotSimplePursuit->atLastPoint()) {
        dist -= centerToDowel;
      }
    case 2:
      
    default:
      STATE = 0;
  }
}

void robot::startPath() {
  STATE = 1;
}

void robot::stopPath() {
  STATE = 0;
}
#ifndef robot_h
#define robot_h

#include "simplePursuit.h"
#include "controller.h"

class robot {
  private:
    simplePursuit *simplePursuit;
    controller *controller; //robot controller
    
    //0 for idle
    //1 for turning
    //2 for path following
    byte STATE;
   
    uint32_t start_us;

    //getting distance from end point
    double centerToDowel;
    
  public:
    robot( 
      simplePursuit *iSimplePursuit,
      controller *iController,
      double iCenterToDowel,
      );
      
    void init();
    void update();
    void startPath();
    void stopPath();

};


#endif
