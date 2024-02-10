#include "simplePursuit.h"
#include "controller.h"
#include "robot.h"

robot::robot
( 
  simplePursuit *iSimplePursuit, controller *iController,
  double iMaxAx, double iMaxAngAx, double iMaxAngVx,
  double iCenterToDowel
)
{
  robotSimplePursuit = iSimplePursuit;
  robotController = iController;
  maxAx = iMaxAx;
  maxAngAx = iMaxAngAx;
  maxAngVx = iMaxAngVx;
  centerToDowel = iCenterToDowel;
}

void robot::init() {
  robotController->init(PI/2);
  robotController->setMaxAx(maxAx);
  robotController->setMaxAngVx(maxAngVx);
  robotController->setMaxAngAx(maxAngAx);
  finalOffset = 0;
  pathMode = 0;
  STATE = 0;
}

void robot::init(double iFinalOffset, int iPathMode) {
  init();
  finalOffset = iFinalOffset;
  pathMode = iPathMode;
}

void robot::update() {
  double dist;
  double theta;
  double deltaTheta;
  double vx;
  switch (STATE) {
    case 0:
      break;
      
    case 1:
      dist = robotSimplePursuit->getCurrentGoalPointDist();
      if (robotSimplePursuit->atLastPoint()) {
        dist -= centerToDowel;
        dist += finalOffset;
      }
      //vx = robotSimplePursuit->getAvgVx();
      vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros()-start_us));
      vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
      robotController->setMaxVx(vx);
      robotController->moveX(dist);
      STATE = 2;
      break;
      
    case 2:
      if (robotController->getState() == 0) {
        if (robotSimplePursuit->atLastPoint()) {
          STATE = 0;
        }
        else {
          STATE = 3;
        }
      }
      robotController->update();
      break;
      
    case 3:
      robotSimplePursuit->nextPoint();
      theta = robotSimplePursuit->getTheta();
      deltaTheta = theta - robotController->getTargetTheta();
      
      while (deltaTheta > PI) {
        deltaTheta -= TWO_PI;
      }
      while (deltaTheta < -PI) {
        deltaTheta += TWO_PI;
      }
      
      
      if ((abs(deltaTheta) == PI) && (pathMode == 1)) {
        dist = robotSimplePursuit->getCurrentGoalPointDist();
        if (robotSimplePursuit->atLastPoint()) {
          dist -= centerToDowel;
          dist += finalOffset;
        }
        //vx = robotSimplePursuit->getAvgVx();
        vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros() - start_us));
        vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
        robotController->setMaxVx(vx);
        robotController->moveX(-dist);
        STATE = 2;
      }
      
      
      else {
        robotController->setTheta(theta);
        STATE = 4;
      }
      break;

    case 4:  
      if (robotController->getState() == 0) {
        STATE = 1;
      }
      robotController->update();
      break;
      
    default:
      STATE = 0;
      break;
  }
}

void robot::startPath() {
  STATE = 1;
  start_us = micros();
}

double robot::stopPath() {
  STATE = 0;
  return (micros() - start_us)/pow(10, 6);
}

uint8_t robot::getState() {
  return STATE;
}
