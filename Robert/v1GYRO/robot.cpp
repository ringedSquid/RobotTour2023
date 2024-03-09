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
  finalOffsetY = 0;
  finalOffsetX = 0;
  pathMode = 0;
  STATE = 0;
}

void robot::init(double iFinalOffsetY, double iFinalOffsetX, int iPathMode) {
  init();
  finalOffsetY = iFinalOffsetY;
  finalOffsetX = iFinalOffsetX;
  pathMode = iPathMode;
}

void robot::update() {
  double dist;
  double theta;
  double deltaTheta;
  double vx;
  double l;
  switch (STATE) {
    case 0:
      break;

    //deciding forward movement 
    case 1:
      dist = robotSimplePursuit->getCurrentGoalPointDist();
      if (robotSimplePursuit->atLastPoint()) {
        //dist -= centerToDowel;
        l = finalOffsetY - sqrt(pow(centerToDowel, 2) - pow(finalOffsetX, 2));
        dist += l;
      }
      //vx = robotSimplePursuit->getAvgVx();
      vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros()-start_us));
      vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
      robotController->setMaxVx(vx);
      robotController->moveX(dist);
      STATE = 2;
      break;
    //actually moving
    case 2:
      if (robotController->getState() == 0) {
        STATE = 3;
      }
      robotController->update();
      break;

    //deciding turns
    case 3:
      if (robotSimplePursuit->atLastPoint()) {
          theta = robotSimplePursuit->getTheta();
          l = finalOffsetY + centerToDowel - sqrt(pow(centerToDowel, 2) - pow(finalOffsetX, 2));
          deltaTheta = theta + atan2(l, finalOffsetX);
          robotController->setTheta(deltaTheta);
          STATE = 6;
      
      }
      else {
        robotSimplePursuit->nextPoint();

        theta = robotSimplePursuit->getTheta()
        deltaTheta = robotSimplePursuit->theta - robotController->getTargetTheta();
        
        while (deltaTheta > PI) {
          deltaTheta -= TWO_PI;
        }
        while (deltaTheta < -PI) {
          deltaTheta += TWO_PI;
        }
  
        //correct heading first;
        if (((abs(deltaTheta) == PI) && (pathMode == 1)) && !(robotSimplePursuit->isAGate())) {
          robotController->setTheta(theta);
          STATE = 5;
        }
  
        else {
          theta = robotSimplePursuit->getTheta();
          robotController->setTheta(theta);
          STATE = 4;
        }
      }
      break;

    case 4:  
      if (robotController->getState() == 0) {
        STATE = 1;
      }
      robotController->update();
      break;

    case 5:
      //go backwards
      if (robotController->getState() == 0) {
        dist = robotSimplePursuit->getCurrentGoalPointDist();
        /*
        if (robotSimplePursuit->atLastPoint()) {
          dist -= centerToDowel;
          dist += finalOffset;
        }
        */
        //vx = robotSimplePursuit->getAvgVx();
        vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros() - start_us));
        vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
        robotController->setMaxVx(vx);
        robotController->moveX(-dist);
        STATE = 2;
      }
      robotController->update();
      break;

    //terminal turn
    case 6:
      if (robotController->getState() == 0) {
        STATE = 0;
      }
      robotController->update();
      
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
