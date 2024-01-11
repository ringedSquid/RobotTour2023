#include "Robot.h"
#include "SimplePursuit.h"
#include "controller.h"
#include "odometry.h"

Robot::Robot( 
      SimplePursuit *iSimplePursuit,
      Controller *iController,
      Odometry *iOdometry,
      double iTarget_t,
      double iCenterToDowel,
      double iEndDistance,
      uint32_t iTurn_us
      )
{
  simplePursuit = iSimplePursuit;
  controller = iController;
  odometry = iOdometry;
  target_t = iTarget_t;
  centerToDowel = iCenterToDowel;
  endDistance = iEndDistance;
  turn_us = iTurn_us;
}

void Robot::init(Vector2d iPose, double iTheta) {
  controller->init();
  odometry->init(iPose, iTheta);
  simplePursuit->init();
  STATE = 0;
}

void Robot::update() {
  controller->update();
  double rTime;
  double rDist;
  switch (STATE) {
    case 0:
      break;
    case 1:
      //Calculate remaining distance, remainting time
      rTime = target_t - (micros() - start_us)/pow(10, 6);
      rDist = simplePursuit->getDistToGoalPoint() + simplePursuit->getPathDist();
      
      //Path following
      controller->setTargetVx(simplePursuit->getVx(rTime, rDist));
      Serial.printf("Spd: %f P: %f, %f, %f\n", simplePursuit->getVx(rTime, rDist), odometry->getX(), odometry->getY(), odometry->getTheta());
      controller->setTargetTheta(simplePursuit->getTheta());
      
      //Check if at next point
      if (simplePursuit->atPoint()) {
        //check if end of path
        if (simplePursuit->nextPoint()) {
          controller->setTargetVx(0);
           STATE = 2;
           buffer_us = micros();
        }
        else {
          STATE = 0;
        }
      }
      break;
    case 2:
      controller->setTargetTheta(simplePursuit->getTheta());
      if (micros() - buffer_us > turn_us) {
        STATE = 1;
      }
      break;
      
    default:
      STATE = 0;
      break;
   }
}

void Robot::startPath() {
  STATE = 1;
  start_us = micros();
  controller->enable();
}

void Robot::stopPath() {
  STATE = 0;
  controller->setTargetVx(0);
  controller->disable();
}

bool Robot::isNearTarget() {
  double dowelPointX = odometry->getX()+centerToDowel*cos(odometry->getTheta());
  double dowelPointY = odometry->getY()+centerToDowel*sin(odometry->getTheta());
  return (simplePursuit->getDist(Vector2d(dowelPointX, dowelPointY), simplePursuit->getEndPoint()) <= endDistance);
}
