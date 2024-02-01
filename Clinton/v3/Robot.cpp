#include "Robot.h"
#include "SimplePursuit.h"
#include "controller.h"
#include "odometry.h"
//#include "imu.h"

Robot::Robot( 
      SimplePursuit *iSimplePursuit,
      Controller *iController,
      Odometry *iOdometry,
      //IMU *iimu,
      double iTarget_t,
      double iCenterToDowel,
      double iEndDistance,
      uint32_t iTurn_us
      )
{
  simplePursuit = iSimplePursuit;
  controller = iController;
  odometry = iOdometry;
  //imu = iimu;
  target_t = iTarget_t;
  centerToDowel = iCenterToDowel;
  endDistance = iEndDistance;
  turn_us = iTurn_us;
}

void Robot::init(Vector2d iPose, double iTheta) {
  controller->init();
  odometry->init(iPose, iTheta);
  simplePursuit->init();
  totalTurnDelay = (simplePursuit->getPathIndexCount()-1)*((turn_us/pow(10, 6)) + 0.5);
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
      //Use avg time over entire track for now
      rTime = target_t - totalTurnDelay; //target_t - (micros() - start_us)/pow(10, 6);
      rDist = simplePursuit->getDistToGoalPoint() + simplePursuit->getPathDist();
      //Serial.printf("Target %f, Elapsed %f, rTime %f, rDist %f\n", target_t, (micros()-start_us)/pow(10, 6), rTime, rDist);
      
      //Path following
      controller->setTargetVx(simplePursuit->getVx(rTime, rDist));
      //Serial.printf("Vx: %f Vrx: %f 350\n", simplePursuit->getVx(rTime, rDist), odometry->getLinVelx());
      controller->setTargetTheta(simplePursuit->getTheta());
      
      //Check if at next point
      if (simplePursuit->atPoint()) {
        controller->setTargetVx(0);
        buffer_us = micros();
        double buffer_speed = simplePursuit->getVx(rTime, rDist);
        while (micros() - buffer_us < 500*pow(10, 3)) {
            controller->update();
        }
        //check if end of path
        if (simplePursuit->nextPoint()) {
           STATE = 2;
           buffer_us = micros();
           controller->resetMotorIntegral();
           controller->setTargetTheta(simplePursuit->getTheta());
        }
        else {
          STATE = 0;
        }
      }
      break;
    case 2:
      //controller->setTargetTheta(simplePursuit->getTheta());
      controller->updateTargetTheta(simplePursuit->getTheta());
      if (micros() - buffer_us > turn_us) {
        controller->resetMotorIntegral();
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
  controller->setTargetTheta(odometry->getTheta());
  controller->setTargetVx(0);
  controller->enable();
}

void Robot::stopPath() {
  STATE = 0;
  controller->resetMotorIntegral();
  controller->setTargetVx(0);
  while (micros() - buffer_us < 500*pow(10, 3)) {
            controller->update();
  }
  controller->disable();
}

bool Robot::isNearTarget() {
  double dowelPointX = odometry->getX()+centerToDowel*cos(odometry->getTheta());
  double dowelPointY = odometry->getY()+centerToDowel*sin(odometry->getTheta());
  return (simplePursuit->atTerminal() && (simplePursuit->getDist(Vector2d(dowelPointX, dowelPointY), simplePursuit->getEndPoint()) <= endDistance));
}
