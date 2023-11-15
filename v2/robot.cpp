#include "robot.h"
#include "DCMotor.h"
#include "odometry.h"
#include "purepursuit.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

Robot::Robot(DCMotor *iMotorL, DCMotor *iMotorR,
          Odometry *iOdo, 
          PurePursuitController *ippc)
{
  motorL = iMotorL;
  motorR = iMotorR;
  odo = iOdo;
  ppc = ippc;
}

void Robot::init() {
  motorL->init();
  motorR->init();
  ppc->init();
  endPoint = Vector2d(INVALID_P, INVALID_P);
}

void Robot::init(Vector2d iPose, double iTheta, Vector2d path[], uint8_t path_size, double iMinEndDist) {
  pose = iPose;
  theta = iTheta;
  motorL->init();
  motorR->init();
  odo->init(iPose, iTheta);
  ppc->init();
  ppc->loadPath(path, path_size);
  endPoint = path[path_size-1];
  minEndDist = iMinEndDist;
  angVelFilter = new FilterOnePole(LOWPASS, 1);
}

void Robot::start() {
  motorL->enable();
  motorR->enable();
}

void Robot::stop() {
  motorL->disable();
  motorR->disable();
}

void Robot::update() {
  motorL->update();
  motorR->update();
  odo->update();
  pose(0) = odo->getX();
  pose(1) = odo->getY();
  theta = odo->getTheta();
  
}

void Robot::followPath() {
  isNearTarget();
  ppc->update(Vector3d(pose(0), pose(1), theta));
  angVelFilter->input(ppc->getTargetAngVel());
  motorL->setRPS(odo->computeLRPS(targetVx, angVelFilter->output()));
  motorR->setRPS(odo->computeRRPS(targetVx, angVelFilter->output()));
}

void Robot::setTargetVx(double newVx) {
  targetVx = newVx;
}

double Robot::getTargetVx() {
  return targetVx;
}

void Robot::isNearTarget() {
  if (ppc->getDist(pose, endPoint) <= minEndDist) {
    nearTarget = true;
  }
  else {
    nearTarget = false;
  }
}

bool Robot::getNearTarget() {
  return nearTarget;
}
