#include "DCMotor.h"
#include "odometry.h";
#include <ArduinoEigenDense.h>
using namespace Eigen;

Odometry::Odometry(DCMotor *iMotorL, DCMotor *iMotorR,
                   double iTrackWidth, double iWheelRadius,
                   uint32_t iintervalus)
{
  motorL = iMotorL;
  motorR = iMotorR;
  trackWidth = iTrackWidth;
  wheelRadius = iWheelRadius;
  intervalus = iintervalus;
}

void Odometry::init(Vector3d iPose, double iTheta) {
  pose = iPose;
  theta = iTheta;

  linVelx = 0;
  angVel = 0;
  
  oldLinVelx = 0;
  oldAngVel = 0;
  oldus = micros();
  
}

void Odometry::update() {
  double currentus = micros();
  double currentML = motorL->getRPS() * wheelRadius;
  double currentMR = motorR->getRPS() * wheelRadius;

  //update the pos;
  if (currentus - oldus > intervalus) {
    linVelx = (wheelRadius/2)*(currentMR+currentML);
    angVel = (wheelRadius/trackWidth)*(currentMR-currentML); //RPS

    theta += (angVel * (currentus-oldus)) / pow(10, 6);
    pose(0) += (linVelx * cos(theta) * (currentus-oldus)) / pow(10, 6);
    pose(1) += (linVelx * sin(theta) * (currentus-oldus)) / pow(10, 6);

    oldLinVelx = linVelx;
    oldAngVel = angVel;
  }
}

void Odometry::setPose(Vector3d newPose) {
  pose = newPose;
}

void Odometry::setX(double newX) {
  pose(0) = newX;
}

void Odometry::setY(double newY) {
  pose(1) = newY;
}

void Odometry::setZ(double newZ) {
  pose(2) = newZ;
}

void Odometry::setXYTheta(Vector3d newXYTheta) {
  pose(0) = newXYTheta(0);
  pose(1) = newXYTheta(1);
  theta = newXYTheta(2);
}

double Odometry::getLinVelx() {
  return linVelx;
}

double Odometry::getAngVel() {
  return angVel;
}

double Odometry::getX() {
  return pose(0);
}

double Odometry::getY() {
  return pose(1);
}

double Odometry::getZ() {
  return pose(2);
}

Vector3d Odometry::getPose() {
  return pose;
}

Vector3d Odometry::getXYTheta() {
  return Vector3d(pose(0), pose(1), theta);
}
