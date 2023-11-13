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

void Odometry::init(Vector2d iPose, double iTheta) {
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
  double currentML = motorL->getRPS();
  double currentMR = motorR->getRPS();

  //update the pos;
  if (currentus - oldus > intervalus) {
    //Serial.printf("X: %f, Y: %f, Theta %f, Rl: %f, Rr: %f\n", pose(0), pose(1), theta, motorL->getTicks()/(3.0*100.37), motorR->getTicks()/(3.0*100.7));
    linVelx = (wheelRadius/2)*(currentMR+currentML);
    angVel = (wheelRadius/trackWidth)*(currentML-currentMR); //RPS

    theta += angVel * ((currentus-oldus) / pow(10, 6));
    pose(0) += linVelx * cos(theta) * ((currentus-oldus) / pow(10, 6));
    pose(1) += linVelx * sin(theta) * ((currentus-oldus) / pow(10, 6));

    oldLinVelx = linVelx;
    oldAngVel = angVel;
    oldus = currentus;

    //Keep theta within [0, 2pi]
    if (theta > TWO_PI) {
      theta -= TWO_PI;
    }

    else if (theta < 0) {
      theta += TWO_PI;
    }
  }
}

void Odometry::setPose(Vector2d newPose) {
  pose = newPose;
}

void Odometry::setX(double newX) {
  pose(0) = newX;
}

void Odometry::setY(double newY) {
  pose(1) = newY;
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


double Odometry::getTheta() {
  return theta;
}

Vector2d Odometry::getPose() {
  return pose;
}

Vector3d Odometry::getXYTheta() {
  return Vector3d(pose(0), pose(1), theta);
}

//inverse kinematics
double Odometry::computeRRPS(double targetLinVelx, double targetAngVel) {
  return (targetLinVelx/wheelRadius)+(trackWidth/2)*(targetAngVel/wheelRadius);
}
double Odometry::computeLRPS(double targetLinVelx, double targetAngVel) {
  return (targetLinVelx/wheelRadius)-(trackWidth/2)*(targetAngVel/wheelRadius);
}
