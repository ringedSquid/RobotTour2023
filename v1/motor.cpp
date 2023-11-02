#include "motor.h"
#include <PID_v1.h>

DCMotor::DCMotor(byte c1, byte c2, byte m1, byte m2, 
                 double ikp, double iki, double ikd, 
                 double updateInt) {
  encoderP1 = c1;
  encoderP2 = c2;
  motorP1 = m1;
  motorP2 = m2;
  kp = ikp;
  ki = iki;
  kd = ikd;
  rpmPID = new PID(&currentRPM, &motorOut, &targetRPM, kp, ki, kd, DIRECT);
}

void DCMotor::init() {
  pinMode(encoderP1, INPUT);
  pinMode(encoderP2, INPUT);
  pinMode(motorP1, OUTPUT);
  pinMode(motorP2, OUTPUT);
  targetRPM = 0;
  currentRPM = 0;
  ticks = 0;
  oldticks = 0;
  oldt = millis();
  
}

void DCMotor::updateTicks() {
  if (digitalRead(encoderP1) == HIGH) {
    ticks++;
  }
  else {
    ticks--;
  }
}

void DCMotor::enable() {
  rpmPID->SetMode(AUTOMATIC);
}

void DCMotor::disable() {
  digitalWrite(motorP1, 0);
  digitalWrite(motorP2, 0);
  rpmPID->SetMode(MANUAL);
}

void DCMotor::enableEncoder() {
  attachInterrupt(digitalPinToInterrupt(encoderP1), updateTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderP2), updateTicks, RISING);
}

void DCMotor::motorUpdate() { 
  currentRPM = getRPM();
  rpmPID->Compute();
  if (motorOut < 0) {
    analogWrite(motorP1, motorOut);
    digitalWrite(motorP2, 0);
  }
  else {
    analogWrite(motorP2, motorOut);
    digitalWrite(motorP1, 0);
  }
}

void DCMotor::setRPM(double rpm) {
  targetRPM = rpm;
}

double DCMotor::getRPM() {
  if (millis() - oldt > updateInt) {
    currentRPM = (ticks-oldticks)/(millis()-oldt)*60*1000/12;
    oldt = millis();
  }
  return currentRPM;
}
