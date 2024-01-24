#include "DCMotor.h"
#include <PID_v1.h>

DCMotor::DCMotor(byte c1, byte c2, byte m1, byte m2, 
                 byte ipchannel1, byte ipchannel2,
                 double ikp, double iki, double ikd, 
                 double uInterval, unsigned long iTpr) {
  encoderP1 = c1;
  encoderP2 = c2;
  motorP1 = m1;
  motorP2 = m2;
  pchannel1 = ipchannel1;
  pchannel2 = ipchannel2;
  updateInt = uInterval;
  kp = ikp;
  ki = iki;
  kd = ikd;
  rpmPID = new PID(&currentAv, &motorOut, &targetAv, kp, ki, kd, DIRECT);
  tpr = iTpr;
  toRadPerSec = 1000000.0 * TWO_PI / tpr;
}

void DCMotor::init() {
  pinMode(encoderP1, INPUT_PULLUP);
  pinMode(encoderP2, INPUT_PULLUP);
  pinMode(motorP1, OUTPUT);
  pinMode(motorP2, OUTPUT);

  ledcSetup(pchannel1, 1000, 15);
  ledcSetup(pchannel2, 1000, 15);
  ledcAttachPin(motorP1, pchannel1);
  ledcAttachPin(motorP2, pchannel2);
  
  rpmPID->SetOutputLimits(-(int)(pow(2, 15)), (int)(pow(2, 15)-1));
  disable();
  targetAv = 0;
  currentAv = 0;
  ticks = 0;
  oldticks = 0;
  oldt = micros();
  
}

void DCMotor::updateTicks() {
  if (digitalRead(encoderP2) == HIGH) {
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
  ledcWrite(pchannel1, 0);
  ledcWrite(pchannel2, 0);
  /*
  digitalWrite(motorP1, 0);
  digitalWrite(motorP2, 0);
  */
  rpmPID->SetMode(MANUAL);
}

void DCMotor::motorUpdate() { 
  computeAv();
  rpmPID->Compute();
  if (motorOut < 0) {
    ledcWrite(pchannel1, abs(motorOut));
    ledcWrite(pchannel2, 0);
    /*
    analogWrite(motorP1, abs(motorOut));
    digitalWrite(motorP2, 0);
    */
  }
  else {
    ledcWrite(pchannel1, 0);
    ledcWrite(pchannel2, abs(motorOut));
    /*
    analogWrite(motorP2, abs(motorOut));
    digitalWrite(motorP1, 0);
    */
  }
}

void DCMotor::computeAv() {
  unsigned long t = micros();
  if (t-oldt > updateInt) {
    currentAv = (ticks-oldticks)/(double)(t-oldt)*toRadPerSec;
    oldt = micros();
    oldticks = ticks;
  }
}

void DCMotor::setTPMi(double tpmi) {
  targetAv = tpmi * toRadPerSec;
}

void DCMotor::setRPS(double rps) {
  targetAv = rps;
}

void DCMotor::setRPM(double rpm) {
  targetAv = (rpm/60)*TWO_PI;
}

double DCMotor::getTPMi() {
  return currentAv/toRadPerSec;
}

double DCMotor::getRPS() {
  return currentAv;
}

double DCMotor::getRPM() {
  return (currentAv*60)/TWO_PI;
}

long DCMotor::getTicks() {
  return ticks;
}
