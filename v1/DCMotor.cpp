#include "DCMotor.h"
#include <PID_v1.h>

DCMotor::DCMotor(byte c1, byte c2, byte m1, byte m2, 
                 byte ipchannel1, byte ipchannel2,
                 double ikp, double iki, double ikd, 
                 double uInterval) {
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
  rpmPID = new PID(&currentRPM, &motorOut, &targetRPM, kp, ki, kd, DIRECT);
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
  targetRPM = 0;
  currentRPM = 0;
  ticks = 0;
  oldticks = 0;
  oldt = millis();
  
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
  currentRPM = getRPM();
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

void DCMotor::setRPM(double rpm) {
  targetRPM = rpm;
}

double DCMotor::getRPM() {
  if (millis() - oldt > updateInt) {
    currentRPM = (ticks-oldticks)/(millis()-oldt)*60*1000/68;
    oldt = millis();
    oldticks = ticks;
  }
  return currentRPM;
}
