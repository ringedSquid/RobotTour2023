#include "DCMotor.h"
#include <PID_v1.h>

DCMotor::DCMotor(uint8_t iEncoderP1, uint8_t iEncoderP2, 
                uint8_t  iMotorP1, uint8_t iMotorP2,
                uint8_t iPWMChannel1, uint8_t iPWMChannel2,
                uint32_t iintervalus, double iTPR,
                double iKp, double iKi, double iKd)
{
  encoderP1 = iEncoderP1;
  encoderP2 = iEncoderP2;
  motorP1 = iMotorP1;
  motorP2 = iMotorP2;
  PWMChannel1 = iPWMChannel1;
  PWMChannel2 = iPWMChannel2;
  intervalus = iintervalus;
  TPR = iTPR;
  toRPS = (TWO_PI * pow(10, 6)) / iTPR;
  toRPM = (6 * pow(10, 7)) / iTPR;
  Kp = iKp;
  Ki = iKi;
  Kd = iKd;
  tpusPID = new PID(&currentTPus, &motorPWM, &targetTPus,
                    Kp, Ki, Kd,
                    DIRECT);
                  
}
                
void DCMotor::init()
{
  pinMode(encoderP1, INPUT_PULLUP);
  pinMode(encoderP2, INPUT_PULLUP);
  pinMode(motorP1, OUTPUT);
  pinMode(motorP2, OUTPUT);

  ledcSetup(PWMChannel1, 1000, 15);
  ledcSetup(PWMChannel2, 1000, 15);
  ledcAttachPin(motorP1, PWMChannel1);
  ledcAttachPin(motorP2, PWMChannel2);
  motorPWM = 0;

  tpusPID->SetOutputLimits(-(int)(pow(2, 15)-1), (int)(pow(2, 15)-1));
  targetTPus = 0;
  currentTPus = 0;
  ticks = 0;
  oldTicks = 0;
  
  oldus = micros();

  disable();
  
}

void DCMotor::setTPus(double newTPus) {
  targetTPus = newTPus;
}

void DCMotor::setRPS(double newRPS) {
  setTPus(newRPS/toRPS);
}

void DCMotor::setRPM(double newRPM) {
  setTPus(newRPM/toRPM);
}

void DCMotor::enable() {
  tpusPID->SetMode(AUTOMATIC);
}

void DCMotor::disable() {
  ledcWrite(PWMChannel1, 0);
  ledcWrite(PWMChannel2, 0);
  tpusPID->SetMode(MANUAL);
}

void DCMotor::computeTPus() {
  double currentus = micros();
  long currentTicks = ticks;
  if (currentus - oldus > intervalus) {
    currentTPus = (double)(ticks-oldTicks)/(currentus-oldus);
    oldus = currentus;
    oldTicks = currentTicks;
  }
}

void DCMotor::update() {
  computeTPus();
  tpusPID->Compute();
  if (motorPWM < 0) {
    ledcWrite(PWMChannel1, 0);
    ledcWrite(PWMChannel2, abs(motorPWM));
  }
  else {
    if (motorPWM < 0) {
      ledcWrite(PWMChannel2, 0);
      ledcWrite(PWMChannel1, abs(motorPWM));
    }
  }
}

void DCMotor::tickEncoder() {
  if (digitalRead(encoderP1) == HIGH) {
    ticks++;
  }
  else {
    ticks--;
  }
}

double DCMotor::getTPus() {
  return currentTPus;
}

double DCMotor::getRPS() {
  return getTPus()*toRPS;
}
 
double DCMotor::getRPM() {
  return getTPus()*toRPM;
}

long DCMotor::getTicks() {
  return ticks;
}
