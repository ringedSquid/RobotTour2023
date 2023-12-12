#include "DCMotor.h"
#include <Filters.h>

DCMotor::DCMotor(uint8_t iEncoderP1, uint8_t iEncoderP2, 
                uint8_t  iMotorP1, uint8_t iMotorP2,
                uint8_t iPWMChannel1, uint8_t iPWMChannel2,
                uint32_t iintervalus, double iTPR,
                double iKp, double iKi, double iKd,
                double iFilterFreq,
                double iF)
{
  encoderP1 = iEncoderP1;
  encoderP2 = iEncoderP2;
  motorP1 = iMotorP1;
  motorP2 = iMotorP2;
  PWMChannel1 = iPWMChannel1;
  PWMChannel2 = iPWMChannel2;
  intervalus = iintervalus;
  TPR = iTPR;
  toRPS = (TWO_PI * pow(10, 6)) / (double)iTPR;
  toRPM = (6 * pow(10, 7)) / (double)iTPR;
  Kp = iKp;
  Ki = iKi;
  Kd = iKd;
  F = iF;
  filterFreq = iFilterFreq;
  tpusPID = new PID_Ballsack(
    &currentTPus, &pidOut, &targetTPus,
    Kp, Ki, Kd,
    DIRECT
    ); 
  lowPassTPus = new FilterOnePole(LOWPASS, filterFreq);
}
                
void DCMotor::init()
{
  pinMode(encoderP1, INPUT);
  pinMode(encoderP2, INPUT);
  pinMode(motorP1, OUTPUT);
  pinMode(motorP2, OUTPUT);

  ledcSetup(PWMChannel1, 1000, 15);
  ledcSetup(PWMChannel2, 1000, 15);
  ledcAttachPin(motorP1, PWMChannel1);
  ledcAttachPin(motorP2, PWMChannel2);
  motorPWM = 0;
  
  targetTPus = 0;
  currentTPus = 0;
  ticks = 0;
  oldTicks = 0;

  tpusPID->SetOutputLimits(-(int)(pow(2, 13)-1), (int)(pow(2, 13)-1));
  tpusPID->SetSampleTime(intervalus/1000);
  oldus = micros();

  disable();
  
}

void DCMotor::setTPus(double newTPus) {
  targetTPus = newTPus;
  tpusPID->ResetSumError();
}

void DCMotor::setRPS(double newRPS) {
  setTPus(newRPS/toRPS);
}

void DCMotor::setRPM(double newRPM) {
  setTPus(newRPM/toRPM);
}

void DCMotor::enable() {
  isEnabled = true;
  tpusPID->SetMode(AUTOMATIC);
}

void DCMotor::disable() {
  isEnabled = false;
  ledcWrite(PWMChannel1, 0);
  ledcWrite(PWMChannel2, 0);
  tpusPID->SetMode(MANUAL);
}

void DCMotor::computeTPus() {
  double currentus = micros();
  long currentTicks = ticks;
  if (currentus - oldus > intervalus) {
    lowPassTPus->input((double)(ticks-oldTicks)/(currentus-oldus));
    currentTPus = lowPassTPus->output();
    oldus = currentus;
    oldTicks = currentTicks;
  }
}

double DCMotor::getFeedForward() {
  return (targetTPus-getTPus()) * F;
}

void DCMotor::update() {
  computeTPus();
  if (isEnabled) {
    tpusPID->Compute();
    motorPWM = pidOut + getFeedForward();
    if (motorPWM < 0) {
      ledcWrite(PWMChannel1, 0);
      ledcWrite(PWMChannel2, abs(motorPWM));
    }
    else {
      ledcWrite(PWMChannel2, 0);
      ledcWrite(PWMChannel1, abs(motorPWM));
    }
  }
}


void DCMotor::tickEncoderA() {
  if (digitalRead(encoderP1) != digitalRead(encoderP2)) {
    ticks++;
  }
  else {
    ticks--;
  }
}

void DCMotor::tickEncoderB() {
  if (digitalRead(encoderP2) == digitalRead(encoderP1)) {
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
