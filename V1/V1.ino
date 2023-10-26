#include "hmap.h"
#include "config.h"
#include <Filters.h>
#include <PID_v1.h>

FilterOnePole RL(LOWPASS, 2);
FilterOnePole LL(LOWPASS, 2);


//Encoders

volatile long rightEncoderPos=100, leftEncoderPos=100;

//Encoder ISR
void rightEncoderTick() {
  rightEncoderPos++;
}

void leftEncoderTick() {
  leftEncoderPos++;

}

//Motors

double Rrpm = 0;
double Rmotor_pwm = 0;
double Rsetpoint = 5;

double Lrpm = 0;
double Lmotor_pwm = 0;
double Lsetpoint = 5;

PID rPID(&Rrpm, &Rmotor_pwm, &Rsetpoint, MR_Kp, MR_Ki, MR_Kd, DIRECT);
PID lPID(&Lrpm, &Lmotor_pwm, &Lsetpoint, ML_Kp, ML_Ki, ML_Kd, DIRECT);
  
void setup()    {
  Serial.begin(115200);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(C1, INPUT);
  pinMode(C4, INPUT);

  digitalWrite(M2, LOW);
  digitalWrite(M3, LOW);

  attachInterrupt(digitalPinToInterrupt(C4), rightEncoderTick, RISING);
  attachInterrupt(digitalPinToInterrupt(C1), leftEncoderTick, RISING);
  rPID.SetMode(AUTOMATIC);
  lPID.SetMode(AUTOMATIC);

  }

void loop() {
  //Get RPM
  double oldt = millis();
  long RoldPos = rightEncoderPos;
  long LoldPos = leftEncoderPos;
  delay(5);
  rPID.Compute();
  lPID.Compute();
  analogWrite(M1, (int)Rmotor_pwm);
  analogWrite(M4, (int)Lmotor_pwm);
  RL.input((((rightEncoderPos - RoldPos)/(millis() - oldt))/60)*6000);
  LL.input((((leftEncoderPos - LoldPos)/(millis() - oldt))/60)*6000);
  Rrpm = RL.output();
  Lrpm = LL.output();
}
