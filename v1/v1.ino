#include "hmap.h"
#include "DCMotor.h"

DCMotor mot1(EC1, EC2, MOT1, MOT2, 0, 1, 500, 0, 0, 3 * 1000, 68);
DCMotor mot2(EC4, EC3, MOT4, MOT3, 2, 3, 60, 10, 0.001, 5 * 1000, 68);

void motorInterruptHandler1() {
  mot1.updateTicks();
}

void motorInterruptHandler2() {
  mot2.updateTicks();
}

void setup() {
  Serial.begin(115200);
  mot1.init();
  mot2.init();
  attachInterrupt(digitalPinToInterrupt(mot1.encoderP1), motorInterruptHandler1, RISING);
  attachInterrupt(digitalPinToInterrupt(mot2.encoderP1), motorInterruptHandler2, RISING);
  mot1.enable();
  mot1.setRPM(200);
}



void loop() {
  Serial.print("200 1000 ");
  Serial.println(mot1.getRPM());
  mot1.motorUpdate();
}
