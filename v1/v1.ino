#include "hmap.h"
#include "DCMotor.h"

DCMotor mot1(C1, C2, M1, M2, 0, 1, 60, 10, 0.001, 15);
DCMotor mot2(C4, C3, M4, M3, 2, 3, 60, 10, 0.001, 15);

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
  mot2.enable();
  mot1.setRPM(500);
  mot2.setRPM(502);
}

void loop() {

  Serial.print(mot1.getRPM());
  Serial.println(", 500, 800");
  mot2.motorUpdate();
  mot1.motorUpdate();
}
