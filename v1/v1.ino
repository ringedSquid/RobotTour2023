#include "hmap.h"
#include "motor.h"


DCMotor mot(C1, C2, M1, M2, 10, 0, 0, 10);

void setup() {
  mot.init();

}

void loop() {
  // put your main code here, to run repeatedly:

}
