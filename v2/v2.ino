#include "hmap.h"
#include "DCMotor.h"
#include "odometry.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

DCMotor motorL(EC3, EC4, M3, M4, 0, 1, (uint32_t)50*pow(10, 3), 7*93, 0, 0, 0);
DCMotor motorR(EC2, EC1, M2, M1, 2, 3, (uint32_t)50*pow(10, 3), 7*93, 0, 0, 0);
Odometry poop(&motorL, &motorR, 114.09, 16, (uint32_t)80*pow(10, 3));

void motorInterruptHandlerL() {
  motorL.tickEncoder();
}

void motorInterruptHandlerR() {
  motorR.tickEncoder();
}

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EC4), motorInterruptHandlerL, RISING);
  attachInterrupt(digitalPinToInterrupt(EC1), motorInterruptHandlerR, RISING);
  motorL.init();
  motorR.init();
  Vector3d initPos(0, 0, 0);
  poop.init(initPos, 0);
}

double oldt = millis();
Vector3d buff;

void loop() {
  motorL.update();
  motorR.update();
  poop.update();
  if (millis() - oldt > 50) {

    buff = poop.getXYTheta();
    Serial.print(buff(0)); Serial.print(" ");
    Serial.print(buff(1)); Serial.print(" ");
    Serial.print(buff(2)); Serial.print("\n");
  }
  

}
