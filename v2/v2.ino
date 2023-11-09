#include "hmap.h"
#include "DCMotor.h"
#include "odometry.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

DCMotor motorL(EC1, EC2, M1, M2, 0, 1, (uint32_t)50*pow(10, 3), 7*93, 0, 0, 0);
DCMotor motorR(EC4, EC3, M3, M4, 2, 3, (uint32_t)50*pow(10, 3), 7*93, 0, 0, 0);
Odometry poop(&motorL, &motorR, 114.09, 32, (uint32_t)80*pow(10, 3));

void motorInterruptHandlerL() {
  motorL.tickEncoder();
}

void motorInterruptHandlerR() {
  motorR.tickEncoder();
}

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EC2), motorInterruptHandlerL, RISING);
  attachInterrupt(digitalPinToInterrupt(EC3), motorInterruptHandlerR, RISING);
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
  if (millis() - oldt > 20) {

    buff = poop.getXYTheta();
    Serial.print(buff(0)); Serial.print(" ");
    Serial.print(buff(1)); Serial.print(" ");
    Serial.print(buff(2)); Serial.print("\n");
   
  }
  

}
