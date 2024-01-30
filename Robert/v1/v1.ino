#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

controller robotController
(
  WHEEL_RADIUS, TRACK_WIDTH,
  &stepperL, &stepperR,
  STEPS_PER_REV
); 

void setup() {
  //init pins
  Serial.begin(115200);
  pinMode(BTN_0, INPUT);
  pinMode(BTN_1, INPUT);

  pinMode(FAULT_L, INPUT);
  pinMode(FAULT_R, INPUT);

  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  //Invert direction
  stepperR.setPinsInverted(true);
  stepperL.setMinPulseWidth(2);
  stepperR.setMinPulseWidth(2);
  
  robotController.init();
  
  robotController.setMaxAx(MAX_ACCEL);
  robotController.setMaxVx(80*PI);


  robotController.moveX(300);
}

void loop() {
  robotController.update();
}
