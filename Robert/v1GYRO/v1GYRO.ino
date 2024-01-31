#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"
#include "simplePursuit.h"
#include "robot.h"

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

SSD1306AsciiWire oled;

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

controller robotController
(
  WHEEL_RADIUS, TRACK_WIDTH,
  &stepperL, &stepperR,
  STEPS_PER_REV, TURN_US
); 

Vector2d path[] = {Vector2d(0, 0), Vector2d(0, 300), Vector2d(300, 300), Vector2d(300, 0), Vector2d(0, 0)};

simplePursuit robotSimplePursuit 
(
  path, 5,
  10, 0
);

robot Robot 
(
  &robotSimplePursuit, &robotController,
  MAX_ACCEL, MAX_ANG_VEL,
  0
);



void setup() {
  //init pins
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);
  
  pinMode(BTN_0, INPUT);
  pinMode(BTN_1, INPUT);

  pinMode(FAULT_L, INPUT);
  pinMode(FAULT_R, INPUT);

  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  //oled init
  oled.begin(&Adafruit128x32, I2C_ADDRESS, OLED_RST);
  oled.setFont(Adafruit5x7);
  oled.clear();

  delay(1000);
  /*
  //Invert direction
  stepperR.setPinsInverted(true);
  stepperL.setMinPulseWidth(2);
  stepperR.setMinPulseWidth(2);
  */
  /*
  
  robotController.init();
  
  robotController.setMaxAx(MAX_ACCEL);
  robotController.setMaxVx(80*PI);
  robotController.setMaxAngVx(40*PI);

  delay(5000);

  for (int i=0; i<4; i++) {
    robotController.moveX(200);
    while (robotController.getState() == 1) {
      robotController.update();
    }
    delay(100);
    robotController.turn(PI/2);
    while (robotController.getState() == 1) {
      robotController.update();
    }
    delay(100);
  }
  oled.set2X();
  oled.print(stepperR.currentPosition());
  */
  Robot.init();
  Robot.startPath();
}

void loop() {
  Robot.update();
  
}
