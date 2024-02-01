#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"
#include "simplePursuit.h"
#include "robot.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "SD.h"
#include "FS.h"
#include "SPI.h"

#include <BMI160Gen.h>

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

int STATE;

Vector2d PATH[100]; //= {Vector2d(0, 0), Vector2d(0, 300), Vector2d(300, 300), Vector2d(300, 0), Vector2d(0, 0)};
uint8_t PATH_SIZE;
double TARGET_TIME;

SSD1306AsciiWire oled;

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

controller robotController
(
  WHEEL_RADIUS, TRACK_WIDTH,
  &stepperL, &stepperR,
  STEPS_PER_REV, TURN_US,
  IMU_UPDATE_US
); 

simplePursuit robotSimplePursuit(TURN_US);

robot Robot 
(
  &robotSimplePursuit, &robotController,
  MAX_ACCEL, MAX_ANG_VEL,
  0
);


/*
 * FILE FORMAT
 * 
 * TARGET TIME:
 * 50.00
 * PATH:
 * A1
 * B2
 * ...
 */

boolean loadPathFromSD(fs::FS &fs);

void setup() {
  STATE = INIT;
 
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);

  //init pins
  pinMode(BTN_0, INPUT);
  pinMode(BTN_1, INPUT);

  pinMode(FAULT_L, INPUT);
  pinMode(FAULT_R, INPUT);

  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  //SD begin
  SD.begin(SD_CS);

  //gyro init
  BMI160.begin(BMI160GenClass::I2C_MODE, Wire, IMU_ADDRESS);

  //oled init
  oled.begin(&Adafruit128x32, I2C_ADDRESS, OLED_RST);
  oled.setFont(Adafruit5x7);
  oled.clear();

  STATE = IDLE;

  robotController.init();
  robotController.setTheta(PI/2);
  /*
  //load Paths
  if (!loadPathFromSD(SD)) {
    STATE = ERROR;
  }

  robotSimplePursuit.init(PATH, PATH_SIZE, TARGET_TIME);
  

  delay(1000);
  */
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
  /*
  Robot.init();
  Robot.startPath();
  */
}

void loop() {
  robotController.update();
  /*
  Robot.update();
  */
}

boolean loadPathFromSD(fs::FS &fs) {
  File file = fs.open(PATH_FILE);
  if (!file) {
    return false;
  }
  PATH_SIZE = 0;
  char buff[5];
  //read in the target time
  //skip first line until newline is reached
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  for (int i=0; i<4; i++) {
    buff[i] = file.read();
  }
  TARGET_TIME = atof(buff);
  
  //skip to next line
  file.read();
  
  //skip line
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }

  //Read in paths
  while (file.available()) {
    buff[0] = file.read();
    buff[1] = file.read();
    //coords
    double pX, pY;
    if (buff[0] != 'E') {
      switch (buff[0]) {
        case 'A':
          pX = 250;
          break;
        case 'B':
          pX = 750;
          break;
        case 'C':
          pX = 1250;
          break;
        case 'D':
          pX = 1750;
          break;
        default:
          return false;
      }
      switch (buff[1]) {
        case 1:
          pY = 250;
          break;
        case 2:
          pY = 750;
          break;
        case 3:
          pY = 1250;
          break;
        case 4:
          pY = 1750;
          break;
        default:
          return false;
      }
    }
    else {
      pY = -DIST_TO_DOWEL;
      switch (buff[1]) {
        case 'A':
          pX = 250;
          break;
        case 'B':
          pX = 750;
          break;
        case 'C':
          pX = 1250;
          break;
        case 'D':
          pX = 1750;
          break;
        default:
          return false;
      }
    }
    PATH[PATH_SIZE] = Vector2d(pX, pY);
    PATH_SIZE++;
    file.read();
  }
  return true;
}
