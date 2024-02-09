#include <mutex>
#include <esp_task_wdt.h>

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

//FSM State
int STATE;

//Path globals
Vector2d PATH[100]; //= {Vector2d(0, 0), Vector2d(0, 300), Vector2d(300, 300), Vector2d(300, 0), Vector2d(0, 0)};
uint8_t PATH_SIZE;
double TARGET_TIME;
double FINAL_OFFSET;

//SD Methods
boolean loadPathFromSD(fs::FS &fs);

//Button related
uint8_t BTN_PINS[] = {BTN_0, BTN_1};
bool BTN_PREV_STATES[] = {LOW, LOW};
bool BTN_STATE(uint8_t index);

SSD1306AsciiWire oled;

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

//Mutexes for stepper instances
std::mutex steppersEngaged_mtx;

//Multicore tasks for engaging steppers
void engageSteppers(void * parameter);
TaskHandle_t engageSteppersHandle = NULL;

controller robotController
(
  WHEEL_RADIUS, TRACK_WIDTH,
  &stepperL, &stepperR,
  STEPS_PER_REV, TURN_US,
  IMU_UPDATE_US, &steppersEngaged_mtx,
  &engageSteppers, &engageSteppersHandle
);

simplePursuit robotSimplePursuit(MAX_VX, DIST_TO_DOWEL);

robot Robot
(
  &robotSimplePursuit, &robotController,
  MAX_ACCEL, MAX_ANG_ACCEL, MAX_ANG_VEL, 
  DIST_TO_DOWEL
);

void setup() {
  //for steppers
  //xTaskCreate(engageSteppers, "engageSteppers Task", 10000, NULL, 1, &engageSteppersHandle);

  //start init
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

  //oled init
  oled.begin(&Adafruit128x32, I2C_ADDRESS, OLED_RST);
  oled.set2X();
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println("INIT...");
  oled.set1X();
  oled.print("DO NOT MOVE ROBOT!");

  delay(1000);

  //gyro init
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, Wire, IMU_ADDRESS)) {
    STATE = IMU_ERROR;
  }
  BMI160.setGyroRate(10);
  BMI160.setFullScaleGyroRange(1); //1000 deg/s
  BMI160.autoCalibrateGyroOffset();
  

  //SD begin
  if (SD.begin(SD_CS) == 0) {
    STATE = SD_ERROR;
    oled.clear();
    oled.set2X();
    oled.println("SD ERROR");
    oled.set1X();
    oled.print("Check connections");

  }

  //load Paths
  if (!loadPathFromSD(SD)) {
    STATE = FILE_ERROR;
    oled.clear();
    oled.set2X();
    oled.println("FILE ERROR");
    oled.set1X();
    oled.println("NO CARD or");
    oled.println("BAD FILE");
  }

  if (STATE == INIT) {
    STATE = IDLE;
    oled.clear();
    oled.set2X();
    oled.println("IDLE");
  }

  /*
  double a = 300;
  double v = 127;
  double d = 400;
  double offset = 0.6;
  robotController.init();
  robotController.setMaxAx(a);
  robotController.setMaxAngVx(v);
  robotController.setMaxVx(v + offset);
  double expected_t = (d + (v * v / a)) / v;
  oled.clear();
  oled.set1X();
  oled.println("START");
  robotController.setTheta(PI/2);
  delay(2);
  uint32_t oldt = micros();
  while (robotController.getState() != 0) {
    robotController.update();
  }
  oled.println((micros() - oldt) / pow(10, 6));
  oled.println(expected_t);
  */
}

void loop() { 
  switch (STATE) {
    case INIT:
      break;
    case IDLE:
      if (BTN_STATE(1)) {
        Robot.init(FINAL_OFFSET);
        robotSimplePursuit.init(PATH, PATH_SIZE, TARGET_TIME, FINAL_OFFSET);
        STATE = READY;
        oled.clear();
        oled.set2X();
        oled.println("READY");
        oled.set1X();
        oled.print("target_t: "); oled.print(TARGET_TIME);
      }
      break;

    case READY:
      //Robot.update();
      if (BTN_STATE(1)) {
        Robot.init(FINAL_OFFSET);
        STATE = READY;
        oled.clear();
        oled.set2X();
        oled.println("READY");
        oled.set1X();
        oled.print("target_t: "); oled.print(TARGET_TIME);
      }
      if (BTN_STATE(0)) {
        Robot.startPath();
        STATE = RUNNING;
        oled.clear();
        oled.set2X();
        oled.println("RUNNING");
        robotController.initTheta(PI/2);
      }
      break;

    case RUNNING:
      Robot.update();
      if (BTN_STATE(1)) {
        STATE = STOPPED;
        oled.clear();
        oled.set2X();
        oled.println("STOPPED");
        oled.set1X();
        oled.print("current_t: "); oled.print(Robot.stopPath());
      }
      if (Robot.getState() == 0) {
        STATE = END_RUN;
        oled.clear();
        oled.set2X();
        oled.println("RUN ENDED");
        oled.set1X();
        oled.print("elapsed_t: "); oled.print(Robot.stopPath());
      }
      break;

    case END_RUN:
      if (BTN_STATE(1)) {
        STATE = IDLE;
        oled.clear();
        oled.set2X();
        oled.println("IDLE");
      }
      if (BTN_STATE(0)) {
        Robot.init(FINAL_OFFSET);
        robotSimplePursuit.init(PATH, PATH_SIZE, TARGET_TIME, FINAL_OFFSET);
        STATE = READY;
        oled.clear();
        oled.set2X();
        oled.println("READY");
        oled.set1X();
        oled.print("target_t: "); oled.print(TARGET_TIME);
      }
      break;

    case STOPPED:
      if (BTN_STATE(1)) {
        STATE = IDLE;
        oled.clear();
        oled.set2X();
        oled.println("IDLE");
      }
      if (BTN_STATE(0)) {
        Robot.init(FINAL_OFFSET);
        robotSimplePursuit.init(PATH, PATH_SIZE, TARGET_TIME, FINAL_OFFSET);
        STATE = READY;
        oled.clear();
        oled.set2X();
        oled.println("READY");
        oled.set1X();
        oled.print("target_t: "); oled.print(TARGET_TIME);
      }
      break;

    case ERROR:
      break;
    case SD_ERROR:
      break;
    case FILE_ERROR:
      break;
    case IMU_ERROR:
      break;
    default:
      STATE = IDLE;
    }
}

void engageSteppers(void * parameter) {
  esp_task_wdt_init(300, false);
  steppersEngaged_mtx.lock();
  while (stepperL.run() && stepperR.run());
  stepperL.setCurrentPosition(stepperL.targetPosition());
  stepperR.setCurrentPosition(stepperR.targetPosition()); 
  steppersEngaged_mtx.unlock();
  vTaskDelete(NULL);
}

boolean loadPathFromSD(fs::FS &fs) {
  /*
     FILE FORMAT

     TARGET TIME:
     50.00
     PATH:
     A1
     B2
     ...
  */
  File file = fs.open(PATH_FILE);
  if (!file) {
    return false;
  }
  PATH_SIZE = 0;
  char buff[5];

  //read in the final offset

  //skip first line until newline is reached
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  for (int i = 0; i < 5; i++) {
    buff[i] = file.read();
  }
  FINAL_OFFSET = atof(buff);
  file.read();

  //read in target time

  //skip first line until newline is reached
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  for (int i = 0; i < 5; i++) {
    buff[i] = file.read();
  }
  TARGET_TIME = atof(buff);
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
        case '1':
          pY = 250;
          break;
        case '2':
          pY = 750;
          break;
        case '3':
          pY = 1250;
          break;
        case '4':
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
    while (file.available()) {
      if (file.read() == '\n') {
        break;
      }
    }
  }
  return true;
}

bool BTN_STATE(uint8_t index) {
  bool buttonstate = digitalRead(BTN_PINS[index]);
  if (buttonstate != BTN_PREV_STATES[index]) {
    BTN_PREV_STATES[index] = buttonstate;
    if (buttonstate == HIGH) {
      return true;
    }
  }
  return false;
}
