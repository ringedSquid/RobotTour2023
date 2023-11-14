#include "CONFIG.h"
#include "PATH0.h"

#include "DCMotor.h"
#include "odometry.h"
#include "purepursuit.h"
#include "robot.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

//Init objects
DCMotor motorL
(
  ENCODER_L_1, ENCODER_L_2, 
  MOTOR_L_1, MOTOR_L_2, 
  0, 1, 
  MOTOR_INTERVAL_US, TICKS_PER_REV, 
  MOTOR_L_KP, MOTOR_L_KI, MOTOR_L_KD
);
               
DCMotor motorR
(
  ENCODER_R_2, ENCODER_R_1, 
  MOTOR_R_2, MOTOR_R_1, 
  2, 3, 
  MOTOR_INTERVAL_US, TICKS_PER_REV, 
  MOTOR_R_KP, MOTOR_R_KI, MOTOR_R_KD
);

Odometry odo
(
  &motorL, &motorR, 
  WIDTH_TRACK, RADIUS_WHEEL, 
  ODOMETRY_INTERVAL_US
);

PurePursuitController ppc
(
  PURE_PURSUIT_LOOKAHEAD,
  PURE_PURSUIT_KP,
  PURE_PURSUIT_MAX_AV,
  PURE_PURSUIT_INTERVAL_US
);

Robot robot
(
  &motorL, &motorR,
  &odo,
  &ppc
);

//Interrupts
void motorInterruptHandlerL() {
  motorL.tickEncoder();
}

void motorInterruptHandlerR() {
  motorR.tickEncoder();
}

//Globals
uint8_t STATE;
Vector2d PATH[] = PATH0;

//Button handling code
uint8_t BTN_PINS[] = {BT1, BT2};
bool BTN_PREV_STATES[] = {LOW, LOW};

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

//LED State updates
void LED_STATE() {
  switch(STATE) {
    case INIT:
      digitalWrite(RED, LOW);
      digitalWrite(GRN, LOW);
      break;
    case IDLE:
      digitalWrite(RED, LOW);
      digitalWrite(GRN, LOW);
      break;
    case READY:
      digitalWrite(RED, LOW);
      digitalWrite(GRN, HIGH);
      break;
    case RUNNING:
      digitalWrite(RED, HIGH);
      digitalWrite(GRN, LOW);
      break;
    default:
      digitalWrite(RED, LOW);
      digitalWrite(GRN, LOW);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  
  //init pins
  pinMode(RED, OUTPUT);
  pinMode(GRN, OUTPUT);
  pinMode(BT1, INPUT);
  pinMode(BT2, INPUT);

  //init state;
  STATE = INIT;
  LED_STATE();

  //Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_1), motorInterruptHandlerR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_2), motorInterruptHandlerL, RISING);

  //Init objects
  robot.init();

  STATE = IDLE;
  LED_STATE();
}

void loop() {
  switch (STATE) {
    case IDLE:
      robot.update();
      //BT1 Pressed
      if (BTN_STATE(0)) {
        robot.init(ROBOT_INIT);
        STATE = READY;
        LED_STATE();
      }
      break;
      
    case READY:
      robot.update();
      if (BTN_STATE(1)) {
        robot.start();
        robot.setTargetVx(150);
        STATE = RUNNING;
        LED_STATE();
      }
      else if (BTN_STATE(0)) { //Re-init
        robot.init(ROBOT_INIT);
        STATE = READY;
        LED_STATE();
      }
      break;

    case RUNNING:
      robot.update();
      robot.followPath();
      if (BTN_STATE(1) || robot.getNearTarget()) {
        robot.stop();
        robot.init(ROBOT_INIT);
        STATE = IDLE;
        LED_STATE();
      }
      break;
      
    default:
      STATE = IDLE;
      LED_STATE(); 
      break;    
  }
  

}
