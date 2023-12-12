#include "CONFIG.h"
#include "PATH0.h"

#include "DCMotor.h"
#include "odometry.h"
#include "controller.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

//Init objects
DCMotor motorL
(
  ENCODER_L_1, ENCODER_L_2, 
  MOTOR_L_1, MOTOR_L_2, 
  0, 1, 
  MOTOR_INTERVAL_US, TICKS_PER_REV, 
  MOTOR_L_KP, MOTOR_L_KI, MOTOR_L_KD,
  MOTOR_FILTER_FREQ,
  MOTOR_L_F
);
               
DCMotor motorR
(
  ENCODER_R_2, ENCODER_R_1, 
  MOTOR_R_2, MOTOR_R_1, 
  2, 3, 
  MOTOR_INTERVAL_US, TICKS_PER_REV, 
  MOTOR_R_KP, MOTOR_R_KI, MOTOR_R_KD,
  MOTOR_FILTER_FREQ,
  MOTOR_R_F
);

Odometry odo
(
  &motorL, &motorR, 
  WIDTH_TRACK, RADIUS_WHEEL, 
  ODOMETRY_INTERVAL_US
);

Controller controller
(
  RADIUS_WHEEL, WIDTH_TRACK,
  &motorL, &motorR,
  MAX_ANG_VEL,
  &odo,
  CONTROLLER_INTERVAL_US,
  POSE_KP, POSE_KI, POSE_KD,
  0
);

//Interrupts
void motorInterruptHandlerLA() {
  motorL.tickEncoderA();
}

void motorInterruptHandlerLB() {
  motorL.tickEncoderB();
}

void motorInterruptHandlerRA() {
  motorR.tickEncoderA();
}

void motorInterruptHandlerRB() {
  motorR.tickEncoderB();
}

//Globals
uint8_t STATE;
double ROBOTSPEED = 0;

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
    case TUNING:
      digitalWrite(RED, LOW);
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_1), motorInterruptHandlerRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_2), motorInterruptHandlerRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_1), motorInterruptHandlerLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_2), motorInterruptHandlerLA, CHANGE);

  //Init objects

  STATE = TUNING;
  LED_STATE();
  
  odo.init(Vector2d(0, 0), 0);
  
  controller.init();
  controller.setTargetTheta(0);
  controller.enable();


}
double oldt = millis();
double thet = 0;
void loop() {
  if (millis() - oldt > 50) {
    Serial.printf("CURR: %f, TARG: %f\n", odo.getTheta(), thet);
  }
  
  if (BTN_STATE(0)) {
    thet += PI/2;
    controller.setTargetTheta(thet);
    
  }
  if (BTN_STATE(1)) {
    thet -= PI/2;
    controller.setTargetTheta(thet);
  }
  
  
  controller.update();
  
}
