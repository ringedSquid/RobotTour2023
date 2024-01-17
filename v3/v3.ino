#include "CONFIG.h"
#include "PATH0.h"

#include "DCMotor.h"
#include "odometry.h"
#include "controller.h"
#include "SimplePursuit.h"
//#include "imu.h"
#include "Robot.h"

//IMU RELATED DO NOT TOUCH /
/*
#include "REG.h"
#include "IMUsdk.h"

#define ACC_UPDATE    0x01
#define GYRO_UPDATE   0x02
#define ANGLE_UPDATE  0x04
#define MAG_UPDATE    0x08
#define READ_UPDATE   0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};

//
*/
#include <ArduinoEigenDense.h>
using namespace Eigen;

//Path constants
Vector2d PATH[] = PATH0;
const byte PATH_SIZE = PATH0_SIZE;

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

SimplePursuit simplePursuit
(
  PATH, PATH_SIZE,
  &odo,
  CHECKRS, TRAFDIST
);
/*
IMU imu
(
  IMU_POLL_INTERVAL_US
);
*/

Robot robot
(
  &simplePursuit,
  &controller,
  &odo,
  //&imu,
  TARGET_TIME,
  CENTER_TO_DOWEL,
  END_DISTANCE,
  TURN_INTERVAL_US
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
    case STOPPED:
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
  /*
  //IMU
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();

  WitWriteReg(AXIS6, ALGRITHM6);
  WitStartMagCali();
  WitSetUartBaud(WIT_BAUD_115200);
  WitSetBandwidth(BANDWIDTH_256HZ);
  
  */
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

  STATE = INIT;
  LED_STATE();

  robot.init(Vector2d(0, 0), 0);

  STATE = IDLE;
  LED_STATE();
}
/*
uint32_t oldus = micros();
float zAngle;
float zAngleOffset;
*/
void loop() {
  /*
  if (micros() - oldus > IMU_POLL_INTERVAL_US) {
    while (Serial2.available()) {
      WitSerialDataIn(Serial2.read());
    }
    if (s_cDataUpdate)
    {
      zAngle = sReg[Roll+2] / 32768.0f * 180.0f;
      if(s_cDataUpdate & ANGLE_UPDATE)
      {
        //odo.setTheta((odo.getTheta()+((zAngle-zAngleOffset))));
        Serial.printf("%f\n", zAngle-zAngleOffset);
        s_cDataUpdate &= ~ANGLE_UPDATE;
      }
      s_cDataUpdate = 0;
    }
  }
  */
  switch (STATE) {
    case IDLE:
      robot.update();
      //Switch to READY
      if (BTN_STATE(1)) {
        robot.init(PATH[0], PI/2);
        STATE = READY;
        LED_STATE();
      }
      break;
    case READY:
      robot.update();
      if (BTN_STATE(1)) {
        //zAngleOffset = zAngle;
        robot.init(PATH[0], PI/2);
        STATE = READY;
        LED_STATE();
      }
      if (BTN_STATE(0)) {
        robot.startPath();
        STATE = RUNNING;
        LED_STATE();
      }
      break;
    case RUNNING:
      robot.update();
      if (BTN_STATE(1)) {
        robot.init(PATH[0], PI/2);
        STATE = READY;
        LED_STATE();
      }
      if (BTN_STATE(0)) {
        robot.stopPath();
        STATE = STOPPED;
        LED_STATE();
      }
      if (robot.isNearTarget()) {
        robot.stopPath();
        STATE = STOPPED;
        LED_STATE;
      }
      break;
    case STOPPED:
      robot.update();
      if (BTN_STATE(1)) {
        robot.init(PATH[0], PI/2);
        STATE = READY;
        LED_STATE();
      }
      if (BTN_STATE(0)) {
        STATE = IDLE;
        LED_STATE();
      }
      break;
    default:
      STATE = IDLE;
      break;
  }
  
}

/*
//IMU SHIT
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  Serial2.write(p_data, uiSize);
  Serial2.flush();
}

static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
  int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
        s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
        s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
        s_cDataUpdate |= READ_UPDATE;
      break;
        }
    uiReg++;
    }
}

static void AutoScanSensor(void)
{
  int i, iRetry;
  
  for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
  {
    Serial2.begin(c_uiBaud[i]);
    Serial2.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do
    {
      WitReadReg(AX, 3);
      delay(200);
      while (Serial2.available())
      {
        WitSerialDataIn(Serial2.read());
      }
      if(s_cDataUpdate != 0)
      {
        Serial.print(c_uiBaud[i]);
        Serial.print(" baud find sensor\r\n\r\n");
        return ;
      }
      iRetry--;
    }while(iRetry);   
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}
*/
