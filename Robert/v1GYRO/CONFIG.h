//Hardware connections

//Buttons for input
#define BTN_0 0
#define BTN_1 0

//Motors
#define STEP_L 26
#define DIR_L 25
#define FAULT_L 32

#define STEP_R 13
#define DIR_R 27
#define FAULT_R 33

#define STEPS_PER_REV 51200

//Oled
#define OLED_ADD 0x3C
#define OLED_RST -1

//LEDS
#define LED_0 16
#define LED_1 17

//SD Card
#define SD_CS 5

//oled
#define I2C_ADDRESS 0x3C

//BMI160 IMU
#define IMU_ADDRESS 0x68
#define IMU_UPDATE_US 0

//Robot physical dimensions
//in mm
#define WHEEL_RADIUS 40.46732
#define TRACK_WIDTH 146.5
#define DIST_TO_DOWEL 0

//Controller tuning values
#define MAX_ACCEL 200
#define MAX_ANG_VEL 200
#define TURN_US 1000*pow(10, 3)


//PATH INFO
#define PATH_FILE "/test.txt"

//STATE DEFINITIONS
#define CALIBRATE -2
#define INIT -1
#define IDLE 0
#define READY 1
#define RUNNING 2
#define END_RUN 3
#define STOPPED 4
#define ERROR 5
