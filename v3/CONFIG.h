//PINOUT
#define MOTOR_R_1 27
#define MOTOR_R_2 13
#define MOTOR_L_1 25
#define MOTOR_L_2 26

#define ENCODER_R_1 36 
#define ENCODER_R_2 39
#define ENCODER_L_1 34
#define ENCODER_L_2 33

#define BT1 21
#define BT2 18

#define GRN 23
#define RED 22

//ROBOT DIMENSIONS
//in mm
#define WIDTH_TRACK 114.09
#define RADIUS_WHEEL 16

//MOTOR TUNING VALUES
#define MOTOR_R_KP 0.0105*pow(10, 7)
#define MOTOR_R_KI 0.01*pow(10, 1)
#define MOTOR_R_KD 0.01*pow(10, 5)
#define MOTOR_R_F 0.268*pow(10, 7)

#define MOTOR_L_KP 0.0105*pow(10, 7)
#define MOTOR_L_KI 0.01*pow(10, 1)
#define MOTOR_L_KD 0.01*pow(10, 5)
#define MOTOR_L_F 0.268*pow(10, 7)

#define MOTOR_FILTER_FREQ 2.5

#define MOTOR_INTERVAL_US (uint32_t)12*pow(10, 3)
#define TICKS_PER_REV 1200

//ODOMETRY TUNING VALUES
#define ODOMETRY_INTERVAL_US (uint32_t)20*pow(10, 3)
#define STARTING_POSE Vector3d(0, 0, 0)
//#define STARTING_THETA 0

//PATH GENERATION PARAMETERS
#define PATH_RES 250
#define STEP_RES 1

#define INVALID_P -2023.0

//PURE PURSUIT TUNING VALUES
//in mm
#define PURE_PURSUIT_LOOKAHEAD 50
#define PURE_PURSUIT_KP_K 0.0075
#define PURE_PURSUIT_MAX_AV 5
#define PURE_PURSUIT_INTERVAL_US (uint32_t)10*pow(10, 3)

#define MIN_END_DIST 20

#define ANG_VEL_FILTER_FREQ 1

//IMU TUNING VALUES
#define IMU_POLL_INTERVAL_US 0
#define IMU_VEL_HIGHPASS_FREQ 0
#define IMU_POSE_HIGHPASS_FREQ 0

//STATE DEFINITIONS
#define INIT 0
#define IDLE 1
#define READY 2
#define RUNNING 3
#define TUNING 4
