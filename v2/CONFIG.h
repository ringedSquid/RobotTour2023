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
#define MOTOR_R_KP 3.3*pow(10, 7)
#define MOTOR_R_KI 0.95*pow(10, 7)
#define MOTOR_R_KD 0.01*pow(10, 7)

#define MOTOR_L_KP 3.3*pow(10, 7)
#define MOTOR_L_KI 0.95*pow(10, 7)
#define MOTOR_L_KD 0.01*pow(10, 7)

#define MOTOR_INTERVAL_US (uint32_t)10*pow(10, 3)
#define TICKS_PER_REV 652

//ODOMETRY TUNING VALUES
#define ODOMETRY_INTERVAL_US (uint32_t)20*pow(10, 3)
#define STARTING_POSE Vector3d(0, 0, 0)
#define STARTING_THETA 0

//PURE PURSUIT TUNING VALUES
//in mm
#define PURE_PURSUIT_LOOKAHEAD 50
#define PURE_PURSUIT_KP 0.5
#define PURE_PURSUIT_MAX_AV 5
#define PURE_PURSUIT_INTERVAL_US (uint32_t)10*pow(10, 3)

#define MIN_END_DIST 30

//IMU TUNING VALUES
#define IMU_POLL_INTERVAL_US 0
#define IMU_VEL_HIGHPASS_FREQ 0
#define IMU_POSE_HIGHPASS_FREQ 0

//STATE DEFINITIONS
#define INIT 0
#define IDLE 1
#define READY 2
#define RUNNING 3
