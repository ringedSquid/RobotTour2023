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
#define WIDTH_TRACK 113.2
#define RADIUS_WHEEL 16

//MOTOR TUNING VALUES
#define MOTOR_R_KP 0.4*pow(10, 7)
#define MOTOR_R_KI 0.6*pow(10, 7)
#define MOTOR_R_KD 0*pow(10, 2)
#define MOTOR_R_F 0*pow(10, 7)

#define MOTOR_L_KP 0.4*pow(10, 7)
#define MOTOR_L_KI 0.6*pow(10, 7)
#define MOTOR_L_KD 0*pow(10, 2)
#define MOTOR_L_F 0*pow(10, 7)

#define MOTOR_FILTER_FREQ 2.5

#define MOTOR_INTERVAL_US (uint32_t)12*pow(10, 3)
#define TICKS_PER_REV 1200

//ODOMETRY TUNING VALUES
#define ODOMETRY_INTERVAL_US (uint32_t)10*pow(10, 3)
//#define STARTING_POSE Vector3d(0, 0, 0)
//#define STARTING_THETA 0

//ROBOT CONTROLLER TUNING VALUES
#define MAX_ANG_VEL 5 //Rad/s
#define CONTROLLER_INTERVAL_US 0

#define POSE_KP 3.1
#define POSE_KI 0.2
#define POSE_KD 0
#define POSE_F  0

//SIMPLE PURSUIT TUNING VALUES 
#define CHECKRS 5.0 //all in mm
#define TRAFDIST 5.0

//ROBOT TUNING VALUES
#define TURN_INTERVAL_US 1500*pow(10, 3)
#define CENTER_TO_DOWEL 80 //mm
#define END_DISTANCE 10.0 //mm

//IMU TUNING VALUES
#define IMU_POLL_INTERVAL_US 1*pow(10, 3)
#define IMU_VEL_HIGHPASS_FREQ 0
#define IMU_POSE_HIGHPASS_FREQ 0

//STATE DEFINITIONS
#define INIT 0
#define IDLE 1
#define READY 2
#define RUNNING 3
#define STOPPED 4
