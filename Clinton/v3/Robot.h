#ifndef Robot_h
#define Robot_h

#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "SimplePursuit.h"
#include "controller.h"
#include "odometry.h"
//#include "imu.h"

class Robot {
  private:
    SimplePursuit *simplePursuit;
    Controller *controller; //robot controller
    Odometry *odometry;
    //IMU *imu;
    
    //0 for idle
    //1 for turning
    //2 for path following
    byte STATE;
    
    double target_t;
    uint32_t start_us;

    //for measuring turning times
    uint32_t buffer_us;
    uint32_t turn_us;

    //getting distance from end point
    double centerToDowel;
    double endDistance;
    double totalTurnDelay;
    
  public:
    Robot( 
      SimplePursuit *iSimplePursuit,
      Controller *iController,
      Odometry *iOdometry,
      //IMU *iimu,
      double iTarget_t,
      double iCenterToDowel,
      double iEndDistance,
      uint32_t iTurn_us
      );
      
    void init(Vector2d iPose, double iTheta);
    void update();
    void startPath();
    void stopPath();

    bool isNearTarget();

};


#endif
