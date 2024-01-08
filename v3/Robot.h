#ifndef Robot_h
#define Robot_h

#include "SimplePursuit.h"
#include "controller.h"


class Robot {
  private:
    SimplePursuit *simplePursuit
    Controller *controller //robot controller
    
    //0 for idle
    //1 for turning
    //2 for path following
    byte STATE;
    
    double target_t;
    double total_d;

    double current_d;
    uint32_t start_us;

    //for measuring turning times
    uint32_t buffer_us;
    uint32_t turn_us;
    
  public:
    Robot( 
      SimplePursuit *iSimplePursuit,
      Controller *iController,
      double iTotal_d, double iTarget_t,
      uint32_t iTurn_us
      );
      
    void init();
    void update();
    void startPath();
    void stopPath();
}


#endif
