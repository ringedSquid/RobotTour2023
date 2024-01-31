#ifndef robot_h
#define robot_h

#include "simplePursuit.h"
#include "controller.h"

class robot {
  private:
    simplePursuit *robotSimplePursuit;
    controller *robotController; //robot controller
    
    //0 for idle
    //1 for deciding
    //2 for moving
    uint8_t STATE;
    uint32_t start_us;

    //getting distance from end point
    double centerToDowel;

    double maxAx;
    double maxAngVx;
    
  public:
    robot( 
      simplePursuit *iSimplePursuit, controller *iController,
      double iMaxAx, double iMaxAngVx,
      double iCenterToDowel
      );
      
    void init();
    void update();
    void startPath();
    void stopPath();

};


#endif
