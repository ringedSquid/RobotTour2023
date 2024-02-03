#ifndef robot_h
#define robot_h

#include "simplePursuit.h"
#include "controller.h"

class robot {
  private:
    simplePursuit *robotSimplePursuit;
    controller *robotController; //robot controller
    
    //0 for idle
    //1 for deciding forward
    //2 for moving
    //3 for deciding turn
    //4 for turning
    uint8_t STATE;
    uint32_t start_us;

    //getting distance from end point
    double centerToDowel;
    double finalOffset;

    double maxAx;
    double maxAngVx;
    
  public:
    robot( 
      simplePursuit *iSimplePursuit, controller *iController,
      double iMaxAx, double iMaxAngVx,
      double iCenterToDowel
      );
      
    void init();
    void init(double iFinalOffset);
    void update();
    void startPath();
    double stopPath();

    uint8_t getState();

};


#endif
