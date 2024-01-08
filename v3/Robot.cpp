#include "Robot.h"
#include "SimplePursuit.h"
#include "controller.h"

Robot::Robot( 
      SimplePursuit *iSimplePursuit,
      Controller *iController,
      double iTotal_d, double iTarget_t,
      uint32_t iTurn_us;
      )
{
  simplePursuit = iSimplePursuit;
  controller = iController;
  total_d = iTotal_d;
  target_t = iTarget_t;
  turn_us = iTurn_us;
}

void Robot::init() {
  STATE = 0;
  current_d = 0;
  controller->disable();
}

void Robot::update() {
  controller->update();
  switch (STATE) {
    case 0:
      break;
    case 1:
      simplePursuit->updateVx();
      controller->setTargetVx(simplePursuit->getVx());
      controller->setTargetTheta(simplePursuit->getTheta());
      if (simplePursuit->atPoint()) {
        //check if end of path
        if (simplePursuit->nextPoint()) {
          controller->setTargetVx(0);
           STATE = 2;
           buffer_us = micros();
        }
        else {
          STATE = 0;
        }
      }
      break;
    case 2:
      controller->setTargetTheta(simplePursuit->getTheta());
      if (micros() - buffer_us > turn_us) {
        STATE = 1;
      }
      break;
    case 3:
      break;
   }
}

void Robot::startPath() {
  STATE = 1;
  start_us = micros();
}

void Robot::stopPath() {
  STATE = 0;
}
