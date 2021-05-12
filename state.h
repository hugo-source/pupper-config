#ifndef STATE_H
#define STATE_H

using namespace std;
//#include <string.h>
#include <Arduino.h>


#include <float.h>
#include <stdint.h>
#include <math.h>

#include "utilities.h"

typedef enum BehaviorState
{
  DEACTIVATED = -1,
  REST = 0,
  TROT = 1,
  HOP = 2,
  FINISHHOP = 3,  
} BehaviorState;


class State 
{

  public:
    String debug_str;
    
    float horizontal_velocity[2] = { 0.0 };
    float yaw_rate;
    float height;
    float pitch;
    float roll;
    bool activation;
    BehaviorState behavior_state; 

    bool verbose = false;
    bool debug = false;

    uint32_t ticks;
    QuadLegs foot_locations;
    QuadLegs joint_angles;


  
    State(  );
    
  private:

};  


#endif
