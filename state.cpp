/*

	State
	
*/

#include "state.h"

State::State(  )
{

  //horizontal_velocity
  yaw_rate = 0.0;
  height = -0.16;
  pitch = 0.0;
  roll = 0.0;
  activation = false;
  behavior_state = REST;

  ticks = 0;
  //foot_locations
  //joint_angles

};
