#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

using namespace std;
#include <Arduino.h>


#include <Arduino.h>


#include <float.h>
#include <stdint.h>
#include <math.h>

#include "utilities.h"
#include "config.h"
#include "state.h"


class HardwareInterface 
{

  public:
    Configuration *Config;
    State *state;

    void set_actuator_postions( void );
    void set_actuator_position( float joint_angle, int16_t axis, int16_t leg );
    void init_servos( void );
  private:

    void send_servo_commands( void );
    void send_servo_command( float joint_angle, int16_t axis,  int16_t leg );
    int32_t angle_to_duty_cycle( float joint_angle, int16_t axis_index, int16_t leg_index );
    int32_t pwm_to_duty_cycle(int32_t pulsewidth_micros );
    int32_t angle_to_pwm( float joint_angle, int16_t axis_index, int16_t leg_index );
    void set_PWM_duty_cycle( int16_t axis, int16_t leg, int32_t duty_cycle );
    int32_t __duty_cycle( int32_t m );
};  


#endif
