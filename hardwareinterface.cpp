/*

	HardwareInterface
	
*/

#include "hardwareinterface.h"

// leg 0 is front-right, 
// 1eg 1 is front-left, 
// leg 2 is back-right, 
// leg 3 is back-left.

  //  axis_index : int
  //      Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
  //  leg_index : int
  //      Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

void HardwareInterface::init_servos( void )
{
  Serial.println( "init_servos:" );
  // Set resolution 
  analogWriteResolution( 12 );

  for ( int leg_index=0; leg_index<4; leg_index++ ) {
    for ( int axis_index=0; axis_index<3; axis_index++ ) {
      analogWriteFrequency( Config->PWMparams.pins[axis_index][leg_index] , Config->PWMparams.freq );
      if ( true ) {
        Serial.printf( " axis=%d, leg=%d, pin=%d\n", axis_index, leg_index, Config->PWMparams.pins[axis_index][leg_index] );      
      };
    };
  };  
};

void HardwareInterface::set_actuator_position( float joint_angle, int16_t axis, int16_t leg )
{
  Serial.print( "radian=" ); Serial.println( joint_angle );
  Serial.print( "angle=" ); Serial.println( joint_angle * 180 / M_PI );
  send_servo_command( joint_angle, axis, leg );
};

int32_t HardwareInterface::__duty_cycle( int32_t m )
{
  // Servos won't respond to a duty cycle > 2100 
  m = constrain( m, 800, 2100 );
  int32_t dc = m * Config->PWMparams.range * Config->PWMparams.freq / 1e6;
  return dc; 
}

void HardwareInterface::set_PWM_duty_cycle( int16_t axis, int16_t leg, int32_t duty_cycle )
{
  analogWrite( Config->PWMparams.pins[axis][leg], __duty_cycle( duty_cycle ) );
  Serial.printf( "set_PWM_duty_cycle=%d\n", duty_cycle ); 
//  Serial.print( ", axis/leg=" ); Serial.print( axis ); Serial.println( leg ); 
};

void HardwareInterface::send_servo_command( float joint_angle, int16_t axis,  int16_t leg )
{
  int32_t duty_cycle = angle_to_duty_cycle( joint_angle, axis, leg );
  set_PWM_duty_cycle( axis, leg, duty_cycle );
};

void HardwareInterface::set_actuator_postions( void )
{
  float a[3][4];
  state->joint_angles.GetAngles( a );
  if ( state->verbose ) {
    for ( int leg_index=0; leg_index<4; leg_index++ ) {
      float joint_angles_a[3];
      for ( int axis_index=0; axis_index<3; axis_index++ ) {
        joint_angles_a[axis_index] = state->joint_angles.Legs[axis_index][leg_index] * 180 / M_PI;
        Serial.print( joint_angles_a[axis_index] ); Serial.print( " " );
      };
      Serial.println( "" );
    };
  };
  send_servo_commands( ); 
};

void HardwareInterface::send_servo_commands( void )
{
  for ( int16_t leg_index=0; leg_index<4; leg_index++ ) {
    for ( int16_t axis_index=0; axis_index<3; axis_index++ ) {
      int32_t duty_cycle = angle_to_duty_cycle(
                state->joint_angles.Legs[axis_index][leg_index],
                axis_index,
                leg_index );
      if ( state->verbose ) {
        Serial.print( "Leg " ); Serial.print( leg_index );
        Serial.print( ", Axis " ); Serial.print( axis_index );
        Serial.print( ", PWM " ); Serial.println( duty_cycle );
      };
      set_PWM_duty_cycle( axis_index, leg_index, duty_cycle );
    };
  };            
};

int32_t HardwareInterface::angle_to_duty_cycle( float joint_angle, int16_t axis_index, int16_t leg_index )
{
  return pwm_to_duty_cycle( angle_to_pwm( joint_angle, axis_index, leg_index ) );
  
};
/*
 *     """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin

    Parameters
    ----------
    pulsewidth_micros : float
        Width of the pwm signal in microseconds
    pwm_params : PWMParams
        PWMParams object

    Returns
    -------
    float
        PWM duty cycle corresponding to the pulse width
    """

 */
int32_t HardwareInterface::pwm_to_duty_cycle(int32_t pulsewidth_micros )
{
  return int32_t( pulsewidth_micros / 1e6 * Config->PWMparams.freq * Config->PWMparams.range );
};
/*
    """Converts a desired servo angle into the corresponding PWM command

    Parameters
    ----------
    angle : float
        Desired servo angle, relative to the vertical (z) axis
    servo_params : ServoParams
        ServoParams object
    axis_index : int
        Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
    leg_index : int
        Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

    Returns
    -------
    float
        PWM width in microseconds
    """ */
int32_t HardwareInterface::angle_to_pwm( float joint_angle, int16_t axis_index, int16_t leg_index )
{
  float angle_deviation = ( joint_angle - ( Config->ServoParams.neutral_angle_degrees[axis_index][leg_index] * M_PI / 180.0 ) ) * 
                            Config->ServoParams.servo_multipliers[axis_index][leg_index];
  float pulse_width_micros = ( Config->ServoParams.neutral_position_pwm + 
                               Config->ServoParams.micros_per_rad * angle_deviation );
  Serial.print( "angle_deviation=" ); Serial.println( angle_deviation );
  Serial.print( "PWM micros=" ); Serial.println( pulse_width_micros );
  
  return pulse_width_micros;
};
