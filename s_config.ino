/*
 * 
 * s_config
 * An interpretation in c++ of the Stanford Pupper Python software by Nathan Kau
 * and others.
 * 
 * Hugh Maclaurin 12.May.2021
 * 
 * Version
 * 1.0    2021/05/12
 *  Teensy 4.0
 * 
 * 
*/

#include <math.h>

#include "config.h"
#include "hardwareinterface.h"
#include "state.h"

Configuration Config;
State state;
HardwareInterface hardware_interface;


String comdata = "";
float set_points[3][4] = { {0, 0, 0, 0}, {45, 45, 45, 45}, {45, 45, 45, 45} };

const char motor_type[3][16] = { "abduction", "inner", "outer" };
const char leg_pos[4][16] = { "front-right", "front-left", "back-right", "back-left" };
 

void setup() {
  Serial.begin(115200); // usb serial
  digitalWrite( LED_BUILTIN, HIGH );
  delay( 1000 );
  hardware_interface.state = &state;
  hardware_interface.Config = &Config;
  hardware_interface.init_servos();

  Serial.println( "The scaling constant for your servo represents how much you have to increase" );
  Serial.println( "the pwm pulse width (in microseconds) to rotate the servo output 1 degree." );
  Serial.print( "This value is currently set to: " ); Serial.println( degrees_to_radians( Config.ServoParams.micros_per_rad ), 3 );
  Serial.println("For newer CLS6336 and CLS6327 servos the value should be 11.333.");

  Serial.println( "Press RETURN to start calibration." );
  Serial.println( "" ); 
};

void loop() {
  char motor_descrip[64];
     
  // read string from serial monitor
  if ( Serial.available() ) {
    comdata = Serial.readStringUntil( '\n' );
    //Serial.println("comdata=" + comdata );
    char cmd = comdata.charAt( 0 );
    if ( cmd == 'x' ) {
      Serial.println( "Exit.." );
      while( true );
    };
    for ( int leg_index=0; leg_index<4; leg_index++ ) {
      for ( int axis=0; axis<3; axis++ ) {
        // Loop until we're satisfied with the calibration
        bool completed = false;
        while ( not completed ) {
          sprintf( motor_descrip, "Calibrating the **%s %s motor",  motor_type[axis], leg_pos[leg_index] );
          Serial.println( motor_descrip );
          
          float set_point = set_points[axis][leg_index];
          // Zero out the neutral angle
          Config.ServoParams.neutral_angle_degrees[axis][leg_index] = 0;

          // Move servo to set_point angle
          hardware_interface.set_actuator_position( degrees_to_radians( set_point ), axis, leg_index );

          // Adjust the angle using keyboard input until it matches the reference angle
          float offset = step_until( axis, leg_index, set_point );

          Serial.print( "Final offset: " ); Serial.println( offset );

          // The upper leg link has a different equation because we're calibrating to make it horizontal, not vertical
          if ( axis == 1 ) {
            Config.ServoParams.neutral_angle_degrees[axis][leg_index] = set_point - offset;
          } else {
            Config.ServoParams.neutral_angle_degrees[axis][leg_index] = -(set_point + offset);
          };
          Serial.print( "Calibrated neutral angle: " ); 
          Serial.println( Config.ServoParams.neutral_angle_degrees[axis][leg_index] );

          // Send the servo command using the new beta value and check that it's ok
          float n[3] = { 0, 45, -45 };
          const char txt[3][16] = { "horizontal", "45 degrees", "45 degrees" };
          char prompt[32];
          hardware_interface.set_actuator_position( degrees_to_radians( n[axis] ), axis, leg_index );
          sprintf( prompt, "The leg should be at exactly **%s**. Are you satisfied? Enter 'y' or 'n to repeat'", txt[axis] );
          Serial.println( prompt );  
          while ( not Serial.available() ); //wait
          comdata = Serial.readStringUntil( '\n' );
          char cmd = comdata.charAt( 0 );
          if ( cmd == 'y' ) completed = true;
        }; // end while
      }; // end for
    }; // end for
    Serial.println( "Finished calibration." ); // response
    PrintMatrix( (float*)Config.ServoParams.neutral_angle_degrees, 3, 4, "neutral_angle_degrees" );
  };
};
    
/*
 *     """Returns the angle offset needed to correct a given link by asking the user for input.

    Returns
    -------
    Float
        Angle offset needed to correct the link.
    """
*/
float step_until( int16_t axis, int16_t leg, float set_point )
{
  char set_names[3][16] = { "horizontal", "horizontal", "vertical" };
  char move_input[80];
  float offset = 0;
  while ( true ) {
    sprintf( move_input, "Enter 'a' or 'b' to move the link until it is **%s**. Enter 'd' when done.", set_names[axis] );
    Serial.println( move_input );  
    while ( not Serial.available() ); //wait
    comdata = Serial.readStringUntil( '\n' );
    char cmd = comdata.charAt( 0 );
    if ( cmd == 'a' ) {
      offset += 1.0;
      hardware_interface.set_actuator_position( degrees_to_radians( set_point + offset ), axis, leg );
    };
    if ( cmd == 'b' ) {
      offset -= 1.0;
      hardware_interface.set_actuator_position( degrees_to_radians( set_point + offset ), axis, leg );
    };
    if ( cmd == 'd' ) {
      Serial.print( "Offset: " ); Serial.println( offset );
      break;
    };
  };
  return offset;
};

    

float degrees_to_radians( float input )
{
   return input * M_PI / 180.0;  
}
float radians_to_degrees( float input )
{
  return input * 180.0 / M_PI;
}
