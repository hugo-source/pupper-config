#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>
#include "utilities.h"

class Configuration {

  public:

  struct {
//    int16_t pins[3][4] = {
//      {2, 14, 18, 23}, 
//      {3, 15, 27, 24}, 
//      {4, 17, 22, 25}
//    };
    int16_t pins[3][4] = {
      {0, 3, 6, 23}, 
      {1, 4, 7, 22}, 
      {2, 5, 8, 9}
    };
    int32_t range = 4000;
    int16_t freq = 250;
  } PWMparams;

  // MICROS_PER_RAD and NEUTRAL_ANGLE_DEGREES should be determined by calibration
  #define MICROS_PER_RAD ( 11.333 * 180.0 / M_PI )
  const float NEUTRAL_ANGLE_DEGREES[3][4] = {
    {  0.0,  0.0,  0.0,  0.0},
    { 45.0, 45.0, 45.0, 45.0},
    {-45.0,-45.0,-45.0,-45.0}
  };

  struct {
    int16_t neutral_position_pwm = 1500;  // Middle position
    float micros_per_rad = MICROS_PER_RAD; // Must be calibrated

    // The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
//    float neutral_angle_degrees[3][4] = {
//      {  0.0,  0.0,  0.0,  0.0},
//      { 45.0, 45.0, 45.0, 45.0},
//      {-45.0,-45.0,-45.0,-45.0}
//    };
    float neutral_angle_degrees[3][4] = {
      {  7.0,  -6.0,  1.0,  4.0},
      { 44.0, 43.0, 46.0, 39.0},
      {-48.0,-44.0,-53.0,-49.0}
    };

    int16_t servo_multipliers[3][4] {
      {1, 1, 1, 1}, 
      {-1, 1, -1, 1},
      {1, -1, 1, -1}
    };
  } ServoParams;

  // The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
  void neutral_angle_degrees( float ang[3][4] ) {
    for ( int i=0; i<3; i++ )
      for ( int j=0; j<4; j++ )
        ang[i][j] = NEUTRAL_ANGLE_DEGREES[i][j];  
    return;
  };
  void leg_neutral_angle_degrees( int16_t LegIndex, LegArray Leg )
  {
    for ( int i=0; i<3; i++ ) Leg[i] = NEUTRAL_ANGLE_DEGREES[i][LegIndex];  
  };

  void neutral_angles( float rads[3][4] ) {
    neutral_angle_degrees( rads );
    for ( int i=0; i<3; i++ )
      for ( int j=0; j<4; j++ )
        rads[i][j] = rads[i][j] * M_PI / 180.0;  // Convert to radians
    return;
  };
  void leg_neutral_angles( int16_t LegIndex, LegArray Leg )
  {
    for ( int i=0; i<3; i++ ) Leg[i] = NEUTRAL_ANGLE_DEGREES[i][LegIndex] * M_PI / 180.0;  // Convert to radians  
  };

  struct {        
    const float max_x_velocity = 0.4;
    const float max_y_velocity = 0.3;
    const float max_yaw_rate = 2.0;
    const float max_pitch = 30.0 * M_PI / 180.0;
        
    const float max_roll = 30.0 * M_PI / 180.0;

    //#################### MOVEMENT PARAMS ####################
    const float z_time_constant = 0.02;
    const float z_speed = 0.03;  // maximum speed [m/s]
    const float pitch_deadband = 0.02;
    const float pitch_time_constant = 0.25;
    const float max_pitch_rate = 0.15;
    const float roll_speed = 0.16;  // maximum roll rate [rad/s]
    const float yaw_time_constant = 0.3;
    const float max_stance_yaw = 1.2;
    const float max_stance_yaw_rate = 2.0;

    //#################### STANCE ####################
    const float delta_x = 0.1;
    const float delta_y = 0.09;
    const float x_shift = 0.0;
    const float default_z_ref = -0.16;

    //#################### SWING ######################
    const float z_coeffs = 0;
    const float z_clearance = 0.07;
    const float alpha = 0.5;  // Ratio between touchdown distance and total horizontal stance movement
    const float beta =  0.5;  // Ratio between touchdown distance and total horizontal stance movement
 
    //#################### GAIT #######################
    const float dt = 0.01;
//    const float dt = 0.5;
    const int16_t num_phases = 4;
    const int16_t contact_phases[4][4] = {
      {1, 1, 1, 0}, 
      {1, 0, 1, 1}, 
      {1, 0, 1, 1}, 
      {1, 1, 1, 0},
    };

    const float overlap_time = 0.10;  // duration of the phase where all four feet are on the ground
    const float swing_time = 0.15;  // duration of the phase when only two feet are on the ground

    //######################## GEOMETRY ######################
    const float LEG_FB = 0.10;  // front-back distance from center line to leg axis
    const float LEG_LR = 0.04;  // left-right distance from center line to leg plane
    const float LEG_L2 = 0.115;
    const float LEG_L1 = 0.1235;
    const float ABDUCTION_OFFSET = 0.03;  // distance from abduction axis to leg
    const float FOOT_RADIUS = 0.01;
    const float HIP_L = 0.0394;
    const float HIP_W = 0.0744;
    const float HIP_T = 0.0214;
    const float HIP_OFFSET = 0.0132;

    const float L = 0.276;
    const float W = 0.100;
    const float T = 0.050;

    const float LEG_ORIGINS[3][4] = {
      {LEG_FB, LEG_FB, -LEG_FB, -LEG_FB},
      {-LEG_LR, LEG_LR, -LEG_LR, LEG_LR},
      {0, 0, 0, 0},
    };

    const float ABDUCTION_OFFSETS[4] = {
      -ABDUCTION_OFFSET, ABDUCTION_OFFSET, -ABDUCTION_OFFSET, ABDUCTION_OFFSET};
 
    //################### INERTIAL ####################
    const float FRAME_MASS = 0.560;  // kg
    const float MODULE_MASS = 0.080;  // kg
    const float LEG_MASS = 0.030;  // kg
    const float MASS = FRAME_MASS + ( MODULE_MASS + LEG_MASS ) * 4;

    // Compensation factor of 3 because the inertia measurement was just
    // of the carbon fiber and plastic parts of the frame and did not
    // include the hip servos and electronics
    /*FRAME_INERTIA = tuple(
            map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3))
        )
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)

        leg_z = 1e-6
        leg_mass = 0.010
        leg_x = 1 / 12 * self.LEG_L1 ** 2 * leg_mass
        leg_y = leg_x
        self.LEG_INERTIA = (leg_x, leg_y, leg_z)
  
    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.x_shift,
                    self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        ) */
    float default_stance[3][4] = { {delta_x + x_shift, delta_x + x_shift, -delta_x + x_shift,-delta_x + x_shift,},
                                   {-delta_y, delta_y, -delta_y, delta_y,},
                                   {0, 0, 0, 0,},
                                 };
  } ConfigParams; 

  //########################### GAIT ####################
  int overlap_ticks( void ) { return trunc( float( ConfigParams.overlap_time / ConfigParams.dt ) ); };

  int swing_ticks( void ) { return trunc( float( ConfigParams.swing_time /  ConfigParams.dt ) ); };

  int stance_ticks( void ) { return ( 2 * overlap_ticks() + swing_ticks() ); };

  int phase_ticks( int phase_t ) {
    switch ( phase_t ) {
      case 0 : return overlap_ticks();
      case 1 : return swing_ticks();
      case 2 : return overlap_ticks();
      case 3 : return swing_ticks();
    };
    return false;
  };

  int phase_length( void ) { return ( 2 * overlap_ticks() + 2 * swing_ticks() ); };


}; // end class

#endif
