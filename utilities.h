#ifndef UTILITIES_H
#define UTILITIES_H

#include <math.h>
#include <Arduino.h>

float clipped_first_order_filter( float input, float target, float max_rate, float tau );

// 3 row X 4 cols/legs array
// Each col is a vector of abduction/hip/leg
typedef float LegsArray[3][4]; 
typedef float LegArray[3];

class QuadLegs
{
  public:
    LegsArray Legs = { { 0.0 } };
    void GetLeg( int16_t LegIndex, LegArray Leg ); // get leg column
    void PutLeg( int16_t LegIndex, const LegArray Leg ); // save a leg column
    void GetAngles( float angles[3][4] );
  private:
};

// euler to rotation matrix c++
typedef float RotMatrix[3][3];
struct EulerAngle { float X,Y,Z; };

// Euler Order enum.
enum EEulerOrder
{
    ORDER_XYZ,
    ORDER_YZX,
    ORDER_ZXY,
    ORDER_ZYX,
    ORDER_YXZ,
    ORDER_XZY
};


void Euler2Mat(const EulerAngle &inEulerAngle, RotMatrix Mx, EEulerOrder EulerOrder);

void PrintMatrix( float* A, int m, int n, String label);


#endif
