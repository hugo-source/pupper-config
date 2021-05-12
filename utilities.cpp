/*

  Utilities
  
  EulerAnglesToMatrix
	
*/

#include "utilities.h"

//def deadband(value, band_radius):
//    return max(value - band_radius, 0) + min(value + band_radius, 0)


float clipped_first_order_filter( float input, float target, float max_rate, float tau )
{
    float rate = ( target - input ) / tau;
    rate = constrain( rate, -max_rate, max_rate );
    return rate;
};

void QuadLegs::GetLeg( int16_t LegIndex, LegArray Leg )
{
  for ( int i=0; i<3; i++ ) Leg[i] = Legs[i][LegIndex];  
};

void QuadLegs::PutLeg( int16_t LegIndex, const LegArray Leg )
{
  for ( int i=0; i<3; i++ ) Legs[i][LegIndex] = Leg[i]; 
};

void QuadLegs::GetAngles( float angles[3][4] )
{
  for ( int i=0; i<3; i++ )
    for ( int j=0; j<4; j++ )
      angles[i][j] = Legs[i][j] * 180 / M_PI;  
};

// euler to rotation matrix c++

void Euler2Mat(const EulerAngle &inEulerAngle, RotMatrix Mx, EEulerOrder EulerOrder)
{
    // Convert Euler Angles passed in a vector of Radians
    // into a rotation matrix.  The individual Euler Angles are
    // processed in the order requested.
    

    const float    Sx    = sinf(inEulerAngle.X);
    const float    Sy    = sinf(inEulerAngle.Y);
    const float    Sz    = sinf(inEulerAngle.Z);
    const float    Cx    = cosf(inEulerAngle.X);
    const float    Cy    = cosf(inEulerAngle.Y);
    const float    Cz    = cosf(inEulerAngle.Z);

    switch(EulerOrder)
    {
    case ORDER_XYZ:
        Mx[0][0]=Cy*Cz;
        Mx[0][1]=-Cy*Sz;
        Mx[0][2]=Sy;
        Mx[1][0]=Cz*Sx*Sy+Cx*Sz;
        Mx[1][1]=Cx*Cz-Sx*Sy*Sz;
        Mx[1][2]=-Cy*Sx;
        Mx[2][0]=-Cx*Cz*Sy+Sx*Sz;
        Mx[2][1]=Cz*Sx+Cx*Sy*Sz;
        Mx[2][2]=Cx*Cy;
        break;

    case ORDER_YZX:
        Mx[0][0]=Cy*Cz;
        Mx[0][1]=Sx*Sy-Cx*Cy*Sz;
        Mx[0][2]=Cx*Sy+Cy*Sx*Sz;
        Mx[1][0]=Sz;
        Mx[1][1]=Cx*Cz;
        Mx[1][2]=-Cz*Sx;
        Mx[2][0]=-Cz*Sy;
        Mx[2][1]=Cy*Sx+Cx*Sy*Sz;
        Mx[2][2]=Cx*Cy-Sx*Sy*Sz;
        break;

    case ORDER_ZXY:
        Mx[0][0]=Cy*Cz-Sx*Sy*Sz;
        Mx[0][1]=-Cx*Sz;
        Mx[0][2]=Cz*Sy+Cy*Sx*Sz;
        Mx[1][0]=Cz*Sx*Sy+Cy*Sz;
        Mx[1][1]=Cx*Cz;
        Mx[1][2]=-Cy*Cz*Sx+Sy*Sz;
        Mx[2][0]=-Cx*Sy;
        Mx[2][1]=Sx;
        Mx[2][2]=Cx*Cy;
        break;

    case ORDER_ZYX:
        Mx[0][0]=Cy*Cz;
        Mx[0][1]=Cz*Sx*Sy-Cx*Sz;
        Mx[0][2]=Cx*Cz*Sy+Sx*Sz;
        Mx[1][0]=Cy*Sz;
        Mx[1][1]=Cx*Cz+Sx*Sy*Sz;
        Mx[1][2]=-Cz*Sx+Cx*Sy*Sz;
        Mx[2][0]=-Sy;
        Mx[2][1]=Cy*Sx;
        Mx[2][2]=Cx*Cy;
        break;

    case ORDER_YXZ:
        Mx[0][0]=Cy*Cz+Sx*Sy*Sz;
        Mx[0][1]=Cz*Sx*Sy-Cy*Sz;
        Mx[0][2]=Cx*Sy;
        Mx[1][0]=Cx*Sz;
        Mx[1][1]=Cx*Cz;
        Mx[1][2]=-Sx;
        Mx[2][0]=-Cz*Sy+Cy*Sx*Sz;
        Mx[2][1]=Cy*Cz*Sx+Sy*Sz;
        Mx[2][2]=Cx*Cy;
        break;

    case ORDER_XZY:
        Mx[0][0]=Cy*Cz;
        Mx[0][1]=-Sz;
        Mx[0][2]=Cz*Sy;
        Mx[1][0]=Sx*Sy+Cx*Cy*Sz;
        Mx[1][1]=Cx*Cz;
        Mx[1][2]=-Cy*Sx+Cx*Sy*Sz;
        Mx[2][0]=-Cx*Sy+Cy*Sx*Sz;
        Mx[2][1]=Cz*Sx;
        Mx[2][2]=Cx*Cy+Sx*Sy*Sz;
        break;
    }
}

void PrintMatrix( float* A, int m, int n, String label)
{
  // A = input matrix (m x n)
  int i, j;
  Serial.println();
  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      Serial.print(A[n * i + j], 4);
      Serial.print("\t");
    }
    Serial.println();
  }
}
