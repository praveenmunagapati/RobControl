#ifndef TRF_TRIG
#define TRF_TRIG

#include <math.h>

/* declaration of some useful trigonometric functions */

#define PI 3.14159265358979

#define sind(x) sin((x) * PI / 180.0)
#define cosd(x) cos((x) * PI / 180.0)
#define tand(x) tan((x) * PI / 180.0)

#define asind(x) 	(asin(x) * 180.0 / PI)
#define acosd(x) 	(acos(x) * 180.0 / PI)
#define atand(x) 	(atan(x) * 180.0 / PI)
#define atan2d(y,x) (atan2(y,x) * 180.0 / PI)

#define sqrt3 sqrt(3.0)
#define sin120 (sqrt(3.0) / 2.0)
#define cos120 -0.5 
#define tan60 sqrt(3.0)
#define sin30 0.5
#define tan30 (1.0 / sqrt(3.0))

#define sign(a) ( (a<0)?-1:1 )

double Modulo2PI(double actual, double desired);
double ModuloPI(double actual, double desired);

#endif

