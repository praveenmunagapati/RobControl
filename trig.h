#ifndef TRF_TRIG
#define TRF_TRIG

/* declaration of some useful trigonometric functions */

#define PI 3.14159265358979f

#define sind(x) sin((x) * PI / 180.0f)
#define cosd(x) cos((x) * PI / 180.0f)
#define tand(x) tan((x) * PI / 180.0f)

#define asind(x) 	(asin(x) * 180.0f / PI)
#define acosd(x) 	(acos(x) * 180.0f / PI)
#define atand(x) 	(atan(x) * 180.0f / PI)
#define atan2d(y,x) (atan2(y,x) * 180.0f / PI)

#define sqrt3 sqrtf(3.0f)
#define sin120 (sqrtf(3.0f) / 2.0f)
#define cos120 -0.5f  
#define tan60 sqrtf(3.0f)
#define sin30 0.5f
#define tan30 (1.0f / sqrtf(3.0f))

#define sign(a) ( (a<0)?-1:1 )

float Modulo2PI(float actual, float desired);
float ModuloPI(float actual, float desired);

#endif

