#include "trig.h"

float Modulo2PI(float actual, float desired)
{// add or subtract multiples of 2PI to the actual angle to bring it closer to the desired value
	float delta = (actual - desired);
	int n = ((int)delta + 180 * sign(delta)) / 360;
	return (actual - (360.0f * n));
}

float ModuloPI(float actual, float desired)
{// add or subtract multiples of PI to the actual angle to bring it closer to the desired value
	actual = Modulo2PI(actual,desired);
	float delta = (actual - desired);
	if (delta > 90)		actual -=180;
	if (delta < -90)	actual +=180;
	return actual;
}

