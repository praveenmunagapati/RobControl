#ifndef TRF_H
#define TRF_H

#include <math.h>
#include "Robots.h"
#include "Misc.h"

/* Declaration of generic transformations function interface */

unsigned short Transformations(struct Mech_Type* Mechanics, unsigned char Mode, float JointAxes[6], float PathAxes[6], float Axes[6]);

#endif

