#ifndef ROBOTS_H
#define ROBOTS_H

#include <math.h>
#include "RobControl.h"
#include "trig.h"
#include "Frame.h"
#include "Misc.h"
#include "constants.h"

/* Declaration of direct and inverse transformations for all kinds of supported robots */

//6-Axis Robot
unsigned short ArmDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6]);
unsigned short ArmInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6]);

//Scara Robot
unsigned short ScaraDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6]);
unsigned short ScaraInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6]);

//Delta Robot
unsigned short DeltaDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6]);
unsigned short DeltaInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6]);

//Pallet Robot
unsigned short PalletDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6]);
unsigned short PalletInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6]);

#endif
