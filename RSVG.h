#ifndef R_SVG_H
#define R_SVG_H

#include "RobControl.h"
#include "constants.h"
#include "PathPlanner.h"
#include <math.h>

/* Declaration of set value generator datatype and function block */

typedef struct RSVG_Type
{
    double TargetPosition;
    double StartPosition;
    double StartSpeed;
    double EndSpeed;
    double StartAcc;	
    struct Robot_Parameter_JointLimits_Type DynamicLimits;
    struct Robot_Parameter_JointLimits_Type DynamicValues;
    double Cycletime;
    double Override;
    double Position;
    double Speed;
    double Acceleration;
    unsigned short Status;
    unsigned char State;
    unsigned char Phase;
    signed char moveDirection;
    double beginPosition;
    double beginSpeed;
    double beginAcc;
    double endPosition;
    double elapsedTime;
    double ds;
    double dt[7];
    double delta;
    double epsilon;
    double v;
    double v1;
    double v2;
    double a0;
    double a1;
    double j;
    double astart_up;
    double astart_down;
    unsigned short Enable;
    unsigned short Start;
    unsigned short Stop;
    unsigned short EStop;
    unsigned short Done;
    unsigned short endLimits;
    double RedFactor;
} RSVG_Type;

void RSVG(struct RSVG_Type* inst);

#endif

