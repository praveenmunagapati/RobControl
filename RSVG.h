#ifndef R_SVG_H
#define R_SVG_H

#include "RobControl.h"
#include "constants.h"
#include "PathPlanner.h"
#include <math.h>

/* Declaration of set value generator datatype and function block */

typedef struct RSVG_Type
{
    float TargetPosition;
    float StartPosition;
    float StartSpeed;
    float EndSpeed;
    float StartAcc;	
    struct Robot_Parameter_JointLimits_Type DynamicLimits;
    struct Robot_Parameter_JointLimits_Type DynamicValues;
    float Cycletime;
    float Override;
    float Position;
    float Speed;
    float Acceleration;
    unsigned short Status;
    unsigned char State;
    unsigned char Phase;
    signed char moveDirection;
    float beginPosition;
    float beginSpeed;
    float beginAcc;
    float endPosition;
    float elapsedTime;
    float ds;
    float dt[7];
    float delta;
    float epsilon;
    float v;
    float v1;
    float v2;
    float a0;
    float a1;
    float j;
    float astart_up;
    float astart_down;
    unsigned short Enable;
    unsigned short Start;
    unsigned short Stop;
    unsigned short EStop;
    unsigned short Done;
    unsigned short endLimits;
    float RedFactor;
} RSVG_Type;

void RSVG(struct RSVG_Type* inst);

#endif

