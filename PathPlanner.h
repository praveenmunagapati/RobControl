#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "RobControl.h"
#include "trig.h"
#include "Misc.h"
#include "constants.h"
#include <math.h>

/* Declaration of common functions used for path planning purposes: path lengths, geometric calculations, vector calculus... */

float LineLength(float P1[6],float P2[6], int Size);
float LineLengthCart(float P1[6],float P2[6]);
float LineLengthAng(float P1[6],float P2[6], int Size);
float MinPathTime(float P1[6], float P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6]);
float VectorLength(float A[3]);
unsigned short CrossProduct(float U[3], float V[3], float (*N)[3]);
float DotProduct(float U[3], float V[3]);
float AngleBetweenVectors(float U[6], float V[6]);
unsigned short PointsToVector(float P1[6], float P2[6], float (*V)[3]);
unsigned short Normalize(float (*V)[3]);
unsigned short EvalCircle(Path_Type *Circle);
unsigned short MaxBlockSpeed(float d, float a, float j, float v_end, float *v_max);
unsigned short MaxMovementDynamics(float d, float a, float j, float v, float v_start, float v_end, float a_start, float *v_max, float *a_up_max, float *a_down_max, float *d_linear);
unsigned short EvaluateBezier (Point_Type P[5], float u, Point_Type *Q,int Size);
float BezierLength(Point_Type P[5], int Size);
float BezierLengthHalf1(Point_Type P[5], int Size);
float BezierLengthHalf2(Point_Type P[5], int Size);
unsigned short StoppingDistance(float v_max, float a_max, float j_max, float v_act, float a_act, float *stopping_distance);


#endif




