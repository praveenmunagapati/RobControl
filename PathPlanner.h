#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "RobControl.h"
#include "trig.h"
#include "Misc.h"
#include "constants.h"
#include <math.h>

#define EDGE_END 0
#define EDGE_START 1

/* Declaration of common functions used for path planning purposes: path lengths, geometric calculations, vector calculus... */

float LineLength(float P1[6],float P2[6], int Size);
float LineLengthCart(float P1[6],float P2[6]);
float LineLengthAng(float P1[6],float P2[6], int Size);
float MinPathTime(float P1[6], float P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6]);
float VectorLength(float A[3]);
unsigned short CrossProduct(float U[3], float V[3], float N[3]);
float DotProduct(float U[3], float V[3]);
float AngleBetweenVectors(float U[6], float V[6]);
unsigned short PointsToVector(float P1[6], float P2[6], float V[3]);
unsigned short Normalize(float V[3]);
unsigned short EvalCircle(Path_Type *Circle);
unsigned short MaxBlockSpeed(float d, float a, float j, float v_end, float *v_max);
unsigned short MaxMovementDynamics(float d, float a, float j, float v, float v_start, float v_end, float a_start, float *v_max, float *a_up_max, float *a_down_max, float *d_linear);
unsigned short EvaluateBezier (Point_Type P[5], float u, Point_Type *Q,int Size, int Order);
float BezierLength(Point_Type P[5], int Size, int Order);
float BezierLengthHalf1(Point_Type P[5], int Size, int Order);
float BezierLengthHalf2(Point_Type P[5], int Size, int Order);
unsigned short StoppingDistance(float v_max, float a_max, float j_max, float v_act, float a_act, float *stopping_distance);
unsigned short DynamicLimitsViolated(float P1[6], float P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6], float CycleTime, float *redFactor);
unsigned short LineCrossBox(float L1[6],float L2[6],float B1[6],float B2[6]);
unsigned short PointInBox(float P[6],float B1[6],float B2[6]);
unsigned short WorkspaceMonitor(unsigned char MovementType, Path_Type* Path, float Tool[6], Robot_Parameter_Workspace_Type Workspace[MAX_ZONE], unsigned short AxesNum, Mech_Type* Mechanics);
unsigned short RoundEdgePoints(MotionPackage_Type* Movement, MotionPackage_Type* MovementPrev, unsigned short AxesNum, Mech_Type* Mechanics);
float PTPLength(Path_Type* Path, unsigned short AxesNum, Mech_Type* Mechanics);
#endif
