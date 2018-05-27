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

double LineLength(double P1[6],double P2[6], int Size);
double LineLengthCart(double P1[6],double P2[6]);
double LineLengthAng(double P1[6],double P2[6], int Size);
double MinPathTime(double P1[6], double P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6]);
double VectorLength(double A[3]);
unsigned short CrossProduct(double U[3], double V[3], double N[3]);
double DotProduct(double U[3], double V[3]);
double AngleBetweenVectors(double U[6], double V[6]);
unsigned short PointsToVector(double P1[6], double P2[6], double V[3]);
unsigned short Normalize(double V[3]);
unsigned short EvalCircle(Path_Type *Circle);
unsigned short MaxBlockSpeed(double d, double a, double j, double v_end, double *v_max);
unsigned short MaxMovementDynamics(double d, double a, double j, double v, double v_start, double v_end, double a_start, double *v_max, double *a_up_max, double *a_down_max, double *d_linear);
unsigned short EvaluateBezier (Frame_Type P[5], double u, Frame_Type *Q,int Size, int Order);
double BezierLength(Frame_Type P[5], int Size, int Order);
double BezierLengthHalf1(Frame_Type P[5], int Size, int Order);
double BezierLengthHalf2(Frame_Type P[5], int Size, int Order);
unsigned short StoppingDistance(double v_max, double a_max, double j_max, double v_act, double a_act, double *stopping_distance);
unsigned short DynamicLimitsViolated(double P1[6], double P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6], double CycleTime, double *redFactor);
unsigned short LineCrossBox(double L1[6],double L2[6],double B1[6],double B2[6]);
unsigned short PointInBox(double P[6],double B1[6],double B2[6]);
unsigned short WorkspaceMonitor(unsigned char MovementType, Path_Type* Path, double Tool[6], Robot_Parameter_Workspace_Type Workspace[MAX_ZONE], unsigned short AxesNum, Mech_Type* Mechanics);
unsigned short RoundEdgePoints(MotionPackage_Type* Movement, MotionPackage_Type* MovementPrev, unsigned short AxesNum, Mech_Type* Mechanics);
double PTPLength(Path_Type* Path, unsigned short AxesNum, Mech_Type* Mechanics);
#endif
