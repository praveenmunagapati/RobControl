#ifndef FRAME_H
#define FRAME_H

#include <math.h>
#include "constants.h"
#include "trig.h"

/* quaternion datatype */
typedef struct Quat_Type
{	
	float w,x,y,z;
} Quat_Type;


/* Declaration of common functions used for frames operations: translations, rotations... */

unsigned short ComposeMatrix(float RM[3][3], float A, float B, float C);
unsigned short DecomposeMatrix(float RM[3][3], float A_actual, float B_actual, float C_actual, float *A, float *B, float *C);
unsigned short MatMult(float M1[3][3],float M2[3][3],float M3[3][3]);
unsigned short SubFrame3D(float P1[6],float F1[6], float A_actual, float B_actual, float C_actual, float P0[6]);
unsigned short AddFrame3D(float P1[6],float F1[6], float A_actual, float B_actual, float C_actual, float P0[6]);
unsigned short SubFrame2D(float P1[6],float F1[6], float P0[6]);
unsigned short AddFrame2D(float P1[6],float F1[6], float P0[6]);
unsigned short SubFrameTool2D(float Path[6], float Frame[6], float Tool[6], float Mount[6]);
unsigned short AddFrameTool2D(float Mount[6], float Frame[6], float Tool[6], float Path[6]);
unsigned short AddFrameTool3D(float Mount[6], float Frame[6], float Tool[6], float A_actual, float B_actual, float C_actual, float Path[6]);
unsigned short SubFrameTool3D(float Path[6], float Frame[6], float Tool[6], float A_actual, float B_actual, float C_actual, float Mount[6]);
unsigned short NormalizeQuat(Quat_Type* q);
unsigned short MatrixToQuat(float RM[3][3], Quat_Type* q);
unsigned short QuatToMatrix(Quat_Type q, float RM[3][3]);
float AngleBetweenQuat(Quat_Type q1, Quat_Type* q2);
unsigned short Slerp(Quat_Type q1, Quat_Type q2, Quat_Type* q, float angle, float u);
unsigned short EulerToQuat(float A, float B, float C, Quat_Type* q);
unsigned short QuatToEuler(Quat_Type q, float A_actual, float B_actual, float C_actual, float *A, float *B, float *C);

#endif

