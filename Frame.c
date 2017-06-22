#include "Frame.h"
#include <math.h>
#include "constants.h"
#include "trig.h"

#define max(a,b) ( (a>b)?a:b )

unsigned short ComposeMatrix(float RM[3][3], float A, float B, float C)
{// compose rotation matrix from RPY angles
	
	RM[0][0] = cosd(C)*cosd(B); 
	RM[0][1] = cosd(C)*sind(B)*sind(A) - sind(C)*cosd(A);
	RM[0][2] = cosd(C)*sind(B)*cosd(A) + sind(C)*sind(A);
	RM[1][0] = sind(C)*cosd(B);
	RM[1][1] = sind(C)*sind(B)*sind(A) + cosd(C)*cosd(A);
	RM[1][2] = sind(C)*sind(B)*cosd(A) - cosd(C)*sind(A);
	RM[2][0] = - sind(B);
	RM[2][1] = cosd(B)*sind(A);
	RM[2][2] = cosd(B)*cosd(A);

	return 0;
}

unsigned short DecomposeMatrix(float RM[3][3], float A_actual, float B_actual, float C_actual, float *A, float *B, float *C)
{// decompose rotation matrix into RPY angles
	
	float A_temp[2],B_temp[2],C_temp[2],ABC_dist[2];
	float cosd_B = sqrtf(1-RM[2][0]*RM[2][0]);

	B_temp[0] = atan2d(-RM[2][0],cosd_B);
	B_temp[1] = atan2d(-RM[2][0],-cosd_B);

	if (fabs(cosd_B)>0.0001f)
	{
		C_temp[0] = atan2d(RM[1][0],RM[0][0]);
		C_temp[1] = atan2d(-RM[1][0],-RM[0][0]);

		A_temp[0] = atan2d(RM[2][1],RM[2][2]); 
		A_temp[1] = atan2d(-RM[2][1],-RM[2][2]); 
	}
	else
	{	//singularity - choose A=0
		A_temp[0] = A_temp[1] = 0;
		C_temp[0] = C_temp[1] = -1.0f * sign(-RM[2][0]) * atan2d(RM[0][1]*sign(-RM[2][0]),RM[1][1]);
	}
	
	//A, C modulo +-2PI to bring them closer to current values
	A_temp[0] = Modulo2PI(A_temp[0],A_actual);
	A_temp[1] = Modulo2PI(A_temp[1],A_actual);
	
	C_temp[0] = Modulo2PI(C_temp[0],C_actual);
	C_temp[1] = Modulo2PI(C_temp[1],C_actual);
	
	// choose closest ABC_temp to ABC_actual 
	ABC_dist[0] = fabs(A_temp[0]-A_actual) + fabs(B_temp[0]-B_actual) + fabs(C_temp[0]-C_actual);
	ABC_dist[1] = fabs(A_temp[1]-A_actual) + fabs(B_temp[1]-B_actual) + fabs(C_temp[1]-C_actual);
	if (ABC_dist[0] <= ABC_dist[1])
	{
		*A=A_temp[0];
		*B=B_temp[0];
		*C=C_temp[0];
	}
	else
	{
		*A=A_temp[1];
		*B=B_temp[1];
		*C=C_temp[1];	
	}

	return 0;
}


unsigned short MatMult(float M1[3][3],float M2[3][3],float M3[3][3])
{// calculate M3 = M1*M2
	int i,j,k;
	
	for(i=0;i<3;i++)
    {
		for(j=0;j<3;j++)
    	{
			M3[i][j] = 0;
		}
	}

	for(i=0;i<3;i++)
    {
		for(j=0;j<3;j++)
    	{
			for (k=0;k<3;k++)
    		{
				M3[i][j] = M3[i][j] + (M1[i][k] * M2[k][j]);
    		}
    	}
    }

	return 0;
}


unsigned short SubFrame3D(float P1[6],float F1[6], float A_actual, float B_actual, float C_actual, float P0[6])
{ //calculate point P0 in base frame given point P1 in frame F1

	float Frame_RotMat[3][3];
	float Start_RotMat[3][3];
	float End_RotMat[3][3];
	float A,B,C;

	ComposeMatrix(Frame_RotMat,F1[3],F1[4],F1[5]);
	ComposeMatrix(Start_RotMat,P1[3],P1[4],P1[5]);

	P0[0] = Frame_RotMat[0][0] * P1[0] + Frame_RotMat[0][1] * P1[1] + Frame_RotMat[0][2] * P1[2] + F1[0]; 
	P0[1] = Frame_RotMat[1][0] * P1[0] + Frame_RotMat[1][1] * P1[1] + Frame_RotMat[1][2] * P1[2] + F1[1]; 
	P0[2] = Frame_RotMat[2][0] * P1[0] + Frame_RotMat[2][1] * P1[1] + Frame_RotMat[2][2] * P1[2] + F1[2]; 
	
	MatMult(Frame_RotMat,Start_RotMat,End_RotMat);

	DecomposeMatrix(End_RotMat,A_actual,B_actual,C_actual,&A,&B,&C);

	P0[3] = A;
	P0[4] = B;
	P0[5] = C;

	return 0;

}

unsigned short AddFrame3D(float P1[6],float F1[6], float A_actual, float B_actual, float C_actual, float P0[6])
{ //calculate point P0 in base frame given point P1 in frame F1

	float Frame_RotMat[3][3];
	float Start_RotMat[3][3];
	float End_RotMat[3][3];
	float A,B,C;

	ComposeMatrix(Frame_RotMat,F1[3],F1[4],F1[5]);
	ComposeMatrix(Start_RotMat,P1[3],P1[4],P1[5]);

	//transpose Frame rotation matrix
	float Temp;
	Temp = Frame_RotMat[0][1];
	Frame_RotMat[0][1] = Frame_RotMat[1][0];
	Frame_RotMat[1][0] = Temp;
	Temp = Frame_RotMat[0][2];
	Frame_RotMat[0][2] = Frame_RotMat[2][0];
	Frame_RotMat[2][0] = Temp;
	Temp = Frame_RotMat[1][2];
	Frame_RotMat[1][2] = Frame_RotMat[2][1];
	Frame_RotMat[2][1] = Temp;

	P0[0] = Frame_RotMat[0][0] * P1[0] + Frame_RotMat[0][1] * P1[1] + Frame_RotMat[0][2] * P1[2] - (Frame_RotMat[0][0] * F1[0] + Frame_RotMat[0][1] * F1[1] + Frame_RotMat[0][2] * F1[2]); 
	P0[1] = Frame_RotMat[1][0] * P1[0] + Frame_RotMat[1][1] * P1[1] + Frame_RotMat[1][2] * P1[2] - (Frame_RotMat[1][0] * F1[0] + Frame_RotMat[1][1] * F1[1] + Frame_RotMat[1][2] * F1[2]); 
	P0[2] = Frame_RotMat[2][0] * P1[0] + Frame_RotMat[2][1] * P1[1] + Frame_RotMat[2][2] * P1[2] - (Frame_RotMat[2][0] * F1[0] + Frame_RotMat[2][1] * F1[1] + Frame_RotMat[2][2] * F1[2]); 
	
	MatMult(Frame_RotMat,Start_RotMat,End_RotMat);

	DecomposeMatrix(End_RotMat,A_actual,B_actual,C_actual,&A,&B,&C);

	P0[3] = A;
	P0[4] = B;
	P0[5] = C;

	return 0;

}

unsigned short SubFrame2D(float P1[6],float F1[6], float P0[6])
{ //calculate point P0 in base frame given point P1 in frame F1 - 2D version, only rotations around Z are possible

	P0[0] = P1[0] * cosd(F1[3]) - P1[1] * sind(F1[3]) + F1[0];
	P0[1] = P1[0] * sind(F1[3]) + P1[1] * cosd(F1[3]) + F1[1];
	P0[2] = P1[2] + F1[2];
	P0[3] = P1[3] + F1[3];
	return 0;	
}

unsigned short AddFrame2D(float P1[6],float F1[6], float P0[6])
{ //calculate point P0 in frame F1 given point P1 in base frame - 2D version, only rotations around Z are possible

	P0[0] = P1[0] * cosd(F1[3]) + P1[1] * sind(F1[3]) - F1[0] * cosd(F1[3]) - F1[1] * sind(F1[3]);
	P0[1] = - P1[0] * sind(F1[3]) + P1[1] * cosd(F1[3]) + F1[0] * sind(F1[3]) - F1[1] * cosd(F1[3]);
	P0[2] = P1[2] - F1[2];
	P0[3] = P1[3] - F1[3];
	return 0;	
}

unsigned short SubFrameTool2D(float Path[6], float Frame[6], float Tool[6], float Mount[6])
{ //calculate mounting point (in base frame) given path axes positions, current frame and tool 

	float TempAxes[6];
	float InvertedTool[6];
	float ZeroFrame[6] = {0,0,0,0,0,0};

	SubFrame2D(Path,Frame,TempAxes);
	AddFrame2D(ZeroFrame,Tool,InvertedTool);
	SubFrame2D(InvertedTool,TempAxes,Mount);

	return 0;
}

unsigned short AddFrameTool2D(float Mount[6], float Frame[6], float Tool[6], float Path[6])
{ //calculate path axes positions given mounting point (in base frame), current frame and tool 

	float TempAxes[6];

	SubFrame2D(Tool,Mount,TempAxes);
	AddFrame2D(TempAxes,Frame,Path);

	return 0;
}

unsigned short AddFrameTool3D(float Mount[6], float Frame[6], float Tool[6], float A_actual, float B_actual, float C_actual, float Path[6])
{ //calculate path axes positions given mounting point (in base frame), current frame and tool 

	float TempAxes[6];

	SubFrame3D(Tool,Mount,A_actual,B_actual,C_actual,TempAxes);
	AddFrame3D(TempAxes,Frame,A_actual,B_actual,C_actual,Path);

	return 0;
}

unsigned short SubFrameTool3D(float Path[6], float Frame[6], float Tool[6], float A_actual, float B_actual, float C_actual, float Mount[6])
{ //calculate mounting point (in base frame) given path axes positions, current frame and tool 

	float TempAxes[6];
	float InvertedTool[6];
	float ZeroFrame[6] = {0,0,0,0,0,0};

	SubFrame3D(Path,Frame,A_actual,B_actual,C_actual,TempAxes);
	AddFrame3D(ZeroFrame,Tool,A_actual,B_actual,C_actual,InvertedTool);
	SubFrame3D(InvertedTool,TempAxes,A_actual,B_actual,C_actual,Mount);

	return 0;
}

unsigned short NormalizeQuat(Quat_Type* q)
{
	float norm = sqrtf(q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
	if (norm >= TRF_EPSILON)
	{
		q->w /= norm;
		q->x /= norm;
		q->y /= norm;
		q->z /= norm;
	}
	return 0;
}

unsigned short MatrixToQuat(float RM[3][3], Quat_Type* q)
{
	q->w = sqrtf( max( 0, 1 + RM[0][0] + RM[1][1] + RM[2][2] )) / 2.0f;
	q->x = sqrtf( max( 0, 1 + RM[0][0] - RM[1][1] - RM[2][2] )) / 2.0f;
	q->y = sqrtf( max( 0, 1 - RM[0][0] + RM[1][1] - RM[2][2] )) / 2.0f;
	q->z = sqrtf( max( 0, 1 - RM[0][0] - RM[1][1] + RM[2][2] )) / 2.0f;
	q->x = copysign(q->x, RM[2][1] - RM[1][2] );
	q->y = copysign(q->y, RM[0][2] - RM[2][0] );
	q->z = copysign(q->z, RM[1][0] - RM[0][1] );

	NormalizeQuat(q);

	return 0;
}

unsigned short QuatToMatrix(Quat_Type q, float RM[3][3])
{
	float w;
	float n = q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
	if (n == 0)
		w = 0;
	else 
		w = 2/n;
		
	float wx = w * q.w * q.x;
	float wy = w * q.w * q.y;
	float wz = w * q.w * q.z;
	float xx = w * q.x * q.x;
	float xy = w * q.x * q.y;
	float xz = w * q.x * q.z;
	float yy = w * q.y * q.y;
	float yz = w * q.y * q.z;
	float zz = w * q.z * q.z;
		
	RM[0][0] = 1 - (yy+zz);
	RM[0][1] = (xy-wz);
	RM[0][2] = (xz+wy);
	RM[1][0] = (xy+wz);
	RM[1][1] = 1 - (xx+zz);
	RM[1][2] = (yz-wx);
	RM[2][0] = (xz-wy);
	RM[2][1] = (yz+wx);
	RM[2][2] = 1 - (yy+xx);
	
	return 0;
}

float AngleBetweenQuat(Quat_Type q1, Quat_Type* q2)
{//calculate angle between quaternions (in radians) -> reverse q2 if angle is obtuse

	float q1_norm = sqrtf(q1.w*q1.w+q1.x*q1.x+q1.y*q1.y+q1.z*q1.z);
	float q2_norm = sqrtf(q2->w*q2->w+q2->x*q2->x+q2->y*q2->y+q2->z*q2->z);
	float q12 = q1.w*q2->w+q1.x*q2->x+q1.y*q2->y+q1.z*q2->z;
	
	float cos_alpha = q12/(q1_norm*q2_norm);
	
	//clamp to avoid numerical errors
	if (cos_alpha > 1)
		cos_alpha = 1;
	else if (cos_alpha < -1)
		cos_alpha = -1;
		
	if (cos_alpha<0)
	{
		q2->w = - q2->w;
		q2->x = - q2->x;
		q2->y = - q2->y;
		q2->z = - q2->z;
	}
	return acos(fabs(cos_alpha));
}

unsigned short Slerp(Quat_Type q1, Quat_Type q2, Quat_Type* q, float angle, float u)
{
	if ((u>1)||(u<0))
		return 255;
	
	if (angle < TRF_EPSILON)
	{ //use linear interpolation for small angles
		q->w = q1.w*(1-u) + q2.w*u; 
		q->x = q1.x*(1-u) + q2.x*u; 
		q->y = q1.y*(1-u) + q2.y*u; 
		q->z = q1.z*(1-u) + q2.z*u; 
	}
	else
	{ //slerp
		q->w = q1.w*sin((1-u)*angle)/sin(angle) + q2.w*sin(u*angle)/sin(angle);		
		q->x = q1.x*sin((1-u)*angle)/sin(angle) + q2.x*sin(u*angle)/sin(angle);		
		q->y = q1.y*sin((1-u)*angle)/sin(angle) + q2.y*sin(u*angle)/sin(angle);		
		q->z = q1.z*sin((1-u)*angle)/sin(angle) + q2.z*sin(u*angle)/sin(angle);		
	}
	
	NormalizeQuat(q);
	
	return 0;
}

unsigned short EulerToQuat(float A, float B, float C, Quat_Type* q)
{
	float RotMat[3][3];
	ComposeMatrix(RotMat,A,B,C);
	MatrixToQuat(RotMat,q);
	return 0;
}

unsigned short QuatToEuler(Quat_Type q, float A_actual, float B_actual, float C_actual, float *A, float *B, float *C)
{
	float RotMat[3][3];
	QuatToMatrix(q,RotMat);
	DecomposeMatrix(RotMat,A_actual,B_actual,C_actual,A,B,C);
	return 0;
}

