#include "Frame.h"
#include <math.h>
#include "constants.h"
#include "trig.h"

#define max(a,b) ( (a>b)?a:b )

unsigned short ComposeMatrix(double RM[3][3], double A, double B, double C)
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

unsigned short DecomposeMatrix(double RM[3][3], double A_actual, double B_actual, double C_actual, double *A, double *B, double *C)
{// decompose rotation matrix into RPY angles
	
	double A_temp[2],B_temp[2],C_temp[2],ABC_dist[2];
	double cosd_B = sqrt(1-RM[2][0]*RM[2][0]);

	B_temp[0] = atan2d(-RM[2][0],cosd_B);
	B_temp[1] = atan2d(-RM[2][0],-cosd_B);

	if (fabs(cosd_B)>TRF_EPSILON) {
		C_temp[0] = atan2d(RM[1][0],RM[0][0]);
		C_temp[1] = atan2d(-RM[1][0],-RM[0][0]);

		A_temp[0] = atan2d(RM[2][1],RM[2][2]); 
		A_temp[1] = atan2d(-RM[2][1],-RM[2][2]); 
	} else {	//singularity - choose A=A_actual
		A_temp[0] = A_temp[1] = A_actual;
		C_temp[0] = C_temp[1] = A_actual - sign(-RM[2][0]) * atan2d(RM[0][1]*sign(-RM[2][0]),RM[1][1]);
	}
	
	//A, C modulo +-2PI to bring them closer to current values
	A_temp[0] = Modulo2PI(A_temp[0],A_actual);
	A_temp[1] = Modulo2PI(A_temp[1],A_actual);
	
	C_temp[0] = Modulo2PI(C_temp[0],C_actual);
	C_temp[1] = Modulo2PI(C_temp[1],C_actual);
	
    //calculate distance of the two solutions from actual values
    ABC_dist[0] = fabs(A_temp[0]-A_actual) + fabs(B_temp[0]-B_actual) + fabs(C_temp[0]-C_actual);
    ABC_dist[1] = fabs(A_temp[1]-A_actual) + fabs(B_temp[1]-B_actual) + fabs(C_temp[1]-C_actual);

    //keep same pose for wrist
    if (B_actual < 0) {//use solution with negative B
        if((B_temp[0]<0)&&(B_temp[1]>=0)) { //use B_temp[0]
            *A=A_temp[0];
            *B=B_temp[0];
            *C=C_temp[0];
        } else if((B_temp[1]<0)&&(B_temp[0]>=0)) { //use B_temp[1]
            *A=A_temp[1];
            *B=B_temp[1];
            *C=C_temp[1];	
        } else { //use closest solution to current values
            if (ABC_dist[0] <= ABC_dist[1]) {
                *A = A_temp[0];
                *B = B_temp[0];
                *C = C_temp[0];
            } else {
                *A = A_temp[1];
                *B = B_temp[1];
                *C = C_temp[1];
            }	    
        }
    } else {//use solution with positive B
        if((B_temp[0]>=0)&&(B_temp[1]<0)) { //use B_temp[0]
            *A = A_temp[0];
            *B = B_temp[0];
            *C = C_temp[0];            
        } else if((B_temp[1]>=0)&&(B_temp[0]<0)) { //use B_temp[1]
            *A = A_temp[1];
            *B = B_temp[1];
            *C = C_temp[1];
        } else { //use closest solution to current values
            if (ABC_dist[0] <= ABC_dist[1]) {
                *A = A_temp[0];
                *B = B_temp[0];
                *C = C_temp[0];
            } else {
                *A = A_temp[1];
                *B = B_temp[1];
                *C = C_temp[1];
            }	    
        }
    }

    //adjust positions of C with +-2PI to bring it closer to desired value
    *C = Modulo2PI(*C,C_actual);
    
	return 0;
}


unsigned short MatMult(double M1[3][3],double M2[3][3],double M3[3][3])
{// calculate M3 = M1*M2
	int i,j,k;
	
	for(i=0;i<3;i++) {
		for(j=0;j<3;j++) {
			M3[i][j] = 0;
		}
	}

	for(i=0;i<3;i++) {
		for(j=0;j<3;j++) {
			for (k=0;k<3;k++) {
				M3[i][j] = M3[i][j] + (M1[i][k] * M2[k][j]);
    		}
    	}
    }

	return 0;
}


unsigned short SubFrame3D(double P1[6],double F1[6], double A_actual, double B_actual, double C_actual, double P0[6])
{ //calculate point P0 in base frame given point P1 in frame F1

	double Frame_RotMat[3][3];
	double Start_RotMat[3][3];
	double End_RotMat[3][3];
	double A,B,C;

	ComposeMatrix(Frame_RotMat,F1[3],F1[4],F1[5]);
	ComposeMatrix(Start_RotMat,P1[3],P1[4],P1[5]);

	P0[0] = Frame_RotMat[0][0] * P1[0] + Frame_RotMat[0][1] * P1[1] + Frame_RotMat[0][2] * P1[2] + F1[0]; 
	P0[1] = Frame_RotMat[1][0] * P1[0] + Frame_RotMat[1][1] * P1[1] + Frame_RotMat[1][2] * P1[2] + F1[1]; 
	P0[2] = Frame_RotMat[2][0] * P1[0] + Frame_RotMat[2][1] * P1[1] + Frame_RotMat[2][2] * P1[2] + F1[2]; 
	
	MatMult(Frame_RotMat,Start_RotMat,End_RotMat);

	DecomposeMatrix(End_RotMat,A_actual,B_actual,C_actual,&A,&B,&C);

    //keep orientation within 2PI from origin
	P0[3] = Modulo2PI(A,0);
	P0[4] = Modulo2PI(B,0);
	P0[5] = Modulo2PI(C,0);

	return 0;

}

unsigned short AddFrame3D(double P1[6],double F1[6], double A_actual, double B_actual, double C_actual, double P0[6])
{ //calculate point P0 in frame F1 given point P1 in base frame

	double Frame_RotMat[3][3];
	double Start_RotMat[3][3];
	double End_RotMat[3][3];
	double A,B,C;

	ComposeMatrix(Frame_RotMat,F1[3],F1[4],F1[5]);
	ComposeMatrix(Start_RotMat,P1[3],P1[4],P1[5]);

	//transpose Frame rotation matrix
	double tmpFrame;
	tmpFrame = Frame_RotMat[0][1];
	Frame_RotMat[0][1] = Frame_RotMat[1][0];
	Frame_RotMat[1][0] = tmpFrame;
	tmpFrame = Frame_RotMat[0][2];
	Frame_RotMat[0][2] = Frame_RotMat[2][0];
	Frame_RotMat[2][0] = tmpFrame;
	tmpFrame = Frame_RotMat[1][2];
	Frame_RotMat[1][2] = Frame_RotMat[2][1];
	Frame_RotMat[2][1] = tmpFrame;

	P0[0] = Frame_RotMat[0][0] * P1[0] + Frame_RotMat[0][1] * P1[1] + Frame_RotMat[0][2] * P1[2] - (Frame_RotMat[0][0] * F1[0] + Frame_RotMat[0][1] * F1[1] + Frame_RotMat[0][2] * F1[2]); 
	P0[1] = Frame_RotMat[1][0] * P1[0] + Frame_RotMat[1][1] * P1[1] + Frame_RotMat[1][2] * P1[2] - (Frame_RotMat[1][0] * F1[0] + Frame_RotMat[1][1] * F1[1] + Frame_RotMat[1][2] * F1[2]); 
	P0[2] = Frame_RotMat[2][0] * P1[0] + Frame_RotMat[2][1] * P1[1] + Frame_RotMat[2][2] * P1[2] - (Frame_RotMat[2][0] * F1[0] + Frame_RotMat[2][1] * F1[1] + Frame_RotMat[2][2] * F1[2]); 
	
	MatMult(Frame_RotMat,Start_RotMat,End_RotMat);

	DecomposeMatrix(End_RotMat,A_actual,B_actual,C_actual,&A,&B,&C);

    //keep orientation within 2PI from origin
    P0[3] = Modulo2PI(A,0);
    P0[4] = Modulo2PI(B,0);
    P0[5] = Modulo2PI(C,0);

	return 0;

}

unsigned short SubFrame2D(double P1[6],double F1[6], double P0[6])
{ //calculate point P0 in base frame given point P1 in frame F1 - 2D version, only rotations around Z are possible

	P0[0] = P1[0] * cosd(F1[3]) - P1[1] * sind(F1[3]) + F1[0];
	P0[1] = P1[0] * sind(F1[3]) + P1[1] * cosd(F1[3]) + F1[1];
	P0[2] = P1[2] + F1[2];
	P0[3] = P1[3] + F1[3];
    //keep orientation within 2PI from origin
    P0[3] = Modulo2PI(P0[3],0);
	return 0;	
}

unsigned short AddFrame2D(double P1[6],double F1[6], double P0[6])
{ //calculate point P0 in frame F1 given point P1 in base frame - 2D version, only rotations around Z are possible

	P0[0] = P1[0] * cosd(F1[3]) + P1[1] * sind(F1[3]) - F1[0] * cosd(F1[3]) - F1[1] * sind(F1[3]);
	P0[1] = - P1[0] * sind(F1[3]) + P1[1] * cosd(F1[3]) + F1[0] * sind(F1[3]) - F1[1] * cosd(F1[3]);
	P0[2] = P1[2] - F1[2];
	P0[3] = P1[3] - F1[3];
    //keep orientation within 2PI from origin
    P0[3] = Modulo2PI(P0[3],0);
    return 0;	
}

unsigned short SubFrameTool2D(double Path[6], double Frame[6], double Tool[6], double Mount[6])
{ //calculate mounting point (in base frame) given path axes positions, current frame and tool 

	double tmpAxes[6];
	double InvertedTool[6];
	double ZeroFrame[6] = {0,0,0,0,0,0};

	SubFrame2D(Path,Frame,tmpAxes);
	AddFrame2D(ZeroFrame,Tool,InvertedTool);
	SubFrame2D(InvertedTool,tmpAxes,Mount);

	return 0;
}

unsigned short AddFrameTool2D(double Mount[6], double Frame[6], double Tool[6], double Path[6])
{ //calculate path axes positions given mounting point (in base frame), current frame and tool 

	double tmpAxes[6];

	SubFrame2D(Tool,Mount,tmpAxes);
	AddFrame2D(tmpAxes,Frame,Path);

	return 0;
}

unsigned short AddFrameTool3D(double Mount[6], double Frame[6], double Tool[6], double A_actual, double B_actual, double C_actual, double Path[6])
{ //calculate path axes positions given mounting point (in base frame), current frame and tool 

	double tmpAxes[6];

	SubFrame3D(Tool,Mount,A_actual,B_actual,C_actual,tmpAxes);
	AddFrame3D(tmpAxes,Frame,A_actual,B_actual,C_actual,Path);

	return 0;
}

unsigned short SubFrameTool3D(double Path[6], double Frame[6], double Tool[6], double A_actual, double B_actual, double C_actual, double Mount[6])
{ //calculate mounting point (in base frame) given path axes positions, current frame and tool 

	double tmpAxes[6];
	double InvertedTool[6];
	double ZeroFrame[6] = {0,0,0,0,0,0};

	SubFrame3D(Path,Frame,A_actual,B_actual,C_actual,tmpAxes);
	AddFrame3D(ZeroFrame,Tool,A_actual,B_actual,C_actual,InvertedTool);
	SubFrame3D(InvertedTool,tmpAxes,A_actual,B_actual,C_actual,Mount);

	return 0;
}

unsigned short NormalizeQuat(Quat_Type* q)
{
	double norm = sqrt(q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
	if (norm >= TRF_EPSILON) {
		q->w /= norm;
		q->x /= norm;
		q->y /= norm;
		q->z /= norm;
	}
	return 0;
}

unsigned short MatrixToQuat(double RM[3][3], Quat_Type* q)
{
	q->w = sqrt( max( 0, 1 + RM[0][0] + RM[1][1] + RM[2][2] )) / 2.0;
	q->x = sqrt( max( 0, 1 + RM[0][0] - RM[1][1] - RM[2][2] )) / 2.0;
	q->y = sqrt( max( 0, 1 - RM[0][0] + RM[1][1] - RM[2][2] )) / 2.0;
	q->z = sqrt( max( 0, 1 - RM[0][0] - RM[1][1] + RM[2][2] )) / 2.0;
	q->x = copysign(q->x, RM[2][1] - RM[1][2] );
	q->y = copysign(q->y, RM[0][2] - RM[2][0] );
	q->z = copysign(q->z, RM[1][0] - RM[0][1] );

	NormalizeQuat(q);

	return 0;
}

unsigned short QuatToMatrix(Quat_Type q, double RM[3][3])
{
	double w;
	double n = q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
	if (fabs(n) < TRF_EPSILON) {
		w = 0;
    } else {
		w = 2/n;
	}
	double wx = w * q.w * q.x;
	double wy = w * q.w * q.y;
	double wz = w * q.w * q.z;
	double xx = w * q.x * q.x;
	double xy = w * q.x * q.y;
	double xz = w * q.x * q.z;
	double yy = w * q.y * q.y;
	double yz = w * q.y * q.z;
	double zz = w * q.z * q.z;
		
	RM[0][0] = 1 - (yy+zz);
	RM[0][1] = (xy-wz);
	RM[0][2] = (xz+wy);
	RM[1][0] = (xy+wz);
	RM[1][1] = 1 - (xx+zz);
	RM[1][2] = (yz-wx);
	RM[2][0] = (xz-wy);
	RM[2][1] = (yz+wx);
	RM[2][2] = 1 - (yy+xx);

    //added for numerical stability - otherwise it is not robust for 90deg angles
    int i, j;
    for (i=0;i<3;i++) {
        for (j=0;j<3;j++) {
            if (RM[i][j]>1) RM[i][j] = 1;
            if (RM[i][j]<-1) RM[i][j] = -1;
        }
    }
	return 0;
}

double AngleBetweenQuat(Quat_Type q1, Quat_Type* q2)
{//calculate angle between quaternions (in radians) -> reverse q2 if angle is obtuse

	double q1_norm = sqrt(q1.w*q1.w+q1.x*q1.x+q1.y*q1.y+q1.z*q1.z);
	double q2_norm = sqrt(q2->w*q2->w+q2->x*q2->x+q2->y*q2->y+q2->z*q2->z);
	double q12 = q1.w*q2->w+q1.x*q2->x+q1.y*q2->y+q1.z*q2->z;
	
	double cos_alpha = q12/(q1_norm*q2_norm);
	
	//clamp to avoid numerical errors
    if (cos_alpha > 1) {
        cos_alpha = 1;
    } else if (cos_alpha < -1) {
            cos_alpha = -1;
    }
    
	if (cos_alpha<0) {
		q2->w = - q2->w;
		q2->x = - q2->x;
		q2->y = - q2->y;
		q2->z = - q2->z;
	}
    
	return acos(fabs(cos_alpha));
}

unsigned short Slerp(Quat_Type q1, Quat_Type q2, Quat_Type* q, double angle, double u)
{
    /* Slerp works also with u outside 0..1 -> it linearly extrapolates
	if ((u>1)||(u<0))
		return 255;
	*/
    
	if (angle < TRF_EPSILON) { //use linear interpolation for small angles
		q->w = q1.w*(1-u) + q2.w*u; 
		q->x = q1.x*(1-u) + q2.x*u; 
		q->y = q1.y*(1-u) + q2.y*u; 
		q->z = q1.z*(1-u) + q2.z*u; 
    } else { //slerp
		q->w = q1.w*sin((1-u)*angle)/sin(angle) + q2.w*sin(u*angle)/sin(angle);		
		q->x = q1.x*sin((1-u)*angle)/sin(angle) + q2.x*sin(u*angle)/sin(angle);		
		q->y = q1.y*sin((1-u)*angle)/sin(angle) + q2.y*sin(u*angle)/sin(angle);		
		q->z = q1.z*sin((1-u)*angle)/sin(angle) + q2.z*sin(u*angle)/sin(angle);		
	}
	
	NormalizeQuat(q);
	
	return 0;
}

unsigned short EulerToQuat(double A, double B, double C, Quat_Type* q)
{
	double RotMat[3][3];
	ComposeMatrix(RotMat,A,B,C);
	MatrixToQuat(RotMat,q);
	return 0;
}

unsigned short QuatToEuler(Quat_Type q, double A_actual, double B_actual, double C_actual, double *A, double *B, double *C)
{
	double RotMat[3][3];
    NormalizeQuat(&q);
	QuatToMatrix(q,RotMat);
	DecomposeMatrix(RotMat,A_actual,B_actual,C_actual,A,B,C);
	return 0;
}



