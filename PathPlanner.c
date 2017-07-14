#include "PathPlanner.h"
#include "RobControl.h"
#include "trig.h"
#include "Misc.h"
#include "constants.h"
#include <math.h>

#define sign(a) ( (a<0)?-1:1 )
#define max(a,b) ( (a>b)?a:b )

static float crt(float x)
{ //cubic root
	if (x<0)
	{
		return -pow(-x,1.0f/3.0f);
	}
	else
	{
		return pow(x,1.0f/3.0f);
	}
} 

float LineLength(float P1[6],float P2[6], int Size)
{ //calculate length of line from point P1 to point P2
	int k;
	float sum;
	sum = 0;	
	for(k=0;k<Size;k++)
		sum += pow(P2[k]-P1[k],2.0f);
	return sqrtf(sum);	
}

float LineLengthCart(float P1[6],float P2[6])
{ //calculate length of line from point P1 to point P2 using only the three cartesian dimensions
	return sqrtf(pow(P2[0]-P1[0],2.0f)+pow(P2[1]-P1[1],2.0f)+pow(P2[2]-P1[2],2.0f));
}

float LineLengthAng(float P1[6],float P2[6], int Size)
{ //calculate length of angular movement from point P1 to point P2 using quaternions
	return sqrtf(pow(P2[3]-P1[3],2.0f)+pow(P2[4]-P1[4],2.0f)+pow(P2[5]-P1[5],2.0f));
}

float MinPathTime(float P1[6], float P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6])
{ //calculate minimum time to complete linear path between P1 and P2 given each axis max speed
	float t;
	float t_min = 0.0f;
	int k;

	for(k=0;k<Size;k++)
	{
		//assume that speed limits are non-zero (must be checked in program!)
		if (P2[k]>=P1[k])
		{
			t = (P2[k]-P1[k])/(Limit[k].VelocityPos);
		}
		else
		{
			t = (P1[k]-P2[k])/(Limit[k].VelocityNeg);
		}
		if (t > t_min)
		{// this axis is slower
			t_min = t;
		}
	}
	
	return t_min;

}


float VectorLength(float A[3])
{	//calculate length of vector A
	return (sqrtf(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]));
}

unsigned short CrossProduct(float U[3], float V[3], float (*N)[3])
{ // calculate scalar components of cross-product UxV
	
	(*N)[0] = U[1]*V[2]-U[2]*V[1];
	(*N)[1] = U[2]*V[0]-U[0]*V[2];
	(*N)[2] = U[0]*V[1]-U[1]*V[0];

	return 0;
}

float DotProduct(float U[3], float V[3])
{
	return (U[0]*V[0] + U[1]*V[1] + U[2]*V[2]);
	
}

float AngleBetweenVectors(float U[6], float V[6])
{// calculate angle between 6-dimensional vectors in degrees

	//calculate vectors lengths
	float U_Length = sqrtf(U[0]*U[0] + U[1]*U[1] + U[2]*U[2] + U[3]*U[3] + U[4]*U[4] + U[5]*U[5]);
	float V_Length = sqrtf(V[0]*V[0] + V[1]*V[1] + V[2]*V[2] + V[3]*V[3] + V[4]*V[4] + V[5]*V[5]);

	//check that input is valid (vector length not zero)
	if ((U_Length < TRF_EPSILON)||(V_Length < TRF_EPSILON))
	{
		return -1;
	}
	
	//calculate dot product between U and V
	float UV_DotProduct = U[0]*V[0] + U[1]*V[1] + U[2]*V[2] + U[3]*V[3] + U[4]*V[4] + U[5]*V[5];
			
	//extract cosine of angle between U and V
	float UV_CosAngle = UV_DotProduct/(U_Length*V_Length);
	
	//limit it do +/- 1 range (in case of numerical errors)
	if(UV_CosAngle > 1)
		UV_CosAngle = 1;
	else if (UV_CosAngle < -1)
		UV_CosAngle = -1;
		
	//return angle between U and V (in 0..180deg range)
	return acosd(UV_CosAngle);

}


unsigned short PointsToVector(float P1[6], float P2[6], float (*V)[3])
{ //make vector V from point P1 to point P2, i.e. V = P2-P1
	(*V)[0] = P2[0]-P1[0];
	(*V)[1] = P2[1]-P1[1];
	(*V)[2] = P2[2]-P1[2];
	return 0;	
}

unsigned short Normalize(float (*V)[3])
{
	float norm = VectorLength((*V));
	if (norm != 0)
	{
		(*V)[0] /= norm;
		(*V)[1] /= norm;
		(*V)[2] /= norm;
	}
	return 0;
}

unsigned short EvalCircle(Path_Type *Circle)
{//calculate all circle parameters
	
	float a,b,c,s;
	float A[3],B[3],C[3];
	float alpha, beta, gamma;
	
	/*** calculate radius and center of cicle using cross and dot product properties ***/
	
	a = LineLengthCart(Circle->MiddlePointPath,Circle->TargetPointPath);
	b = LineLengthCart(Circle->StartPointPath,Circle->TargetPointPath);
	c = LineLengthCart(Circle->StartPointPath,Circle->MiddlePointPath);

	PointsToVector(Circle->MiddlePointPath,Circle->TargetPointPath,&A);
	PointsToVector(Circle->StartPointPath,Circle->TargetPointPath,&B);
	PointsToVector(Circle->StartPointPath,Circle->MiddlePointPath,&C);
	
	CrossProduct(C,A,&Circle->Normal);
	s = VectorLength(Circle->Normal);
	
	/* check that the three points are not collinear */
	if (s<TRF_EPSILON)
	{ 
		return ERR_PP_CIRCLEPOINTS;	
	}

	Circle->Radius = a*b*c /(2.0f*s); //radius

	Normalize(&Circle->Normal);	//normal versor	
	
	alpha = a*a * DotProduct(B,C) / (2.0f*s*s);
	beta = b*b * -DotProduct(A,C) / (2.0f*s*s);
	gamma = c*c * DotProduct(B,A) / (2.0f*s*s);

	/* circle center point coordinates */
	Circle->Center[0] = alpha * Circle->StartPointPath[0] + beta * Circle->MiddlePointPath[0] + gamma * Circle->TargetPointPath[0];
	Circle->Center[1] = alpha * Circle->StartPointPath[1] + beta * Circle->MiddlePointPath[1] + gamma * Circle->TargetPointPath[1];
	Circle->Center[2] = alpha * Circle->StartPointPath[2] + beta * Circle->MiddlePointPath[2] + gamma * Circle->TargetPointPath[2];

	
	/* calculate vectors from center to start, middle and target point */
	PointsToVector(Circle->Center,Circle->StartPointPath,&Circle->StartVersor);
	Normalize(&Circle->StartVersor);
	
	PointsToVector(Circle->Center,Circle->MiddlePointPath,&Circle->MiddleVersor);	
	Normalize(&Circle->MiddleVersor);

	PointsToVector(Circle->Center,Circle->TargetPointPath,&Circle->EndVersor);	
	Normalize(&Circle->EndVersor);

	/* calculate cross versor */
	CrossProduct(Circle->Normal,Circle->StartVersor,&Circle->CrossVersor);	
	Normalize(&Circle->CrossVersor);
	
	/* calculate arc length to end point */
    float Test[3];
    CrossProduct(Circle->StartVersor,Circle->EndVersor,&Test);
    float TempDotProd = DotProduct(Circle->StartVersor,Circle->EndVersor);
    //argument of acos must be between +/- 1 (avoid numerical errors)
    if (TempDotProd > 1)
        TempDotProd = 1;
    else if (TempDotProd < -1)
        TempDotProd = -1;
    Circle->Length = Circle->Radius * acos(TempDotProd);
    if (sign(Circle->Normal[2])*sign(Test[2]) < 0)
    {
        Circle->Length = Circle->Radius * 2.0f * PI - Circle->Length;
    }
	
	/* calculate arc length to middle point */
	CrossProduct(Circle->StartVersor,Circle->MiddleVersor,&Test);
	TempDotProd = DotProduct(Circle->StartVersor,Circle->MiddleVersor);
	//argument of acos must be between +/- 1 (avoid numerical errors)
	if (TempDotProd > 1)
		TempDotProd = 1;
	else if (TempDotProd < -1)
		TempDotProd = -1;
	Circle->MiddleLength = Circle->Radius * acos(TempDotProd);
	if (sign(Circle->Normal[2])*sign(Test[2]) < 0)
	{
		Circle->MiddleLength = Circle->Radius * 2.0f * PI - Circle->MiddleLength;
	}
	
	if ((Circle->Length <= 0)||(Circle->MiddleLength <= 0))
		return ERR_PP_CIRCLE_LENGTH;
	    
	return 0;
}


unsigned short MaxBlockSpeed(float d, float a, float j, float v_end, float *v_max)
{ //find maximum (initial) speed of a block with length d, acceleration a, jerk j, and final speed v_end

	if ((d<0)||(v_end<0)||(a<=0)||(j<=0))
	{// incorrect input -> error
		return 255;
	}
	
	if (d==0)
	{ //zero length -> no movement possible
		*v_max = v_end;
		return 0;	
	}
		
	float Delta = 2.0f*a*v_end/j + a*a*a/j/j;	
	if (d<=Delta)
	{// a cannot be reached -> reduce it. There is no linear acceleration section, it's all jerk limited movement
		float p3 = 2.0f*v_end*j/3.0f;
		float q2 = -d*j*j/2.0f;
		float Det = p3*p3*p3+q2*q2;
		float alpha = crt(-q2+sqrtf(Det));
		float betha = crt(-q2-sqrtf(Det));
		float a_max = alpha + betha;
		*v_max = (v_end + a_max*a_max/j);
		return 0;
	}
	else
	{// a can be reached, there is a linear acceleration section
		float Det = a*a/j/j/4.0f + 2.0f*d/a + v_end*v_end/a/a - v_end/j;
		*v_max = a*(sqrtf(Det) - a/j/2.0f);
		return 0;
	}

}



unsigned short MaxMovementDynamics(float d, float a, float j, float v, float v_start, float v_end, float a_start, float *v_max, float *a_up_max, float *a_down_max, float *d_linear)
{ //find maximum speed and acceleration of a movement with length d, speed v, acceleration a, jerk j, initial and final speed v_start and v_end, initial acceleration a_start
	//also returns size of linear speed interval
	
	float d_start, d_end;
	float t_start, t_end;
	float a_max, a_up, a_down;
	float dv_start, dv_end;
	
	float step, Delta, v_high, v_low;
	
	if ((d<0)||(v_end<0)||(v_end>v)||(v_start<0)||(a<=0)||(j<=0)||(v<=0)||(fabs(a_start)>a+TRF_EPSILON))
	//note that programmed speed for path planner cannot be negative
	//note that starting acceleration cannot be higher than max acc
	{// incorrect input -> error
		return 255;
	}
	
	if (d==0)
	{ //zero length -> no movement possible
		if (v_start!=v_end)
		{
			return 250;
		}
		else
		{	
			*v_max = v_start;
			*a_up_max = 0;
			*a_down_max = 0;
			*d_linear = 0;
			return 0;	
		}
	}
	
	/*** check if v_end can be reached ***/
	dv_start = v_end - v_start;
	a_max = sqrtf(j*fabs(dv_start)+0.5f*a_start*a_start); //max acc (no linear segment) - normally limited by "a" (limit)

	/* REMOVED because v_end is not max speed and it can be exceeded. Think for example of a case when v_end=v_start.
	if (fabs(a_start)>a_max+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
		return 251;
	*/
	if (a_max < fabs(a_start)) a_max = fabs(a_start);
	if (a_max > a) a_max = a;
	if (a_max != 0)
	{
		float t0 = (a_max-fabs(a_start))/j;
		if (t0<0) t0=0;
		
		float t1 = 1/a_max * (fabs(dv_start) -0.5f*a_max*a_max/j -fabs(a_start)*(a_max-fabs(a_start))/j -0.5f*(a_max-fabs(a_start))*(a_max-fabs(a_start))/j);
		if (t1<0) t1=0;
		
		float t2 = a_max/j;

		t_start = t0 + t1 + t2;
		
		float v1 = v_start + (fabs(a_start) * t0 + 0.5f *j *t0*t0)*sign(dv_start); //speed at end of t0
		float v2 = v1 + (a_max*t1)*sign(dv_start); //speed at end of t1

		d_start = (v_start * t0 + (j * t0*t0*t0 / 6.0f + 0.5f * fabs(a_start) * t0*t0)* sign(dv_start));
		d_start += (v1 * t1 + (0.5f * a_max * t1*t1) * sign(dv_start));
		d_start += (v2 * t2 + (0.5f * a_max * t2*t2 - j * t2*t2*t2 / 6.0f) * sign(dv_start));
	}
	else
	{
		t_start = 0;
		d_start = 0;
	}
	if (d_start > d+TRF_EPSILON)
	{
		//v_end is too high (or too low) and cannot be reached with current dynamic limits -> movement will be interrupted while running with a!=0
		*v_max = v_end;
		*a_up_max = a_max*sign(dv_start);
		*a_down_max = 0;
		*d_linear = 0;
		return 0;
	}	
	
	
	
	/* assume that v can be reached */

	// interval from v_start to v
	dv_start = v - v_start;
	a_up = sqrtf(j*fabs(dv_start)+0.5f*a_start*a_start); //max acc (no linear segment) - normally limited by a (limit)
	if (fabs(a_start)>a_up+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
		return 252;
	if (a_up > a) a_up = a;
	if (a_up != 0)
	{
		float t0 = (a_up-fabs(a_start))/j;
		float t1 = 1/a_up * (fabs(dv_start) -0.5f*a_up*a_up/j -fabs(a_start)*(a_up-fabs(a_start))/j -0.5f*(a_up-fabs(a_start))*(a_up-fabs(a_start))/j);
		float t2 = a_up/j;

		t_start = t0 + t1 + t2;
		
		float v1 = v_start + (fabs(a_start) * t0 + 0.5f *j *t0*t0)*sign(dv_start); //speed at end of t0
		float v2 = v1 + (a_up*t1)*sign(dv_start); //speed at end of t1

		d_start = (v_start * t0 + (j * t0*t0*t0 / 6.0f + 0.5f * fabs(a_start) * t0*t0)* sign(dv_start));
		d_start += (v1 * t1 + (0.5f * a_up * t1*t1) * sign(dv_start));
		d_start += (v2 * t2 + (0.5f * a_up * t2*t2 - j * t2*t2*t2 / 6.0f) * sign(dv_start));
	}
	else
	{
		t_start = 0;
		d_start = 0;
	}
	
	// interval from v to v_end
	dv_end = fabs(v - v_end);
	a_down = sqrtf(j*(dv_end));
	if (a_down > a) a_down = a;
	if (a_down != 0)
	{
		t_end = dv_end/a_down + a_down/j;
		d_end = 0.5f * (dv_end*dv_end/a_down + dv_end*a_down/j);
		if (v<v_end)
			d_end += v * t_end;
		else
			d_end += v_end * t_end;			
	}
	else
	{
		t_end = 0;
		d_end = 0;
	}
	
	Delta = d - d_start - d_end; //constant speed section

	
	if (Delta >= 0)
	{// v can be reached
		*v_max = v; //if v<v_start then this v_max is not really maximum but speed of constant section
		*a_up_max = a_up * sign(dv_start);
		*a_down_max = a_down;
		*d_linear = Delta;
		if (fabs(a_start)>a_up+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
			return 253;
		return 0;
	}
	/* removed - we already checked above for the case that v_end cannot be reached! 
	it works when you have many small blocks in acceleration but it does not work when you have one single block and need to
	complete the movement by the end of the block.
	else if (d >= d_start)
	{// v can be reached (d>=d_start) but v_end cannot (Delta<0) -> start movement and let next blocks complete it (TODO what if there are no more blocks after it?!?!??!)
		*v_max = v;
		*a_up_max = a_up*sign(dv_start);
		*a_down_max = a;
		*d_linear = 0;
		if (fabs(a_start)>a_up+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
			return 248;
		return 0;
	}
	*/
	else
	{// v cannot be reached, reduce v
		
		if (v_start > v)
		{// in this case v is not needed, just decelerate from v_start to v_end
			*v_max = v_end; //no linear speed interval
			
			//calculate a_max from d1+d2+d3=d

			float aaa = (v_start+v_end)/j+fabs(a_start)/j/j-fabs(a_start)*fabs(a_start)/j/j;
			float bbb = d + v_start*fabs(a_start)/j - fabs(a_start)*fabs(a_start)*fabs(a_start)/3.0f/j/j;
			float ccc = (v_start-v_end+fabs(a_start)*fabs(a_start)/2.0f/j)*(v_start+v_end+fabs(a_start)*fabs(a_start)/j);
			float ddd = bbb*bbb-aaa*ccc;		
			
			if ((ddd<0)||(aaa==0))
				return 254;	//should never happen with current settings
			
			*a_up_max = - (bbb - sqrtf(ddd)) / aaa; //acceleration is negative because movement is speeding down to v_end

			*a_down_max = 0; //no ending interval
			*d_linear = 0; // no linear speed interval
			if (fabs(a_start)>fabs(*a_up_max)+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
				return 255;
			return 0;
		}
		
		step = 0;
		v_high = v;
		v_low = max(v_start,v_end);
		do
		{
			step += 0.01f;

			if (Delta < 0)
			{
				v_high = v;
				v = (v_high + v_low)/2.0f;
			}
			else if (Delta > 0)
			{
				v_low = v;
				v = (v_high + v_low)/2.0f;
			}
			else if (Delta == 0)
			{
				step = 1;	
			}

			// interval from v_start to v
			dv_start = v - v_start;
			a_up = sqrtf(j*fabs(dv_start)+0.5f*a_start*a_start); //max acc (no linear segment) - normally limited by a (limit)
			if (a_up > a) a_up = a;
			if (a_up != 0)
			{
				float t0 = (a_up-fabs(a_start))/j;
				float t1 = 1/a_up * (fabs(dv_start) -0.5f*a_up*a_up/j -fabs(a_start)*(a_up-fabs(a_start))/j -0.5f*(a_up-fabs(a_start))*(a_up-fabs(a_start))/j);
				float t2 = a_up/j;

				t_start = t0 + t1 + t2;
		
				float v1 = v_start + (fabs(a_start) * t0 + 0.5f *j *t0*t0)*sign(dv_start); //speed at end of t0
				float v2 = v1 + (a_up*t1)*sign(dv_start); //speed at end of t1

				d_start = (v_start * t0 + (j * t0*t0*t0 / 6.0f + 0.5f * fabs(a_start) * t0*t0)* sign(dv_start));
				d_start += (v1 * t1 + (0.5f * a_up * t1*t1) * sign(dv_start));
				d_start += (v2 * t2 + (0.5f * a_up * t2*t2 - j * t2*t2*t2 / 6.0f) * sign(dv_start));
			}
			else
			{
				t_start = 0;
				d_start = 0;
			}
	
			// interval from v to v_end
			dv_end = v - v_end;
			a_down = sqrtf(j*(dv_end));
			if (a_down > a) a_down = a;
			if (a_down != 0)
			{
				t_end = dv_end/a_down + a_down/j;
				d_end = 0.5f * (dv_end*dv_end/a_down + dv_end*a_down/j);
				if (v<v_end)
					d_end += v * t_end;
				else
					d_end += v_end * t_end;			
			}
			else
			{
				t_end = 0;
				d_end = 0;
			}

			Delta = d - d_start - d_end;
					
		} while (step < 1);

		*v_max = v;
		*a_up_max = a_up;
		*a_down_max = a_down;
		*d_linear = 0; // no linear speed interval
				
		if (fabs(a_start)>a_up+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
		{
			return 249;
		}
		
		return 0;
	}

}


unsigned short EvaluateBezier (Point_Type P[5], float u, Point_Type *Q,int Size)
{// input are Bezier control points P0..P4, time u and size of coordinate system (number of robot's axes); output is point Q
	
	unsigned char k,i,j;
	unsigned char n=4; // n=4 for quartic Bezier curves

	if((u<0)||(u>1))
		return 255;
	
	Point_Type Point[5];	
	for(i=0;i<=n;i++) Point[i]=P[i];
	
	for (k=1; k<=n; k++)
	{
		for (i=0; i<=n-k; i++)
		{
			for (j=0; j<Size; j++)
				Point[i].Axes[j] = (1.0f-u)*Point[i].Axes[j] + u*Point[i+1].Axes[j];
		}
	}
	
	for (j=0; j<Size; j++)
		Q->Axes[j] = Point[0].Axes[j];
	
	return 0;
}

float BezierLength(Point_Type P[5], int Size)
{// input are Bezier control points P0..P4 and size of coordinate system (number of robot's axes)

	Point_Type Point[2];
	int i;
	int k = 10; //increase number of points for more accuracy
	float Length = 0;
	
	Point[0] = P[0];
	
	//evaluate the BezierCurve at different points and then add the distances between them
	for(i=1;i<=k;i++)
	{
		EvaluateBezier(P,(float)(i) / (float) (k),&(Point[1]),Size);
		Length += LineLength(Point[0].Axes,Point[1].Axes,Size);
		Point[0] = Point[1];
	}
	
	return Length;

}

float BezierLengthHalf1(Point_Type P[5], int Size)
{// input are Bezier control points P0..P4 and size of coordinate system (number of robot's axes)
 // calculates only length of first half P0..P2
	
	Point_Type Point[2];
	int i;
	int k = 10; //increase number of points for more accuracy
	float Length = 0;
	
	Point[0] = P[0];
	
	//evaluate the BezierCurve at different points and then add the distances between them
	for(i=1;i<=k/2;i++)
	{
		EvaluateBezier(P,(float)(i) / (float) (k),&(Point[1]),Size);
		Length += LineLength(Point[0].Axes,Point[1].Axes,Size);
		Point[0] = Point[1];
	}
	
	return Length;

}

float BezierLengthHalf2(Point_Type P[5], int Size)
{// input are Bezier control points P0..P4 and size of coordinate system (number of robot's axes)
	// calculates only length of second half P2..P4

	Point_Type Point[2];
	int i;
	int k = 10; //increase number of points for more accuracy
	float Length = 0;
	
	EvaluateBezier(P,0.5f,&(Point[0]),Size);
	
	//evaluate the BezierCurve at different points and then add the distances between them
	for(i=k/2+1;i<=k;i++)
	{
		EvaluateBezier(P,(float)(i) / (float) (k),&(Point[1]),Size);
		Length += LineLength(Point[0].Axes,Point[1].Axes,Size);
		Point[0] = Point[1];
	}
	
	return Length;

}


unsigned short StoppingDistance(float v_max, float a_max, float j_max, float v_act, float a_act, float* stopping_distance)
{//returns the stopping distance of a movement running at v_act and a_act given the
    //v_max, a_max and j_max costraints

    //NOTE: assumes positive direction of motion!!!
    
    /* check input parameters */
    if ((v_max<=0)||(a_max<=0)||(j_max<=0)||(v_act<=0))
    {
        *stopping_distance = 0;
        return 255;   
    }
    
    float dt[4],ds[4];
    float v_top,a_top;
    
    if(a_act > TRF_EPSILON)
    {// movement is accelerating -> need to bring acceleration to zero first
 
        dt[0] = a_act/j_max; 
        v_top = v_act + 0.5f*a_act*a_act/j_max; //speed reached at the end of the first section
        
        dt[2] = v_top/a_max - a_max/j_max;
        if (dt[2] <= 0) //no linear section
            a_max = sqrtf(v_top*j_max);

        dt[1] = a_max/j_max;
        dt[2] = v_top/a_max - a_max/j_max;
        dt[3] = a_max/j_max;
        
        ds[0] = v_act*dt[0] + 0.5f*a_act*dt[0]*dt[0] - j_max*dt[0]*dt[0]*dt[0]/6.0f;
        ds[1] = v_top*dt[1] - (j_max * dt[1]*dt[1]*dt[1] /6.0f);
        ds[2] = v_top*dt[2] - (0.5f * a_max *dt[2]*dt[2] + 0.5f * a_max*a_max *dt[2]*dt[2] / j_max);
        ds[3] = v_top*dt[3] - (- j_max*dt[3]*dt[3]*dt[3]/6.0f + 0.5f * a_max *dt[3]*dt[3] + (v_top-0.5f*a_max*a_max/j_max)*dt[3]);
 
    }
    else if (a_act < -TRF_EPSILON)
    {// movement is decelerating already -> find out what deceleration is needed to reach zero
				
        dt[0] = 0;
        a_act = fabs(a_act);
        v_top = v_act;
        
        a_top = sqrtf(j_max*v_act+0.5f*a_act*a_act);
        if (a_top < a_act)
            a_top = a_act;
        if (a_top > a_max)
            a_top = a_max;

        dt[1] = (a_top-a_act)/j_max;
        dt[2] = 1/a_top * (v_act -0.5f*a_top*a_top/j_max -a_act*(a_top-a_act)/j_max -0.5f*(a_top-a_act)*(a_top-a_act)/j_max);
        dt[3] = a_top/j_max;

        float v1 = v_top - (a_act*dt[1] + 0.5f *j_max * dt[1]*dt[1]);
        float v2 = v1 - a_top*dt[2];

        ds[0] = 0;
        ds[1] = v_top*dt[1] - (j_max * dt[1]*dt[1]*dt[1] /6.0f)- 0.5f * a_act * dt[1]*dt[1];
        ds[2] = v1 *dt[2] - (0.5f * a_top *dt[2]*dt[2]);
        ds[3] = v2 * dt[3] - (-j_max *dt[3]*dt[3]*dt[3] /6.0f + 0.5f * a_top* dt[3]*dt[3]);
 
    }
    else
    {// movement is running at constant speed -> decelerate with default acceleration
			
        dt[0] = 0;
        v_top = v_act;
      
        dt[2] = v_top/a_max - a_max/j_max;
        if (dt[2] <= 0) //no linear section
            a_max = sqrtf(v_top*j_max);

        dt[1] = a_max/j_max;
        dt[2] = v_top/a_max - a_max/j_max;
        dt[3] = a_max/j_max;
  
        ds[0] = 0;
        ds[1] = v_top*dt[1] - (j_max * dt[1]*dt[1]*dt[1] /6.0f);
        ds[2] = v_top*dt[2] - (0.5f * a_max *dt[2]*dt[2] + 0.5f * a_max*a_max *dt[2]*dt[2] / j_max);
        ds[3] = v_top*dt[3] - (- j_max*dt[3]*dt[3]*dt[3]/6.0f + 0.5f * a_max *dt[3]*dt[3] + (v_top-0.5f*a_max*a_max/j_max)*dt[3]);

    }
        
    *stopping_distance = ds[0]+ds[1]+ds[2]+ds[3];
    
    return 0;
    
}


