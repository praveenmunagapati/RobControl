#include "PathPlanner.h"
#include "RobControl.h"
#include "trig.h"
#include "Misc.h"
#include "constants.h"
#include "Transformations.h"
#include <math.h>

#define sign(a) ( (a<0)?-1:1 )
#define max(a,b) ( (a>=b)?a:b )
#define min(a,b) ( (a<b)?a:b )

static double crt(double x)
{ //cubic root
	if (x<0)
	{
		return -pow(-x,1.0/3.0);
	}
	else
	{
		return pow(x,1.0/3.0);
	}
} 

double LineLength(double P1[6],double P2[6], int Size)
{ //calculate length of line from point P1 to point P2
	int k;
	double sum;
	sum = 0;	
	for(k=0;k<Size;k++)
		sum += pow(P2[k]-P1[k],2.0);
	return sqrt(sum);	
}

double LineLengthCart(double P1[6],double P2[6])
{ //calculate length of line from point P1 to point P2 using only the three cartesian dimensions
	return sqrt(pow(P2[0]-P1[0],2.0)+pow(P2[1]-P1[1],2.0)+pow(P2[2]-P1[2],2.0));
}

double LineLengthAng(double P1[6],double P2[6], int Size)
{ //calculate length of angular movement from point P1 to point P2 using quaternions
	return sqrt(pow(P2[3]-P1[3],2.0)+pow(P2[4]-P1[4],2.0)+pow(P2[5]-P1[5],2.0));
}

double MinPathTime(double P1[6], double P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6])
{ //calculate minimum time to complete linear path between P1 and P2 given each axis max speed
	double t;
	double t_min = 0.0;
	int k;

	for(k=0;k<Size;k++)
	{
		if (P2[k]>=P1[k] && Limit[k].VelocityPos != 0) {
			t = (P2[k]-P1[k])/Limit[k].VelocityPos;
		} else if (P2[k]<P1[k] && Limit[k].VelocityNeg != 0){
			t = (P1[k]-P2[k])/Limit[k].VelocityNeg;
		}
		if (t > t_min)
		{// this axis is slower
			t_min = t;
		}
	}
	
	return t_min;

}


double VectorLength(double A[3])
{	//calculate length of vector A
	return (sqrt(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]));
}

unsigned short CrossProduct(double U[3], double V[3], double N[3])
{ // calculate scalar components of cross-product UxV
	
	N[0] = U[1]*V[2]-U[2]*V[1];
	N[1] = U[2]*V[0]-U[0]*V[2];
	N[2] = U[0]*V[1]-U[1]*V[0];

	return 0;
}

double DotProduct(double U[3], double V[3])
{
	return (U[0]*V[0] + U[1]*V[1] + U[2]*V[2]);
	
}

double AngleBetweenVectors(double U[6], double V[6])
{// calculate angle between 6-dimensional vectors in degrees

	//calculate vectors lengths
	double U_Length = sqrt(U[0]*U[0] + U[1]*U[1] + U[2]*U[2] + U[3]*U[3] + U[4]*U[4] + U[5]*U[5]);
	double V_Length = sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2] + V[3]*V[3] + V[4]*V[4] + V[5]*V[5]);

	//check that input is valid (vector length not zero)
	if ((U_Length < TRF_EPSILON)||(V_Length < TRF_EPSILON))
	{
		return -1;
	}
	
	//calculate dot product between U and V
	double UV_DotProduct = U[0]*V[0] + U[1]*V[1] + U[2]*V[2] + U[3]*V[3] + U[4]*V[4] + U[5]*V[5];
			
	//extract cosine of angle between U and V
	double UV_CosAngle = UV_DotProduct/(U_Length*V_Length);
	
	//limit it do +/- 1 range (in case of numerical errors)
	if(UV_CosAngle > 1)
		UV_CosAngle = 1;
	else if (UV_CosAngle < -1)
		UV_CosAngle = -1;
		
	//return angle between U and V (in 0..180deg range)
	return acosd(UV_CosAngle);

}


unsigned short PointsToVector(double P1[6], double P2[6], double V[3])
{ //make vector V from point P1 to point P2, i.e. V = P2-P1
	V[0] = P2[0]-P1[0];
	V[1] = P2[1]-P1[1];
	V[2] = P2[2]-P1[2];
	return 0;	
}

unsigned short Normalize(double V[3])
{
	double norm = VectorLength(V);
	if (norm != 0)
	{
		V[0] /= norm;
		V[1] /= norm;
		V[2] /= norm;
	}
	return 0;
}

unsigned short EvalCircle(Path_Type *Circle)
{//calculate all circle parameters
	
	double a,b,c,s;
	double A[3],B[3],C[3];
	double alpha, beta, gamma;
	
	/*** calculate radius and center of cicle using cross and dot product properties ***/
	
	a = LineLengthCart(Circle->MiddlePointPath,Circle->TargetPointPath);
	b = LineLengthCart(Circle->StartPointPath,Circle->TargetPointPath);
	c = LineLengthCart(Circle->StartPointPath,Circle->MiddlePointPath);

	PointsToVector(Circle->MiddlePointPath,Circle->TargetPointPath,A);
	PointsToVector(Circle->StartPointPath,Circle->TargetPointPath,B);
	PointsToVector(Circle->StartPointPath,Circle->MiddlePointPath,C);
	
	CrossProduct(C,A,Circle->Normal);
	s = VectorLength(Circle->Normal);
	
	/* check that the three points are not collinear */
	if (s<TRF_EPSILON)
	{ 
		return ERR_PP_CIRCLEPOINTS;	
	}

	Circle->Radius = a*b*c /(2.0*s); //radius

	Normalize(Circle->Normal);	//normal versor	
	
	alpha = a*a * DotProduct(B,C) / (2.0*s*s);
	beta = b*b * -DotProduct(A,C) / (2.0*s*s);
	gamma = c*c * DotProduct(B,A) / (2.0*s*s);

	/* circle center point coordinates */
	Circle->Center[0] = alpha * Circle->StartPointPath[0] + beta * Circle->MiddlePointPath[0] + gamma * Circle->TargetPointPath[0];
	Circle->Center[1] = alpha * Circle->StartPointPath[1] + beta * Circle->MiddlePointPath[1] + gamma * Circle->TargetPointPath[1];
	Circle->Center[2] = alpha * Circle->StartPointPath[2] + beta * Circle->MiddlePointPath[2] + gamma * Circle->TargetPointPath[2];

	
	/* calculate vectors from center to start, middle and target point */
	PointsToVector(Circle->Center,Circle->StartPointPath,Circle->StartVersor);
	Normalize(Circle->StartVersor);
	
	PointsToVector(Circle->Center,Circle->MiddlePointPath,Circle->MiddleVersor);	
	Normalize(Circle->MiddleVersor);

	PointsToVector(Circle->Center,Circle->TargetPointPath,Circle->EndVersor);	
	Normalize(Circle->EndVersor);

	/* calculate cross versor */
	CrossProduct(Circle->Normal,Circle->StartVersor,Circle->CrossVersor);	
	Normalize(Circle->CrossVersor);
	
	/* calculate arc length to end point */
    double Test[3];
    CrossProduct(Circle->StartVersor,Circle->EndVersor,Test);
    double tmpDotProd = DotProduct(Circle->StartVersor,Circle->EndVersor);
    //argument of acos must be between +/- 1 (avoid numerical errors)
    if (tmpDotProd > 1)
        tmpDotProd = 1;
    else if (tmpDotProd < -1)
        tmpDotProd = -1;
    Circle->Length = Circle->Radius * acos(tmpDotProd);
    if (sign(Circle->Normal[2])*sign(Test[2]) < 0)
    {
        Circle->Length = Circle->Radius * 2.0 * PI - Circle->Length;
    }
	
	/* calculate arc length to middle point */
	CrossProduct(Circle->StartVersor,Circle->MiddleVersor,Test);
	tmpDotProd = DotProduct(Circle->StartVersor,Circle->MiddleVersor);
	//argument of acos must be between +/- 1 (avoid numerical errors)
	if (tmpDotProd > 1)
		tmpDotProd = 1;
	else if (tmpDotProd < -1)
		tmpDotProd = -1;
	Circle->MiddleLength = Circle->Radius * acos(tmpDotProd);
	if (sign(Circle->Normal[2])*sign(Test[2]) < 0)
	{
		Circle->MiddleLength = Circle->Radius * 2.0 * PI - Circle->MiddleLength;
	}
	
	if ((Circle->Length <= 0)||(Circle->MiddleLength <= 0))
		return ERR_PP_CIRCLE_LENGTH;
	    
	return 0;
}


unsigned short MaxBlockSpeed(double d, double a, double j, double v_end, double *v_max)
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
		
	double Delta = 2.0*a*v_end/j + a*a*a/j/j;	
	if (d<=Delta)
	{// a cannot be reached -> reduce it. There is no linear acceleration section, it's all jerk limited movement
		double p3 = 2.0*v_end*j/3.0;
		double q2 = -d*j*j/2.0;
		double Det = p3*p3*p3+q2*q2;
		double alpha = crt(-q2+sqrt(Det));
		double betha = crt(-q2-sqrt(Det));
		double a_max = alpha + betha;
		*v_max = (v_end + a_max*a_max/j);
		return 0;
	}
	else
	{// a can be reached, there is a linear acceleration section
		double Det = a*a/j/j/4.0 + 2.0*d/a + v_end*v_end/a/a - v_end/j;
		*v_max = a*(sqrt(Det) - a/j/2.0);
		return 0;
	}

}



unsigned short MaxMovementDynamics(double d, double a, double j, double v, double v_start, double v_end, double a_start, double *v_max, double *a_up_max, double *a_down_max, double *d_linear)
{ //find maximum speed and acceleration of a movement with length d, speed v, acceleration a, jerk j, initial and final speed v_start and v_end, initial acceleration a_start
	//also returns size of linear speed interval
	
	double d_start, d_end;
	double t_start, t_end;
	double a_max, a_up, a_down;
	double dv_start, dv_end;
	
	double step, Delta, v_high, v_low;
	
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
	a_max = sqrt(j*fabs(dv_start)+0.5*a_start*a_start); //max acc (no linear segment) - normally limited by "a" (limit)

	/* REMOVED because v_end is not max speed and it can be exceeded. Think for example of a case when v_end=v_start.
	if (fabs(a_start)>a_max+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
		return 251;
	*/
	if (a_max < fabs(a_start)) a_max = fabs(a_start);
	if (a_max > a) a_max = a;
	if (a_max != 0)
	{
		double t0 = (a_max-fabs(a_start))/j;
		if (t0<0) t0=0;
		
		double t1 = 1/a_max * (fabs(dv_start) -0.5*a_max*a_max/j -fabs(a_start)*(a_max-fabs(a_start))/j -0.5*(a_max-fabs(a_start))*(a_max-fabs(a_start))/j);
		if (t1<0) t1=0;
		
		double t2 = a_max/j;

		t_start = t0 + t1 + t2;
		
		double v1 = v_start + (fabs(a_start) * t0 + 0.5 *j *t0*t0)*sign(dv_start); //speed at end of t0
		double v2 = v1 + (a_max*t1)*sign(dv_start); //speed at end of t1

		d_start = (v_start * t0 + (j * t0*t0*t0 / 6.0 + 0.5 * fabs(a_start) * t0*t0)* sign(dv_start));
		d_start += (v1 * t1 + (0.5 * a_max * t1*t1) * sign(dv_start));
		d_start += (v2 * t2 + (0.5 * a_max * t2*t2 - j * t2*t2*t2 / 6.0) * sign(dv_start));
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
	a_up = sqrt(j*fabs(dv_start)+0.5*a_start*a_start); //max acc (no linear segment) - normally limited by a (limit)
	if (fabs(a_start)>a_up+TRF_EPSILON)	//a_start too high, max speed will be exceeded!
		return 252;
	if (a_up > a) a_up = a;
	if (a_up != 0)
	{
		double t0 = (a_up-fabs(a_start))/j;
		double t1 = 1/a_up * (fabs(dv_start) -0.5*a_up*a_up/j -fabs(a_start)*(a_up-fabs(a_start))/j -0.5*(a_up-fabs(a_start))*(a_up-fabs(a_start))/j);
		double t2 = a_up/j;

		t_start = t0 + t1 + t2;
		
		double v1 = v_start + (fabs(a_start) * t0 + 0.5 *j *t0*t0)*sign(dv_start); //speed at end of t0
		double v2 = v1 + (a_up*t1)*sign(dv_start); //speed at end of t1

		d_start = (v_start * t0 + (j * t0*t0*t0 / 6.0 + 0.5 * fabs(a_start) * t0*t0)* sign(dv_start));
		d_start += (v1 * t1 + (0.5 * a_up * t1*t1) * sign(dv_start));
		d_start += (v2 * t2 + (0.5 * a_up * t2*t2 - j * t2*t2*t2 / 6.0) * sign(dv_start));
	}
	else
	{
		t_start = 0;
		d_start = 0;
	}
	
	// interval from v to v_end
	dv_end = fabs(v - v_end);
	a_down = sqrt(j*(dv_end));
	if (a_down > a) a_down = a;
	if (a_down != 0)
	{
		t_end = dv_end/a_down + a_down/j;
		d_end = 0.5 * (dv_end*dv_end/a_down + dv_end*a_down/j);
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

			double aaa = (v_start+v_end)/j+fabs(a_start)/j/j-fabs(a_start)*fabs(a_start)/j/j;
			double bbb = d + v_start*fabs(a_start)/j - fabs(a_start)*fabs(a_start)*fabs(a_start)/3.0/j/j;
			double ccc = (v_start-v_end+fabs(a_start)*fabs(a_start)/2.0/j)*(v_start+v_end+fabs(a_start)*fabs(a_start)/j);
			double ddd = bbb*bbb-aaa*ccc;		
			
			if ((ddd<0)||(aaa==0))
				return 254;	//should never happen with current settings
			
			*a_up_max = - (bbb - sqrt(ddd)) / aaa; //acceleration is negative because movement is speeding down to v_end

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
			step += 0.01;

			if (Delta < 0)
			{
				v_high = v;
				v = (v_high + v_low)/2.0;
			}
			else if (Delta > 0)
			{
				v_low = v;
				v = (v_high + v_low)/2.0;
			}
			else if (Delta == 0)
			{
				step = 1;	
			}

			// interval from v_start to v
			dv_start = v - v_start;
			a_up = sqrt(j*fabs(dv_start)+0.5*a_start*a_start); //max acc (no linear segment) - normally limited by a (limit)
			if (a_up > a) a_up = a;
			if (a_up != 0)
			{
				double t0 = (a_up-fabs(a_start))/j;
				double t1 = 1/a_up * (fabs(dv_start) -0.5*a_up*a_up/j -fabs(a_start)*(a_up-fabs(a_start))/j -0.5*(a_up-fabs(a_start))*(a_up-fabs(a_start))/j);
				double t2 = a_up/j;

				t_start = t0 + t1 + t2;
		
				double v1 = v_start + (fabs(a_start) * t0 + 0.5 *j *t0*t0)*sign(dv_start); //speed at end of t0
				double v2 = v1 + (a_up*t1)*sign(dv_start); //speed at end of t1

				d_start = (v_start * t0 + (j * t0*t0*t0 / 6.0 + 0.5 * fabs(a_start) * t0*t0)* sign(dv_start));
				d_start += (v1 * t1 + (0.5 * a_up * t1*t1) * sign(dv_start));
				d_start += (v2 * t2 + (0.5 * a_up * t2*t2 - j * t2*t2*t2 / 6.0) * sign(dv_start));
			}
			else
			{
				t_start = 0;
				d_start = 0;
			}
	
			// interval from v to v_end
			dv_end = v - v_end;
			a_down = sqrt(j*(dv_end));
			if (a_down > a) a_down = a;
			if (a_down != 0)
			{
				t_end = dv_end/a_down + a_down/j;
				d_end = 0.5 * (dv_end*dv_end/a_down + dv_end*a_down/j);
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


unsigned short EvaluateBezier (Frame_Type P[5], double u, Frame_Type *Q,int Size,int Order)
{// input are Bezier control points P0..P4, time u and size of coordinate system (number of robot's axes); output is point Q
	
	unsigned char k,i,j;
	int n = Order; // n=4 for quartic Bezier curves (used for round edges), n=3 for cubic Bezier curves (used for MS movement)

	if((u<0)||(u>1))
		return 255;
	
	Frame_Type Point[5];	
	for(i=0;i<=n;i++) Point[i]=P[i];
	
	for (k=1; k<=n; k++)
	{
		for (i=0; i<=n-k; i++)
		{
			for (j=0; j<Size; j++)
				Point[i].Axes[j] = (1.0-u)*Point[i].Axes[j] + u*Point[i+1].Axes[j];
		}
	}
	
	for (j=0; j<Size; j++)
		Q->Axes[j] = Point[0].Axes[j];
	
	return 0;
}

double BezierLength(Frame_Type P[5], int Size, int Order)
{// input are Bezier control points P0..P4 and size of coordinate system (number of robot's axes)

	Frame_Type Point[2];
	int i;
	int k = 10; //increase number of points for more accuracy
	double Length = 0;
	
	Point[0] = P[0];
	
	//evaluate the BezierCurve at different points and then add the distances between them
	for(i=1;i<=k;i++)
	{
		EvaluateBezier(P,(double)(i) / (double) (k),&(Point[1]),Size,Order);
		Length += LineLength(Point[0].Axes,Point[1].Axes,Size);
		Point[0] = Point[1];
	}
	
	return Length;

}

double BezierLengthHalf1(Frame_Type P[5], int Size, int Order)
{// input are Bezier control points P0..P4 and size of coordinate system (number of robot's axes)
 // calculates only length of first half P0..P2
	
	Frame_Type Point[2];
	int i;
	int k = 10; //increase number of points for more accuracy
	double Length = 0;
	
	Point[0] = P[0];
	
	//evaluate the BezierCurve at different points and then add the distances between them
	for(i=1;i<=k/2;i++)
	{
		EvaluateBezier(P,(double)(i) / (double) (k),&(Point[1]),Size,Order);
		Length += LineLength(Point[0].Axes,Point[1].Axes,Size);
		Point[0] = Point[1];
	}
	
	return Length;

}

double BezierLengthHalf2(Frame_Type P[5], int Size, int Order)
{// input are Bezier control points P0..P4 and size of coordinate system (number of robot's axes)
	// calculates only length of second half P2..P4

	Frame_Type Point[2];
	int i;
	int k = 10; //increase number of points for more accuracy
	double Length = 0;
	
	EvaluateBezier(P,0.5,&(Point[0]),Size,Order);
	
	//evaluate the BezierCurve at different points and then add the distances between them
	for(i=k/2+1;i<=k;i++)
	{
		EvaluateBezier(P,(double)(i) / (double) (k),&(Point[1]),Size,Order);
		Length += LineLength(Point[0].Axes,Point[1].Axes,Size);
		Point[0] = Point[1];
	}
	
	return Length;

}


unsigned short StoppingDistance(double v_max, double a_max, double j_max, double v_act, double a_act, double* stopping_distance)
{//returns the stopping distance of a movement running at v_act and a_act given the
    //v_max, a_max and j_max costraints

    //NOTE: assumes positive direction of motion!!!
    
    /* check input parameters */
    if ((v_max<=0)||(a_max<=0)||(j_max<=0)||(v_act<=0))
    {
        *stopping_distance = 0;
        return 255;   
    }
    
    double dt[4],ds[4];
    double v_top,a_top;
    
    if(a_act > TRF_EPSILON)
    {// movement is accelerating -> need to bring acceleration to zero first
 
        dt[0] = a_act/j_max; 
        v_top = v_act + 0.5*a_act*a_act/j_max; //speed reached at the end of the first section
        
        dt[2] = v_top/a_max - a_max/j_max;
        if (dt[2] <= 0) //no linear section
            a_max = sqrt(v_top*j_max);

        dt[1] = a_max/j_max;
        dt[2] = v_top/a_max - a_max/j_max;
        dt[3] = a_max/j_max;
        
        ds[0] = v_act*dt[0] + 0.5*a_act*dt[0]*dt[0] - j_max*dt[0]*dt[0]*dt[0]/6.0;
        ds[1] = v_top*dt[1] - (j_max * dt[1]*dt[1]*dt[1] /6.0);
        ds[2] = v_top*dt[2] - (0.5 * a_max *dt[2]*dt[2] + 0.5 * a_max*a_max *dt[2]*dt[2] / j_max);
        ds[3] = v_top*dt[3] - (- j_max*dt[3]*dt[3]*dt[3]/6.0 + 0.5 * a_max *dt[3]*dt[3] + (v_top-0.5*a_max*a_max/j_max)*dt[3]);
 
    }
    else if (a_act < -TRF_EPSILON)
    {// movement is decelerating already -> find out what deceleration is needed to reach zero
				
        dt[0] = 0;
        a_act = fabs(a_act);
        v_top = v_act;
        
        a_top = sqrt(j_max*v_act+0.5*a_act*a_act);
        if (a_top < a_act)
            a_top = a_act;
        if (a_top > a_max)
            a_top = a_max;

        dt[1] = (a_top-a_act)/j_max;
        dt[2] = 1/a_top * (v_act -0.5*a_top*a_top/j_max -a_act*(a_top-a_act)/j_max -0.5*(a_top-a_act)*(a_top-a_act)/j_max);
        dt[3] = a_top/j_max;

        double v1 = v_top - (a_act*dt[1] + 0.5 *j_max * dt[1]*dt[1]);
        double v2 = v1 - a_top*dt[2];

        ds[0] = 0;
        ds[1] = v_top*dt[1] - (j_max * dt[1]*dt[1]*dt[1] /6.0)- 0.5 * a_act * dt[1]*dt[1];
        ds[2] = v1 *dt[2] - (0.5 * a_top *dt[2]*dt[2]);
        ds[3] = v2 * dt[3] - (-j_max *dt[3]*dt[3]*dt[3] /6.0 + 0.5 * a_top* dt[3]*dt[3]);
 
    }
    else
    {// movement is running at constant speed -> decelerate with default acceleration
			
        dt[0] = 0;
        v_top = v_act;
      
        dt[2] = v_top/a_max - a_max/j_max;
        if (dt[2] <= 0) //no linear section
            a_max = sqrt(v_top*j_max);

        dt[1] = a_max/j_max;
        dt[2] = v_top/a_max - a_max/j_max;
        dt[3] = a_max/j_max;
  
        ds[0] = 0;
        ds[1] = v_top*dt[1] - (j_max * dt[1]*dt[1]*dt[1] /6.0);
        ds[2] = v_top*dt[2] - (0.5 * a_max *dt[2]*dt[2] + 0.5 * a_max*a_max *dt[2]*dt[2] / j_max);
        ds[3] = v_top*dt[3] - (- j_max*dt[3]*dt[3]*dt[3]/6.0 + 0.5 * a_max *dt[3]*dt[3] + (v_top-0.5*a_max*a_max/j_max)*dt[3]);

    }
        
    *stopping_distance = ds[0]+ds[1]+ds[2]+ds[3];
    
    return 0;
    
}


unsigned short DynamicLimitsViolated(double P1[6], double P2[6], int Size, struct Robot_Parameter_JointLimits_Type Limit[6], double CycleTime, double *redFactor)
{ //checks if the dynamic limits of the joints are being violated during the current cycle time
    //returns indexes(+1) of axes where largest violation occurs - e.g. 01010010 means violation on axes 1,4,6
    int k;
    double tmpRedFactor = 1.0;
   
    unsigned short BadAxes = 0;
    
    for(k=0;k<Size;k++)
    {
        double JointSpeed = (P2[k]-P1[k]) / CycleTime;
        if (JointSpeed != 0)
        {
            if ((JointSpeed > 0)&&(Limit[k].VelocityPos / JointSpeed < tmpRedFactor))
            {
                tmpRedFactor = Limit[k].VelocityPos / JointSpeed;
                BadAxes |= 1<<(k+1);
            }
            if ((JointSpeed < 0)&&(-Limit[k].VelocityNeg / JointSpeed < tmpRedFactor))
            {
                tmpRedFactor = -Limit[k].VelocityNeg / JointSpeed;
                BadAxes |= 1<<(k+1);
            }
        }
    }
	
    *redFactor = tmpRedFactor;

    return BadAxes;

}


unsigned short  LineCrossBox(double L1[6],double L2[6],double B1[6],double B2[6])
{ //checks if the segment from L1 to L2 crosses the box defined by B1 and B2
    //note that line (as defined by L1-L2) might still cross box but outside L1-L2 segment
    //in that case the function returns false
    //https://tavianator.com/fast-branchless-raybounding-box-intersections/
    
    double t[6];
    double D[3]; //line direction
    PointsToVector(L1,L2,D);
   
    double LARGE_NUMBER = 1e+10;
    
    int i;
    for (i=0;i<3;i++)
    {
        if (D[i]!=0)
        {
            t[2*i+0] = (B1[i] - L1[i])/D[i];
            t[2*i+1] = (B2[i] - L1[i])/D[i];
        }
        else if (L1[i]>=min(B1[i],B2[i]) && L1[i]<=max(B1[i],B2[i]))
        {
            t[2*i+0] = LARGE_NUMBER;
            t[2*i+1] = -LARGE_NUMBER;        
        }
        else
        {
            t[2*i+0] = LARGE_NUMBER;
            t[2*i+1] = LARGE_NUMBER;        
        }
    }

    double tmin = max(max(min(t[0], t[1]), min(t[2], t[3])), min(t[4], t[5]));
    double tmax = min(min(max(t[0], t[1]), max(t[2], t[3])), max(t[4], t[5]));
        
    if (tmax<0 || tmin>1 || tmin>tmax)
        return 0;
    else
        return 1;

}


unsigned short PointInBox(double P[6],double B1[6],double B2[6])
{ //checks if the point P is inside the box (returns 1) or outside (returns 0)
    // XYZ says what axes are in and out of the box limits
    
    int i;
    unsigned short tmpRet = 1;
            
    for (i=0;i<3;i++)
        tmpRet *= (unsigned short) (P[i]<=max(B1[i],B2[i]) && P[i]>=min(B1[i],B2[i]));
    
    return tmpRet;
}



unsigned short WorkspaceMonitor(unsigned char MovementType, Path_Type* Path, double Tool[6], Robot_Parameter_Workspace_Type Workspace[MAX_ZONE], unsigned short AxesNum, Mech_Type* Mechanics)
{ // check if planned movement violates the defined workspace
    // returns index of violated zone (1..MAX_ZONE), 0 otherwise

    //path workspace monitoring
    short calculatePoints = 0;
    double subInc,tmpRotAngle;
    int k,j;
    short sub;
    for (k=0;k<MAX_ZONE;k++)
    {
        //check for allowed and forbidden zones
        if (!Workspace[k].Type)
            continue;

        Frame_Type subPoints[WS_SUBPOINTS];

        memcpy(subPoints[0].Axes,Path->StartPointPath,sizeof(subPoints[0].Axes));
                                    
        //calculate inner subpoints in path only if not done yet
        //do not calculate at all if no zones are defined
        //avoid repeating calculations for following zones
        if (!calculatePoints)
        {
            calculatePoints = 1;
            subInc = 1.0/(WS_SUBPOINTS-1);                
            
            if (MovementType == MOVE_CIRCLE)
            {
                tmpRotAngle = Path->Length / Path->Radius;
                if (tmpRotAngle > 2*PI)
                    tmpRotAngle = 2*PI; //consider only one full rotation at maximum                                           
            }
            

            for (sub=1;sub<WS_SUBPOINTS;sub++)
            {

                Frame_Type tmpTargetJ, tmpTargetX;
                                            
                //interpolate movement at BlockLength/(WS_SUBPOINTS-1)*sub
                double u = sub * subInc;
                
                switch(MovementType)
                {
                    case MOVE_PTP:
                        for(j=0;j<AxesNum;j++)
                        {
                            tmpTargetJ.Axes[j] = (1-u) * Path->StartPointJoint[j] + u * Path->TargetPointJoint[j];
                        }           
                        //calculate corresponding TCP with direct TRF
                        Transformations(Mechanics,TRF_DIRECT,tmpTargetJ.Axes,Path->StartPointPath,tmpTargetX.Axes);
                        break;

                    case MOVE_CIRCLE:
                        for(j=0;j<3;j++)
                        {//X,Y,Z on circle
                            //P = C + R cos(t) U + R sin(t) V
                            tmpTargetX.Axes[j] = Path->Center[j] + Path->Radius * cos(tmpRotAngle * u) * Path->StartVersor[j] + 
                                Path->Radius * sin(tmpRotAngle * u) * Path->CrossVersor[j];
                        }
                        break;

                    case MOVE_SPLINE:      
                        EvaluateBezier(Path->Spline.CtrlPoint,u,&tmpTargetX,BEZIER_XYZ,BEZIER_CUBIC);
                        break;
                }   
            
                //add tool
                if (AxesNum < 5)
                {
                    SubFrame2D(Tool,tmpTargetX.Axes,subPoints[sub].Axes);
                }
                else
                {
                    SubFrame3D(Tool,tmpTargetX.Axes,0,0,0,subPoints[sub].Axes);
                }		                                
            }
        }

                                
        //check that lines between subpoints do not violate workspace
        for (sub=1;sub<WS_SUBPOINTS;sub++)
        {
            unsigned short tmpInsideL1 = PointInBox(subPoints[sub-1].Axes,Workspace[k].PositionMin,Workspace[k].PositionMax);
            unsigned short tmpInsideL2 = PointInBox(subPoints[sub].Axes,Workspace[k].PositionMin,Workspace[k].PositionMax);
            unsigned short tmpHitBox = LineCrossBox(subPoints[sub-1].Axes,subPoints[sub].Axes,Workspace[k].PositionMin,Workspace[k].PositionMax);
            if (((!tmpInsideL1 || !tmpInsideL2) && Workspace[k].Type==ZONE_SAFE) || (tmpHitBox && Workspace[k].Type==ZONE_FORBIDDEN))
            {                   
                return (k+1);
            }                
        }
    }            
    
    return 0; //no violation occurred if execution arrives here
    
}


//local help function
unsigned short ControlPoints(unsigned char MovementType,Path_Type* Path, double BlockLength, Frame_Type CtrlPoints[2], unsigned char Edge, unsigned short AxesNum, Mech_Type* Mechanics)
{//calculate two control points of a round edge
        
    int j;
    double DistA, DistB;
  
    switch (MovementType)
    {
        case MOVE_LINE:
            //linear interpolation
            if (Edge == EDGE_END)
            {
                DistA = 1 - Path->EndEdge.Radius / BlockLength;
                DistB = 1 - Path->EndEdge.Radius / BlockLength / 2.0;
            }
            else //EDGE_START
            {
                DistA = Path->StartEdge.Radius / BlockLength / 2.0;
                DistB = Path->StartEdge.Radius / BlockLength;
            }

            for (j=0;j<BEZIER_XYZ;j++) //note that only cartesian coordinates are included
            {
                CtrlPoints[0].Axes[j] = (1-DistA) * Path->StartPointPath[j] + DistA * Path->TargetPointPath[j];
                CtrlPoints[1].Axes[j] = (1-DistB) * Path->StartPointPath[j] + DistB * Path->TargetPointPath[j];
            }
            break;
            
        case MOVE_CIRCLE:
            
            if (Edge == EDGE_END)
            {
                DistA = 1 - Path->EndEdge.Radius / BlockLength;
                //find control point #0
                for (j=0;j<BEZIER_XYZ;j++) //note that only cartesian coordinates are included
                {
                    //P = C + R cos(t) U + R sin(t) V
                    CtrlPoints[0].Axes[j] = Path->Center[j] + Path->Radius * cos(DistA*BlockLength/Path->Radius) * Path->StartVersor[j] + Path->Radius * sin(DistA*BlockLength/Path->Radius) * Path->CrossVersor[j];
                }
                //find control point #1 along tangent at distance Path->EndEdge.Radius/2
                double tmpCrossVersor[3];
                PointsToVector(Path->Center,CtrlPoints[0].Axes,tmpCrossVersor);
                double tmpEndVersor[3];
                CrossProduct(Path->Normal,tmpCrossVersor,tmpEndVersor);
                Normalize(tmpEndVersor);
                for (j=0;j<BEZIER_XYZ;j++)
                {
                    CtrlPoints[1].Axes[j] = CtrlPoints[0].Axes[j] + tmpEndVersor[j] * Path->EndEdge.Radius/2;
                }
            }
            else //EDGE_START
            {
                DistA = Path->StartEdge.Radius / BlockLength;
                //find control point #4
                for (j=0;j<BEZIER_XYZ;j++) //note that only cartesian coordinates are included
                {
                    //P = C + R cos(t) U + R sin(t) V
                    CtrlPoints[1].Axes[j] = Path->Center[j] + Path->Radius * cos(DistA*BlockLength/Path->Radius) * Path->StartVersor[j] + Path->Radius * sin(DistA*BlockLength/Path->Radius) * Path->CrossVersor[j];
                }
                //find control point #3 along tangent at distance Path->EndEdge.Radius/2
                double tmpCrossVersor[3];
                PointsToVector(Path->Center,CtrlPoints[1].Axes,tmpCrossVersor);
                double tmpEndVersor[3];
                CrossProduct(Path->Normal,tmpCrossVersor,tmpEndVersor);
                Normalize(tmpEndVersor);
                for (j=0;j<BEZIER_XYZ;j++)
                {//negative because point #3 is closer to beginning of movement
                    CtrlPoints[0].Axes[j] = CtrlPoints[1].Axes[j] - tmpEndVersor[j] * Path->StartEdge.Radius/2;
                }
            }

            break;

        case MOVE_PTP:
            //first interpolate linearly in joints coords, then transform results into path coords
            //note that for PTP movements some control points should also hold the orientation (#0 for end edge and #4 for start edge)
            if (Edge == EDGE_END)
            {
                DistA = 1 - Path->EndEdge.Radius / BlockLength;
                
                //find control point #5 (is #0 in joint world)
                for(j=0;j<AxesNum;j++)
                {
                    CtrlPoints[5].Axes[j] = (1-DistA) * Path->StartPointJoint[j] + DistA * Path->TargetPointJoint[j];
                }

                //find control point #0
                Transformations(Mechanics,TRF_DIRECT,CtrlPoints[5].Axes,Path->TargetPointPath,CtrlPoints[0].Axes);
  
                //find control point #1 along tangent at distance Path->EndEdge.Radius/2
                DistA += (1.0/Path->EndEdge.Radius/10.0);
                if (DistA > 1)
                    DistA = 1;
                double tmpAxesValues[6];
                for(j=0;j<AxesNum;j++)
                {
                    tmpAxesValues[j] = (1-DistA) * Path->StartPointJoint[j] + (DistA) * Path->TargetPointJoint[j];
                }

                double tmpPathValues[6];
                Transformations(Mechanics,TRF_DIRECT,tmpAxesValues,Path->TargetPointPath,tmpPathValues);
                double tmpTangent[3];
                for (j=0;j<BEZIER_XYZ;j++)
                {
                    tmpTangent[j] = tmpPathValues[j] - CtrlPoints[0].Axes[j];
                }

                Normalize(tmpTangent);
                for (j=0;j<BEZIER_XYZ;j++)
                {
                    CtrlPoints[1].Axes[j] = CtrlPoints[0].Axes[j] + tmpTangent[j] * Path->EndEdge.Radius/2;
                }
            }
            else //EDGE_START
            {
                DistA = Path->StartEdge.Radius / BlockLength;
                
                //find control point #6 (is #4 in joint world)
                for(j=0;j<AxesNum;j++)
                {
                    CtrlPoints[3].Axes[j] = (1-DistA) * Path->StartPointJoint[j] + DistA * Path->TargetPointJoint[j];
                }
                
                //find control point #4
                Transformations(Mechanics,TRF_DIRECT,CtrlPoints[3].Axes,Path->StartPointPath,CtrlPoints[1].Axes);
  
                //find control point #3 along tangent at distance Path->StartEdge.Radius/2
                DistA -= (1.0/Path->StartEdge.Radius/10.0);
                if (DistA < 0)
                    DistA = 0;
                double tmpAxesValues[6];
                for(j=0;j<AxesNum;j++)
                {
                    tmpAxesValues[j] = (1-DistA) * Path->StartPointJoint[j] + DistA * Path->TargetPointJoint[j];
                }

                double tmpPathValues[6];
                Transformations(Mechanics,TRF_DIRECT,tmpAxesValues,Path->StartPointPath,tmpPathValues);
                double tmpTangent[3];
                for (j=0;j<BEZIER_XYZ;j++)
                {
                    tmpTangent[j] = tmpPathValues[j] - CtrlPoints[1].Axes[j];
                }

                Normalize(tmpTangent);
                for (j=0;j<BEZIER_XYZ;j++)
                {
                    CtrlPoints[0].Axes[j] = CtrlPoints[1].Axes[j] + tmpTangent[j] * Path->StartEdge.Radius/2;
                }
            }

            break;

    }
    
    return 0;

}


unsigned short RoundEdgePoints(MotionPackage_Type* Movement, MotionPackage_Type* MovementPrev, unsigned short AxesNum, Mech_Type* Mechanics)
{

    int k;
    
    //limit radius size to half of the block length
    Movement->Path.StartEdge.Radius = min(MovementPrev->Round,Movement->BlockLengthIdeal/2.0);
    MovementPrev->Path.EndEdge.Radius = min(MovementPrev->Round,MovementPrev->BlockLengthIdeal/2.0);
    
    //in case of circle also limit radius to half circumference (because BlockLengthIdeal could be many revolutions)
    if (Movement->MovementType == MOVE_CIRCLE)
    {
        Movement->Path.StartEdge.Radius = min(Movement->Path.StartEdge.Radius,Movement->Path.Radius*PI);
    }
    if (MovementPrev->MovementType == MOVE_CIRCLE)
    {
        MovementPrev->Path.EndEdge.Radius = min(MovementPrev->Path.EndEdge.Radius,MovementPrev->Path.Radius*PI);
    }
    
    if (Movement->BlockLengthIdeal != 0 && MovementPrev->BlockLengthIdeal != 0 && MovementPrev->MovementType != MOVE_SPLINE)
    {//both blocks have non-zero length -> can apply round edge
              
        //control points 0 and 1 (first half)
        ControlPoints(MovementPrev->MovementType,&MovementPrev->Path,MovementPrev->BlockLengthIdeal,&Movement->Path.StartEdge.CtrlPoint[0],EDGE_END,AxesNum,Mechanics);
        
        //control point 2 (middle point)
        memcpy(Movement->Path.StartEdge.CtrlPoint[2].Axes, Movement->Path.StartPointPath,sizeof(Movement->Path.StartEdge.CtrlPoint[2].Axes));

        //control points 3 and 4 (second half)        
        ControlPoints(Movement->MovementType,&Movement->Path,Movement->BlockLengthIdeal,&Movement->Path.StartEdge.CtrlPoint[3],EDGE_START,AxesNum,Mechanics);

        //calculate start edge length (only part included in current block) and previous block's end edge length (only part included in previous block)
        Movement->Path.StartEdge.Length = BezierLengthHalf2(Movement->Path.StartEdge.CtrlPoint,BEZIER_XYZ,BEZIER_QUARTIC);
        MovementPrev->Path.EndEdge.Length = BezierLengthHalf1(Movement->Path.StartEdge.CtrlPoint,BEZIER_XYZ,BEZIER_QUARTIC);

        // adjust length of previous and current block considering start edge
        //NOTE: Edge Radius and Length are in mm, but blocklength could be in deg (e.g. PTP or ML/MC with FA programmed)
        //solution: use percentage change over full block length
        Movement->BlockLength += ((Movement->Path.StartEdge.Length - Movement->Path.StartEdge.Radius)/Movement->BlockLengthIdeal * Movement->BlockLength);
        MovementPrev->BlockLength += ((MovementPrev->Path.EndEdge.Length - MovementPrev->Path.EndEdge.Radius)/MovementPrev->BlockLengthIdeal * MovementPrev->BlockLength);

    }
    else //there is no round edge -> all points are coincident at transition
    {				
        for(k=0;k<5;k++)
        {
            memcpy(Movement->Path.StartEdge.CtrlPoint[k].Axes,Movement->Path.StartPointPath,sizeof(Movement->Path.StartEdge.CtrlPoint[k].Axes));
        }
        for(k=5;k<7;k++)
        {
            memcpy(Movement->Path.StartEdge.CtrlPoint[k].Axes,Movement->Path.StartPointJoint,sizeof(Movement->Path.StartEdge.CtrlPoint[k].Axes));
        }
        Movement->Path.StartEdge.Length = 0;
        Movement->Path.StartEdge.Radius = 0;
        MovementPrev->Path.EndEdge.Length = 0;										
        MovementPrev->Path.EndEdge.Radius = 0;
    }
				
    //copy points to previous block
    memcpy(MovementPrev->Path.EndEdge.CtrlPoint,Movement->Path.StartEdge.CtrlPoint,sizeof(Frame_Type)*7);
    
    return 0;
}



double PTPLength(Path_Type* Path, unsigned short AxesNum, Mech_Type* Mechanics)
{ //calculates approx. cartesian length of PTP movement (required when adding round edge to PTP)
    
    Frame_Type Point[2];
    int i,j;
    int k = 10; //increase number of points for more accuracy
    double Length = 0;
	
    memcpy(Point[0].Axes,Path->StartPointPath,sizeof(Point[0].Axes));
	
    //evaluate the BezierCurve at different points and then add the distances between them
    for(i=1;i<=k;i++)
    {
        double tmpAxesValues[6];
        //interpolate PTP to find next point
        for(j=0;j<AxesNum;j++)
        {
            tmpAxesValues[j] = (1-(double)i/(double)k) * Path->StartPointJoint[j] + (double)i/(double)k * Path->TargetPointJoint[j];
        }
        
        //transform in path coords. (assuming direct TRF never fail...)					
        Transformations(Mechanics,TRF_DIRECT,tmpAxesValues,Point[0].Axes,Point[1].Axes);

        //calculate segment length
        Length += LineLength(Point[0].Axes,Point[1].Axes,BEZIER_XYZ);
        Point[0] = Point[1];
    }
	
    return Length;

}


