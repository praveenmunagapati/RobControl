#include "Robots.h"
#include <math.h>
#include "trig.h"
#include "RobControl.h"
#include "Frame.h"
#include "Misc.h"
#include "constants.h"

unsigned short ArmDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6])
{ //direct transformations for 6ax robot
	
    //simplify notation
    float Q1 = JointAxes[0];
    float Q2 = JointAxes[1];
    float Q3 = JointAxes[2];
    float Q4 = JointAxes[3];
    float Q5 = JointAxes[4];
    float Q6 = JointAxes[5];
	
    float a1x = Links[1].Offset.X;
    //float a1y = Links[1].Offset.Y;
    float a1z = Links[1].Offset.Z;
    //float a2x = Links[2].Offset.X;
    //float a2y = Links[2].Offset.Y;
    float a2z = Links[2].Offset.Z;
    float a3x = Links[3].Offset.X;
    //float a3y = Links[3].Offset.Y;
    float a3z = Links[3].Offset.Z;
    float a4x = Links[4].Offset.X;
    float a5x = Links[5].Offset.X;
	
    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.X;
    ZeroFrame[4] = Links[0].Rotation.Y;
    ZeroFrame[5] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6];

    /* check mechanical parameters consistency */
    if ((a1z < 0)||(a2z <= 0)||(a3z < 0)||((a3x+a4x) <= 0)||(a5x <= 0))
    {
        return ERR_TRF_MECH;
    }

    float aaa = cosd(Q3)*(a3x+a4x+cosd(Q5)*a5x) + sind(Q3)*(a3z-cosd(Q4)*sind(Q5)*a5x);
    float bbb = -sind(Q3)*(a3x+a4x+cosd(Q5)*a5x) + cosd(Q3)*(a3z-cosd(Q4)*sind(Q5)*a5x);
    float ddd = a1x + cosd(Q2)*aaa + sind(Q2)*(bbb+a2z);
    float fff = a1z - sind(Q2)*aaa + cosd(Q2)*(bbb+a2z);

    /* X axis */
    tmpAxes[0] = cosd(Q1)*ddd - sind(Q1)*sind(Q4)*sind(Q5)*a5x;

    /* Y axis */	
    tmpAxes[1] = sind(Q1)*ddd + cosd(Q1)*sind(Q4)*sind(Q5)*a5x;
	
    /* Z axis */
    tmpAxes[2] = fff;

	
    /* compose R_total from all joints rotations */	
    float R_total[3][3];
    float temp01 = -sind(Q1)*cosd(Q4)+cosd(Q1)*sind(Q2+Q3)*sind(Q4);
    float temp02 = cosd(Q1)*cosd(Q2+Q3)*sind(Q5)+cosd(Q5)*sind(Q1)*sind(Q4)+cosd(Q5)*cosd(Q1)*sind(Q2+Q3)*cosd(Q4);
    float temp11 = cosd(Q1)*cosd(Q4)+sind(Q1)*sind(Q2+Q3)*sind(Q4);
    float temp12 = sind(Q1)*cosd(Q2+Q3)*sind(Q5)-cosd(Q1)*sind(Q4)*cosd(Q5)+cosd(Q5)*sind(Q1)*sind(Q2+Q3)*cosd(Q4);
    float temp21 = cosd(Q2+Q3)*sind(Q4);
    float temp22 = -sind(Q2+Q3)*sind(Q5)+cosd(Q2+Q3)*cosd(Q4)*cosd(Q5);

    R_total[0][0] = cosd(Q1)*cosd(Q2+Q3)*cosd(Q5)-sind(Q1)*sind(Q4)*sind(Q5)-cosd(Q1)*sind(Q2+Q3)*cosd(Q4)*sind(Q5);
    R_total[0][1] = cosd(Q6)*temp01 + sind(Q6)*temp02;
    R_total[0][2] = -sind(Q6)*temp01 + cosd(Q6)*temp02;
	
    R_total[1][0] = sind(Q1)*cosd(Q2+Q3)*cosd(Q5)+cosd(Q1)*sind(Q4)*sind(Q5)-sind(Q1)*sind(Q2+Q3)*cosd(Q4)*sind(Q5);
    R_total[1][1] = cosd(Q6)*temp11 + sind(Q6)*temp12;
    R_total[1][2] = -sind(Q6)*temp11 + cosd(Q6)*temp12;
	
    R_total[2][0] = -sind(Q2+Q3)*cosd(Q5)-cosd(Q2+Q3)*cosd(Q4)*sind(Q5);
    R_total[2][1] = cosd(Q6)*temp21 + sind(Q6)*temp22;
    R_total[2][2] = -sind(Q6)*temp21 + cosd(Q6)*temp22;
	
    /* A,B,C axis */
    float A,B,C;
    DecomposeMatrix(R_total,PathAxes[3],PathAxes[4],PathAxes[5],&A,&B,&C);
    tmpAxes[3] = A;
    tmpAxes[4] = B;
    tmpAxes[5] = C;

    /* consider zero offset (position and orientation of base point with respect to world origin) */
    SubFrame3D(tmpAxes,ZeroFrame,PathAxes[3],PathAxes[4],PathAxes[5],Axes);

    int i=0;
    for(i=0;i<6;i++)
    {
        Axes[i] = RoundToEpsilon(Axes[i]);
    }

    return STATUS_OK;

} 



unsigned short ArmInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6])
{ //inverse transformations for 6ax robot

    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.X;
    ZeroFrame[4] = Links[0].Rotation.Y;
    ZeroFrame[5] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6], WP[6];

    /* consider zero offset (position and orientation of base point with respect to world origin) */
    AddFrame3D(PathAxes,ZeroFrame,PathAxes[3],PathAxes[4],PathAxes[5],tmpAxes);

    //simplify notation
    float X = tmpAxes[0];
    float Y = tmpAxes[1];
    float Z = tmpAxes[2];
    float A = tmpAxes[3];
    float B = tmpAxes[4];
    float C = tmpAxes[5];

    float a1x = Links[1].Offset.X;
    //float a1y = Links[1].Offset.Y;
    float a1z = Links[1].Offset.Z;
    //float a2x = Links[2].Offset.X;
    //float a2y = Links[2].Offset.Y;
    float a2z = Links[2].Offset.Z;
    float a3x = Links[3].Offset.X;
    //float a3y = Links[3].Offset.Y;
    float a3z = Links[3].Offset.Z;
    float a4x = Links[4].Offset.X;
    float a5x = Links[5].Offset.X;
	
    //keep same pose as current joints configuration
    short Pose = 0;
    if (JointAxes[2] < -90)
    {
        Pose |= TRF_POSE_CONCAVE;
    }
    else
    {
        Pose |= TRF_POSE_CONVEX;
    }

    if (JointAxes[4] < 0)
    {
        Pose |= TRF_POSE_NEGATIVE;
    }
    else
    {
        Pose |= TRF_POSE_POSITIVE;
    }

    /* check mechanical parameters consistency */
    if ((a1z < 0)||(a2z <= 0)||(a3z < 0)||((a3x+a4x) <= 0)||(a5x <= 0))
    {
        return ERR_TRF_MECH;
    }

    /* compose rotation matrix  */
    float R_total[3][3];
    ComposeMatrix(R_total,A,B,C);

    /* calculate wrist point WP from mounting point MP */
    WP[0] = X - a5x * R_total[0][0];
    WP[1] = Y - a5x * R_total[1][0];
    WP[2] = Z - a5x * R_total[2][0];
	
    /* calculate Q1 */	
    /* check for singularity */
    if ((fabs(WP[0]) < TRF_EPSILON)&&(fabs(WP[1]) < TRF_EPSILON))
    {
        Axes[0] = JointAxes[0];
    }
    else
    {
        Axes[0] = atan2d(WP[1],WP[0]);
    }

    /* adjust positions of Q1 with +-PI to bring it closer to desired values */
    Axes[0] = ModuloPI(Axes[0],JointAxes[0]);


    /* consider axis 3 shoulder */
    float b3x = sqrtf((a3x+a4x)*(a3x+a4x)+a3z*a3z);

    /* calculate length and height of triangle formed by a2z and b3x */
    float height = WP[2] - a1z;
    float length = sqrtf(WP[0]*WP[0] + WP[1]*WP[1]);

    //flip sign of length if Q1 was chosen 180 deg away from atan(Y,X)
    if (fabs(Modulo2PI(Axes[0] - atan2d(WP[1],WP[0]),0))>90)
    {
        length = -length;
    }

    //add a1x correction
    length -= a1x;

    float rho = sqrtf(length*length + height*height);

    //check for workspace violations
    if ( (rho > (a2z + b3x + TRF_EPSILON))||(rho < (fabs(a2z-b3x) - TRF_EPSILON)))
    {
        return ERR_TRF_WORKSPACE;
    }
    else if (rho > (a2z + b3x)) //adjust impreciseness
    {
        rho = (a2z + b3x);
    }
    else if (rho < fabs(a2z-b3x)) //adjust impreciseness
    {
        rho = fabs(a2z-b3x);
    }

    float alpha = atan2d(height,length);
    float cos_beta = (rho*rho + a2z*a2z - b3x*b3x) / (2.0f*a2z*rho);
    float beta = atan2d(sqrtf(1-cos_beta*cos_beta),cos_beta);
    float cos_gamma = (a2z*a2z + b3x*b3x - rho*rho) / (2.0f*a2z*b3x);
    float gamma = 180.0f - atan2d(sqrtf(1-cos_gamma*cos_gamma),cos_gamma);


    if (Pose & TRF_POSE_CONCAVE)
    {
        Axes[1] = 90.0f - alpha + beta;
        Axes[2] = - gamma - atan2d(a3x+a4x,a3z);
    }
    else
    {
        Axes[1] = 90.0f - alpha - beta;
        Axes[2] = gamma - atan2d(a3x+a4x,a3z);
    }

    int i=0;
    for(i=0;i<3;i++)
    {
        Axes[i] = RoundToEpsilon(Axes[i]);
    }

    /* compute wrist rotation matrix */
    /* R_wrist = (R_arm)^T * R_total*/
    float R_arm[3][3];
    float R_wrist[3][3];

    //R_arm = Rz(Q1) * Ry(Q2+Q3)
    float Qy = Axes[1] + Axes[2];
    float Qz = Axes[0];
    R_arm[0][0] = cosd(Qz)*cosd(Qy);
    R_arm[0][1] = -sind(Qz);
    R_arm[0][2] = sind(Qy)*cosd(Qz);
    R_arm[1][0] = cosd(Qy)*sind(Qz);
    R_arm[1][1] = cosd(Qz);
    R_arm[1][2] = sind(Qy)*sind(Qz);
    R_arm[2][0] = -sind(Qy);
    R_arm[2][1] = 0;
    R_arm[2][2] = cosd(Qy);


    //transpose R_arm
    float tmpR;
    tmpR = R_arm[0][1];
    R_arm[0][1] = R_arm[1][0];
    R_arm[1][0] = tmpR;
    tmpR = R_arm[0][2];
    R_arm[0][2] = R_arm[2][0];
    R_arm[2][0] = tmpR;
    tmpR = R_arm[1][2];
    R_arm[1][2] = R_arm[2][1];
    R_arm[2][1] = tmpR;

    MatMult(R_arm,R_total,R_wrist);

    /* extract Q4,Q5,Q6 from wrist rotation matrix as XYX Euler angles */ 
    /* note that this angle type is not the same as the one used in the decompose matrix function in the frame.h file */
	
    float A_temp[2],B_temp[2],C_temp[2],ABC_dist[2];
    float A_actual = JointAxes[3];
    float B_actual = JointAxes[4];
    float C_actual = JointAxes[5];

    B_temp[0] = atan2d(sqrtf(1-R_wrist[0][0]*R_wrist[0][0]), R_wrist[0][0]);
    B_temp[1] = atan2d(-sqrtf(1-R_wrist[0][0]*R_wrist[0][0]), R_wrist[0][0]);

    if (fabs(B_temp[0])>TRF_EPSILON)
    {
        C_temp[0] = atan2d(R_wrist[0][1],R_wrist[0][2]);
        C_temp[1] = atan2d(-R_wrist[0][1],-R_wrist[0][2]);

        A_temp[0] = atan2d(R_wrist[1][0],-R_wrist[2][0]); 
        A_temp[1] = atan2d(-R_wrist[1][0],R_wrist[2][0]); 
    }
    else
    {	//singularity - choose A=currentQ4
        A_temp[0] = A_temp[1] = A_actual;
        C_temp[0] = C_temp[1] = atan2d(-R_wrist[1][2],R_wrist[2][2]) - A_actual;
    }
	
    //A, C modulo +-2PI to bring them closer to current values
    A_temp[0] = Modulo2PI(A_temp[0],JointAxes[3]);
    A_temp[1] = Modulo2PI(A_temp[1],JointAxes[3]);

    C_temp[0] = Modulo2PI(C_temp[0],JointAxes[5]);
    C_temp[1] = Modulo2PI(C_temp[1],JointAxes[5]);

    //calculate distance of the two solutions from actual values
    ABC_dist[0] = fabs(A_temp[0]-A_actual) + fabs(B_temp[0]-B_actual) + fabs(C_temp[0]-C_actual);
    ABC_dist[1] = fabs(A_temp[1]-A_actual) + fabs(B_temp[1]-B_actual) + fabs(C_temp[1]-C_actual);
    
    //keep same pose for wrist
    if (Pose & TRF_POSE_NEGATIVE)
    { //use solution with negative Q5
        if((B_temp[0]<0)&&(B_temp[1]>=0))
        { //use B_temp[0]
            Axes[3] = A_temp[0];
            Axes[4] = B_temp[0];
            Axes[5] = C_temp[0];            
        }
        else if((B_temp[1]<0)&&(B_temp[0]>=0))
        { //use B_temp[1]
            Axes[3] = A_temp[1];
            Axes[4] = B_temp[1];
            Axes[5] = C_temp[1];
        }
        else
        { //use closest solution to current values
            if (ABC_dist[0] <= ABC_dist[1])
            {
                Axes[3] = A_temp[0];
                Axes[4] = B_temp[0];
                Axes[5] = C_temp[0];
            }
            else
            {
                Axes[3] = A_temp[1];
                Axes[4] = B_temp[1];
                Axes[5] = C_temp[1];
            }	    
        }
    }
    else
    { //use solution with positive Q5
        if((B_temp[0]>=0)&&(B_temp[1]<0))
        { //use B_temp[0]
            Axes[3] = A_temp[0];
            Axes[4] = B_temp[0];
            Axes[5] = C_temp[0];            
        }
        else if((B_temp[1]>=0)&&(B_temp[0]<0))
        { //use B_temp[1]
            Axes[3] = A_temp[1];
            Axes[4] = B_temp[1];
            Axes[5] = C_temp[1];
        }
        else
        { //use closest solution to current values
            if (ABC_dist[0] <= ABC_dist[1])
            {
                Axes[3] = A_temp[0];
                Axes[4] = B_temp[0];
                Axes[5] = C_temp[0];
            }
            else
            {
                Axes[3] = A_temp[1];
                Axes[4] = B_temp[1];
                Axes[5] = C_temp[1];
            }	    
        }
    }

    //adjust positions of Q6 with +-2PI to bring it closer to desired value
    Axes[3] = Modulo2PI(Axes[3],JointAxes[3]);
    Axes[5] = Modulo2PI(Axes[5],JointAxes[5]);
    
    for(i=3;i<6;i++)
    {
        Axes[i] = RoundToEpsilon(Axes[i]);
    }

    return STATUS_OK;

} 



unsigned short ScaraDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6])
{ //direct transformations for Scara robot
	
    //simplify notation
    float Q1 = JointAxes[0];
    float Q2 = JointAxes[1];
    float Q3 = JointAxes[2];
    float Q4 = JointAxes[3];
    float a1x = Links[1].Offset.X;
    float a1z = Links[1].Offset.Z;
    float a2x = Links[2].Offset.X;
    float a2z = Links[2].Offset.Z;
    float a4z = Links[4].Offset.Z;

    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6];

    /* check mechanical parameters consistency */
    if ((a1x <= 0)||(a2x <= 0))
    {
        return ERR_TRF_MECH;
    }

    /* X axis */
    tmpAxes[0] = a1x * cosd(Q1) + a2x * cosd(Q1+Q2);

    /* Y axis */	
    tmpAxes[1] = a1x * sind(Q1) + a2x * sind(Q1+Q2);
	
    /* Z axis */
    tmpAxes[2] = a1z + a2z + Q3 + (a4z * Q4 / 360.0f);

    /* C axis */
    tmpAxes[3] = Q1 + Q2 + Q4;

    /* adjust positions of axis C with +-2PI to bring it closer to the desired value */
    tmpAxes[3] = Modulo2PI(tmpAxes[3],PathAxes[3]);

    /* consider zero offset (position and orientation of base point with respect to world origin) */
    SubFrame2D(tmpAxes,ZeroFrame,Axes);

    return 0; //STATUS_OK;

} 


unsigned short ScaraInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6])
{ //inverse transformations for Scara robot

    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6];

    /* consider zero offset (position and orientation of base point with respect to world origin) */
    AddFrame2D(PathAxes,ZeroFrame,tmpAxes);

    //simplify notation
    float X = tmpAxes[0];
    float Y = tmpAxes[1];
    float Z = tmpAxes[2];
    float C = tmpAxes[3];
    float a1x = Links[1].Offset.X;
    float a1z = Links[1].Offset.Z;
    float a2x = Links[2].Offset.X;
    float a2z = Links[2].Offset.Z;
    float a4z = Links[4].Offset.Z;

    //keep same pose as current joints configuration
    short Pose;
    if (JointAxes[1] < 0)
    {
        Pose = TRF_POSE_LEFT;
    }
    else
    {
        Pose = TRF_POSE_RIGHT;
    }


    /* check mechanical parameters consistency */
    if ((a1x <= 0)||(a2x <= 0))
    {
        return ERR_TRF_MECH;
    }

    float D = (X*X + Y*Y - a1x*a1x - a2x*a2x) / (2 * a1x * a2x);

    if ((D*D) > 1 + TRF_EPSILON)
    {
        return ERR_TRF_WORKSPACE;
    }
    else if ((D*D) > 1)	//round numerical impreciseness back to 1
    {
        D = 1;	
    }

    if ((X <= TRF_EPSILON)&&(Y <= TRF_EPSILON))	//TCP in origin
    {
        return ERR_TRF_WORKSPACE;
    }

    /* Q1 and Q2 axis */
    if (Pose == TRF_POSE_RIGHT)
    {
        Axes[0] = atan2d(Y,X) - atan2d(a2x*sqrtf(1-D*D),a1x + a2x*D);
        Axes[1] = atan2d(sqrtf(1-D*D),D);
    }
    else if (Pose == TRF_POSE_LEFT)
    {
        Axes[0] = atan2d(Y,X) - atan2d(-a2x*sqrtf(1-D*D), a1x + a2x*D);
        Axes[1] = atan2d(-sqrtf(1-D*D),D);
    }
    else
    {
        return(ERR_TRF_POSE);
    }

    /* adjust positions of Q1 and Q2 with +-2PI to bring them closer to desired values */
    Axes[0] = Modulo2PI(Axes[0],JointAxes[0]);
    Axes[1] = Modulo2PI(Axes[1],JointAxes[1]);
    float Q1 = Axes[0];
    float Q2 = Axes[1];

    /* Q4 axis */
    Axes[3] = C - Q1 - Q2;
    /* adjust positions of Q3 with +-2PI to bring it closer to desired values */
    Axes[3] = Modulo2PI(Axes[3],JointAxes[3]);
    float Q4 = Axes[3];
	
    /* Q3 axis */
    Axes[2] = Z - a1z - a2z - (a4z * Q4 / 360.0f);

    return STATUS_OK;

} 




unsigned short PalletDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6])
{ //direct transformations for Palletizer robot
	
    //simplify notation
    float Q1 = JointAxes[0];
    float Q2 = JointAxes[1];
    //	float Q3 = JointAxes[2] - JointAxes[1]; // mechanical coupling
    float Q3 = JointAxes[2];
    float Q4 = JointAxes[3];
    float a1x = Links[1].Offset.X;
    float a1z = Links[1].Offset.Z;
    float a2z = Links[2].Offset.Z;
    float a3x = Links[3].Offset.X;
    float a4x = Links[4].Offset.X;
    float a4z = Links[4].Offset.Z;

    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6];

    /* check mechanical parameters consistency */
    if ((a1z < 0)||(a2z <= 0)||(a3x <= 0)||(a4x < 0)||(a4z < 0))
    {
        return ERR_TRF_MECH;
    }

    float rho = a1x + a2z*sind(Q2) + a3x*cosd(Q2+Q3) + a4x;

    /* X axis */
    tmpAxes[0] = rho * cosd(Q1);

    /* Y axis */	
    tmpAxes[1] = rho * sind(Q1);
	
    /* Z axis */
    tmpAxes[2] = a1z + a2z*cosd(Q2) - a3x*sind(Q2+Q3) - a4z;

    /* C axis */
    tmpAxes[3] = Q1 + Q4;
    /* adjust positions of axis C with +-2PI to bring it closer to the desired value */
    tmpAxes[3] = Modulo2PI(tmpAxes[3],PathAxes[3]);

    /* consider zero offset (position and orientation of base point with respect to world origin) */
    SubFrame2D(tmpAxes,ZeroFrame,Axes);

    return STATUS_OK;

} 


unsigned short PalletInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6])
{ //inverse transformations for Palletizer robot

    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6];

    /* consider zero offset (position and orientation of base point with respect to world origin) */
    AddFrame2D(PathAxes,ZeroFrame,tmpAxes);

    //simplify notation
    float X = tmpAxes[0];
    float Y = tmpAxes[1];
    float Z = tmpAxes[2];
    float C = tmpAxes[3];
    float a1x = Links[1].Offset.X;
    float a1z = Links[1].Offset.Z;
    float a2z = Links[2].Offset.Z;
    float a3x = Links[3].Offset.X;
    float a4x = Links[4].Offset.X;
    float a4z = Links[4].Offset.Z;

    /* check mechanical parameters consistency */
    if ((a1z < 0)||(a2z <= 0)||(a3x <= 0)||(a4x < 0)||(a4z < 0))
    {
        return ERR_TRF_MECH;
    }

    /* calculate Q1 */	
    /* check for singularity */
    if ((fabs(X) < TRF_EPSILON)&&(fabs(Y) < TRF_EPSILON))
    {
        Axes[0] = JointAxes[0];
    }
    else
    {
        Axes[0] = atan2d(Y,X);
    }

    /* adjust positions of Q1 with +-PI to bring it closer to desired values */
    Axes[0] = ModuloPI(Axes[0],JointAxes[0]);
    float Q1 = Axes[0];


    /* calculate length and height of triangle formed by a2z and a3x */
    float height = Z - a1z + a4z;
    float length = sqrtf(X*X + Y*Y);
    //flip sign of length if Q1 was chosen 180 deg away from atan(Y,X)
    if (fabs(Modulo2PI(Axes[0] - atan2d(Y,X),0))>90)
    {
        length = -length;
    }
    //add a1x correction
    length -= (a1x+a4x);

    float rho = sqrtf(length*length + height*height);

    //check for workspace violations
    if ( (rho > (a2z + a3x + TRF_EPSILON))||(rho < (fabs(a2z-a3x) - TRF_EPSILON)))
    {
        return ERR_TRF_WORKSPACE;
    }
    else if (rho > (a2z + a3x)) //adjust impreciseness
    {
        rho = (a2z + a3x);
    }
    else if (rho < fabs(a2z-a3x)) //adjust impreciseness
    {
        rho = fabs(a2z-a3x);
    }


    float alpha = atan2d(height,length);
    float cos_beta = (rho*rho + a2z*a2z - a3x*a3x) / (2.0f*a2z*rho);
    float beta = atan2d(sqrtf(1-cos_beta*cos_beta),cos_beta);
    float cos_gamma = (a2z*a2z + a3x*a3x - rho*rho) / (2.0f*a2z*a3x);
    float gamma = 180.0f - atan2d(sqrtf(1-cos_gamma*cos_gamma),cos_gamma);


    /* Q2 axis */
    Axes[1] = 90.0f - alpha - beta;
	
    /* Q3 axis */
    Axes[2] = gamma - 90;

    /* Q4 axis */
    Axes[3] = C - Q1;
    //adjust positions of Q4 with +-2PI to bring it closer to desired values
    Axes[3] = Modulo2PI(Axes[3],JointAxes[3]);

    return STATUS_OK;

} 



unsigned short DeltaDirect(Link_Type Links[6], float JointAxes[6], float PathAxes[6], float Axes[6])
{ //direct transformations for Delta robot
	
    //simplify notation
    float Q1 = JointAxes[0];
    float Q2 = JointAxes[1];
    float Q3 = JointAxes[2];
    float Q4 = JointAxes[3];
    float a1x = Links[1].Offset.X;	//radius top
    float a1z = Links[1].Offset.Z;	//arm top
    float a2x = Links[2].Offset.X;	//radius bottom
    float a2z = Links[2].Offset.Z;	//arm bottom
    float a3z = Links[3].Offset.Z;	//bottom vertical offset

    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6];

    /* check mechanical parameters consistency */
    if ((a1x <= 0)||(a1z <= 0)||(a2x <= 0)||(a2z <= 0)||(a3z < 0))
    {
        return ERR_TRF_MECH;
    }

    float a1y = a1x * 2.0f * sqrt3;	//base side top
    float a2y = a2x * 2.0f * sqrt3;	//base side bottom

    float t = (a1y - a2y) * tan30 / 2.0f;

    float y1 = -(t + a1z * cosd(Q1));
    float z1 = -a1z * sind(Q1);
 
    float y2 = (t + a1z * cosd(Q2)) * sin30;
    float x2 = y2 * tan60;
    float z2 = -a1z * sind(Q2);
	 
    float y3 = (t + a1z * cosd(Q3)) * sin30;
    float x3 = -y3 * tan60;
    float z3 = -a1z * sind(Q3);
	 
    float dnm = (y2-y1) * x3 - (y3-y1) * x2;
    if (fabs(dnm) < TRF_EPSILON) 
    {
        return ERR_TRF_WORKSPACE;
    }
 
    float w1 = y1*y1 + z1*z1;
    float w2 = x2*x2 + y2*y2 + z2*z2;
    float w3 = x3*x3 + y3*y3 + z3*z3;
     
    // x = (a1*z + b1)/dnm
    float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0f;
 
    // y = (a2*z + b2)/dnm;
    float a2 = -(z2-z1)*x3+(z3-z1)*x2;
    float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0f;
 
    // a*z^2 + b*z + c = 0
    float a = a1*a1 + a2*a2 + dnm*dnm;
    float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - a2z*a2z);
  
    // discriminant
    float d = b*b - (float)4.0f*a*c;
    if (d < TRF_EPSILON) 
    {
        return ERR_TRF_WORKSPACE;
    }
    else 
    {

        /* Z axis */
        tmpAxes[2] = -0.5*(b+sqrtf(d))/a -a3z;
        float Z = Axes[2];

        /* X axis */
        tmpAxes[0] = (a1 * Z + b1)/dnm;
	
        /* Y axis */	
        tmpAxes[1] = (a2 * Z + b2)/dnm;
			
        /* C axis */
        tmpAxes[3] = Q4;		
        /* adjust positions of axis C with +-2PI to bring it closer to the desired value */
        tmpAxes[3] = Modulo2PI(tmpAxes[3],PathAxes[3]);

        /* consider zero offset (position and orientation of base point with respect to world origin) */
        SubFrame2D(tmpAxes,ZeroFrame,Axes);
		
        return STATUS_OK;
	
    }

} 

static int delta_calcAngleYZ(float a1y, float a1z, float a2y, float a2z, float X, float Y, float Z, float *theta)
{

    float y1 = -a1y / 2.0f / sqrt3;
    Y -= (a2y / 2.0f / sqrt3);    // shift center to edge

    // z = a + b*y
    float a = (X*X + Y*Y + Z*Z + a1z*a1z - a2z*a2z - y1*y1)/(2.0f*Z);
    float b = (y1-Y)/Z;

    // discriminant
    float d = -(a+b*y1)*(a+b*y1)+a1z*(b*b*a1z+a1z); 
    if (d < TRF_EPSILON)
    {
        return ERR_TRF_WORKSPACE;
    }
    else
    {
        float yj = (y1 - a*b - sqrtf(d))/(b*b + 1);
        float zj = a + b*yj;
        *theta = atan2d(-zj,(y1 - yj));
        return STATUS_OK;
    }
}


unsigned short DeltaInverse(Link_Type Links[6], float PathAxes[6], float JointAxes[6], float Axes[6])
{ //inverse transformations for Delta robot

    /* base offset from world frame */
    float ZeroFrame[6];
    ZeroFrame[0] = Links[0].Offset.X;
    ZeroFrame[1] = Links[0].Offset.Y;
    ZeroFrame[2] = Links[0].Offset.Z;
    ZeroFrame[3] = Links[0].Rotation.Z;

    //temporary axes values
    float tmpAxes[6];

    /* consider zero offset (position and orientation of base point with respect to world origin) */
    AddFrame2D(PathAxes,ZeroFrame,tmpAxes);

    //simplify notation
    float X = tmpAxes[0];
    float Y = tmpAxes[1];
    float Z = tmpAxes[2];
    float C = tmpAxes[3];
    float a1x = Links[1].Offset.X;	//radius top
    float a1z = Links[1].Offset.Z;	//arm top
    float a2x = Links[2].Offset.X;	//radius bottom
    float a2z = Links[2].Offset.Z;	//arm bottom
    float a3z = Links[3].Offset.Z;	//bottom vertical offset
	
    /* check mechanical parameters consistency */
    if ((a1x <= 0)||(a1z <= 0)||(a2x <= 0)||(a2z <= 0)||(a3z < 0))
    {
        return ERR_TRF_MECH;
    }

    float a1y = a1x * 2.0f * sqrt3;	//base side top
    float a2y = a2x * 2.0f * sqrt3;	//base side bottom

    /* remove bottom vertical offset */
    Z += a3z;

    if (Z >= 0)
    {
        return ERR_TRF_WORKSPACE;
    }

    float Q1, Q2, Q3;
    int status;

    /* Q1 axis */
    status = delta_calcAngleYZ(a1y, a1z, a2y, a2z, X, Y, Z, &Q1);
    if (status == STATUS_OK)
    {
        Axes[0] = Q1;
    }
    else
    {
        return ERR_TRF_WORKSPACE;
    }

    /* Q2 axis */
    status = delta_calcAngleYZ(a1y, a1z, a2y, a2z, X*cos120 + Y*sin120, Y*cos120-X*sin120, Z, &Q2);
    if (status == STATUS_OK)
    {
        Axes[1] = Q2;
    }
    else
    {
        return ERR_TRF_WORKSPACE;
    }

    /* Q3 axis */
    status = delta_calcAngleYZ(a1y, a1z, a2y, a2z, X*cos120 - Y*sin120, Y*cos120+X*sin120, Z, &Q3);
    if (status == STATUS_OK)
    {
        Axes[2] = Q3;
    }
    else
    {
        return ERR_TRF_WORKSPACE;
    }


    /* Q4 axis */
    Axes[3] = C;
    /* adjust positions of axis Q4 with +-2PI to bring it closer to the desired value */
    Axes[3] = Modulo2PI(Axes[3],JointAxes[3]);
	

    return STATUS_OK;

} 




