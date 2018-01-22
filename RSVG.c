#include "RSVG.h"
#include "RobControl.h"
#include "constants.h"
#include "PathPlanner.h"
#include <math.h>

#define sign(a) ( (a<0)?-1:1 )

#define STATE_STANDSTILL 0
#define STATE_MOVING 1
#define STATE_STOPPING 2
#define STATE_ESTOPPING 3
#define STATE_ERROR 4

/* Set-value generator */
void RSVG(struct RSVG_Type* inst)
{
	
	if (!inst->Enable)
	{
		inst->Status = ERR_DISABLED;
		inst->State = STATE_STANDSTILL;
		inst->Start = 0;
		inst->Stop = 0;
		inst->EStop = 0;
		inst->Speed = 0;
		inst->Acceleration = 0;
		return;	
	}
	else if (inst->Status == ERR_DISABLED)	
	{// reset status at enable positive edge
		inst->Status = STATUS_OK;
	}
		
	double oldPosition,oldSpeed;
	double v,a0,a1,j,t,d;
	double astart_up, astart_down;
	
	oldPosition = inst->Position;
	oldSpeed = inst->Speed;
		
	/*************** simplify variable notations ***************/
	v = inst->v;
	a0 = inst->a0;
	a1 = inst->a1;
	j = inst->j;
	astart_up = inst->astart_up;
	astart_down = inst->astart_down;
	double cycletime = inst->Cycletime * inst->RedFactor;    //this is the magic of optimized motion (limited speed only)
	if (cycletime < 0)
	{
		cycletime = 0;
		inst->Status = ERR_SPG_CYCLETIME;
	}
	
	double override = inst->Override;
	if (override < 0)
	{
		override = 0;
		inst->Status = ERR_SPG_OVERRIDE;
	}
	
	/*************** evaluate INPUT commands ***************/
	
	if ((inst->Start)&&(inst->State == STATE_STANDSTILL)) //currently only works from standstill
	{
		inst->Start = 0;
		inst->Status = STATUS_OK;

		/* new movement? -> set start position */	
		if (inst->State == STATE_STANDSTILL)
		{
			inst->Position = inst->StartPosition;
			inst->beginPosition = inst->StartPosition;
			oldPosition = inst->Position;
		}
		else
		{// start from current position 
			inst->beginPosition = inst->Position;
		}
		
		/* determine moving direction */
		inst->moveDirection = sign(inst->TargetPosition - inst->Position);

		/* simplify variable notations */
		if (inst->moveDirection >0)
		{ //positive move
			v = inst->DynamicValues.VelocityPos;
			a0 = inst->DynamicValues.AccelerationPos;
			a1 = inst->DynamicValues.AccelerationPos;
			j = inst->DynamicValues.JerkPos;	
			//check that given movement parameters are within dynamic limits
			if ((v<=0)||(v>inst->DynamicLimits.VelocityPos))
			{
				inst->Status = ERR_SPG_LIMIT_VEL;		
			}
			if ((a0<=0)||(a0>inst->DynamicLimits.AccelerationPos)||(a1<=0)||(a1>inst->DynamicLimits.AccelerationPos))
			{
				inst->Status = ERR_SPG_LIMIT_ACC;				
			}
			if ((j<=0)||(j>inst->DynamicLimits.JerkPos))
			{
				inst->Status = ERR_SPG_LIMIT_JERK;				
			}
		}
		else
		{ //negative move
			v = inst->DynamicValues.VelocityNeg;
			a0 = inst->DynamicValues.AccelerationNeg;
			a1 = inst->DynamicValues.AccelerationNeg;
			j = inst->DynamicValues.JerkNeg;
			//check that given movement parameters are within dynamic limits
			if ((v<=0)||(v>inst->DynamicLimits.VelocityNeg))
			{
				inst->Status = ERR_SPG_LIMIT_VEL;		
			}
			if ((a0<=0)||(a0>inst->DynamicLimits.AccelerationNeg)||(a1<=0)||(a1>inst->DynamicLimits.AccelerationNeg))
			{
				inst->Status = ERR_SPG_LIMIT_ACC;			
			}
			if ((j<=0)||(j>inst->DynamicLimits.JerkNeg))
			{
				inst->Status = ERR_SPG_LIMIT_JERK;				
			}
		}

		/* limit end speed to max speed */
		if (inst->EndSpeed > v)
			inst->EndSpeed = v;
		
		/* check static limits for start position */		
		if (inst->DynamicLimits.PositionNeg >= inst->DynamicLimits.PositionPos)
		{
			inst->Status = ERR_SPG_LIMIT_POS;
		}
		if ((inst->beginPosition > inst->DynamicLimits.PositionPos)||(inst->beginPosition < inst->DynamicLimits.PositionNeg))
		{
			inst->Status = ERR_SPG_LIMIT_POS;	//start position error
		}

		/* check static limits for target position */
		if (inst->TargetPosition > inst->DynamicLimits.PositionPos)
		{
			inst->endPosition = inst->DynamicLimits.PositionPos;
			inst->endLimits = 1;
		}
		else if (inst->TargetPosition < inst->DynamicLimits.PositionNeg)
		{
			inst->endPosition = inst->DynamicLimits.PositionNeg;
			inst->endLimits = 1; 
		}
		else
		{
			inst->endPosition = inst->TargetPosition;
			inst->endLimits = 0;
		}
		
		/* absolute value of total movement distance */
		d = (inst->endPosition - inst->Position)*inst->moveDirection;

		if ((inst->Status == STATUS_OK)&&(inst->State == STATE_STANDSTILL)) //currently only works from standstill
		{ //start movement
			inst->Done = 0;
			inst->State = STATE_MOVING;		
			inst->elapsedTime = 0.0;

			// numerical computation of dynamic values

				
			double tmp = MaxMovementDynamics(d,a0,j,v,inst->StartSpeed,inst->EndSpeed,inst->StartAcc,&v,&a0,&a1,&inst->delta);
			if (tmp != STATUS_OK)
			{
				inst->Status = ERR_SPG_DYNCALC;
				inst->State = STATE_ERROR;
			}			

			
			/* calculate curve time zones according to previously calculated a and v */
			inst->dt[0] = 0;
			inst->dt[1] = 0;
			inst->dt[2] = 0;
			inst->dt[3] = 0;
			inst->dt[4] = 0;
			inst->dt[5] = 0;
			inst->dt[6] = 0;									
			if (a0!=0)
			{ 
				a0 = fabs(a0); //note that a0 is negative if v_start > v
				inst->StartAcc = fabs(inst->StartAcc);
				
				inst->dt[0] = (a0-inst->StartAcc)/j;
				inst->dt[1] = inst->dt[0] + 1/a0 * (fabs(v-inst->StartSpeed) -0.5*a0*a0/j -inst->StartAcc*(a0-inst->StartAcc)/j -0.5*(a0-inst->StartAcc)*(a0-inst->StartAcc)/j);
				inst->dt[2] = inst->dt[1] + a0/j;

				//speed at end of dt[0]
				inst->v1 = inst->StartSpeed + (inst->StartAcc * inst->dt[0] + 0.5 *j *inst->dt[0]*inst->dt[0])*sign(v-inst->StartSpeed);
				inst->v2 = inst->v1 + a0*(inst->dt[1]-inst->dt[0])*sign(v-inst->StartSpeed);					
			}
			if (v!=0)
			{
				inst->dt[3] = inst->dt[2] + inst->delta/v;
			}
			else
			{
				inst->dt[3] = inst->dt[2];
			}
			if (a1!=0)
			{//note that a1 cannot be negative because v_end is always smaller than v
				
				inst->dt[4] = inst->dt[3] + a1/j;
				inst->dt[5] = inst->dt[4] + (v-inst->EndSpeed)/a1-a1/j;
				inst->dt[6] = inst->dt[5] + a1/j;
			}
			else
			{
				inst->dt[4] = inst->dt[3];
				inst->dt[5] = inst->dt[3];
				inst->dt[6] = inst->dt[3];					
			}
		}
		else if (inst->Status != STATUS_OK)
		{
			inst->State = STATE_ERROR;
		}
	}
	
	if (inst->Stop)
	{
		inst->Stop = 0;
		if ((inst->State != STATE_STOPPING)&&(inst->State != STATE_ESTOPPING)) //stop all movements (that are not already stopping)
		{	
			
			if (inst->State == STATE_STANDSTILL) //continue stopping from previous block
			{
				inst->beginPosition = inst->StartPosition;
				oldPosition = inst->beginPosition;
				inst->endPosition = inst->TargetPosition;
			}
			else
			{// stop current block
				inst->beginPosition = inst->Position;
			}
			
			inst->Done = 0;
			inst->State = STATE_STOPPING;	
            if (override > 0)
			    inst->beginSpeed = inst->Speed * inst->RedFactor / override; //added RedFactor here so that movements that are slowed down already stop quickly
            else
                inst->beginSpeed = 0;
			inst->beginAcc = fabs(inst->Acceleration);
			inst->elapsedTime = 0;
			
			
			if(inst->Phase < 1)
			{// movement is accelerating -> need to bring acceleration to zero first
				inst->dt[0] = inst->beginAcc/j; 
				v = inst->beginSpeed + (inst->beginAcc * inst->dt[0] - 0.5 * j *inst->dt[0]*inst->dt[0]); //speed reached at the end of the first section
				
				astart_up = inst->beginAcc;
				astart_down = 0;
					
				// rest of the movement uses default acceleration
				a0 = inst->DynamicValues.AccelerationPos;
						
				v = fabs(v);
				inst->dt[2] = (v/a0)-(a0/j);
				if (inst->dt[2] <= 0)
				{
					a0 = sqrt(v*j);
				}
				if ((a0!=0)&&(v!=0))
				{
					inst->dt[1] = inst->dt[0] + a0/j;
					inst->dt[2] = inst->dt[1] + v/a0-a0/j;
					inst->dt[3] = inst->dt[2] + a0/j;
					inst->v1 = v - 0.5*a0*a0/j;
					inst->v2 = inst->v1 - v + a0*a0/j;
				}
				else
				{
					inst->dt[1] = 0;
					inst->dt[2] = 0;
					inst->dt[3] = 0;				
					inst->v1 = 0;
					inst->v2 = 0;
				}
			}
			else if (inst->Phase == 1)
			{// movement is running at constant speed -> decelerate with default acceleration
				
				inst->dt[0] = 0;

				astart_up = 0;
				astart_down = 0;
				a0 = inst->DynamicValues.AccelerationPos;
						
				v = fabs(inst->beginSpeed);
				inst->dt[2] = (v/a0)-(a0/j);
				if (inst->dt[2] <= 0)
				{
					a0 = sqrt(v*j);
				}
				if ((a0!=0)&&(v!=0))
				{
					inst->dt[1] = inst->dt[0] + a0/j;
					inst->dt[2] = inst->dt[1] + v/a0-a0/j;
					inst->dt[3] = inst->dt[2] + a0/j;
					inst->v1 = v - 0.5*a0*a0/j;
					inst->v2 = inst->v1 - v + a0*a0/j;
				}
				else
				{
					inst->dt[1] = 0;
					inst->dt[2] = 0;
					inst->dt[3] = 0;				
					inst->v1 = 0;
					inst->v2 = 0;
				}	
                                
			}
			else if (inst->Phase > 1)
			{// movement is decelerating already -> find out what deceleration is needed to reach zero
				
				inst->dt[0] = 0;
				v = fabs(inst->beginSpeed);

				astart_up = 0;
				astart_down = inst->beginAcc;
				
				a0 = sqrt(j*v+0.5*inst->beginAcc*inst->beginAcc);
				if (a0<inst->beginAcc) a0=inst->beginAcc;
				if (a0 > inst->DynamicValues.AccelerationPos) a0 = inst->DynamicValues.AccelerationPos;
				if (a0!=0)
				{ 
					inst->dt[1] = (a0-inst->beginAcc)/j;
					inst->dt[2] = inst->dt[1] + 1/a0 * (v -0.5*a0*a0/j -inst->beginAcc*(a0-inst->beginAcc)/j -0.5*(a0-inst->beginAcc)*(a0-inst->beginAcc)/j);
					inst->dt[3] = inst->dt[2] + a0/j;
					inst->v1 = v - (astart_down*inst->dt[1] + 0.5 *j * inst->dt[1]*inst->dt[1]);
					inst->v2 = inst->v1 - a0*(inst->dt[2] - inst->dt[1]);
				}
				else
				{
					inst->dt[1] = 0;
					inst->dt[2] = 0;
					inst->dt[3] = 0;				
					inst->v1 = 0;
					inst->v2 = 0;
				}	
			}
			
		}
	}	
		
	
	if (inst->EStop)
	{

		if (inst->State == STATE_STANDSTILL) //continue stopping from previous block
		{
			inst->beginPosition = inst->StartPosition;
			oldPosition = inst->beginPosition;
			inst->endPosition = inst->TargetPosition;
		}
		else
		{// stop current block
			inst->beginPosition = inst->Position;
		}
			
		inst->Done = 0;
		inst->EStop = 0;
		inst->State = STATE_ESTOPPING;
		inst->elapsedTime = 0;
        if (override > 0)
		    v = inst->Speed / override; //path speed always positive
        else
            v = 0;
		a0 = -inst->DynamicLimits.AccelerationPos; //e-stop acceleration always negative
		inst->dt[0] = -v/a0;
	}	
	
	
	
	/***************  STATE MACHINE ***************/
	
	switch (inst->State)
	{
		case STATE_STANDSTILL:
			inst->Status = STATUS_OK;
			inst->Acceleration = 0;
			break;
		
		case STATE_MOVING:
			inst->Status = 1;	
			inst->elapsedTime += (cycletime * override);
			
			if (inst->elapsedTime > inst->dt[6])
			{//movement completed
				inst->Status = STATUS_OK;
				inst->Done = 1;
				if (inst->endLimits)
				{
					inst->Status = ERR_SPG_LIMITS_REACHED;
				}
			}
			else if (inst->elapsedTime > inst->dt[5])
			{
				inst->Phase = 2;
				t = inst->dt[0];
				inst->ds = (inst->StartSpeed * t + (j * t*t*t / 6.0 + 0.5 * inst->StartAcc * t*t)* sign(v-inst->StartSpeed));
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (inst->v1 * t + (0.5 * a0 * t*t) * sign(v-inst->StartSpeed));
				t = inst->dt[2]-inst->dt[1];
				inst->ds += (inst->v2*t + (0.5 * a0 * t*t - j * t*t*t / 6.0) * sign(v-inst->StartSpeed));
				t = inst->dt[3]-inst->dt[2];
				inst->ds += (v*t);
				t = inst->dt[4]-inst->dt[3];
				inst->ds += (v*t - (j * t*t*t /6.0));
				t = inst->dt[5]-inst->dt[4];
				inst->ds += (v*t - (0.5 * a1 *t*t + 0.5 * a1*a1/j * t));
				t = inst->elapsedTime - inst->dt[5];
				inst->ds += (v*t - (-j *t*t*t /6.0 + 0.5 * a1* t*t + t*((v-inst->EndSpeed)-0.5*a1*a1/j)));
				inst->Acceleration = -(a1 - j*t);


			}
			else if (inst->elapsedTime > inst->dt[4])
			{
				inst->Phase = 2;
				t = inst->dt[0];
				inst->ds = (inst->StartSpeed * t + (j * t*t*t / 6.0 + 0.5 * inst->StartAcc * t*t)* sign(v-inst->StartSpeed));
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (inst->v1 * t + (0.5 * a0 * t*t) * sign(v-inst->StartSpeed));
				t = inst->dt[2]-inst->dt[1];
				inst->ds += (inst->v2*t + (0.5 * a0 * t*t - j * t*t*t / 6.0) * sign(v-inst->StartSpeed));
				t = inst->dt[3]-inst->dt[2];
				inst->ds += (v*t);
				t = inst->dt[4]-inst->dt[3];
				inst->ds += (v*t - (j * t*t*t /6.0));
				t = inst->elapsedTime - inst->dt[4];
				inst->ds += (v*t - (0.5 * a1 *t*t + 0.5 * a1*a1/j * t));
				inst->Acceleration = -a1;

			}
			else if (inst->elapsedTime > inst->dt[3])
			{
				inst->Phase = 2;
				t = inst->dt[0];
				inst->ds = (inst->StartSpeed * t + (j * t*t*t / 6.0 + 0.5 * inst->StartAcc * t*t)* sign(v-inst->StartSpeed));
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (inst->v1 * t + (0.5 * a0 * t*t) * sign(v-inst->StartSpeed));
				t = inst->dt[2]-inst->dt[1];
				inst->ds += (inst->v2*t + (0.5 * a0 * t*t - j * t*t*t / 6.0) * sign(v-inst->StartSpeed));
				t = inst->dt[3]-inst->dt[2];
				inst->ds += (v*t);
				t = inst->elapsedTime - inst->dt[3];
				inst->ds += (v*t - (j * t*t*t /6.0));
				inst->Acceleration = -j * t;

			}
			else if (inst->elapsedTime > inst->dt[2])
			{
				inst->Phase = 1;
				t = inst->dt[0];
				inst->ds = (inst->StartSpeed * t + (j * t*t*t / 6.0 + 0.5 * inst->StartAcc * t*t)* sign(v-inst->StartSpeed));
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (inst->v1 * t + (0.5 * a0 * t*t) * sign(v-inst->StartSpeed));
				t = inst->dt[2]-inst->dt[1];
				inst->ds += (inst->v2*t + (0.5 * a0 * t*t - j * t*t*t / 6.0) * sign(v-inst->StartSpeed));
				t = inst->elapsedTime - inst->dt[2];
				inst->ds += (v*t);
				inst->Acceleration = 0;

			}
			else if (inst->elapsedTime > inst->dt[1])
			{
				inst->Phase = 0;
				t = inst->dt[0];
				inst->ds = (inst->StartSpeed * t + (j * t*t*t / 6.0 + 0.5 * inst->StartAcc * t*t)* sign(v-inst->StartSpeed));
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (inst->v1 * t + (0.5 * a0 * t*t) * sign(v-inst->StartSpeed));
				t = inst->elapsedTime - inst->dt[1];
				inst->ds += (inst->v2*t + (0.5 * a0 * t*t - j * t*t*t / 6.0) * sign(v-inst->StartSpeed));
				inst->Acceleration = (a0 - j * t) * sign(v-inst->StartSpeed);

			}
			else if (inst->elapsedTime > inst->dt[0])
			{
				inst->Phase = 0;
				t = inst->dt[0];
				inst->ds = (inst->StartSpeed * t + (j * t*t*t / 6.0 + 0.5 * inst->StartAcc * t*t)* sign(v-inst->StartSpeed));
				t = inst->elapsedTime - inst->dt[0];
				inst->ds += (inst->v1 * t + (0.5 * a0 * t*t) * sign(v-inst->StartSpeed));
				inst->Acceleration = a0 * sign(v-inst->StartSpeed);

			}
			else
			{
				inst->Phase = 0;
				t = inst->elapsedTime;
				inst->ds = (inst->StartSpeed * t + (j * t*t*t / 6.0 + 0.5 * inst->StartAcc * t*t)* sign(v-inst->StartSpeed));
				inst->Acceleration = (inst->StartAcc + j * t) * sign(v-inst->StartSpeed);

            }
			
			inst->Position = inst->beginPosition + (inst->ds * inst->moveDirection);
			
			if (inst->Done)
			{
				inst->Position = inst->endPosition;
			}

			//check if movement has completed already but speed and acceleration have not reached zero yet -> movement must continue in next tangential block!
			//moveDirection negative is needed for manual jogging in negative direction! 
			if ((!inst->Done)&&(((inst->Position >= inst->endPosition)&&(inst->moveDirection > 0))||((inst->Position <= inst->endPosition)&&(inst->moveDirection < 0))))
			{
				inst->Status = STATUS_ABORT;
				inst->Done = 1;				
			}
				
			break;
		
		
		case STATE_STOPPING:

			inst->Status = 1;	
			inst->elapsedTime += (cycletime * override);
						
            //movement already stopped by zero override
            if (override == 0)
            {
                inst->Status = STATUS_OK;
                inst->Done = 1;
            }

            if (inst->elapsedTime > inst->dt[3])
			{//movement completed
			
				//used to increase precision in case dt[0] is shorter than one cycle time
				t = inst->dt[0];
				inst->ds = (fabs(inst->beginSpeed) * t + 0.5* astart_up * t*t - j * t*t*t / 6.0);
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (v*t - (j * t*t*t /6.0)- 0.5 * astart_down * t*t);
				t = inst->dt[2]-inst->dt[1];
				inst->ds += (inst->v1 *t - (0.5 * a0 *t*t));
				t = inst->dt[3] - inst->dt[2];
				inst->ds += (inst->v2 * t - (-j *t*t*t /6.0 + 0.5 * a0* t*t));
				inst->Acceleration = 0;
				
				inst->Status = STATUS_OK;
				inst->Done = 1;

			}
			else if (inst->elapsedTime > inst->dt[2])
			{
				inst->Phase = 2;
				t = inst->dt[0];
				inst->ds = (fabs(inst->beginSpeed) * t + 0.5* astart_up * t*t - j * t*t*t / 6.0);
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (v*t - (j * t*t*t /6.0)- 0.5 * astart_down * t*t);
				t = inst->dt[2]-inst->dt[1];
				inst->ds += (inst->v1 *t - (0.5 * a0 *t*t));
				t = inst->elapsedTime - inst->dt[2];
				inst->ds += (inst->v2 * t - (-j *t*t*t /6.0 + 0.5 * a0* t*t));
				inst->Acceleration = -(a0-j*t);

			}
			else if (inst->elapsedTime > inst->dt[1])
			{
				inst->Phase = 2;
				t = inst->dt[0];
				inst->ds = (fabs(inst->beginSpeed) * t + 0.5* astart_up * t*t - j * t*t*t / 6.0);
				t = inst->dt[1]-inst->dt[0];
				inst->ds += (v*t - (j * t*t*t /6.0)- 0.5 * astart_down * t*t);
				t = inst->elapsedTime - inst->dt[1];
				inst->ds += (inst->v1 *t - (0.5 * a0 *t*t));
				inst->Acceleration = -a0;
				
			}
			else if (inst->elapsedTime > inst->dt[0])
			{
				inst->Phase = 2;
				t = inst->dt[0];
				inst->ds = (fabs(inst->beginSpeed) * t + 0.5* astart_up * t*t - j * t*t*t / 6.0);
				t = inst->elapsedTime - inst->dt[0];
				inst->ds += (v*t - (j * t*t*t /6.0)- 0.5 * astart_down * t*t);
				inst->Acceleration = -j * t - astart_down;

			}
			else
			{
				inst->Phase = 0;
				t = inst->elapsedTime;
				inst->ds = (fabs(inst->beginSpeed) * t + 0.5* astart_up * t*t - j * t*t*t / 6.0);
				inst->Acceleration = astart_up - j * t;

			}
			inst->Position = inst->beginPosition + (inst->ds * inst->moveDirection);
			
			//check if movement has completed already but speed has not reached zero yet -> movement must stop in next tangential block!
            //moveDirection negative is needed for manual jogging in negative direction! 
            if ((!inst->Done)&&(((inst->Position >= inst->endPosition)&&(inst->moveDirection > 0))||((inst->Position <= inst->endPosition)&&(inst->moveDirection < 0))))
			{
				inst->Status = STATUS_ABORT;
				inst->Done = 1;				

			}

			
			break;

		
		case STATE_ESTOPPING:

			inst->Status = 1;
			inst->elapsedTime += (cycletime * override);

            //movement already stopped by zero override
            if (override == 0)
            {
            inst->Status = STATUS_OK;
            inst->Done = 1;
            }

			if (inst->elapsedTime > inst->dt[0])
			{//movement completed
				t = inst->dt[0];
				inst->ds = (0.5*a0*t*t+v*t);	//used to increase precision in case dt[0] is shorter than one cycle time
				inst->Status = STATUS_OK;
				inst->Done = 1;
				inst->Acceleration = 0;
			}
			else
			{
				t = inst->elapsedTime;
				inst->ds = (0.5*a0*t*t+v*t);
				inst->Acceleration = a0;
			}
			inst->Position = inst->beginPosition + inst->ds;
			
			//check if movement has completed already but speed has not reached zero yet -> movement must stop in next tangential block!
            //moveDirection negative is needed for manual jogging in negative direction! 
            if ((!inst->Done)&&(((inst->Position >= inst->endPosition)&&(inst->moveDirection > 0))||((inst->Position <= inst->endPosition)&&(inst->moveDirection < 0))))
            {
				inst->Status = STATUS_ABORT;
				inst->Done = 1;				
			}
						
			break;
		
		case STATE_ERROR:
			//return to standstill
			inst->State = STATE_STANDSTILL;
		
	}

	
	/***************  update OUTPUT values ***************/

	if (cycletime != 0)
	{
		inst->Speed = (inst->Position - oldPosition) / cycletime;
		if (inst->Done)
		{
			if ((inst->State == STATE_ESTOPPING)||(inst->State == STATE_STOPPING))
			{
				if (inst->Status == STATUS_OK)
				{
					inst->Speed = 0; //zero speed at end of movement (not if aborted -> could be non-zero speed and stopping must continue in next movement)
					inst->Acceleration = 0;
				}
			}
			else
			{
				if (inst->Status == STATUS_OK) //not STATUS_ABORT -> movement is completed with EndSpeed and zero acc
				{
					inst->Speed = inst->EndSpeed * override; //stable speed at end of movement
					inst->Acceleration = 0;
				}
				//else STATUS_ABORT -> path blending (end speed not necessarily reached, acc not zero)
			}
			inst->State = STATE_STANDSTILL;
		}
		
	}
	else
	{
		inst->Speed = 0;
		inst->Acceleration = 0;		
	}

	/***************  reset all commands ***************/
	inst->Start = 0;
	inst->Stop = 0;
	inst->EStop = 0;
	
	
	/*************** simplify variable notations ***************/	
	inst->v = v;
	inst->a0 = a0;
	inst->a1 = a1;
	inst->j = j;
	inst->astart_up = astart_up;
	inst->astart_down = astart_down;
	
	
}

