#include "RobControl.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "PathPlanner.h"
#include "RSVG.h"
#include "static.h"
#include "Transformations.h"
#include "Interpreter.h"
#include "Misc.h"
#include "Frame.h"
#include "trig.h"
#include "constants.h"
#include "Calibration.h"

#define sign(a) ( (a<0)?-1:1 )
#define min(a,b) ( (a<b)?a:b )


unsigned short RobotControl(unsigned long Robots, unsigned char RobotsNumber)
{
    Robot_Type *gRobot[RobotsNumber];

    unsigned char M_count, M_value, M_synch;
    unsigned short Trf_Status;
    unsigned short Circle_Status;
    unsigned char AxesMoved = 0;
	
    double OldAxesValues[6];
    double tmpAxesValues[6];
    double tmpJointsValues[6];
    double tmpPathValues[6];
    double BlockTime;
    unsigned short tmpViolate;
	
    char line[MAX_BLOCK_SIZE];	
	
    if (!CheckConstDone) {
        CheckConstDone = CheckConst();
        return ERR_CHECKSUM;
    }
    
    if (RobotsNumber > MAX_ROBOTS) {
        memset((Robot_Type*)Robots,0,sizeof(Robot_Type)*RobotsNumber);
        return ERR_MAX_ROBOTS;
    }
	
    /* repeat all operations for configured axes */
    int i,j,k;
    for(i=0;i<RobotsNumber;i++) {
        
        gRobot[i] = (Robot_Type*) (Robots + sizeof(Robot_Type)*i);
        
        //LICENSE CHECK!
        unsigned long* p1 = 0x5e0;
        unsigned long* p2 = 0x5e4;
        unsigned long License = (~(*p1)^(*p2))^(gRobot[i]->Parameters.License);
        if (License) return ERR_ROBOT_LICENSE;

        unsigned long License = 0;

        /* prevent user from changing old monitor values (except for M-functions, DI_ and TrackSynch) */
        memcpy(&OldMonitor[i].M,&gRobot[i]->Monitor.M,sizeof(OldMonitor[i].M));
        memcpy(&OldMonitor[i].DI_,&gRobot[i]->Monitor.DI_,sizeof(OldMonitor[i].DI_));
        OldMonitor[i].TrackSynch = gRobot[i]->Monitor.TrackSynch;
        if (gRobot[i]->Monitor.State == STANDSTILL_STATE) OldMonitor[i].Tool = gRobot[i]->Monitor.Tool;
        memcpy(&gRobot[i]->Monitor,&OldMonitor[i],sizeof(gRobot[i]->Monitor));
		
        /* monitor cycletime */
        double TaskCycleTime = gRobot[i]->Parameters.CycleTime;
        if ((TaskCycleTime <= 0)&&(gRobot[i]->Monitor.State!=ERROR_STATE)) {
            gRobot[i]->Monitor.ActiveError = ERR_CYCLETIME;
            gRobot[i]->Monitor.ErrorLine = 0;
        }
		
        /* increase evaluation time -> the more robots the faster it runs off! */
        EvaluationTime += ((unsigned long) (gRobot[i]->Parameters.CycleTime * 1000.0));

        /* overwrite Tool[0] and Frame[0] so that omitting them in the programmed block is equivalent to working with no tool and no frame */
        memset(&gRobot[i]->Parameters.Tool[0],0,sizeof(gRobot[i]->Parameters.Tool[0]));
        memset(&gRobot[i]->Parameters.Frame[0],0,sizeof(gRobot[i]->Parameters.Frame[0]));
        /* overwrite Points[0] so that HOME goes to P0=0 */
        memset(&gRobot[i]->Parameters.Points[0],0,sizeof(gRobot[i]->Parameters.Points[0]));
		
        /* prevent negative values at path limits */
        if (gRobot[i]->Parameters.PathLimits.Linear.Velocity < 0) gRobot[i]->Parameters.PathLimits.Linear.Velocity = 0;
        if (gRobot[i]->Parameters.PathLimits.Linear.Acceleration < 0) gRobot[i]->Parameters.PathLimits.Linear.Acceleration = 0;            
        if (gRobot[i]->Parameters.PathLimits.Linear.Jerk < 0) gRobot[i]->Parameters.PathLimits.Linear.Jerk = 0;            
        if (gRobot[i]->Parameters.PathLimits.Angular.Velocity < 0) gRobot[i]->Parameters.PathLimits.Angular.Velocity = 0;            
        if (gRobot[i]->Parameters.PathLimits.Angular.Acceleration < 0) gRobot[i]->Parameters.PathLimits.Angular.Acceleration = 0;            
        if (gRobot[i]->Parameters.PathLimits.Angular.Jerk < 0) gRobot[i]->Parameters.PathLimits.Angular.Jerk = 0;
		
        /* automatically assign jerk values if not given by user */
        if (gRobot[i]->Parameters.PathLimits.Linear.Jerk == 0) gRobot[i]->Parameters.PathLimits.Linear.Jerk = gRobot[i]->Parameters.PathLimits.Linear.Acceleration * 10;
        if (gRobot[i]->Parameters.PathLimits.Angular.Jerk == 0) gRobot[i]->Parameters.PathLimits.Angular.Jerk = gRobot[i]->Parameters.PathLimits.Angular.Acceleration * 10;

        /* prevent user from setting jerk too low */
        if (gRobot[i]->Parameters.PathLimits.Linear.Jerk < gRobot[i]->Parameters.PathLimits.Linear.Acceleration) gRobot[i]->Parameters.PathLimits.Linear.Jerk = gRobot[i]->Parameters.PathLimits.Linear.Acceleration;
        if (gRobot[i]->Parameters.PathLimits.Angular.Jerk < gRobot[i]->Parameters.PathLimits.Angular.Acceleration) gRobot[i]->Parameters.PathLimits.Angular.Jerk = gRobot[i]->Parameters.PathLimits.Angular.Acceleration;
            
        /* prevent negative values at joint limits and jerk too low */
        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {
            if (gRobot[i]->Parameters.JointLimits[k].VelocityNeg < 0) gRobot[i]->Parameters.JointLimits[k].VelocityNeg = 0;
            if (gRobot[i]->Parameters.JointLimits[k].VelocityPos < 0) gRobot[i]->Parameters.JointLimits[k].VelocityPos = 0;
            if (gRobot[i]->Parameters.JointLimits[k].AccelerationPos < 0) gRobot[i]->Parameters.JointLimits[k].AccelerationPos = 0;
            if (gRobot[i]->Parameters.JointLimits[k].AccelerationNeg < 0) gRobot[i]->Parameters.JointLimits[k].AccelerationNeg = 0;
            if (gRobot[i]->Parameters.JointLimits[k].JerkPos < 0) gRobot[i]->Parameters.JointLimits[k].JerkPos = 0;
            if (gRobot[i]->Parameters.JointLimits[k].JerkNeg < 0) gRobot[i]->Parameters.JointLimits[k].JerkNeg = 0;
            if (gRobot[i]->Parameters.JointLimits[k].JerkPos == 0) gRobot[i]->Parameters.JointLimits[k].JerkPos = gRobot[i]->Parameters.JointLimits[k].AccelerationPos * 10;
            if (gRobot[i]->Parameters.JointLimits[k].JerkNeg == 0) gRobot[i]->Parameters.JointLimits[k].JerkNeg = gRobot[i]->Parameters.JointLimits[k].AccelerationNeg * 10;
            if (gRobot[i]->Parameters.JointLimits[k].JerkPos < gRobot[i]->Parameters.JointLimits[k].AccelerationPos) gRobot[i]->Parameters.JointLimits[k].JerkPos = gRobot[i]->Parameters.JointLimits[k].AccelerationPos;
            if (gRobot[i]->Parameters.JointLimits[k].JerkNeg < gRobot[i]->Parameters.JointLimits[k].AccelerationNeg) gRobot[i]->Parameters.JointLimits[k].JerkNeg = gRobot[i]->Parameters.JointLimits[k].AccelerationNeg;
        }
		
        /* mapp dynamic path limits internally */
        /* because the Parameters inteface is different from the SVG interface */
        Robot_Parameter_JointLimits_Type PathLimits;
        PathLimits.PositionPos = 1e30; //has no meaning
        PathLimits.PositionNeg = -1e30; //has no meaning
        PathLimits.VelocityPos = gRobot[i]->Parameters.PathLimits.Linear.Velocity;
        PathLimits.VelocityNeg = gRobot[i]->Parameters.PathLimits.Linear.Velocity; //has no meaning
        PathLimits.AccelerationPos = gRobot[i]->Parameters.PathLimits.Linear.Acceleration;
        PathLimits.AccelerationNeg = gRobot[i]->Parameters.PathLimits.Linear.Acceleration; //has no meaning
        PathLimits.JerkPos = gRobot[i]->Parameters.PathLimits.Linear.Jerk;
        PathLimits.JerkNeg = gRobot[i]->Parameters.PathLimits.Linear.Jerk; //has no meaning
		
        /* these are only used for jogging ABC angles */
        Robot_Parameter_JointLimits_Type AngularPathLimits;
        AngularPathLimits.PositionPos = 1e30; //has no meaning
        AngularPathLimits.PositionNeg = -1e30; //has no meaning
        AngularPathLimits.VelocityPos = gRobot[i]->Parameters.PathLimits.Angular.Velocity;
        AngularPathLimits.VelocityNeg = gRobot[i]->Parameters.PathLimits.Angular.Velocity; //has no meaning
        AngularPathLimits.AccelerationPos = gRobot[i]->Parameters.PathLimits.Angular.Acceleration;
        AngularPathLimits.AccelerationNeg = gRobot[i]->Parameters.PathLimits.Angular.Acceleration; //has no meaning
        AngularPathLimits.JerkPos = gRobot[i]->Parameters.PathLimits.Angular.Jerk;
        AngularPathLimits.JerkNeg = gRobot[i]->Parameters.PathLimits.Angular.Jerk; //has no meaning
		
        /* select number of significant joint axes for this robot */
        if (gRobot[i]->Parameters.Mechanics.Type == CNC) {
            gRobot[i]->Monitor.AxesNum = 3;
        } else if ((gRobot[i]->Parameters.Mechanics.Type == SCARA)||(gRobot[i]->Parameters.Mechanics.Type == DELTA)||(gRobot[i]->Parameters.Mechanics.Type == PALLETIZER)) {
            gRobot[i]->Monitor.AxesNum = 4;
        } else if (gRobot[i]->Parameters.Mechanics.Type == RTCP) {
            gRobot[i]->Monitor.AxesNum = 6;
        } else if (gRobot[i]->Parameters.Mechanics.Type == USER) {
            gRobot[i]->Monitor.AxesNum = gRobot[i]->Parameters.Mechanics.UserTrf.AxesNum;
            if (gRobot[i]->Monitor.AxesNum <= 0) {
                gRobot[i]->Monitor.ActiveError = ERR_TRF_AXESNUM;
                gRobot[i]->Monitor.ErrorLine = 0;				
            }
        } else { //all other cases - protection for future additions
            gRobot[i]->Monitor.AxesNum = 6;
        }
		
        /* Override */
        // limit override to max 200%
        if (gRobot[i]->Parameters.Override > 200) gRobot[i]->Parameters.Override = 200;
        // limit override to min 0% and do not consider values lower than 1 (because they create numerical issues)
        if (gRobot[i]->Parameters.Override < 1) gRobot[i]->Parameters.Override = 0;
		
        //filter override so that it does not jump
        if (gRobot[i]->Monitor.Moving) {
            FilterOverride[i].Enable = 1;
        } else {
            FilterOverride[i].Enable = 0;			
        }

        //calculate filter window
        if ((fRSVG[i].DynamicValues.AccelerationPos != 0)&&(fRSVG[i].DynamicValues.VelocityPos != 0)&&(fRSVG[i].DynamicValues.JerkPos != 0))
        { // (v/a+a/j) is the time needed to ramp up to the speed -> filter window is half that time
            FilterOverride[i].Window = (unsigned short) round(0.5 * (fRSVG[i].DynamicValues.VelocityPos / fRSVG[i].DynamicValues.AccelerationPos + fRSVG[i].DynamicValues.AccelerationPos / fRSVG[i].DynamicValues.JerkPos) / TaskCycleTime);
        } else {
            FilterOverride[i].Window = 1;
        }
		
        // ramp down override if halt command was called
        if ((gRobot[i]->Monitor.Halted)&&(HaltedByCmd[i])) { 
            Override[i] = 0;
        } else { // ramp up override
            Override[i] = gRobot[i]->Parameters.Override;
        }

        FilterOverride[i].InputValue = Override[i];
        GaussianFilter((struct Filter_Type*)&FilterOverride[i]);	

        fRSVG[i].Override =  FilterOverride[i].OutputValue / 100.0; 			
        fRSVG[i].Cycletime = TaskCycleTime;
        fRSVG[i].RedFactor = RedFactor[i]; //reduction factor is <1 when hitting dynamic limits of joints		


        /* joints units scaling handling */
        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {				

            /* prevent user from changing units scaling when axes are moving */
            if (gRobot[i]->Monitor.Moving) {
                gRobot[i]->Parameters.UnitsRatio[k] = OldUnitsRatio[i][k];
            } else {
                OldUnitsRatio[i][k] = gRobot[i]->Parameters.UnitsRatio[k];
            }
		
            /* Units scaling check */
            if (((gRobot[i]->Parameters.UnitsRatio[k].MotorUnits <= 0)||(gRobot[i]->Parameters.UnitsRatio[k].AxisUnits <= 0)||(gRobot[i]->Parameters.UnitsRatio[k].Direction == 0)||(gRobot[i]->Parameters.UnitsRatio[k].Direction > 1)||(gRobot[i]->Parameters.UnitsRatio[k].Direction < -1))&&(gRobot[i]->Monitor.State!=ERROR_STATE)) {
                //ignore axis Q6 of RTCP
                if (gRobot[i]->Parameters.Mechanics.Type != RTCP || k != 5) {
                    gRobot[i]->Monitor.ActiveError = ERR_UNITS_SCALING;
                    gRobot[i]->Monitor.ErrorLine = 0;
                }
            }
		
        }
		
		
        /* Filter Time handling */
		
        //prevent user from changing filter time while movement is active
        if (gRobot[i]->Monitor.Moving) {
            gRobot[i]->Parameters.FilterTime = OldFilterTime[i];
            StoppedTime[i] = 0; //this counts time after movement has stopped to let filter settle down
        } else {
            OldFilterTime[i] = gRobot[i]->Parameters.FilterTime;
        }		
		
        //Reduce value of filter time within limits (0 - 500 ms)
        if (gRobot[i]->Parameters.FilterTime < 0) gRobot[i]->Parameters.FilterTime = 0;
        if (gRobot[i]->Parameters.FilterTime > 0.5) gRobot[i]->Parameters.FilterTime = 0.5;

        //prevent user from activating single step mode while movement is active
        if ((gRobot[i]->Monitor.Moving)&&(OldSingleStep[i] == 0)) {
            gRobot[i]->Parameters.SingleStep = OldSingleStep[i];
        } else {
            OldSingleStep[i] = gRobot[i]->Parameters.SingleStep;
        }		
		
		
        /* MaxTransitionAngle */
		
        //prevent user from changing angle value while movement is active
        if (gRobot[i]->Monitor.Moving) {
            gRobot[i]->Parameters.MaxTransitionAngle = OldTransitionAngle[i];
        } else {
            OldTransitionAngle[i] = gRobot[i]->Parameters.MaxTransitionAngle;
        }		
		
        //Reduce angle value within limits (0 - 180 deg)
        if (gRobot[i]->Parameters.MaxTransitionAngle < 0) {
            gRobot[i]->Parameters.MaxTransitionAngle = 0;		
        }
        if (gRobot[i]->Parameters.MaxTransitionAngle > 180.0) {
            gRobot[i]->Parameters.MaxTransitionAngle = 180.0;
        }
        
		
        /******************** evaluate input commands ********************/
			
        /* reset command */
        if (gRobot[i]->Commands.Reset) {
            gRobot[i]->Commands.Reset = 0;
            if (gRobot[i]->Monitor.State == ERROR_STATE) {
                gRobot[i]->Monitor.ActiveError = 0;
                gRobot[i]->Monitor.ErrorLine = 0;
                gRobot[i]->Monitor.State = STANDSTILL_STATE;
            }
        }
		
        /* set joints command */
        if (gRobot[i]->Commands.SetJoints) {
            gRobot[i]->Commands.SetJoints = 0;
            if ((gRobot[i]->Monitor.State == STANDSTILL_STATE)||(gRobot[i]->Monitor.State == ERROR_STATE)) {
                //scale axes units
                for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {					
                    if (gRobot[i]->Parameters.UnitsRatio[k].MotorUnits != 0) {
                        gRobot[i]->Monitor.JointPosition[k] = (double) (gRobot[i]->Parameters.ActFromDrives[k] - (double) gRobot[i]->Parameters.UnitsRatio[k].HomeOffset) * (double) gRobot[i]->Parameters.UnitsRatio[k].Direction * (double) gRobot[i]->Parameters.UnitsRatio[k].AxisUnits / (double) gRobot[i]->Parameters.UnitsRatio[k].MotorUnits;
                    } else {
                        gRobot[i]->Monitor.JointPosition[k] = 0;
                    }
                }
                // remove mechanical coupling
                if (gRobot[i]->Parameters.Mechanics.Coupling[5]!=0) gRobot[i]->Monitor.JointPosition[5] += (gRobot[i]->Monitor.JointPosition[4]*gRobot[i]->Parameters.Mechanics.Coupling[5]);
                if (gRobot[i]->Parameters.Mechanics.Coupling[4]!=0) gRobot[i]->Monitor.JointPosition[5] += (gRobot[i]->Monitor.JointPosition[3]*gRobot[i]->Parameters.Mechanics.Coupling[4]);
                if (gRobot[i]->Parameters.Mechanics.Coupling[3]!=0) gRobot[i]->Monitor.JointPosition[4] += (gRobot[i]->Monitor.JointPosition[3]*gRobot[i]->Parameters.Mechanics.Coupling[3]);
                if (gRobot[i]->Parameters.Mechanics.Coupling[1]!=0) gRobot[i]->Monitor.JointPosition[2] += (gRobot[i]->Monitor.JointPosition[1]*gRobot[i]->Parameters.Mechanics.Coupling[1]);
            }							
        }
		
        /* JogAxis command */
        if (gRobot[i]->Commands.JogAxis) {
            if ((gRobot[i]->Monitor.State == STANDSTILL_STATE)&&(fRSVG[i].Status == ERR_DISABLED)) {
                //check for jog parameters consistency
                if (((gRobot[i]->Parameters.Jog.Mode != JOG_JOINTS)&&(gRobot[i]->Parameters.Jog.Mode != JOG_BASE)&&(gRobot[i]->Parameters.Jog.Mode != JOG_TOOL))
                    ||((gRobot[i]->Parameters.Jog.Mode == JOG_TOOL)&&(gRobot[i]->Monitor.AxesNum != 6))
                    ||(gRobot[i]->Parameters.Jog.Direction > JOG_GOTO)
                    ||(gRobot[i]->Parameters.Jog.AxisIndex > 5)
                ) {					
                    gRobot[i]->Monitor.ActiveError = ERR_JOG_PAR;
                    gRobot[i]->Monitor.ErrorLine = 0;
                } else {	
                    gRobot[i]->Monitor.State = JOGGING;
                    Jog[i] = gRobot[i]->Parameters.Jog;
                    if (Jog[i].Mode == JOG_JOINTS) {
                        memcpy(&fRSVG[i].DynamicLimits,&gRobot[i]->Parameters.JointLimits[Jog[i].AxisIndex],sizeof(fRSVG[i].DynamicLimits));
                        memcpy(&fRSVG[i].DynamicValues,&gRobot[i]->Parameters.JointLimits[Jog[i].AxisIndex],sizeof(fRSVG[i].DynamicValues));
                        
                        /* modulate max speed according to set override */
                        fRSVG[i].DynamicLimits.VelocityPos *= fRSVG[i].Override;
                        fRSVG[i].DynamicLimits.VelocityNeg *= fRSVG[i].Override;
                        fRSVG[i].DynamicValues.VelocityPos *= fRSVG[i].Override;
                        fRSVG[i].DynamicValues.VelocityNeg *= fRSVG[i].Override;
                        
                        fRSVG[i].StartPosition = gRobot[i]->Monitor.JointPosition[Jog[i].AxisIndex];
						
                        if (Jog[i].Direction == JOG_POSITIVE) {
                            fRSVG[i].TargetPosition = gRobot[i]->Parameters.JointLimits[Jog[i].AxisIndex].PositionPos;
                            fRSVG[i].DynamicLimits.PositionNeg = -1e37; //ignore negative limit when jogging in positive direction
                        } else if (Jog[i].Direction == JOG_NEGATIVE) {
                            fRSVG[i].TargetPosition = gRobot[i]->Parameters.JointLimits[Jog[i].AxisIndex].PositionNeg;
                            fRSVG[i].DynamicLimits.PositionPos = 1e37; //ignore positive limit when jogging in negative direction
                        } else if (Jog[i].Direction == JOG_GOTO) {
                            //check for goto limits
                            if ((Jog[i].GotoPos > gRobot[i]->Parameters.JointLimits[Jog[i].AxisIndex].PositionPos)||(Jog[i].GotoPos < gRobot[i]->Parameters.JointLimits[Jog[i].AxisIndex].PositionNeg)) {					
                                gRobot[i]->Monitor.ActiveError = ERR_JOG_GOTOPOS;
                                gRobot[i]->Monitor.ErrorLine = 0;
                            } else if (Jog[i].GotoPos >= gRobot[i]->Monitor.JointPosition[Jog[i].AxisIndex]) {
                                // move in positive direction
                                fRSVG[i].TargetPosition = Jog[i].GotoPos;
                                fRSVG[i].DynamicLimits.PositionNeg = -1e37; //ignore negative limit when jogging in positive direction								
                            } else {
                                // move in negative direction
                                fRSVG[i].TargetPosition = Jog[i].GotoPos;
                                fRSVG[i].DynamicLimits.PositionPos = 1e37; //ignore positive limit when jogging in negative direction
                            }
                        }
                    }// end JOG_JOINTS

                    else if (Jog[i].Mode == JOG_BASE) {
                        /* use linear or angular limits according to what path axis is being jogged */
                        if (Jog[i].AxisIndex < 3) {
                            memcpy(&fRSVG[i].DynamicLimits,&PathLimits,sizeof(fRSVG[i].DynamicLimits));
                            memcpy(&fRSVG[i].DynamicValues,&PathLimits,sizeof(fRSVG[i].DynamicValues));
                        } else {
                            memcpy(&fRSVG[i].DynamicLimits,&AngularPathLimits,sizeof(fRSVG[i].DynamicLimits));
                            memcpy(&fRSVG[i].DynamicValues,&AngularPathLimits,sizeof(fRSVG[i].DynamicValues));                            
                        }
                        
                        /* modulate max speed according to set override */
                        fRSVG[i].DynamicLimits.VelocityPos *= fRSVG[i].Override;
                        fRSVG[i].DynamicLimits.VelocityNeg *= fRSVG[i].Override;
                        fRSVG[i].DynamicValues.VelocityPos *= fRSVG[i].Override;
                        fRSVG[i].DynamicValues.VelocityNeg *= fRSVG[i].Override;
                        
                        fRSVG[i].StartPosition = gRobot[i]->Monitor.MountBasePosition[Jog[i].AxisIndex];

                        // no path limits are set when jogging path axes because limits would depend on current tool and orientation of robot axes
                        // the user needs to manually check its limits
                        // joints are limited anyway through the workspace monitoring
						
                        if (Jog[i].Direction == JOG_POSITIVE) {
                            fRSVG[i].TargetPosition = fRSVG[i].DynamicLimits.PositionPos;
                        } else if (Jog[i].Direction == JOG_NEGATIVE) {
                            fRSVG[i].TargetPosition = fRSVG[i].DynamicLimits.PositionNeg;
                        } else if (Jog[i].Direction == JOG_GOTO) {
                            fRSVG[i].TargetPosition = Jog[i].GotoPos;
                        }
                    }// end JOG_BASE

                    else if (Jog[i].Mode == JOG_TOOL) {
                        /* use linear or angular limits according to what path axis is being jogged */
                        if (Jog[i].AxisIndex < 3) {
                            memcpy(&fRSVG[i].DynamicLimits,&PathLimits,sizeof(fRSVG[i].DynamicLimits));
                            memcpy(&fRSVG[i].DynamicValues,&PathLimits,sizeof(fRSVG[i].DynamicValues));
                        } else {
                            memcpy(&fRSVG[i].DynamicLimits,&AngularPathLimits,sizeof(fRSVG[i].DynamicLimits));
                            memcpy(&fRSVG[i].DynamicValues,&AngularPathLimits,sizeof(fRSVG[i].DynamicValues));                            
                        }

                        /* modulate max speed according to set override */
                        fRSVG[i].DynamicLimits.VelocityPos *= fRSVG[i].Override;
                        fRSVG[i].DynamicLimits.VelocityNeg *= fRSVG[i].Override;
                        fRSVG[i].DynamicValues.VelocityPos *= fRSVG[i].Override;
                        fRSVG[i].DynamicValues.VelocityNeg *= fRSVG[i].Override;
                        
                        fRSVG[i].StartPosition = 0;

                        // no path limits are set when jogging path axes because limits would depend on current tool and orientation of robot axes
                        // the user needs to manually check its limits
                        // joints are limited anyway through the workspace monitoring
						
                        if (Jog[i].Direction == JOG_POSITIVE) {
                            fRSVG[i].TargetPosition = fRSVG[i].DynamicLimits.PositionPos;
                        } else if (Jog[i].Direction == JOG_NEGATIVE) {
                            fRSVG[i].TargetPosition = fRSVG[i].DynamicLimits.PositionNeg;
                        } else if (Jog[i].Direction == JOG_GOTO) {
                            fRSVG[i].TargetPosition = Jog[i].GotoPos;
                        }
                    }

                    fRSVG[i].StartSpeed = 0;
                    fRSVG[i].EndSpeed = 0;
                    fRSVG[i].StartAcc = 0;
                    fRSVG[i].Start = 1;
                    fRSVG[i].Enable = 1;
                    OldSVGPos[i] = fRSVG[i].StartPosition;
                }
            }
        }
            //stop jogging when jog input command goes to zero 
        else if(gRobot[i]->Monitor.State == JOGGING) {
            gRobot[i]->Commands.Stop = 1;
        }
		
        //override set to 100% when jogging, max speed was already reduced when starting movement
        if(gRobot[i]->Monitor.State == JOGGING) {
            fRSVG[i].Override = 1.0;
        }
        
        
        /* run program command */
        if (gRobot[i]->Commands.RunProgram) {
            
            gRobot[i]->Commands.RunProgram = 0;
            
            if ((gRobot[i]->Monitor.State == STANDSTILL_STATE)&&(fRSVG[i].Status == ERR_DISABLED)) {
                memset(&Buffer[i],0,sizeof(Buffer[i])); //all buffer indexes and flags are reset here
                if (gRobot[i]->Parameters.StartLine > 0) {
                    Buffer[i].IP_PrgCount = gRobot[i]->Parameters.StartLine;
                } else {
                    Buffer[i].IP_PrgCount = 1; //start from line 1 of nc program
                }
                //use current position as target position of last block (for PP to plan first block)
                memcpy(&Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointJoint,&gRobot[i]->Monitor.JointPosition,sizeof(Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointJoint));
                memcpy(&Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointPath,&gRobot[i]->Monitor.MountBasePosition,sizeof(Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointPath));
                Buffer[i].PP_Index_Prev = BUFFER_LENGTH;
				
                //set last block speed to zero (for EXEC to start first block at zero speed)
                Buffer[i].MotionPackage[BUFFER_LENGTH].EndSpeed = 0;
                Buffer[i].EXEC_Index_Prev = BUFFER_LENGTH;
				
                /* dynamically allocate memory for the program string */

                 Robot_Program[i] = calloc(strlen(gRobot[i]->Parameters.Program) + 1,sizeof(char));

                if (Robot_Program[i] == 0) { //memory allocation error
                    gRobot[i]->Monitor.ActiveError = ERR_FILE_NOMEMORY;
                    gRobot[i]->Monitor.ErrorLine = 0;
                } else { // copy program string into new memory so that it will not be modified during program execution
                    strcpy(Robot_Program[i], gRobot[i]->Parameters.Program);
                    if (strlen(Robot_Program[i]) == 0) { // empty string
                        gRobot[i]->Monitor.ActiveError = ERR_FILE_EMPTY;
                        gRobot[i]->Monitor.ErrorLine = 0;
                    }
                }

                gRobot[i]->Monitor.State = MOVING;
            }
        }
		
        /* run block command */
        if (gRobot[i]->Commands.RunBlocks) {
            
            gRobot[i]->Commands.RunBlocks = 0;
            
            if ((gRobot[i]->Monitor.State == STANDSTILL_STATE)&&(fRSVG[i].Status == ERR_DISABLED)) {
                memset(&Buffer[i],0,sizeof(Buffer[i])); //all buffer indexes and flags are reset here
				
                Buffer[i].IP_PrgCount = 0; //start from block 0
				
                //use current position as target position of last block (for PP to plan first block)
                memcpy(&Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointJoint,&gRobot[i]->Monitor.JointPosition,sizeof(Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointJoint));
                memcpy(&Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointPath,&gRobot[i]->Monitor.MountBasePosition,sizeof(Buffer[i].MotionPackage[BUFFER_LENGTH].Path.TargetPointPath));
                Buffer[i].PP_Index_Prev = BUFFER_LENGTH;
				
                //set last block speed to zero (for EXEC to start first block at zero speed)
                Buffer[i].MotionPackage[BUFFER_LENGTH].EndSpeed = 0;
                Buffer[i].EXEC_Index_Prev = BUFFER_LENGTH;

                gRobot[i]->Monitor.State = MOVING;				
            }
        }
		
        /* stop command */
        if (gRobot[i]->Commands.Stop) {
            gRobot[i]->Commands.Stop = 0;
            if ((gRobot[i]->Monitor.State == JOGGING)||(gRobot[i]->Monitor.State == MOVING)) {
                if ((!gRobot[i]->Monitor.Halted)||(HaltedByCmd[i])) {
                    fRSVG[i].Stop = 1;
                }
                StoppingError[i] = -1;
            }
        }
		
        /* halt command */
        if (gRobot[i]->Commands.Halt) {
            gRobot[i]->Commands.Halt = 0;
            if ((gRobot[i]->Monitor.State == MOVING)&&(!gRobot[i]->Monitor.Halted)) {
                HaltedByCmd[i] = 1; //needed to differentiate between halted by command and by single step mode
                gRobot[i]->Monitor.Halted = 1;
            }
        }

        /* continue command */
        if (gRobot[i]->Commands.Continue) {
            //gRobot[i]->Commands.Continue = 0; -> do not reset here, it is needed later for single step restart -> reset at end of cycle
            if (gRobot[i]->Monitor.State == MOVING) {
                gRobot[i]->Monitor.Halted = 0;
                HaltedByCmd[i] = 0;
            }
        }

        /* tool calibration command */
        //this is not really a motion command and can run synchronous to the rest of the application
        if (gRobot[i]->Parameters.Calibration.Tool.Start) {
            gRobot[i]->Parameters.Calibration.Tool.Start = 0;
            gRobot[i]->Parameters.Calibration.Tool.Status = ToolCalibration(gRobot[i]->Parameters.Calibration.Tool.Points,&gRobot[i]->Parameters.Calibration.Tool.Result);
        }
		
		
        /******************** call motion functions ********************/

        RSVG((struct RSVG_Type*)&fRSVG[i]);
        
        RedFactor[i] = 1.0; //reset every cycle
        
        
        /******************** cyclic workspace monitoring ********************/

        //make sure that joint axes limits are set correctly
        if (gRobot[i]->Monitor.State != ERROR_STATE) {
            //so that we do not keep adding a new error every cycle
            for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {			
                if ((gRobot[i]->Parameters.JointLimits[k].PositionPos < gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                    gRobot[i]->Monitor.ActiveError = ERR_WRONG_JOINT_LIMITS;
                    gRobot[i]->Monitor.ErrorLine = 0;
                }
            }							
        }
		
        if (gRobot[i]->Monitor.State == MOVING) {
            /* WORKSPACE MONITORING (cyclic) */
            
            for (k=0;k<MAX_ZONE;k++) {
                //check for allowed and forbidden zones
                if (!gRobot[i]->Parameters.Workspace[k].Type)   continue;

                unsigned short tmpInside = PointInBox(gRobot[i]->Monitor.ToolBasePosition,gRobot[i]->Parameters.Workspace[k].PositionMin,gRobot[i]->Parameters.Workspace[k].PositionMax);
                if ((!tmpInside && gRobot[i]->Parameters.Workspace[k].Type==ZONE_SAFE) || (tmpInside && gRobot[i]->Parameters.Workspace[k].Type==ZONE_FORBIDDEN)) {                   
                    fRSVG[i].EStop = 1;
                    StoppingError[i] = ERR_WORKSPACE_ZONE1 + k;
                    StoppingLine[i] = gRobot[i]->Monitor.LineNumber;
                    break;
                }                
            }            
            
            for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                //check for joint axes limits - only if these are set (max>min)
                if (((gRobot[i]->Monitor.JointPosition[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(gRobot[i]->Monitor.JointPosition[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))&&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                    fRSVG[i].EStop = 1;
                    StoppingError[i] = ERR_LIMIT_J1 + k;
                    StoppingLine[i] = gRobot[i]->Monitor.LineNumber;
                }
            }							            
        }

        if ((gRobot[i]->Monitor.State == JOGGING)&&((Jog[i].Mode == JOG_BASE)||(Jog[i].Mode == JOG_TOOL))) {
            // do not check limits when jogging joints - user needs to be able to move back to workspace with manual movements
            for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {					
                //check for joint axes limits - only if these are set (max>min)
                if (((gRobot[i]->Monitor.JointPosition[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(gRobot[i]->Monitor.JointPosition[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))&&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                    fRSVG[i].EStop = 1;
                    StoppingError[i] = ERR_LIMIT_J1 + k;
                }
            }							
        }


		
        /******************** check for active errors ********************/
	
        if (gRobot[i]->Monitor.ActiveError != 0) {
            // force to error state
            gRobot[i]->Monitor.State = ERROR_STATE;
        }
			
        
			
        /******************** robot state machine ********************/
        switch (gRobot[i]->Monitor.State) {
            case STANDSTILL_STATE: {
                
                    gRobot[i]->Monitor.Moving = 0;
                    gRobot[i]->Monitor.Halted = 0;
                    HaltedByCmd[i] = 0;
                    gRobot[i]->Monitor.PathSpeed = 0;
                    gRobot[i]->Monitor.TrackSynch = 0;
                    gRobot[i]->Monitor.TrackActive = 0;
                    gRobot[i]->Monitor.TangActive = 0;
                    gRobot[i]->Monitor.TangOffset = 0;
				
                    fRSVG[i].Enable = 0;
                    if (StoppedTime[i] <= gRobot[i]->Parameters.FilterTime) StoppedTime[i] += gRobot[i]->Parameters.CycleTime;
                    strcpy(gRobot[i]->Monitor.CurrentBlock,"");
                    gRobot[i]->Monitor.LineNumber = 0;
                    gRobot[i]->Monitor.TargetPoint = 0;
                    gRobot[i]->Monitor.BlockLength = 0;
                    gRobot[i]->Monitor.CompletedBlockLength = 0;
                    memset(gRobot[i]->Monitor.M,0,sizeof(gRobot[i]->Monitor.M));
			
                    if (Robot_Program[i] != 0) {
                        //free program string memory

                    	free(Robot_Program[i]);
				
                        Robot_Program[i] = NULL;
                    }
				
                    // Tool and frame are not reset here: they stay valid also at standstill, otherwise path axes monitor positions are not correct
                    //However, the tool can be manually modified (e.g. if a new tool is mounted)
                    //call direct TRF to realign path axes
                    memcpy(&OldAxesValues,&gRobot[i]->Monitor.MountBasePosition,sizeof(OldAxesValues));
                    Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,gRobot[i]->Monitor.JointPosition,OldAxesValues,gRobot[i]->Monitor.MountBasePosition);
                    if (Trf_Status != STATUS_OK) {
                        gRobot[i]->Monitor.ActiveError = Trf_Status;
                        gRobot[i]->Monitor.ErrorLine = 0;
                    }
                    break;
                }

            case JOGGING: {
                    gRobot[i]->Monitor.Moving = 1;
								
                    if (fRSVG[i].Done) {
                        // jog completed (stopped or axis reached limits)
                        if (StoppingError[i] > 0) {
                            //movement stopped because of error
                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                            gRobot[i]->Monitor.ErrorLine = 0;
                            gRobot[i]->Monitor.State = ERROR_STATE;
                        } else {
                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                        }			
                        StoppingError[i] = 0;
                        StoppingLine[i] = 0;
                    }
                
					
                    if (fRSVG[i].Status >= ERR_SPG) { 
                        // error in SVG call
                        gRobot[i]->Monitor.ActiveError = fRSVG[i].Status;
                        gRobot[i]->Monitor.ErrorLine = 0;
                    }
				
                    /* call Trf to calculate all axes positions */
                    if (Jog[i].Mode == JOG_JOINTS) {
                        gRobot[i]->Monitor.JointPosition[Jog[i].AxisIndex] = fRSVG[i].Position;
                        memcpy(&OldAxesValues,&gRobot[i]->Monitor.MountBasePosition,sizeof(OldAxesValues));
                        Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,gRobot[i]->Monitor.JointPosition,OldAxesValues,gRobot[i]->Monitor.MountBasePosition);
                        if (Trf_Status != STATUS_OK) {
                            gRobot[i]->Monitor.ActiveError = Trf_Status;
                            gRobot[i]->Monitor.ErrorLine = 0;
                        }
                    } else if (Jog[i].Mode == JOG_BASE) {
                        short WorkspaceFlag = 0;
                        double OldPath[6];
                        memcpy(&OldPath,&gRobot[i]->Monitor.MountBasePosition,sizeof(OldPath));
                        memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                
                        // OptMot:
                        // try to execute movement at given speed
                        // check if joint dynamic limits are violated
                        // if it fails then execute it again at lower speed
                        unsigned short MovementAllowed = 1;
                        double JogPosition;
                
                        do {
                        
                            if (MovementAllowed) {
                                //first time we attempt movement - try full speed
                                JogPosition = fRSVG[i].Position;
                            } else {
                                //previous movement attempt failed - repeat with new target position (calculated at limited speed)
                                JogPosition = fRSVG[i].Position - fRSVG[i].Speed * TaskCycleTime * (1 - RedFactor[i]);
                                MovementAllowed = 255; // do not repeat test again
                            }

                            if ((gRobot[i]->Monitor.AxesNum == 6)&&(Jog[i].AxisIndex >=3)) {
                                //orientation of 6ax robot around base axes
                                /*
                                1. calculate R0 from current ABC
                                2. calculate R+ from additive ABC
                                3. endmatrix R1 = R+ x R0
                                4. extract target ABC
                                */
                                double A_old = gRobot[i]->Monitor.MountBasePosition[3];
                                double B_old = gRobot[i]->Monitor.MountBasePosition[4];
                                double C_old = gRobot[i]->Monitor.MountBasePosition[5];
                                double R_old[3][3];
                                ComposeMatrix(R_old,A_old,B_old,C_old);
                                double R_add[3][3];
                                double A_add = 0;
                                double B_add = 0;
                                double C_add = 0;
                                if (Jog[i].AxisIndex == 3) A_add = JogPosition - OldSVGPos[i];
                                if (Jog[i].AxisIndex == 4) B_add = JogPosition - OldSVGPos[i];
                                if (Jog[i].AxisIndex == 5) C_add = JogPosition - OldSVGPos[i];
                                ComposeMatrix(R_add,A_add,B_add,C_add);
                                double R_new[3][3];
                                MatMult(R_add,R_old,R_new);
                                double A_new, B_new, C_new;
                                DecomposeMatrix(R_new,A_old,B_old,C_old,&A_new,&B_new,&C_new);
                                gRobot[i]->Monitor.MountBasePosition[3] = A_new;
                                gRobot[i]->Monitor.MountBasePosition[4] = B_new;
                                gRobot[i]->Monitor.MountBasePosition[5] = C_new;
                            } else {
                                gRobot[i]->Monitor.MountBasePosition[Jog[i].AxisIndex] = JogPosition;
                            }
                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
                       
                            if (Trf_Status == STATUS_OK) {
                                //make sure joints are within limits
                                for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {					
                                    //check for joint axes limits - only if these are set (max>min)
                                    if (((gRobot[i]->Monitor.JointPosition[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(gRobot[i]->Monitor.JointPosition[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))&&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                                        WorkspaceFlag = k+1;
                                    }
                                }							
                            }
                            if ((Trf_Status != STATUS_OK)||(WorkspaceFlag)) {
                                // ignore movement and reload old position
                                memcpy(&gRobot[i]->Monitor.MountBasePosition,&OldPath,sizeof(gRobot[i]->Monitor.MountBasePosition));
                                memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                if (WorkspaceFlag == 0) gRobot[i]->Monitor.ActiveError = Trf_Status;
                                else gRobot[i]->Monitor.ActiveError = ERR_LIMIT_J1 + WorkspaceFlag-1;
                                gRobot[i]->Monitor.ErrorLine = 0;
                                gRobot[i]->Monitor.State = STANDSTILL_STATE;
                            }
                    
                            //reduce path speed if speed of any joint axis is exceeded
                            // MovementAllowed is set to 255 after test already failed once - no need to repeat again
                            if (MovementAllowed != 255) {
                                double tmpRedFactor;
                                unsigned short BadAxes = DynamicLimitsViolated(OldAxesValues,gRobot[i]->Monitor.JointPosition,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits,TaskCycleTime,&tmpRedFactor);
                            
                                if (BadAxes) {
                                    RedFactor[i] = tmpRedFactor;
                            
                                    if (RedFactor[i] > 1) {
                                        //should never happen!
                                        RedFactor[i] = 1;
                                        gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                    }
                                            
                                    if (RedFactor[i] < 0) {
                                        //should never happen!
                                        RedFactor[i] = 0;
                                        gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                    }
                                            
                                    //repeat movement with slower speed
                                    MovementAllowed = 0;
                                    memcpy(&gRobot[i]->Monitor.MountBasePosition,&OldPath,sizeof(gRobot[i]->Monitor.MountBasePosition));
                                    memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                }               
                            } else {
                                for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                    //manually truncate joint movements -> WARNING! TCP will leave programmed path!
                                    //TODO - pack this in FUB!!!
                                    double JointSpeed = (gRobot[i]->Monitor.JointPosition[j]-OldAxesValues[j]) / TaskCycleTime; //do not scale cycle time here!
                                    if (JointSpeed != 0) {
                                        if ((JointSpeed > 0)&&(gRobot[i]->Parameters.JointLimits[j].VelocityPos / JointSpeed < 1)) {
                                            gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] + gRobot[i]->Parameters.JointLimits[j].VelocityPos * TaskCycleTime;
                                        }
                                        if ((JointSpeed < 0)&&(-gRobot[i]->Parameters.JointLimits[j].VelocityNeg / JointSpeed < 1)) {
                                            gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] - gRobot[i]->Parameters.JointLimits[j].VelocityNeg * TaskCycleTime;
                                        }
                                    }

                                }
                            }
                        
                        } while (MovementAllowed == 0);
                
                    } else if (Jog[i].Mode == JOG_TOOL) {
                        
                        short WorkspaceFlag = 0;
                        double OldPath[6];
                        double OldMount[6];
                        memcpy(OldPath,gRobot[i]->Monitor.ToolBasePosition,sizeof(OldPath)); //include active tool
                        memcpy(OldMount,gRobot[i]->Monitor.MountBasePosition,sizeof(OldMount));
                        memcpy(OldAxesValues,gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                        double NewPath[6] = {0,0,0,0,0,0};

                        // OptMot:
                        // try to execute movement at given speed
                        // check if joint dynamic limits are violated
                        // if it fails then execute it again at lower speed
                        unsigned short MovementAllowed = 1;
                
                        do {
                                        
                            if (MovementAllowed) {
                                //first time we attempt movement - try full speed
                                NewPath[Jog[i].AxisIndex] = fRSVG[i].Speed * TaskCycleTime;
                            } else {
                                //previous movement attempt failed - repeat with new target position (calculated at limited speed)
                                NewPath[Jog[i].AxisIndex] = fRSVG[i].Speed * TaskCycleTime * RedFactor[i];
                                MovementAllowed = 255; // do not repeat test again
                            }

                
                            //calculate target point in base frame given it in tool frame
                            if (gRobot[i]->Monitor.AxesNum == 6) {
                                SubFrameTool3D(NewPath,OldPath,gRobot[i]->Parameters.Tool[gRobot[i]->Monitor.Tool].Axes,OldMount[3],OldMount[4],OldMount[5],gRobot[i]->Monitor.MountBasePosition);						
                            } else {
                                SubFrameTool2D(NewPath,OldPath,gRobot[i]->Parameters.Tool[gRobot[i]->Monitor.Tool].Axes,gRobot[i]->Monitor.MountBasePosition);
                            }

                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);	
                            if (Trf_Status == STATUS_OK) {
                                //make sure joints are within limits
                                for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {
                                    //check for joint axes limits - only if these are set (max>min)
                                    if (((gRobot[i]->Monitor.JointPosition[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(gRobot[i]->Monitor.JointPosition[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))&&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                                        WorkspaceFlag = k+1;
                                    }
                                }							
                            }
                            if ((Trf_Status != STATUS_OK)||(WorkspaceFlag)) {
                                // ignore movement and reload old position
                                memcpy(&gRobot[i]->Monitor.MountBasePosition,&OldMount,sizeof(gRobot[i]->Monitor.MountBasePosition));
                                memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                memcpy(&gRobot[i]->Monitor.ToolBasePosition,&OldPath,sizeof(gRobot[i]->Monitor.ToolBasePosition)); //TODO - do we need this???!
                                if (WorkspaceFlag == 0) gRobot[i]->Monitor.ActiveError = Trf_Status;
                                else gRobot[i]->Monitor.ActiveError = ERR_LIMIT_J1 + WorkspaceFlag-1;
                                gRobot[i]->Monitor.ErrorLine = 0;
                                gRobot[i]->Monitor.State = STANDSTILL_STATE;
                            }  
                        
                            //reduce path speed if speed of any joint axis is exceeded
                            // MovementAllowed is set to 255 after test already failed once - no need to repeat again
                            if (MovementAllowed != 255) {
                                double tmpRedFactor;
                                unsigned short BadAxes = DynamicLimitsViolated(OldAxesValues,gRobot[i]->Monitor.JointPosition,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits,TaskCycleTime,&tmpRedFactor);
                            
                                if (BadAxes) {
                                    RedFactor[i] = tmpRedFactor;
                            
                                    if (RedFactor[i] > 1) {
                                        //should never happen!
                                        RedFactor[i] = 1;
                                        gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                    }
                                            
                                    if (RedFactor[i] < 0) {
                                        //should never happen!
                                        RedFactor[i] = 0;
                                        gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                    }
                                            
                                    //repeat movement with slower speed
                                    MovementAllowed = 0;
                                    memcpy(&gRobot[i]->Monitor.MountBasePosition,&OldMount,sizeof(gRobot[i]->Monitor.MountBasePosition));
                                    memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                    memcpy(&gRobot[i]->Monitor.ToolBasePosition,&OldPath,sizeof(gRobot[i]->Monitor.ToolBasePosition)); //TODO - do we need this?!?!?
                                }               
                            } else {
                                for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) { 
                                    //manually truncate joint movements -> WARNING! TCP will leave programmed path!
                                    double JointSpeed = (gRobot[i]->Monitor.JointPosition[j]-OldAxesValues[j]) / TaskCycleTime; //do not scale cycle time here!
                                    if (JointSpeed != 0) {
                                        if ((JointSpeed > 0)&&(gRobot[i]->Parameters.JointLimits[j].VelocityPos / JointSpeed < 1)) {
                                            gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] + gRobot[i]->Parameters.JointLimits[j].VelocityPos * TaskCycleTime;
                                        }
                                        if ((JointSpeed < 0)&&(-gRobot[i]->Parameters.JointLimits[j].VelocityNeg / JointSpeed < 1)) {
                                            gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] - gRobot[i]->Parameters.JointLimits[j].VelocityNeg * TaskCycleTime;
                                        }
                                    }
                                }
                            }
                        } while (MovementAllowed == 0);
                    }
                    OldSVGPos[i] = fRSVG[i].Position;
                    break;
                }
                
            case MOVING: {
                    
                    gRobot[i]->Monitor.Moving = 1;

                    /* NOTE - any error while program is running (either in interpreter or path-planner or svg) should cause e-stop, not jump directly to error_state */
                    //fRSVG[i].EStop = 1;
                    //StoppingError[i] = ERR...;
				
                    /*** ---- IP ---- ***/
                    /*** interpret next block to keep buffer full ***/
                    unsigned short IP_LoopCount = 0;
                    //loop until buffer full or end of file or active error (or stop)
                    while ((Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_UNDEF)&&(IP_LoopCount < MAX_LOOP_COUNT)&&(Buffer[i].Eof == 0)&&(StoppingError[i] == 0)) {
                        IP_LoopCount++; //used to avoid long loops in case NC program is full of comments or empty lines; infinite loops caused by GOTO; or long searches for labels
                        if ( ((Robot_Program[i] != 0)&&(LineFromString(Robot_Program[i], line, Buffer[i].IP_PrgCount) == STATUS_OK)) ||		//move_program
                        ((Robot_Program[i] == 0)&&(LineFromString(gRobot[i]->Parameters.Blocks[Buffer[i].IP_PrgCount], line, 0) == STATUS_OK)) ) {		//move_blocks
                            // line read correctly from file
                            Buffer[i].MotionPackage[Buffer[i].IP_Index].Feedrate = Buffer[i].ModalFeedrate; //keep modal feedrate (if exists)
                            Buffer[i].MotionPackage[Buffer[i].IP_Index].FeedrateType = Buffer[i].ModalFeedrateType; //keep modal feedrate (if exists)
                            unsigned short IP_Status = Interpreter(line,&(Buffer[i].MotionPackage[Buffer[i].IP_Index]));
                            
                            if (IP_Status != STATUS_OK) { //interpreter error -> abort program with error
                                fRSVG[i].EStop = 1;
                                StoppingError[i] = IP_Status;
                                StoppingLine[i] = Buffer[i].IP_PrgCount;
                            }
                                
                            else if ((Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_GOTO)||(Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_SUB)) { 
                                //found GOTO/SUB line -> interrupt interpreter and start looking for label
                                
                                if (Robot_Program[i] == 0) {	//GOTO/SUB not supported for move_blocks          
                                    fRSVG[i].EStop = 1;
                                    StoppingError[i] = ERR_IP_JUMP;
                                    StoppingLine[i] = Buffer[i].IP_PrgCount;								
                                } else {	
                                    //check if this loop is already active
                                    short tmpLoopFound = 0;
                                    for(j=0;j<MAX_SUBLEVEL;j++)
                                    {
                                        if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_GOTO && Buffer[i].GotoBuffer[j].Line == Buffer[i].IP_PrgCount)
                                        {//loop already exists - check how many iterations are left
                                            tmpLoopFound = 1;
                                            if (Buffer[i].GotoBuffer[j].Counter > 0)
                                            {//next iteration
                                                Buffer[i].GotoBuffer[j].Counter--;
                                                Buffer[i].IP_PrgCount = Buffer[i].GotoBuffer[j].Index; //jump to target label
                                                Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_UNDEF;	//delete GOTO/SUB command from buffer	
                                            } else if (Buffer[i].GotoBuffer[j].Counter < 0) {//infinite loop
                                                Buffer[i].GotoBuffer[j].Counter = -1;
                                                Buffer[i].IP_PrgCount = Buffer[i].GotoBuffer[j].Index; //jump to target label
                                                Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_UNDEF;	//delete GOTO/SUB command from buffer	                                            
                                            } else {//no more iterations
                                                Buffer[i].IP_PrgCount++; //jump to next line
                                                Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_UNDEF;	//delete GOTO/SUB command from buffer
                                                //delete loop from buffer (this is important because the same loop might be accessed later again and the counter will have to start again)
                                                Buffer[i].GotoBuffer[j].Index = 0;
                                                Buffer[i].GotoBuffer[j].Line = 0;
                                                Buffer[i].GotoBuffer[j].Counter = 0;
                                            }
                                            break;
                                        } else if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_SUB && Buffer[i].SubBuffer[j].Line == Buffer[i].IP_PrgCount) {
                                            tmpLoopFound = 1;
                                            if (Buffer[i].SubBuffer[j].Counter > 0) {//next iteration
                                                Buffer[i].SubBuffer[j].Counter--;
                                                Buffer[i].IP_SubLevel++;
                                                Buffer[i].IP_PrgCount = Buffer[i].SubBuffer[j].Index; //jump to target label
                                                Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_UNDEF;	//delete GOTO/SUB command from buffer	
                                            } else {//no more iterations
                                                Buffer[i].IP_PrgCount++; //jump to next line
                                                Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_UNDEF;	//delete GOTO/SUB command from buffer
                                                //delete loop from buffer (this is important because the same loop might be accessed later again and the counter will have to start again)
                                                Buffer[i].SubBuffer[j].Index = 0;
                                                Buffer[i].SubBuffer[j].Line = 0;
                                                Buffer[i].SubBuffer[j].Counter = 0;
                                            }
                                            break;                                        
                                        }
                                    }
                                
                                    if (!tmpLoopFound) {//loop not found - create a new one at first available space in buffer
                                        short tmpCreated = 0; 
                                        for(j=0;j<MAX_SUBLEVEL;j++) {
                                            if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_GOTO && Buffer[i].GotoBuffer[j].Line == 0) {
                                                Buffer[i].GotoBuffer[j].Line = Buffer[i].IP_PrgCount;
                                                Buffer[i].GotoBuffer[j].Counter = Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Counter-1;
                                                Buffer[i].GotoBuffer[j].Index = 0;
                                                tmpCreated = 1;
                                                break;
                                            } else if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_SUB && Buffer[i].SubBuffer[j].Line == 0) {
                                                Buffer[i].SubBuffer[j].Line = Buffer[i].IP_PrgCount;
                                                Buffer[i].SubBuffer[j].Counter = Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Counter-1;
                                                Buffer[i].SubBuffer[j].Index = 0;
                                                tmpCreated = 1;
                                                break;
                                            }
                                        }
                                    
                                        Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Found = 0; //look for label to jump to
                                        Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Index = 0;

                                        //error if buffer is already full
                                        if (!tmpCreated) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_IP_SUBLEVEL;
                                            StoppingLine[i] = Buffer[i].IP_PrgCount;
                                        }
                                    }
                                }
                            }
                                
                            else if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_END) {
                                if (Buffer[i].IP_SubLevel == 0) {//END of main program -> stop interpreter
                                    Buffer[i].Eof = 1;
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].LineNumber = Buffer[i].IP_PrgCount;
                                } else {//END of subprogram -> return to calling program
                                    Buffer[i].IP_PrgCount = Buffer[i].IP_ReturnIdx[Buffer[i].IP_SubLevel];
                                    Buffer[i].IP_SubLevel--;
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_UNDEF;	//delete END command from buffer
                                }
                            }
                    
                            else if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType != MOVE_UNDEF)
                            { //found movement block -> save in buffer
                                Buffer[i].MotionPackage[Buffer[i].IP_Index].LineNumber = Buffer[i].IP_PrgCount;
                                strcpy(Buffer[i].MotionPackage[Buffer[i].IP_Index].BlockString,line);
                                Buffer[i].ModalFeedrate = Buffer[i].MotionPackage[Buffer[i].IP_Index].Feedrate; //save new modal feedrate
                                Buffer[i].ModalFeedrateType = Buffer[i].MotionPackage[Buffer[i].IP_Index].FeedrateType; //save new modal feedrate type
                                if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_HOME) {// transform HOME into MJ P0
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_PTP;
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].TargetPoint = 0;
                                } else if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_TOOL) {// transform Tx into MJ Pact Tx
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_PTP;
                                    //use dirty trick to tell PTP that target position is the same as start position
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].DelayTime = POINT_SAME;
                                } else if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_TRK) { //IP error if tracking index was not programmed correctly!
                                    if (((Buffer[i].MotionPackage[Buffer[i].IP_Index].TrkIndex == 0)&&(Buffer[i].IP_TrkIndex == 0))||
                                        ((Buffer[i].MotionPackage[Buffer[i].IP_Index].TrkIndex > 0)&&(Buffer[i].IP_TrkIndex != 0))) {
                                        fRSVG[i].EStop = 1;
                                        StoppingError[i] = ERR_IP_TRK_INDEX;
                                        StoppingLine[i] = Buffer[i].IP_PrgCount;									
                                    } else {
                                        Buffer[i].IP_TrkIndex = Buffer[i].MotionPackage[Buffer[i].IP_Index].TrkIndex;
                                    }
                                }
                                Buffer[i].IP_Index++;
                                if (Buffer[i].IP_Index >= BUFFER_LENGTH) { //circular buffer
                                    Buffer[i].IP_Index = 0;
                                }
                                //move to next line
                                Buffer[i].IP_PrgCount++;
                                if ((Robot_Program[i] == 0)&&(Buffer[i].IP_PrgCount >= RING_BUFFER_SIZE)) { //consider ring buffer for move_blocks
                                    Buffer[i].IP_PrgCount -= RING_BUFFER_SIZE;
                                }
                            }
                                
                            else
                            { //useless line (e.g. empty or comment) -> advance to next line (for move_program) or abort (for move_blocks) 
                                if (Robot_Program[i] == 0) {
                                    Buffer[i].Eof = 1;
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].LineNumber = Buffer[i].IP_PrgCount;
                                } else {
                                    Buffer[i].IP_PrgCount++;
                                }
                            }
                        } else {// cannot read line -> end of file reached, stop interpreter here
                            Buffer[i].Eof = 1;
                        }
                
                        //stop reading program at StopLine
                        if ((Robot_Program[i] != 0)&&(Buffer[i].IP_PrgCount > gRobot[i]->Parameters.StopLine)&&(gRobot[i]->Parameters.StopLine > 0)) {
                            Buffer[i].Eof = 1;
                        }              
                
                    } //while

                    /* GOTO/SUB block -> look for label to jump to */
                    while(((Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_GOTO)||(Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_SUB))&&(Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Found == 0)&&(IP_LoopCount < MAX_LOOP_COUNT)&&(StoppingError[i] == 0)) {
                        IP_LoopCount++; //used to avoid long searches for labels
                        Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Index++;
                        if(LineFromString(Robot_Program[i], line, Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Index) == STATUS_OK) {//look for label in this line

                            char *tmp;
                            /* remove comment section */
                            tmp = strstr(line,"//");
                            if (tmp != 0) {
                                *tmp = 0; //cut string here
                            }
												
                            /* look for Label sign */
                            tmp = my_strcasestr(line,":");
                            if (tmp != 0) { //label found -> now check if it is the one we are searching for
                                *tmp =0; //cut string here
                                tmp = line;
                                while(*tmp == ' ') tmp++; //remove leading spaces
                                char* end = tmp + strlen(tmp) - 1;
                                while((end > tmp) && (*end == ' ')) end--;
                                *(end+1) = 0; //cut string here (to remove possible following empty spaces)
                                if (strcmp(tmp,Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Name) == 0) {
                                    //label match! -> reposition program counter
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Found = 1;
                                    if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_SUB) {
                                        //increase Subroutine level and record return position for program counter
                                        Buffer[i].IP_SubLevel++;
                                        if (Buffer[i].IP_SubLevel >= MAX_SUBLEVEL) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_IP_SUBLEVEL;
                                            StoppingLine[i] = Buffer[i].IP_PrgCount;
                                        } else {
                                            Buffer[i].IP_ReturnIdx[Buffer[i].IP_SubLevel] = Buffer[i].IP_PrgCount; //return to same line in case a counter was defined
                                        }
                                    }
                                    //save target line
                                    if (Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType == MOVE_GOTO) {
                                        for (j=0;j<MAX_SUBLEVEL;j++) {
                                            if (Buffer[i].GotoBuffer[j].Line == Buffer[i].IP_PrgCount) {
                                                Buffer[i].GotoBuffer[j].Index = Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Index;
                                                break;
                                            }
                                        }
                                    } else {
                                        for (j=0;j<MAX_SUBLEVEL;j++) {
                                            if (Buffer[i].SubBuffer[j].Line == Buffer[i].IP_PrgCount) {
                                                Buffer[i].SubBuffer[j].Index = Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Index;
                                                break;
                                            }
                                        }
                                    }
                                    Buffer[i].IP_PrgCount = Buffer[i].MotionPackage[Buffer[i].IP_Index].Label.Index; //jump to new line
                                    Buffer[i].MotionPackage[Buffer[i].IP_Index].MovementType = MOVE_UNDEF;	//delete GOTO/SUB command from buffer	
                                }
                            }
                        }
                        else { //eof and label not found -> abort program with error
                            fRSVG[i].EStop = 1;
                            StoppingError[i] = ERR_IP_LABEL;
                            StoppingLine[i] = Buffer[i].IP_PrgCount;
                        }
                    }


                    /*** --- PP --- ***/
                    /*** plan block ***/
                    if (Buffer[i].Synch) {
                        //signal from EXEC -> PP can move onto next block after halting for M-synch
                        Buffer[i].Synch = 0;
                        Buffer[i].PP_Index++;
                        //use current position as target position of last block (for PP to plan first block)
                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointJoint,&gRobot[i]->Monitor.JointPosition,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointJoint));
                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath,&gRobot[i]->Monitor.MountBasePosition,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath));
                    }

                    if (Buffer[i].PP_Index >= BUFFER_LENGTH) {
                        Buffer[i].PP_Index = 0;	
                    }
				
                    //plan the next unplanned block 
                    if (StoppingError[i] == 0) {
                        //if the block is already planned then the buffer is full and the PP needs to wait for the EXEC to consume a block
                        //if the block is a synch M-function or TRK then the PP needs to halt until reset by the EXEC
                        if ((!Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned)||(Buffer[i].MotionPackage[Buffer[i].PP_Index].MovementType == MOVE_MCODE_SYNCH)||(Buffer[i].MotionPackage[Buffer[i].PP_Index].MovementType == MOVE_TRK)) {
                            
                            switch (Buffer[i].MotionPackage[Buffer[i].PP_Index].MovementType) {
                                
                                case MOVE_UNDEF: {
                                        // nothing programmed in this block, either program is completed or interpreter has not written anything yet
                                        if (Buffer[i].Eof) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].MovementType = MOVE_END;
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Round = -1;
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1; //non-tangential transition
                                            Buffer[i].Planned = 1;
                                        }
                                        break;
                                    }
				
                                case MOVE_LINE: {
                                        /* START POINT */
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath));
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointJoint,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint));

                                        /* END POINT */
                                        if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_JOINTS) {
                                            // POINT_JOINTS
                                            memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,&gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint));
                                            // transform target point from joints to path coordinates
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_PATH) {
                                            // POINT_PATH
                                            // remove tool and frame to find target mountbaseframe
                                            if (gRobot[i]->Monitor.AxesNum < 5) {
                                                SubFrameTool2D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            } else {
                                                SubFrameTool3D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,0,0,0,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            }							
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint);
                                    
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else {// Point type not supported
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_POINT_TYPE;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }
				
                                        //check that line connecting start and end point (plus planned tool) does not violate workspace zones
                                        double tmpStartPoint[6];
                                        double tmpTargetPoint[6];
                                        if (gRobot[i]->Monitor.AxesNum < 5) {
                                            SubFrame2D(gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,tmpStartPoint);
                                            SubFrame2D(gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,tmpTargetPoint);
                                        } else {
                                            SubFrame3D(gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,0,0,0,tmpStartPoint);
                                            SubFrame3D(gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,0,0,0,tmpTargetPoint);
                                        }		
                                
                                        //path workspace monitoring
                                        for (k=0;k<MAX_ZONE;k++) {
                                            //check for allowed and forbidden zones
                                            if (!gRobot[i]->Parameters.Workspace[k].Type)   continue;
                                    
                                            unsigned short tmpInsideL1 = PointInBox(tmpStartPoint,gRobot[i]->Parameters.Workspace[k].PositionMin,gRobot[i]->Parameters.Workspace[k].PositionMax);
                                            unsigned short tmpInsideL2 = PointInBox(tmpTargetPoint,gRobot[i]->Parameters.Workspace[k].PositionMin,gRobot[i]->Parameters.Workspace[k].PositionMax);
                                            unsigned short tmpHitBox = LineCrossBox(tmpStartPoint,tmpTargetPoint,gRobot[i]->Parameters.Workspace[k].PositionMin,gRobot[i]->Parameters.Workspace[k].PositionMax);
                                            if (((!tmpInsideL1 || !tmpInsideL2) && gRobot[i]->Parameters.Workspace[k].Type==ZONE_SAFE) || (tmpHitBox && gRobot[i]->Parameters.Workspace[k].Type==ZONE_FORBIDDEN)) {                   
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_WORKSPACE_ZONE1 + k;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }                
                                        }            
                                
                                        //joints workspace monitoring
                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {
                                            //check for joint axes limits - only if these are set (max>min)
                                            if (((Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))&&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)){
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_LIMIT_J1 + k;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        }							

                                        //use quaternions for 6ax robots
                                        if (gRobot[i]->Monitor.AxesNum == 6) {//convert Euler angles into quaternions and calculate angle between them (in radians)
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat);
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                        } else {  //TODO - this is only valid for 4 axes robots!
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3]) * PI / 180.0 ; //quatang is in rad!
                                        }
				
                                        //calculate block length according to selected feedrate configuration
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType == FEED_CART) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLengthCart(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
					
                                            //fall back on default configuration if angular axes are not programmed
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength < TRF_EPSILON) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,gRobot[i]->Monitor.AxesNum);
                                            }
                                        } else if (Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType == FEED_ANG) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle) * 180.0 / PI; //blocklength in deg								

                                            //fall back on default configuration if angular axes are not programmed
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength < TRF_EPSILON) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,gRobot[i]->Monitor.AxesNum);
                                            }
                                        } else {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,gRobot[i]->Monitor.AxesNum);								
                                        }

                                        //blocklengthideal does not consider round edges and is cartesian only!
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLengthIdeal = LineLengthCart(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);

                                        /* calculate START and END direction vectors in path space */
                                        PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);
                                        Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[0] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[0];
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[1] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[1];
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[2] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[2];
                            
                                        /* calculate START direction vector in joint space */

                                        // interpolate linearly to find next point (at 0.1% of total block length)
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {//note that all axes are interpolated linearly, even orientation (should not be a big issue)
                                            tmpAxesValues[j] = 0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j] + 0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j];
                                        }
			
                                        Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,tmpAxesValues,tmpJointsValues);
                                        if (Trf_Status != STATUS_OK){
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = Trf_Status;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }					

                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].StartJointVector[k] = tmpJointsValues[k] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint[k];
                                        }

                                        /* calculate END direction vector in joint space */

                                        // interpolate linearly to find previous point (at 99.9% of total block length)
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {//note that all axes are interpolated linearly, even orientation (should not be a big issue)
                                            tmpAxesValues[j] = 0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j] + 0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j];
                                        }
			
                                        Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,tmpAxesValues,tmpJointsValues);
                                        if (Trf_Status != STATUS_OK) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = Trf_Status;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }					

                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].EndJointVector[k] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] - tmpJointsValues[k];
                                        }


                                        /* set default values for limit edge points -> used in EXEC even if no round edge is programmed */
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[4].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j];
                                        }
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndEdge.CtrlPoint[0].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j];
                                        }

				
                                        /* calculate angle from previous block*/
				
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Round < 0) {
                                            // no round edge was defined in previous block
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = AngleBetweenVectors(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartJointVector,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndJointVector);
                                        } else if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Round == 0) {
                                            // R0 was programmed in previous block -> non-tangential transition required
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1;
                                        } else {
                                            // Rxxx was programmed in previous block -> transition is tangential -> evaluate round edge parameters
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
				
                                            //calculate control points
                                            RoundEdgePoints(&Buffer[i].MotionPackage[Buffer[i].PP_Index], &Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev],gRobot[i]->Monitor.AxesNum,&gRobot[i]->Parameters.Mechanics);
                                
                                            //modify end path vector of previous block if it was a circle so that tangential angle is calculated correctly
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_CIRCLE) {
                                                double tmpCrossVersor[3];
                                                PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Center,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes,tmpCrossVersor);
                                                CrossProduct(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Normal,tmpCrossVersor,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector);
                                                Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector);
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.tangAngleStart = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[0]);
                                            }

                                            //modify end edge quaterion of previous block if it was a PTP
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_PTP) {
                                                //calculate quaternion of end edge
                                                EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[3],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[4],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.EdgeQuat);
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.EdgeQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndQuat);
                                            }

                                        }
                              
                                        //calculate initial and final tangential angle of current movement
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.tangAngleEnd = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[0]);
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndEdge.tangAngleStart = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[0]);
                                        //recall final tangential angle of previous movement
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.tangAngleStart = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.tangAngleStart;
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.tangAngleEnd = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.tangAngleEnd;
                                
                                        /* replan end of previous block if it was a spline */
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_SPLINE) {
                                
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                
                                            for(j=0;j<3;j++) {
                                                //end tangent of spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j];

                                                //third point of spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint[2].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath[j] - Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.LengthB;
                                            }                                                                        
                                    
                                            //spline length
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].BlockLength = BezierLength(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint,BEZIER_XYZ,BEZIER_CUBIC);
                                
                                            //path workspace monitoring of previous spline block
                                            tmpViolate = WorkspaceMonitor(MOVE_SPLINE,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Tool].Axes,gRobot[i]->Parameters.Workspace,gRobot[i]->Monitor.AxesNum,0);
                                            if (tmpViolate) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_WORKSPACE_ZONE1 + (tmpViolate-1);
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].LineNumber;                                
                                            }
                                
                                        }
				
                                        //limit feedrate: time to complete block cannot exceed time needed from joints to reach end position
                                        BlockTime = MinPathTime(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits);
                                        if (BlockTime != 0) {
                                            double tempSpeed = Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength / BlockTime;
				
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > tempSpeed) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = tempSpeed;
                                            }
                                        }
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > PathLimits.VelocityPos) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = PathLimits.VelocityPos;
                                        }
				
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].PP_Index_Prev = Buffer[i].PP_Index;
                                        Buffer[i].PP_Index++;

                                        break;
                                    }
				
                                case MOVE_PTP: {
                                        /* START POINT */
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath));
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointJoint,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint));
		
                                        /* END POINT */
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].DelayTime == POINT_SAME) { //target point equal to start point (used for tool change)
                                            memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath));
                                            memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointJoint,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint));                               
                                        } else if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_JOINTS) {// target point already in joints coordinates
                                            memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,&gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint));	
                                            // transform target point from joints to path coordinates
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_PATH) {// transform target point from path to joints coordinates

                                            // remove tool and frame to find target mountbaseframe
                                            if (gRobot[i]->Monitor.AxesNum < 5) {
                                                SubFrameTool2D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            } else {
                                                SubFrameTool3D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,0,0,0,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            }						
			
                                            memcpy(&OldAxesValues,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,sizeof(OldAxesValues));								
                                            //allow for reference joints values to modify the target joints values
                                            for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {
                                                if (Buffer[i].MotionPackage[Buffer[i].PP_Index].RefPoint.Defined[k]){
                                                    OldAxesValues[k] = Buffer[i].MotionPackage[Buffer[i].PP_Index].RefPoint.Axes[k];
                                                }
                                            }
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else {// Point type not supported
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_POINT_TYPE;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }

                                        //use quaternions for 6ax robots
                                        if (gRobot[i]->Monitor.AxesNum == 6) {//convert Euler angles into quaternions and calculate angle between them (in radians)
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat);
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                        } else { //TODO - this is only valid for 4 axes robots!
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3]) * PI / 180.0 ; //quatang is in rad!
                                        }
                                
                                        //calculate block length according to selected feedrate configuration
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType == FEED_CART) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = PTPLength(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path, gRobot[i]->Monitor.AxesNum, &gRobot[i]->Parameters.Mechanics);
					
                                            //fall back on default configuration if there is no cartesian displacement
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength < TRF_EPSILON) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum);
                                            }
                                        } else if (Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType == FEED_ANG) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle) * 180.0 / PI; //blocklength in deg								

                                            //fall back on default configuration if there is no angular displacement
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength < TRF_EPSILON) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum);
                                            }
                                        } else {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum);
                                        }
                                                
                                        //blocklengthideal does not consider round edges and is cartesian only!
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLengthIdeal = PTPLength(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path, gRobot[i]->Monitor.AxesNum, &gRobot[i]->Parameters.Mechanics);
     
                                        //path workspace monitoring
                                        tmpViolate = WorkspaceMonitor(MOVE_PTP,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,gRobot[i]->Parameters.Workspace,gRobot[i]->Monitor.AxesNum,&gRobot[i]->Parameters.Mechanics);
                                        if (tmpViolate) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_WORKSPACE_ZONE1 + (tmpViolate-1);
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;                                
                                        }
                            
                                        //joint monitoring
                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                                            //check for joint axes limits - only if these are set (max>min)
                                            if (((Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))&&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_LIMIT_J1 + k;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        }							
		
                                        /* calculate START and END direction vectors in path space */
                                
                                        // interpolate linearly to find next point (at 0.1% of total block length)
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                            tmpAxesValues[j] = 0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint[j] + 0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[j];
                                        }
							
                                        Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,tmpAxesValues,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,tmpPathValues);
                                        if (Trf_Status != STATUS_OK) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = Trf_Status;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }					
	
                                        for (k=0;k<3;k++) {	
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[k] = tmpPathValues[k] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[k];
                                        }
                                        Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);

                                        // interpolate linearly to find previous point (at 99.9% of total block length)
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                            tmpAxesValues[j] = 0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint[j] + 0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[j];
                                        }
							
                                        Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,tmpAxesValues,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,tmpPathValues);
                                        if (Trf_Status != STATUS_OK) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = Trf_Status;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }					
	
                                        for (k=0;k<3;k++) {	
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[k] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[k] - tmpPathValues[k];
                                        }
                                        Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector);

                                        /* calculate start and end direction vector in joint space */
                                        /* they are actually the same vector because a PTP is a linear interpolation in the joint space */
                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].StartJointVector[k] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint[k];
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].EndJointVector[k] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartJointVector[k];
                                        }

                                        /* set default values for limit edge points -> used in EXEC even if no round edge is programmed */
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[6].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[6].Axes));
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndEdge.CtrlPoint[5].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndEdge.CtrlPoint[5].Axes));

                                        /* calculate angle from previous block*/
				
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Round < 0) {
                                            // no round edge was defined in previous block
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = AngleBetweenVectors(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartJointVector,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndJointVector);
                                        } else if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Round == 0) {
                                            // R0 was programmed in previous block -> non-tangential transition required
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1;
                                        } else {
                                            // Rxxx was programmed in previous block -> transition is tangential
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;	

                                            //calculate control points
                                            RoundEdgePoints(&Buffer[i].MotionPackage[Buffer[i].PP_Index], &Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev],gRobot[i]->Monitor.AxesNum,&gRobot[i]->Parameters.Mechanics);

                                            //calculate quaternion of start edge
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[4].Axes[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[4].Axes[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[4].Axes[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.EdgeQuat);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.EdgeQuat);
                                    
                                            //modify end path vector of previous block if it was a circle so that tangential angle is calculated correctly
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_CIRCLE) {
                                                double tmpCrossVersor[3];
                                                PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Center,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes,tmpCrossVersor);
                                                CrossProduct(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Normal,tmpCrossVersor,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector);
                                                Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector);
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.tangAngleStart = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[0]);
                                            }

                                            //modify end edge quaterion of previous block if it was a PTP
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_PTP) {
                                                //calculate quaternion of end edge
                                                EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[3],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[4],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.EdgeQuat);
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.EdgeQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndQuat);
                                            }
                                        }
				
                                        /* replan end of previous block if it was a spline */
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_SPLINE) {
                                
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                
                                            for(j=0;j<3;j++) {
                                                //end tangent of spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j];
    
                                                //third point of spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint[2].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath[j] - Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.LengthB;
                                            }                                                                        
                                        
                                            //spline length
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].BlockLength = BezierLength(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint,BEZIER_XYZ,BEZIER_CUBIC);

                                            //path workspace monitoring of previous spline block
                                            tmpViolate = WorkspaceMonitor(MOVE_SPLINE,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Tool].Axes,gRobot[i]->Parameters.Workspace,gRobot[i]->Monitor.AxesNum,0);
                                            if (tmpViolate) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_WORKSPACE_ZONE1 + (tmpViolate-1);
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].LineNumber;                                
                                            }
                                        }		

                                        //limit feedrate: time to complete block cannot exceed time needed from joints to reach end position
                                        BlockTime = MinPathTime(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits);
                                        if (BlockTime != 0) {
                                            double tempSpeed = Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength / BlockTime;
				
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > tempSpeed) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = tempSpeed;
                                            }
                                        }
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > PathLimits.VelocityPos) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = PathLimits.VelocityPos;
                                        }
		
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].PP_Index_Prev = Buffer[i].PP_Index;
                                        Buffer[i].PP_Index++;
					
                                        break;
                                    }
				
                                case MOVE_CIRCLE: {
                                        /* START POINT */
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath));
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointJoint,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint));
						
                                        /* MIDDLE POINT */
                                        if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].CenterPoint].Mode == POINT_JOINTS) { // POINT_JOINTS
                                            memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint,&gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].CenterPoint].Axes,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint));
                                            // transform middle point from joints to path coordinates
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath);
                                            if (Trf_Status != STATUS_OK){
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].CenterPoint].Mode == POINT_PATH) {// POINT_PATH
                                            // remove tool and frame to find target mountbaseframe
                                            if (gRobot[i]->Monitor.AxesNum < 5) {
                                                SubFrameTool2D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].CenterPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath);
                                            } else {
                                                SubFrameTool3D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].CenterPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,0,0,0,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath);
                                            }							
							
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else {// Point type not supported
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_POINT_TYPE;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }
						
                                        /* END POINT */
                                        if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_JOINTS) {// POINT_JOINTS
                                            memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,&gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint));
                                            // transform target point from joints to path coordinates
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_PATH) {// POINT_PATH
                                            // remove tool and frame to find target mountbaseframe
                                            if (gRobot[i]->Monitor.AxesNum < 5) {
                                                SubFrameTool2D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            } else {
                                                SubFrameTool3D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,0,0,0,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            }							

                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else {// Point type not supported
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_POINT_TYPE;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }

						
                                        /* evaluate circle parameters */
                                        Circle_Status = EvalCircle(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path);
                                        if (Circle_Status != STATUS_OK) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_PP_CIRCLEPOINTS;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;								
                                        }
                                
                                        //rotation angle defined by user -> calculate new target point (position and orientation)
                                        // TODO - add handling of negative angles (invert rotation direction)
                                        if(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.RotAngle > 0) {
                                            //simplify notation
                                            Path_Type *Circle = &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path;
                                    
                                            double tmpMiddlePoint[6], tmpTargetPoint[6];
                                            memcpy(&tmpTargetPoint,&Circle->TargetPointPath,sizeof(tmpTargetPoint));
                                            memcpy(&tmpMiddlePoint,&Circle->MiddlePointPath,sizeof(tmpMiddlePoint));

                                            double UserLength = Circle->Radius * Circle->RotAngle / 180.0 * PI;
  
                                            //calculate new target point in path world
                                            double u = UserLength / Circle->Length;
                                            Circle->uM = Circle->MiddleLength / Circle->Length;
        
                                            //POSITION
                                            for(j=0;j<3;j++) {//X,Y,Z on circle
                                                //P = C + R cos(t) U + R sin(t) V
                                                tmpTargetPoint[j] = Circle->Center[j] + Circle->Radius * cos(Circle->Length * u / Circle->Radius) * Circle->StartVersor[j] + 
                                                    Circle->Radius * sin(Circle->Length * u / Circle->Radius) * Circle->CrossVersor[j];
                                        
                                                if (u <= Circle->uM) {
                                                    double tmp_u = u * 0.5;
                                                    tmpMiddlePoint[j] = Circle->Center[j] + Circle->Radius * cos(Circle->Length * tmp_u / Circle->Radius) * Circle->StartVersor[j] + 
                                                        Circle->Radius * sin(Circle->Length * tmp_u / Circle->Radius) * Circle->CrossVersor[j];
                                                }
                                            }

                                            //ORIENTATION
                                            if (gRobot[i]->Monitor.AxesNum != 6) {//orientation interpolated linearly
                                                for(j=3;j<gRobot[i]->Monitor.AxesNum;j++) {
                                                    if (u <= Circle->uM) {
                                                        double tmp_u = u/Circle->uM;
                                                        tmpTargetPoint[j] = (1-tmp_u) * Circle->StartPointPath[j] + tmp_u * Circle->MiddlePointPath[j];
                                                    
                                                        tmp_u *= 0.5;
                                                        tmpMiddlePoint[j] = (1-tmp_u) * Circle->StartPointPath[j] + tmp_u * Circle->MiddlePointPath[j];
                                                    } else {
                                                        double tmp_u = (u-Circle->uM)/(1-Circle->uM);
                                                        tmpTargetPoint[j] = (1-tmp_u) * Circle->MiddlePointPath[j] + tmp_u * Circle->TargetPointPath[j]; //this also works for tmp_u>1
                                                    }
                                                }
                                            } else {//orientation uses quaternions slerp
                                        
                                                EulerToQuat(Circle->StartPointPath[3],Circle->StartPointPath[4],Circle->StartPointPath[5], &Circle->StartQuat);
                                                EulerToQuat(Circle->MiddlePointPath[3],Circle->MiddlePointPath[4],Circle->MiddlePointPath[5], &Circle->MiddleQuat);
                                                EulerToQuat(Circle->TargetPointPath[3],Circle->TargetPointPath[4],Circle->TargetPointPath[5], &Circle->EndQuat);
                                                Circle->QuatAngle = AngleBetweenQuat(Circle->StartQuat,&Circle->EndQuat);
                                                Circle->QuatAngle1 = AngleBetweenQuat(Circle->StartQuat,&Circle->MiddleQuat);
                                                Circle->QuatAngle2 = AngleBetweenQuat(Circle->MiddleQuat,&Circle->EndQuat);

                                                double tmpA,tmpB,tmpC;
                                                Quat_Type tmpQuat;
                                                if (u <= Circle->uM) {
                                                    double tmp_u = u/Circle->uM;
                                                    Slerp(Circle->StartQuat,Circle->MiddleQuat,&tmpQuat,Circle->QuatAngle1,tmp_u);
                                                    QuatToEuler(tmpQuat,Circle->TargetPointPath[3],Circle->TargetPointPath[4],Circle->TargetPointPath[5],&tmpA,&tmpB,&tmpC);
                                                    tmpTargetPoint[3] = tmpA;
                                                    tmpTargetPoint[4] = tmpB;
                                                    tmpTargetPoint[5] = tmpC;

                                                    tmp_u *= 0.5;
                                                    Slerp(Circle->StartQuat,Circle->MiddleQuat,&tmpQuat,Circle->QuatAngle1,tmp_u);
                                                    QuatToEuler(tmpQuat,Circle->TargetPointPath[3],Circle->TargetPointPath[4],Circle->TargetPointPath[5],&tmpA,&tmpB,&tmpC);
                                                    tmpMiddlePoint[3] = tmpA;
                                                    tmpMiddlePoint[4] = tmpB;
                                                    tmpMiddlePoint[5] = tmpC;
                                                } else {
                                                    double tmp_u = (u-Circle->uM)/(1-Circle->uM);
                                                    Slerp(Circle->MiddleQuat,Circle->EndQuat,&tmpQuat,Circle->QuatAngle2,tmp_u); //Slerp works also with tmp_u>1
                                                    QuatToEuler(tmpQuat,Circle->TargetPointPath[3],Circle->TargetPointPath[4],Circle->TargetPointPath[5],&tmpA,&tmpB,&tmpC);
                                                    tmpTargetPoint[3] = tmpA;
                                                    tmpTargetPoint[4] = tmpB;
                                                    tmpTargetPoint[5] = tmpC;
                                                }
                                            }                                
                                
                                            memcpy(&Circle->TargetPointPath,&tmpTargetPoint,sizeof(Circle->TargetPointPath));
                                            memcpy(&Circle->MiddlePointPath,&tmpMiddlePoint,sizeof(Circle->MiddlePointPath));
                                
                                            //calculate new target point in joint world
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Circle->MiddlePointJoint,Circle->TargetPointPath,Circle->TargetPointJoint);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                
                                            Circle->Length = UserLength;  
                                                                                
                                            if (u <= Circle->uM) {
                                                //calculate new middle point in joint world
                                                Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Circle->StartPointJoint,Circle->MiddlePointPath,Circle->MiddlePointJoint);
                                                if (Trf_Status != STATUS_OK) {
                                                    fRSVG[i].EStop = 1;
                                                    StoppingError[i] = Trf_Status;
                                                    StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                    break;
                                                }
                                                Circle->MiddleLength = Circle->Length * 0.5;
                                            }
                                        }
                                
                                        //note that Circle was a temporary pointer to Buffer[i].MotionPackage[Buffer[i].PP_Index].Path, so all the Buffer values are already updated here!

                                        //path workspace monitoring
                                        tmpViolate = WorkspaceMonitor(MOVE_CIRCLE,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,gRobot[i]->Parameters.Workspace,gRobot[i]->Monitor.AxesNum,0);
                                        if (tmpViolate) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_WORKSPACE_ZONE1 + (tmpViolate-1);
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;                                
                                        }
                                
                                        //check for joint axes limits at middle and end point of circle - only if limits are set (max>min)								
                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                                            if (((Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos) || (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg) ||
                                                (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos) || (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))
                                            &&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_LIMIT_J1 + k;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        }							
                                
                                        //use quaternions for 6ax robots
                                        if (gRobot[i]->Monitor.AxesNum == 6) {//convert Euler angles into quaternions and calculate angle between them
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat);
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddleQuat);
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1 = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddleQuat);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle2 = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddleQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                        } else {  //TODO - this is only valid for 4 axes robots!
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3]) * PI / 180.0 ; //quatang is in rad!
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1 = (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath[3] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3]) * PI / 180.0 ; //quatang is in rad!
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle2 = (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointPath[3]) * PI / 180.0 ; //quatang is in rad!
                                        }

                                        //calculate block length according to selected feedrate configuration
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType == FEED_ANG) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = (fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1) + fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1))  * 180.0 / PI; //blocklength in deg								
																
                                            //fall back on default configuration if angular axes are not programmed
                                            if ((fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1) <= TRF_EPSILON)||(fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle2) <= TRF_EPSILON)) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType = FEED_DEFAULT;
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length;
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.uM = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddleLength / Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length;
                                            } else {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.uM = fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1) / (fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1) + fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle1));
                                            }
                                        } else { //F and FC are equivalent for circle
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length;
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.uM = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddleLength / Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length;
                                        }
								
                                        //make sure that uM is in interval (0..1) - should always be the case anyway...
                                        if ((Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.uM <= 0)||(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.uM >= 1)) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_PP_CIRCLE_LENGTH;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;								
                                        }

                                        //blocklengthideal does not consider round edges and is cartesian only!
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLengthIdeal = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length;
                                
                                        /* calculate START and END direction vectors in path space */

                                        for(j=0;j<3;j++) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.CrossVersor[j];
                                        }
                                                                      
                                        double tmpEndVersor[3];
                                        PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Center,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,tmpEndVersor);
                                        CrossProduct(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Normal,tmpEndVersor,Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector);
                                        Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector);
                                
                                        /* calculate START direction vector in joint space */

                                        // interpolate linearly to find next point (at 0.1% of total block length)
                                        for(j=0;j<3;j++) {//X,Y,Z on circle
                                            //P = C + R cos(t) U + R sin(t) V
                                            tmpAxesValues[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Center[j] + Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius * cos(0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length/Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartVersor[j] + Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius * sin(0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length/Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.CrossVersor[j];
                                        }
                                        for(j=3;j<6;j++) {
                                            tmpAxesValues[j] = 0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j] + 0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j];
                                        }
							
                                        Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,tmpAxesValues,tmpJointsValues);
                                        if (Trf_Status != STATUS_OK) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = Trf_Status;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }							

                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].StartJointVector[k] = tmpJointsValues[k] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint[k];
                                        }


                                        /* calculate END direction vector in joint space */

                                        // interpolate linearly to find next point (at 1% of total block length)
                                        for(j=0;j<3;j++) {//X,Y,Z on circle
                                            //P = C + R cos(t) U + R sin(t) V
                                            tmpAxesValues[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Center[j] + Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius * cos(0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length/Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartVersor[j] + Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius * sin(0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Length/Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.CrossVersor[j];
                                        }
                                        for(j=3;j<6;j++) {
                                            tmpAxesValues[j] = 0.001 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j] + 0.999 * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j];
                                        }
							
                                        Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,tmpAxesValues,tmpJointsValues);
                                        if (Trf_Status != STATUS_OK) {
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = Trf_Status;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }							

                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {	
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].EndJointVector[k] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint[k] - tmpJointsValues[k];
                                        }

                                
                                        /* calculate angle from previous block*/
								
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Round < 0) {// no round edge was defined in previous block
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = AngleBetweenVectors(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartJointVector,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndJointVector);
                                        } else if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Round == 0) {// R0 was programmed in previous block -> non-tangential transition required
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1;
                                        } else {// Rxxx was programmed in previous block -> transition is tangential
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                
                                            //calculate control points
                                            RoundEdgePoints(&Buffer[i].MotionPackage[Buffer[i].PP_Index], &Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev],gRobot[i]->Monitor.AxesNum,&gRobot[i]->Parameters.Mechanics);                                

                                            //modify start path vector so that tangential angle is calculated correctly
                                            double tmpCrossVersor[3];
                                            PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Center,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[4].Axes,tmpCrossVersor);
                                            CrossProduct(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Normal,tmpCrossVersor,Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);
                                            Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);                                

                                            //modify end path vector of previous block if it was a circle so that tangential angle is calculated correctly
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_CIRCLE) {
                                                PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Center,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes,tmpCrossVersor);
                                                CrossProduct(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Normal,tmpCrossVersor,Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector);
                                                Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector);
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.tangAngleStart = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[0]);
                                            }
                                
                                            //modify end edge quaterion of previous block if it was a PTP
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_PTP) {
                                                //calculate quaternion of end edge
                                                EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[3],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[4],Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.CtrlPoint[0].Axes[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.EdgeQuat);
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.EdgeQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndQuat);
                                            }
                                        }
                                                               
                                        //calculate initial and final tangential angle of current movement
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.tangAngleEnd = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[0]);
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndEdge.tangAngleStart = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[0]);
                                        //recall final tangential angle of previous movement
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.tangAngleStart = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.tangAngleStart;
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.EndEdge.tangAngleEnd = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.tangAngleEnd;
                                

                                        /* replan end of previous block if it was a spline */
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_SPLINE) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                
                                            for(j=0;j<3;j++) {
                                                //end tangent of spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j];
    
                                                //third point of spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint[2].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath[j] - Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.LengthB;
                                            }                                                                        
                                        
                                            //spline length
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].BlockLength = BezierLength(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint,BEZIER_XYZ,BEZIER_CUBIC);
                                
                                            //path workspace monitoring of previous spline block
                                            tmpViolate = WorkspaceMonitor(MOVE_SPLINE,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Tool].Axes,gRobot[i]->Parameters.Workspace,gRobot[i]->Monitor.AxesNum,0);
                                            if (tmpViolate) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_WORKSPACE_ZONE1 + (tmpViolate-1);
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].LineNumber;                                
                                            }
                                        }
								
                                        //limit feedrate: time to complete block cannot exceed time needed from joints to reach end position
                                        BlockTime = MinPathTime(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits) + MinPathTime(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.MiddlePointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits);
                                        if (BlockTime != 0) {
                                            double tempSpeed = Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength / BlockTime;
								
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > tempSpeed) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = tempSpeed;
                                            }
                                        }
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > PathLimits.VelocityPos) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = PathLimits.VelocityPos;
                                        }
								
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].PP_Index_Prev = Buffer[i].PP_Index;
                                        Buffer[i].PP_Index++;
					
                                        break;
                                    }

                                case MOVE_SPLINE: {
                                        /* START POINT */
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath));
                                        memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointJoint,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint));

                                        /* END POINT */
                                        if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_JOINTS) {// POINT_JOINTS
                                            memcpy(&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,&gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,sizeof(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint));
                                            // transform target point from joints to path coordinates
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        } else if (gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Mode == POINT_PATH) {// POINT_PATH
                                            // remove tool and frame to find target mountbaseframe
                                            if (gRobot[i]->Monitor.AxesNum < 5) {
                                                SubFrameTool2D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            } else {
                                                SubFrameTool3D(gRobot[i]->Parameters.Points[Buffer[i].MotionPackage[Buffer[i].PP_Index].TargetPoint].Axes,gRobot[i]->Parameters.Frame[Buffer[i].MotionPackage[Buffer[i].PP_Index].Frame].Axes,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index].Tool].Axes,0,0,0,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
                                            }							
                                            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint);
                                        
                                            if (Trf_Status != STATUS_OK) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = Trf_Status;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }

                                        } else {// Point type not supported
                                            fRSVG[i].EStop = 1;
                                            StoppingError[i] = ERR_POINT_TYPE;
                                            StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                            break;
                                        }
                                
                                        //no path workspace monitoring here yet, because spline path will only be defined in next movement 
                                
                                        //check for joint axes limits at end point - only if these are set (max>min)								
                                        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {
                                            if (((Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg))&&(gRobot[i]->Parameters.JointLimits[k].PositionPos > gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_LIMIT_J1 + k;
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                                break;
                                            }
                                        }							

                                        //use quaternions for 6ax robots
                                        if (gRobot[i]->Monitor.AxesNum == 6) {//convert Euler angles into quaternions and calculate angle between them (in radians)
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat);
                                            EulerToQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[4],Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[5], &Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = AngleBetweenQuat(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartQuat,&Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndQuat);
                                        } else {  //TODO - this is only valid for 4 axes robots!
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle = (Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[3] - Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[3]) * PI / 180.0 ; //quatang is in rad!
                                        }
								
                                        //calculate block length according to selected feedrate configuration
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType == FEED_CART) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLengthCart(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath);
									
                                            //fall back on default configuration if angular axes are not programmed
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength < TRF_EPSILON) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,gRobot[i]->Monitor.AxesNum);
                                            }
                                        } else if (Buffer[i].MotionPackage[Buffer[i].PP_Index].FeedrateType == FEED_ANG){
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.QuatAngle) * 180.0 / PI; //blocklength in deg								

                                            //fall back on default configuration if angular axes are not programmed
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength < TRF_EPSILON) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,gRobot[i]->Monitor.AxesNum);
                                            }
                                        } else {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath,gRobot[i]->Monitor.AxesNum);								
                                        }
								
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLengthIdeal = Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength; //blocklengthideal does not consider round edges
							
                                        /* set default values for limit edge points -> used in EXEC even if no round edge is programmed */
                                        //TODO - do we need this for spline?!?!
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartEdge.CtrlPoint[4].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j];
                                        }
                                        for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.EndEdge.CtrlPoint[0].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j];
                                        }

                                        // we do not calculate start and end direction vectors here
                                        // splines are by definition always tangential to previous and next movement (although in the path world!)
                                        // in the joint world the tangentiality should be conserved in a neighborhood of the transition point
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                
                                        /* calculate length and control points of spline */
                                
                                        //the start and end tangent length are currently set equal to each other, but this might change in the future
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.LengthA = Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLengthIdeal * BEZIER_SEGMENT;
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.LengthB = Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLengthIdeal * BEZIER_SEGMENT;

                                        //first point is start point
                                        for(j=0;j<3;j++) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint[0].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j];
                                        }
                                
                                        //fourth point is target point
                                        for(j=0;j<3;j++) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint[3].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j];
                                        }
                                
                                        //second point is calculated from path tangent of previous movement (if exists) and start tangent length
                                        if (VectorLength(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector) > TRF_EPSILON) {
                                            //previous movement has tangent -> spline should follow it
                                            for(j=0;j<3;j++) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j];
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint[1].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j] + Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.LengthA;
                                            }                                                                        
                                        } else {//previous movement has no tangent -> spline starts straight towards end point
                                            PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);
                                            Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);
                                            for(j=0;j<3;j++) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint[1].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j] + Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.LengthA;
                                            }                                    
                                        }
                                                              
                                        //third point cannot be calculated now, it depends on next movement and will be calculated there
                                        //however, we set it temporarily straight towards target point
                                        PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector);
                                        Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector);
                                        for(j=0;j<3;j++) {                                
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint[2].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath[j] - Buffer[i].MotionPackage[Buffer[i].PP_Index].EndPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.LengthB;
                                        }
                                
                                        //temporary block length -> might be adjusted in next block
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = BezierLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint,BEZIER_XYZ,BEZIER_CUBIC);
                                
 
                                        /* replan end of previous block (and beginning of current movement) if it was a spline */
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].MovementType == MOVE_SPLINE) {
                                            PointsToVector(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.StartPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointPath, Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);
                                            Normalize(Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector);
                                    
                                            for(j=0;j<3;j++) {
                                                //second point of current spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint[1].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointPath[j] + Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.LengthA;
                                                //third point of previous spline
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index].StartPathVector[j];
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint[2].Axes[j] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath[j] - Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].EndPathVector[j] * Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.LengthB;
                                            }                                                                        
                                        
                                            //update spline lengths
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength = BezierLength(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.Spline.CtrlPoint,BEZIER_XYZ,BEZIER_CUBIC);
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].BlockLength = BezierLength(Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.Spline.CtrlPoint,BEZIER_XYZ,BEZIER_CUBIC);
                                
                                            //path workspace monitoring of previous spline block
                                            tmpViolate = WorkspaceMonitor(MOVE_SPLINE,&Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path,gRobot[i]->Parameters.Tool[Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Tool].Axes,gRobot[i]->Parameters.Workspace,gRobot[i]->Monitor.AxesNum,0);
                                            if (tmpViolate) {
                                                fRSVG[i].EStop = 1;
                                                StoppingError[i] = ERR_WORKSPACE_ZONE1 + (tmpViolate-1);
                                                StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].LineNumber;                                
                                            }
                                        }
																
                                        //limit feedrate: time to complete block cannot exceed time needed from joints to reach end position
                                        BlockTime = MinPathTime(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits);
                                        if (BlockTime != 0) {
                                            double tempSpeed = Buffer[i].MotionPackage[Buffer[i].PP_Index].BlockLength / BlockTime;
								
                                            if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > tempSpeed) {
                                                Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = tempSpeed;
                                            }
                                        }
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate > PathLimits.VelocityPos) {
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].Feedrate = PathLimits.VelocityPos;
                                        }
								
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].PP_Index_Prev = Buffer[i].PP_Index;
                                        Buffer[i].PP_Index++;
					
                                        break;
                                    }
                                
                                case MOVE_TRK: {                                
                                        //halt PP until TRK is reset by EXEC
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1;	
                                        Buffer[i].Planned = 1; //let the EXEC start working if not moving already
                                        break;
                                    }

                                case MOVE_SETDO: {
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                        Buffer[i].PP_Index++; //only continue to next block if all M-functions are non-synch												
                                        break;
                                    }

                                case MOVE_RESETDO: {
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                        Buffer[i].PP_Index++; //only continue to next block if all M-functions are non-synch												
                                        break;
                                    }

                                case MOVE_MCODE: {
                                        M_count = 0;
                                        M_value = Buffer[i].MotionPackage[Buffer[i].PP_Index].M_Index[M_count];
                                        M_synch = 0; //flag to check if any of the called M funcs are synch
                                        while((M_value > 0)&&(M_count < MAX_MFUNC_INLINE)) {
                                            if (gRobot[i]->Parameters.M_synch[M_value]) {/* synch M func found */
                                                M_synch = 1;
                                            }
                                            M_count++;
                                            M_value = Buffer[i].MotionPackage[Buffer[i].PP_Index].M_Index[M_count];
                                        }
				
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;

                                        if (M_synch) { // some of the M funcs are synch - mark movement type as Synch and make transition non-tangential (speed dip)
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].MovementType = MOVE_MCODE_SYNCH; //halt PP until reset by EXEC						
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1;	
                                            Buffer[i].Planned = 1; //let the EXEC start working if not moving already
                                        } else { // all M funcs are non-synch, movement is tangential (no speed dip)								
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                            //Buffer[i].PP_Index_Prev = Buffer[i].PP_Index; //do not update prev index here so M func does not interrupt geometry
                                            Buffer[i].PP_Index++; //only continue to next block if all M-functions are non-synch
                                        }
												
                                        break;
                                    }

                                case MOVE_MCODE_SYNCH: {
                                        //halt PP until M function is reset by EXEC
                                        break;
                                    }
				
                                case MOVE_WAITDI: {
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1; //non-tangential transition
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        //Buffer[i].PP_Index_Prev = Buffer[i].PP_Index; //do not update prev index here so WaitDI does not interrupt geometry
                                        Buffer[i].PP_Index++;
                                        break;
                                    }
				
                                case MOVE_DELAY: {
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1; //non-tangential transition
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        //Buffer[i].PP_Index_Prev = Buffer[i].PP_Index; //do not update prev index here so Delay does not interrupt geometry
                                        Buffer[i].PP_Index++;
                                        break;
                                    }

                                case MOVE_TANG: {
                                        //next movement has not been planned already, so we cannot know next starting tangent here
                                        //TODO - introduce PP_Index_Next ?!?
                                        //double oldTangAxis = Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.TargetPointPath[gRobot[i]->Monitor.AxesNum-Buffer[i].MotionPackage[Buffer[i].PP_Index].TangCmd]; //TangCmd = 1,2,3 (tangent around Z,Y,X)
                                        //double newPathTangent = atan2d(Buffer[i].MotionPackage[Buffer[i].PP_Index_Next].StartPathVector[1],Buffer[i].MotionPackage[Buffer[i].PP_Index_Next].StartPathVector[0]);                                    
                                        if (Buffer[i].MotionPackage[Buffer[i].PP_Index].TangCmd &&
                                            //Buffer[i].MotionPackage[Buffer[i].PP_Index_Next].MovementType != MOVE_PTP &&
                                            (!Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].TangCmd || //&& fabs(newPathTangent-oldTangAxis)>TRF_EPSILON) ||
                                            (Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].TangCmd && fabs(Buffer[i].MotionPackage[Buffer[i].PP_Index].Path.RotAngle-Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].Path.RotAngle)>TRF_EPSILON))
                                            )
                                        {//either just started new tangential section (with new path tangent not equal to the current C angle), or modified offset angle 
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1; //non-tangential transition
                                        }
                                        else
                                        {// either tang off or useless tang on
                                            //TODO axes will jump after TANG 0 -> take care of that here or let the OptMot do the job?!?
                                            Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = 0;
                                        }
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        //Buffer[i].PP_Index_Prev = Buffer[i].PP_Index; //do not update prev index here so Tang does not interrupt geometry (e.g. for splines after TANG 0)
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index_Prev].TangCmd = Buffer[i].MotionPackage[Buffer[i].PP_Index].TangCmd; //needs this to compensate for previous line
                                        Buffer[i].PP_Index++;
                                        break;
                                    }

                                case MOVE_END: {
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].TransitionAngle = -1; //non-tangential transition
                                        Buffer[i].MotionPackage[Buffer[i].PP_Index].Planned = 1;
                                        Buffer[i].Planned = 1;
                                        //end of program - do nothing
                                        break;
                                    }

                                default: {
                                        fRSVG[i].EStop = 1;
                                        StoppingError[i] = ERR_NOT_SUPPORTED;
                                        StoppingLine[i] = Buffer[i].MotionPackage[Buffer[i].PP_Index].LineNumber;
                                        break;
                                    }
                            }
                        } else {
                            Buffer[i].Planned = 1; //note that this is never reset during active program because every packet consumed by EXEC will immediately be replaced by IP and PP (except for synch M-func)
                        }
                    }

                    /*** --- EXEC --- ***/
                    /*** execute block ***/
                    if (Buffer[i].EXEC_Index >= BUFFER_LENGTH) {
                        Buffer[i].EXEC_Index = 0;	
                    }
                    strcpy(gRobot[i]->Monitor.CurrentBlock,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockString);
                    gRobot[i]->Monitor.LineNumber = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber;
                    gRobot[i]->Monitor.TargetPoint = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].TargetPoint;

                    if (Buffer[i].Planned) {// only start moving when lookahead buffer is full (which can also happen if PP has reached Eof)
					
                        unsigned short ContinueExec = 0; //this flag is needed to execute the next block immediately in case of: (1) non-synch M-func; (2) jumps to Subroutines; (3) movements with zero length. The loop avoids wasting a cycle time and causing speed dips.
                        unsigned short EXEC_LoopCount = 0;
                        do {
                            ContinueExec = 0;
                            EXEC_LoopCount++;//avoid endless loop
                            if (Buffer[i].EXEC_Index >= BUFFER_LENGTH) {
                                Buffer[i].EXEC_Index = 0;	
                            }
                            
                            switch (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].MovementType) {
                                case MOVE_UNDEF: {
                                        // nothing programmed in this block, either program is completed (quit) or interpreter has not written anything yet (wait here)
                                        if (Buffer[i].Eof) {
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;
                                        break;
                                    }
				
                                case MOVE_LINE: {
                                        if (!fRSVG[i].Enable) { //svg not active yet -> movement not started
                                        
                                            gRobot[i]->Monitor.BlockLength = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
								
                                            gRobot[i]->Monitor.Tool = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Tool;
                                            gRobot[i]->Monitor.Frame = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Frame;
							
                                            /* do not start movement if block length is zero */
                                            if(gRobot[i]->Monitor.BlockLength < TRF_EPSILON) {
                                                gRobot[i]->Monitor.CompletedBlockLength = 0;
                                                fRSVG[i].Enable = 0;
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                double tmpEndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;
                                                double tmpEndAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acc for next block to read
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK)
                                                    ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                                break;
                                            }
							
                                            memcpy(&fRSVG[i].DynamicLimits,&PathLimits,sizeof(fRSVG[i].DynamicLimits));
                                            memcpy(&fRSVG[i].DynamicValues,&PathLimits,sizeof(fRSVG[i].DynamicValues));

                                            fRSVG[i].DynamicValues.VelocityPos = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate;
									
                                            //calculate end speed of current block based on future path
                                            unsigned short lookAhead = 1;
                                            unsigned char endBuffer = 0;
                                            double v_max;
                                            do {//look for end of tangential path
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH)
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                if ((Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle < 0)||(Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle > gRobot[i]->Parameters.MaxTransitionAngle)) {
                                                    //non-tangential transition found ---> stop here
                                                    endBuffer = 1;
                                                    lookAhead--; //do not consider this block!
                                                }	
                                                lookAhead++;
                                            } while ((lookAhead < BUFFER_LENGTH)&&(endBuffer == 0));

                                            double EndSpeed = 0;
                                            double TotalBlockLength = 0;
                                            double SameFeedBlockLength = 0;
                                            double SameFeedEndSpeed = 0;
                                            while (lookAhead > 1) { //go backwards from here and find max end speed of current block
                                                lookAhead--;
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH) {
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                }
                                                if (Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_MCODE || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_SETDO || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_RESETDO) {
                                                    continue; //non-synch M-functions do not affect path speed
                                                }
                                                if (!MaxBlockSpeed(Buffer[i].MotionPackage[lookAheadIndex].BlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,EndSpeed,&v_max)) { //AccPos and JerkPos are used here because SVG over a block always moves in positive direction
                                                    TotalBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    //EndSpeed = min(v_max,Buffer[i].MotionPackage[lookAheadIndex].Feedrate); //this should not be used here because it reduces the start speed too much
                                                    EndSpeed = Buffer[i].MotionPackage[lookAheadIndex].Feedrate;
                                                    if (Buffer[i].MotionPackage[lookAheadIndex].Feedrate >= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate) {
                                                        SameFeedBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    } else {
                                                        SameFeedBlockLength = 0;
                                                        SameFeedEndSpeed = EndSpeed;
                                                    }
                                                } else {
                                                    EndSpeed = 0;
                                                    TotalBlockLength = 0;
                                                    SameFeedBlockLength = 0;
                                                    SameFeedEndSpeed = 0;
                                                }										
                                            }

                                            TotalBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
                                            SameFeedBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
											
                                            //end speed is zero if running in single step mode
                                            if (gRobot[i]->Parameters.SingleStep) {
                                                EndSpeed = 0;
                                            } else {										
                                                //end speed cannot be higher than feedrate!
                                                EndSpeed = min(EndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate);

                                                //end speed cannot be higher than what current dynamics allow until end of tangential path
                                                double tmpV, tmpA0, tmpA1,tmpDelta;
                                                double tmpMaxDyn = MaxMovementDynamics(TotalBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,0,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
										
                                                //however, if feedrate is not constant along path, we should only consider path until current feedrate drops
                                                tmpMaxDyn = MaxMovementDynamics(SameFeedBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,SameFeedEndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
                                            }
									
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = EndSpeed;
									
                                            fRSVG[i].StartPosition = 0;
                                            fRSVG[i].TargetPosition = gRobot[i]->Monitor.BlockLength;
                                            fRSVG[i].StartSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;		//start with the end speed of previous block (was adjusted at exec time)
                                            fRSVG[i].StartAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;		//start with the end acceleration of previous block
                                            if (!StoppingError[i]) {
                                                fRSVG[i].EndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;			 // note that this end speed is not necessarily reached (will be adjusted at end of movement)		
                                                fRSVG[i].Start = 1;
                                            } else if (StoppingError[i]<0) { //movement is already stopping -> reduce end speed
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].Stop = 1;
                                            } else { // e-stopping because of active error -> continue e-stop
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].EStop = 1;										
                                            }
                                            fRSVG[i].Enable = 1;
                                            OldSVGPos[i] = 0;
                                            fRSVG[i].Position = 0;
                                        } else {//movement started -> wait for movement to complete
							
                                            if (fRSVG[i].Status >= ERR_SPG) { // error in SVG call
                                                gRobot[i]->Monitor.ActiveError = fRSVG[i].Status;
                                                gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                            }
                                    
                                            // OptMot:
                                            // try to execute movement at given speed
                                            // check if joint dynamic limits are violated
                                            // if it fails then execute it again at lower speed
                                            unsigned short MovementAllowed = 1;
                                            memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                                            double tangAngle;
                                    
                                            //save old X,Y for tangential angle calculation
                                            double oldX = gRobot[i]->Monitor.MountBasePosition[0];
                                            double oldY = gRobot[i]->Monitor.MountBasePosition[1];

                                            do {
                                                                                
                                                if (MovementAllowed) {//first time we attempt movement - try full speed
                                                    gRobot[i]->Monitor.CompletedBlockLength = fRSVG[i].Position;
                                                } else {//previous movement attempt failed - repeat with new target position (calculated at limited speed)
                                                    gRobot[i]->Monitor.CompletedBlockLength = OldSVGPos[i] + (fRSVG[i].Position - OldSVGPos[i]) * (1 - RedFactor[i]);
                                                    MovementAllowed = 255; // do not repeat test again
                                                }
                                        
                                                //protect from small overshoots
                                                if (gRobot[i]->Monitor.CompletedBlockLength > gRobot[i]->Monitor.BlockLength) {
                                                    gRobot[i]->Monitor.CompletedBlockLength = gRobot[i]->Monitor.BlockLength;
                                                }
                                                
                                                double u = gRobot[i]->Monitor.CompletedBlockLength/gRobot[i]->Monitor.BlockLength;

                                                double u_start = 0;
                                                double u_end = 0;
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal > 0) {
                                                    u_start = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.Radius/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal;
                                                    u_end = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.Radius/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal;
                                                }

                                                //interpolate POSITION
                                                unsigned short phase; //used for debugging, not really needed otherwise
                                                Point_Type tmpPoint;
                                                if (u < u_start) {//start edge -> interpolate Bezier with u = 0.5 ... 1
                                                    phase = PHASE_START;
                                            
                                                    double u_tmp = 0.5 + u / u_start * 0.5;
                                                    EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint,u_tmp,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                    for(j=0;j<BEZIER_XYZ;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = tmpPoint.Axes[j];
                                                    }

                                                    //tangential auto mode: force C axis to follow path tangent
                                                    if(gRobot[i]->Monitor.TangActive) {
                                                        //calculate tangent angle on XY plane
                                                        double tang_u = u_tmp + (1.0/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.Length/10.0); //move on spline by 0.1mm (note that increment is purely geometrical, independent of path speed)
                                                        if (tang_u<1) {
                                                            EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint,tang_u,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                            tangAngle = atan2d(tmpPoint.Axes[1]-oldY,tmpPoint.Axes[0]-oldX);
                                                        } else {//spline close to end - use tangent of next block
                                                            tangAngle = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.tangAngleEnd;
                                                        }
                                                    }
                                                } else if ((1-u) < u_end) {//end edge -> interpolate Bezier with u = 0 ... 0.5
                                                    phase = PHASE_END;	
                                                    double alpha = 1 - u_end;

                                                    //interpolate POSITION
                                                    double u_tmp = (u-alpha)/u_end * 0.5;
                                                    EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint,u_tmp,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                    for(j=0;j<BEZIER_XYZ;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = tmpPoint.Axes[j];
                                                    }

                                                    //tangential auto mode: force C axis to follow path tangent
                                                    if(gRobot[i]->Monitor.TangActive) {
                                                        //calculate tangent angle on XY plane
                                                        double tang_u = u_tmp + (1.0/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.Length/10.0); //move on spline by 0.1mm (note that increment is purely geometrical, independent of path speed)
                                                        if (tang_u<1) {
                                                            EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint,tang_u,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                            tangAngle = atan2d(tmpPoint.Axes[1]-oldY,tmpPoint.Axes[0]-oldX);
                                                        } else {//spline close to end - use tangent of next block
                                                            tangAngle = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.tangAngleEnd;
                                                        }
                                                    }
                                                } else {//middle block -> interpolate linearly with u = 0 ... 1
                                                    phase = PHASE_MIDDLE;	
                                                    double u_tmp = (u-u_start) / (1-u_end-u_start);
                                                    for(j=0;j<BEZIER_XYZ;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = (1-u_tmp) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint[4].Axes[j] + u_tmp * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint[0].Axes[j];
                                                    }
                                                    tangAngle = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.tangAngleEnd;
                                                }
                                        
                                                //interpolate ORIENTATION
                                                if (gRobot[i]->Monitor.AxesNum != 6) {//orientation interpolated linearly
                                                    for(j=3;j<gRobot[i]->Monitor.AxesNum;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = (1-u) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointPath[j] + u * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.TargetPointPath[j];
                                                    }
                                                } else {//orientation uses quaternions slerp       
                                                    double tmpA,tmpB,tmpC;
                                                    Quat_Type tmpQuat;
                                                    Slerp(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndQuat,&tmpQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.QuatAngle,u);
                                                    //QuatToEuler(tmpQuat,gRobot[i]->Monitor.MountBasePosition[3],gRobot[i]->Monitor.MountBasePosition[4],gRobot[i]->Monitor.MountBasePosition[5],&tmpA,&tmpB,&tmpC);
                                                    QuatToEuler(tmpQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointPath[3],Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointPath[4],Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointPath[5],&tmpA,&tmpB,&tmpC);
                                                    gRobot[i]->Monitor.MountBasePosition[3] = tmpA;
                                                    gRobot[i]->Monitor.MountBasePosition[4] = tmpB;
                                                    gRobot[i]->Monitor.MountBasePosition[5] = tmpC;         
                                                }
									                                        
                                                //tangential auto mode: force C axis to follow path tangent
                                                if(gRobot[i]->Monitor.TangActive) {
                                                    //TODO - this assumes 4 or 6 axis robot                                            
                                                    gRobot[i]->Monitor.MountBasePosition[gRobot[i]->Monitor.AxesNum-1] = tangAngle + gRobot[i]->Monitor.TangOffset;
                                                }
                                        
                                                //memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                                                AxesMoved = 1; //movement active -> add whole conveyor offset when tracking
                                                Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
                                                if (Trf_Status != STATUS_OK) {
                                                    gRobot[i]->Monitor.ActiveError = Trf_Status;
                                                    gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                    //resume old joints positions to prevent jumps
                                                    memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                }							
                                        
                                                //reduce path speed if speed of any joint axis is exceeded
                                                // MovementAllowed is set to 255 after test already failed once - no need to repeat again
                                                if (MovementAllowed != 255) {
                                                    double tmpRedFactor;
                                                    unsigned short BadAxes = DynamicLimitsViolated(OldAxesValues,gRobot[i]->Monitor.JointPosition,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits,TaskCycleTime,&tmpRedFactor);
                                            
                                                    if (BadAxes) {
                                                        RedFactor[i] = tmpRedFactor;
                                                
                                                        if (RedFactor[i] > 1) {//should never happen!
                                                            RedFactor[i] = 1;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                            
                                                        if (RedFactor[i] < 0) {//should never happen!
                                                            RedFactor[i] = 0;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                            
                                                        //repeat movement with slower speed
                                                        MovementAllowed = 0;
                                                        memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                    }
                                            
                                                } else {
                                                    for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) { //manually truncate joint movements -> WARNING! TCP will leave programmed path!
                                                
                                                        double JointSpeed = (gRobot[i]->Monitor.JointPosition[j]-OldAxesValues[j]) / TaskCycleTime; //do not scale cycle time here!
                                                        if (JointSpeed != 0) {                                                    
                                                            if ((JointSpeed > 0)&&(gRobot[i]->Parameters.JointLimits[j].VelocityPos / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] + gRobot[i]->Parameters.JointLimits[j].VelocityPos * TaskCycleTime;
                                                            }
                                                            if ((JointSpeed < 0)&&(-gRobot[i]->Parameters.JointLimits[j].VelocityNeg / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] - gRobot[i]->Parameters.JointLimits[j].VelocityNeg * TaskCycleTime;
                                                            }
                                                        }
                                                    }
                                                }
                                            } while (MovementAllowed == 0);

                                            if (fRSVG[i].Done) { // movement completed
                                                if ((StoppingError[i] == 0)&&(gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                                    //single step mode -> wait for continue command
                                                    gRobot[i]->Monitor.Halted = 1;
                                                    if (gRobot[i]->Commands.Continue) {
                                                        //TODO -> what if single step is activated during movement
                                                        gRobot[i]->Commands.Continue = 0;
                                                        gRobot[i]->Monitor.Halted = 0;
                                                        fRSVG[i].Enable = 0;
                                                        if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                            strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                        }
                                                        memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                        Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                        Buffer[i].EXEC_Index++;
                                                    }
                                                } else if ((StoppingError[i] == 0)||(fRSVG[i].Status == STATUS_ABORT)) {
                                                    //movement was completed correctly or aborted but non completed yet -> move on to next block
                                                    fRSVG[i].Enable = 0;
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                    }
                                                    double tmpEndSpeed = fRSVG[i].Speed / fRSVG[i].Override; //Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;
                                                    double tmpEndAcc = fRSVG[i].Acceleration; //SVG output acceleration not affected by override
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acceleration for next block to read
                                                    if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                        ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                                    }
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                } else if (StoppingError[i] > 0) {//movement stopped because of error
                                                    gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                                    gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                                    gRobot[i]->Monitor.State = ERROR_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                } else if (StoppingError[i] < 0) { //movement stopped from command
                                                    gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                }
                                            }														
                                        }
                                        break;
                                    }
				
                                case MOVE_PTP: {
                                        if (!fRSVG[i].Enable) { //svg not active yet -> movement not started
                                            //recalculate starting point and blocklength if previous position was not reached correctly because of interpolated movement programmed in joint world
                                            //exclude case of round edge (because target point is never reached)
                                            if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.Length <= 0) {
                                                memcpy(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointJoint,&gRobot[i]->Monitor.JointPosition,sizeof(gRobot[i]->Monitor.JointPosition));
                                                memcpy(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint[6].Axes,&gRobot[i]->Monitor.JointPosition,sizeof(gRobot[i]->Monitor.JointPosition));
                                        
                                                //calculate block length according to selected feedrate configuration
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].FeedrateType == FEED_CART) {
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength = PTPLength(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path, gRobot[i]->Monitor.AxesNum, &gRobot[i]->Parameters.Mechanics);
					
                                                    //fall back on default configuration if there is no cartesian displacement
                                                    if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength < TRF_EPSILON) {
                                                        Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum);
                                                    }
                                                } else if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].FeedrateType == FEED_ANG) {
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength = fabs(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.QuatAngle) * 180.0 / PI; //blocklength in deg								

                                                    //fall back on default configuration if there is no angular displacement
                                                    if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength < TRF_EPSILON) {
                                                        Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum);
                                                    }
                                                } else {
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength = LineLength(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointJoint,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.TargetPointJoint,gRobot[i]->Monitor.AxesNum);
                                                }
                                            }
                                    
                                            gRobot[i]->Monitor.BlockLength = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
								
                                            gRobot[i]->Monitor.Tool = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Tool;
                                            gRobot[i]->Monitor.Frame = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Frame;

                                            /* do not start movement if block length is zero */
                                            if(gRobot[i]->Monitor.BlockLength < TRF_EPSILON) {
                                                gRobot[i]->Monitor.CompletedBlockLength = 0;
                                                fRSVG[i].Enable = 0;
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                double tmpEndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;
                                                double tmpEndAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acc for next block to read
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                    ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                                }
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                                break;
                                            }

                                            memcpy(&fRSVG[i].DynamicLimits,&PathLimits,sizeof(fRSVG[i].DynamicLimits));
                                            memcpy(&fRSVG[i].DynamicValues,&PathLimits,sizeof(fRSVG[i].DynamicValues));

                                            fRSVG[i].DynamicValues.VelocityPos = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate;

                                            //calculate end speed of current block based on future path
                                            unsigned short lookAhead = 1;
                                            unsigned char endBuffer = 0;
                                            double v_max;
                                            do {//look for end of tangential path
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH) {
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                }
                                                if ((Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle < 0)||(Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle > gRobot[i]->Parameters.MaxTransitionAngle)) {//non-tangential transition found ---> stop here
                                                    endBuffer = 1;
                                                    lookAhead--; //do not consider this block!
                                                }	
                                                lookAhead++;
                                            } while ((lookAhead < BUFFER_LENGTH)&&(endBuffer == 0));

                                            double EndSpeed = 0;
                                            double TotalBlockLength = 0;
                                            double SameFeedBlockLength = 0;
                                            double SameFeedEndSpeed = 0;
                                            while (lookAhead > 1) { //go backwards from here and find max end speed of current block
                                                lookAhead--;
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH) {
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                }
                                                if (Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_MCODE || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_SETDO || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_RESETDO) {
                                                    continue; //non-synch M-functions do not affect path speed
                                                }
                                                if (!MaxBlockSpeed(Buffer[i].MotionPackage[lookAheadIndex].BlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,EndSpeed,&v_max)) { //AccPos and JerkPos are used here because SVG over a block always moves in positive direction
                                                    TotalBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    //EndSpeed = min(v_max,Buffer[i].MotionPackage[lookAheadIndex].Feedrate); //this should not be used here because it reduces the start speed too much
                                                    EndSpeed = Buffer[i].MotionPackage[lookAheadIndex].Feedrate;
                                                    if (Buffer[i].MotionPackage[lookAheadIndex].Feedrate >= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate) {
                                                        SameFeedBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    } else {
                                                        SameFeedBlockLength = 0;
                                                        SameFeedEndSpeed = EndSpeed;
                                                    }
                                                } else {
                                                    EndSpeed = 0;
                                                    TotalBlockLength = 0;
                                                    SameFeedBlockLength = 0;
                                                    SameFeedEndSpeed = 0;
                                                }										
                                            }

                                            TotalBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
                                            SameFeedBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
													
                                            //end speed is zero if running in single step mode
                                            if (gRobot[i]->Parameters.SingleStep) {
                                                EndSpeed = 0;
                                            } else {										
                                                //end speed cannot be higher than feedrate!
                                                EndSpeed = min(EndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate);

                                                //end speed cannot be higher than what current dynamics allow until end of tangential path
                                                double tmpV, tmpA0, tmpA1,tmpDelta;
                                                double tmpMaxDyn = MaxMovementDynamics(TotalBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,0,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
										
                                                //however, if feedrate is not constant along path, we should only consider path until current feedrate drops
                                                tmpMaxDyn = MaxMovementDynamics(SameFeedBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,SameFeedEndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
                                            }
									
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = EndSpeed;

                                            fRSVG[i].StartPosition = 0;
                                            fRSVG[i].TargetPosition = gRobot[i]->Monitor.BlockLength;
                                            fRSVG[i].StartSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;		//start with the end speed of previous block (was adjusted at exec time)
                                            fRSVG[i].StartAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;		//start with the end acceleration of previous block
                                            if (!StoppingError[i]) {
                                                fRSVG[i].EndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;					
                                                fRSVG[i].Start = 1;
                                            } else if (StoppingError[i]<0) { //movement is already stopping -> reduce end speed	
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].Stop = 1;
                                            } else { // e-stopping because of active error -> continue e-stop
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].EStop = 1;										
                                            }
                                            fRSVG[i].Enable = 1;							
                                            OldSVGPos[i] = 0;
                                            fRSVG[i].Position = 0;
                                        } else {//movement started -> wait for movement to complete
												
                                            if (fRSVG[i].Status >= ERR_SPG) { // error in SVG call
                                                gRobot[i]->Monitor.ActiveError = fRSVG[i].Status;
                                                gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                            }
							
                                            // OptMot:
                                            // try to execute movement at given speed
                                            // check if joint dynamic limits are violated
                                            // if it fails then execute it again at lower speed
                                            // NOTE: PTP movements can violate joint speed limits only in extreme cases
                                            // for example when the planned position was not reached correctly
                                            // after an ML P0 move where P0 was programmed in joint world
                                            unsigned short MovementAllowed = 1;

                                            memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                                            AxesMoved = 1; //movement active -> add whole conveyor offset when tracking									

                                            do {

                                                if (MovementAllowed) {//first time we attempt movement - try full speed
                                                    gRobot[i]->Monitor.CompletedBlockLength = fRSVG[i].Position;
                                                } else {//previous movement attempt failed - repeat with new target position (calculated at limited speed)
                                                    gRobot[i]->Monitor.CompletedBlockLength = OldSVGPos[i] + (fRSVG[i].Position - OldSVGPos[i]) * (1 - RedFactor[i]);
                                                    MovementAllowed = 255; // do not repeat test again
                                                }

                                                //protect from small overshoots
                                                if (gRobot[i]->Monitor.CompletedBlockLength > gRobot[i]->Monitor.BlockLength) {
                                                    gRobot[i]->Monitor.CompletedBlockLength = gRobot[i]->Monitor.BlockLength;
                                                }
							
                                                double u = gRobot[i]->Monitor.CompletedBlockLength/gRobot[i]->Monitor.BlockLength;
                                        
                                                double u_start = 0;
                                                double u_end = 0;
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal > 0) {
                                                    u_start = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.Radius/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal;
                                                    u_end = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.Radius/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal;
                                                }
                                        
                                                //check what phase of the block is currently being executed
                                                unsigned short phase;
                                                Point_Type tmpPoint;
                                                if (u < u_start) {//start edge -> interpolate Bezier with u = 0.5 ... 1
                                                    phase = PHASE_START;
                                            
                                                    //interpolate POSITION
                                                    double u_tmp = 0.5 + u / u_start * 0.5;
                                                    EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint,u_tmp,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                    for(j=0;j<BEZIER_XYZ;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = tmpPoint.Axes[j];
                                                    }	

                                                    //interpolate ORIENTATION
                                                    u_tmp = u / u_start;
                                                    if (gRobot[i]->Monitor.AxesNum != 6) {//orientation interpolated linearly
                                                        for(j=3;j<gRobot[i]->Monitor.AxesNum;j++) {
                                                            gRobot[i]->Monitor.MountBasePosition[j] = (1-u_tmp) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointPath[j] + u_tmp * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint[4].Axes[j];
                                                        }
                                                    } else {//orientation uses quaternions slerp       
                                                        double tmpA,tmpB,tmpC;
                                                        Quat_Type tmpQuat;
                                                        Slerp(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.EdgeQuat,&tmpQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.QuatAngle,u_tmp);
                                                        QuatToEuler(tmpQuat,gRobot[i]->Monitor.MountBasePosition[3],gRobot[i]->Monitor.MountBasePosition[4],gRobot[i]->Monitor.MountBasePosition[5],&tmpA,&tmpB,&tmpC);
                                                        gRobot[i]->Monitor.MountBasePosition[3] = tmpA;
                                                        gRobot[i]->Monitor.MountBasePosition[4] = tmpB;
                                                        gRobot[i]->Monitor.MountBasePosition[5] = tmpC;
                                                    }

                                                    Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
                                                    if (Trf_Status != STATUS_OK) {
                                                        gRobot[i]->Monitor.ActiveError = Trf_Status;
                                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        //resume old joints positions to prevent jumps
                                                        memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                    }							
                                                } else if ((1-u) < u_end) {//end edge -> interpolate Bezier with u = 0 ... 0.5
                                                    phase = PHASE_END;
                                                    double alpha = 1 - u_end;

                                                    //interpolate POSITION
                                                    double u_tmp = (u-alpha)/u_end * 0.5;
                                                    EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint,u_tmp,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                    for(j=0;j<BEZIER_XYZ;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = tmpPoint.Axes[j];
                                                    }
                                                                
                                                    //interpolate ORIENTATION
                                                    u_tmp = (u-alpha) / u_end;
                                                    if (gRobot[i]->Monitor.AxesNum != 6) {//orientation interpolated linearly
                                                        for(j=3;j<gRobot[i]->Monitor.AxesNum;j++) {
                                                            gRobot[i]->Monitor.MountBasePosition[j] = (1-u_tmp) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint[0].Axes[j] + u_tmp * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.TargetPointPath[j];
                                                        }
                                                    } else {//orientation uses quaternions slerp                                            
                                                        double tmpA,tmpB,tmpC;
                                                        Quat_Type tmpQuat;
                                                        Slerp(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.EdgeQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndQuat,&tmpQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.QuatAngle,u_tmp);
                                                        QuatToEuler(tmpQuat,gRobot[i]->Monitor.MountBasePosition[3],gRobot[i]->Monitor.MountBasePosition[4],gRobot[i]->Monitor.MountBasePosition[5],&tmpA,&tmpB,&tmpC);
                                                        gRobot[i]->Monitor.MountBasePosition[3] = tmpA;
                                                        gRobot[i]->Monitor.MountBasePosition[4] = tmpB;
                                                        gRobot[i]->Monitor.MountBasePosition[5] = tmpC;
                                                    }

                                                    Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
                                                    if (Trf_Status != STATUS_OK) {
                                                        gRobot[i]->Monitor.ActiveError = Trf_Status;
                                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        //resume old joints positions to prevent jumps
                                                        memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                    }							
                                                } else {//middle block -> interpolate joints linearly (this is the real PTP movement)
                                                    if (phase != PHASE_MIDDLE) { // TODO - first time in here -> adjust starting point to avoid jumps
                                                        //memcpy(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint[6].Axes,&gRobot[i]->Monitor.JointPosition,sizeof(gRobot[i]->Monitor.JointPosition));
                                                    }
                                                    phase = PHASE_MIDDLE;	
                                                    double u_tmp = (u-u_start) / (1-u_end-u_start);
                                                    for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) {
                                                        gRobot[i]->Monitor.JointPosition[j] = (1-u_tmp) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint[6].Axes[j] + u_tmp * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint[5].Axes[j];
                                                    }

                                                    Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_DIRECT,gRobot[i]->Monitor.JointPosition,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.MountBasePosition);
                                                    if (Trf_Status != STATUS_OK) {
                                                        gRobot[i]->Monitor.ActiveError = Trf_Status;
                                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                    }
                                                }
                                        
                                                //tangential auto mode does not exist for PTP movements, not even during their half of the round edge transitions
                                        
                                                //reduce path speed if speed of any joint axis is exceeded
                                                // MovementAllowed is set to 255 after test already failed once - no need to repeat again
                                                if (MovementAllowed != 255) {
                                                    double tmpRedFactor;
                                                    unsigned short BadAxes = DynamicLimitsViolated(OldAxesValues,gRobot[i]->Monitor.JointPosition,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits,TaskCycleTime,&tmpRedFactor);
                                            
                                                    if (BadAxes) {
                                                        RedFactor[i] = tmpRedFactor;
                                                
                                                        if (RedFactor[i] > 1) {//should never happen!
                                                            RedFactor[i] = 1;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                            
                                                        if (RedFactor[i] < 0){//should never happen!
                                                            RedFactor[i] = 0;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                            
                                                        //repeat movement with slower speed
                                                        MovementAllowed = 0;
                                                        memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                    }
                                            
                                                } else {
                                                    for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) { //manually truncate joint movements -> WARNING! TCP will leave programmed path!
                                                
                                                        double JointSpeed = (gRobot[i]->Monitor.JointPosition[j]-OldAxesValues[j]) / TaskCycleTime; //do not scale cycle time here!
                                                        if (JointSpeed != 0) {                                                    
                                                            if ((JointSpeed > 0)&&(gRobot[i]->Parameters.JointLimits[j].VelocityPos / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] + gRobot[i]->Parameters.JointLimits[j].VelocityPos * TaskCycleTime;
                                                            }
                                                            if ((JointSpeed < 0)&&(-gRobot[i]->Parameters.JointLimits[j].VelocityNeg / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] - gRobot[i]->Parameters.JointLimits[j].VelocityNeg * TaskCycleTime;
                                                            }
                                                        }
                                                    }
                                                }
                                            } while (MovementAllowed == 0);
                                        
                                            if (fRSVG[i].Done) { // movement completed
										
                                                if ((StoppingError[i] == 0)&&(gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                                    //single step mode -> wait for continue command
                                                    gRobot[i]->Monitor.Halted = 1;
                                                    if (gRobot[i]->Commands.Continue) {
                                                        //TODO -> what if single step is activated during movement
                                                        gRobot[i]->Commands.Continue = 0;
                                                        gRobot[i]->Monitor.Halted = 0;
                                                        fRSVG[i].Enable = 0;
                                                        if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                            strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                        }
                                                        memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                        Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                        Buffer[i].EXEC_Index++;
                                                    }
                                                } else if ((StoppingError[i] == 0)||(fRSVG[i].Status == STATUS_ABORT)) {//movement was completed correctly or aborted but non completed yet -> move on to next block
                                                    fRSVG[i].Enable = 0;
                                                    double tmpEndSpeed = fRSVG[i].Speed / fRSVG[i].Override; //Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;	
                                                    double tmpEndAcc = fRSVG[i].Acceleration; //SVG output acceleration not affected by override
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                    }
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acceleration for next block to read
                                                    if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                        ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                                    }
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                } else if (StoppingError[i] > 0) {//movement stopped because of error
                                                    gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                                    gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                                    gRobot[i]->Monitor.State = ERROR_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                } else if (StoppingError[i] < 0) { //movement stopped from command
                                                    gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                }
                                            }
                                        }				
                                        break;
                                    }
				
                                case MOVE_CIRCLE: {
                                        if (!fRSVG[i].Enable) { //svg not active yet -> movement not started
                                            gRobot[i]->Monitor.BlockLength = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;

                                            gRobot[i]->Monitor.Tool = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Tool;
                                            gRobot[i]->Monitor.Frame = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Frame;

                                            /* do not start movement if block length is zero */
                                            if(gRobot[i]->Monitor.BlockLength < TRF_EPSILON) {
                                                gRobot[i]->Monitor.CompletedBlockLength = 0;
                                                fRSVG[i].Enable = 0;
                                                double tmpEndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;					
                                                double tmpEndAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;					
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acc for next block to read
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                    ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                                }
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                                break;
                                            }

                                            memcpy(&fRSVG[i].DynamicLimits,&PathLimits,sizeof(fRSVG[i].DynamicLimits));
                                            memcpy(&fRSVG[i].DynamicValues,&PathLimits,sizeof(fRSVG[i].DynamicValues));
									
                                            fRSVG[i].DynamicValues.VelocityPos = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate;

                                            //calculate end speed of current block based on future path
                                            unsigned short lookAhead = 1;
                                            unsigned char endBuffer = 0;
                                            double v_max;
                                            do {//look for end of tangential path
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH) {
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                }
                                                if ((Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle < 0)||(Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle > gRobot[i]->Parameters.MaxTransitionAngle)) {
                                                    //non-tangential transition found ---> stop here
                                                    endBuffer = 1;
                                                    lookAhead--; //do not consider this block!
                                                }	
                                                lookAhead++;
                                            } while ((lookAhead < BUFFER_LENGTH)&&(endBuffer == 0));

                                            double EndSpeed = 0;
                                            double TotalBlockLength = 0;
                                            double SameFeedBlockLength = 0;
                                            double SameFeedEndSpeed = 0;
                                            while (lookAhead > 1) { //go backwards from here and find max end speed of current block
                                                lookAhead--;
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH) {
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                }
                                                if (Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_MCODE || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_SETDO || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_RESETDO) {
                                                    continue; //non-synch M-functions do not affect path speed
                                                }
                                                if (!MaxBlockSpeed(Buffer[i].MotionPackage[lookAheadIndex].BlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,EndSpeed,&v_max)) { //AccPos and JerkPos are used here because SVG over a block always moves in positive direction
                                                    TotalBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    //EndSpeed = min(v_max,Buffer[i].MotionPackage[lookAheadIndex].Feedrate); //this should not be used here because it reduces the start speed too much
                                                    EndSpeed = Buffer[i].MotionPackage[lookAheadIndex].Feedrate;
                                                    if (Buffer[i].MotionPackage[lookAheadIndex].Feedrate >= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate) {
                                                        SameFeedBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    } else {
                                                        SameFeedBlockLength = 0;
                                                        SameFeedEndSpeed = EndSpeed;
                                                    }
                                                } else {
                                                    EndSpeed = 0;
                                                    TotalBlockLength = 0;
                                                    SameFeedBlockLength = 0;
                                                    SameFeedEndSpeed = 0;
                                                }										
                                            }

                                            TotalBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
                                            SameFeedBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
																				
                                            //end speed is zero if running in single step mode
                                            if (gRobot[i]->Parameters.SingleStep) {
                                                EndSpeed = 0;
                                            } else {										
                                                //end speed cannot be higher than feedrate!
                                                EndSpeed = min(EndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate);

                                                //end speed cannot be higher than what current dynamics allow until end of tangential path
                                                double tmpV, tmpA0, tmpA1,tmpDelta;
                                                double tmpMaxDyn = MaxMovementDynamics(TotalBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,0,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
										
                                                //however, if feedrate is not constant along path, we should only consider path until current feedrate drops
                                                tmpMaxDyn = MaxMovementDynamics(SameFeedBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,SameFeedEndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
                                            }
									
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = EndSpeed;
																	
                                            fRSVG[i].StartPosition = 0;
                                            fRSVG[i].TargetPosition = gRobot[i]->Monitor.BlockLength;
                                            fRSVG[i].StartSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;		//start with the end speed of previous block (was adjusted at exec time)
                                            fRSVG[i].StartAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;		//start with the end acceleration of previous block									
                                            if (!StoppingError[i]) {
                                                fRSVG[i].EndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;			 // note that this end speed is not necessarily reached (will be adjusted at end of movement)		
                                                fRSVG[i].Start = 1;
                                            } else if (StoppingError[i]<0) { //movement is already stopping -> reduce end speed
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].Stop = 1;
                                            } else { // e-stopping because of active error -> continue e-stop
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].EStop = 1;										
                                            }
                                            fRSVG[i].Enable = 1;							
                                            OldSVGPos[i] = 0;
                                            fRSVG[i].Position = 0;
                                        } else {//movement started -> wait for movement to complete
							
                                            if (fRSVG[i].Status >= ERR_SPG) { // error in SVG call
                                                gRobot[i]->Monitor.ActiveError = fRSVG[i].Status;
                                                gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                            }

                                            // OptMot:
                                            // try to execute movement at given speed
                                            // check if joint dynamic limits are violated
                                            // if it fails then execute it again at lower speed
                                            unsigned short MovementAllowed = 1;
                                            memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                                            double tangAngle;
                                    
                                            //save old X,Y for tangential angle calculation
                                            double oldX = gRobot[i]->Monitor.MountBasePosition[0];
                                            double oldY = gRobot[i]->Monitor.MountBasePosition[1];

                                            do {
                                        
                                                if (MovementAllowed) {//first time we attempt movement - try full speed
                                                    gRobot[i]->Monitor.CompletedBlockLength = fRSVG[i].Position;
                                                } else {//previous movement attempt failed - repeat with new target position (calculated at limited speed)
                                                    gRobot[i]->Monitor.CompletedBlockLength = OldSVGPos[i] + (fRSVG[i].Position - OldSVGPos[i]) * (1 - RedFactor[i]);
                                                    MovementAllowed = 255; // do not repeat test again
                                                }
                                        
                                                //protect from small overshoots
                                                if (gRobot[i]->Monitor.CompletedBlockLength > gRobot[i]->Monitor.BlockLength) {
                                                    gRobot[i]->Monitor.CompletedBlockLength = gRobot[i]->Monitor.BlockLength;
                                                }

                                                double u = gRobot[i]->Monitor.CompletedBlockLength/gRobot[i]->Monitor.BlockLength;

                                                double u_start = 0;
                                                double u_end = 0;
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal > 0) {
                                                    u_start = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.Radius/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal;
                                                    u_end = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.Radius/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLengthIdeal;
                                                }
                                        
                                                //interpolate POSITION
                                                unsigned short phase; //used for debugging, not really needed otherwise
                                                Point_Type tmpPoint;
                                                if (u < u_start) {//start edge -> interpolate Bezier with u = 0.5 ... 1
                                                    phase = PHASE_START;
                                            
                                                    double u_tmp = 0.5 + u / u_start * 0.5;
                                                    EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint,u_tmp,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                    for(j=0;j<BEZIER_XYZ;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = tmpPoint.Axes[j];
                                                    }				
                                            
                                                    //tangential auto mode: force C axis to follow path tangent
                                                    if(gRobot[i]->Monitor.TangActive) {
                                                        //calculate tangent angle on XY plane
                                                        double tang_u = u_tmp + (1.0/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.Length/10.0); //move on spline by 0.1mm (note that increment is purely geometrical, independent of path speed)
                                                        if (tang_u<1){
                                                            EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.CtrlPoint,tang_u,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                            tangAngle = atan2d(tmpPoint.Axes[1]-oldY,tmpPoint.Axes[0]-oldX);
                                                        } else {//spline close to end - use tangent of next block
                                                            tangAngle = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartEdge.tangAngleEnd;
                                                        }
                                                    }
                                                } else if ((1-u) < u_end) {//end edge -> interpolate Bezier with u = 0 ... 0.5
                                                    phase = PHASE_END;	
                                                    double alpha = 1 - u_end;
                                            
                                                    double u_tmp = (u-alpha)/u_end * 0.5;
                                                    EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint,u_tmp,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                    for(j=0;j<BEZIER_XYZ;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = tmpPoint.Axes[j];
                                                    }		
                                                    //tangential auto mode: force C axis to follow path tangent
                                                    if(gRobot[i]->Monitor.TangActive) {
                                                        //calculate tangent angle on XY plane
                                                        double tang_u = u_tmp + (1.0/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.Length/10.0); //move on spline by 0.1mm (note that increment is purely geometrical, independent of path speed)
                                                        if (tang_u<1) {
                                                            EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.CtrlPoint,tang_u,&tmpPoint,BEZIER_XYZ,BEZIER_QUARTIC);
                                                            tangAngle = atan2d(tmpPoint.Axes[1]-oldY,tmpPoint.Axes[0]-oldX);
                                                        } else {//spline close to end - use tangent of next block
                                                            tangAngle = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndEdge.tangAngleEnd;
                                                        }
                                                    }
                                                } else {//middle block -> interpolate linearly with u = 0 ... 1
                                                    phase = PHASE_MIDDLE;	
                                                    //division by zero cannot happen because it would be one of the two cases above
                                                    double u_tmp = u; //(u-u_start) / (1-u_end-u_start);
                                            
                                                    if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].FeedrateType == FEED_ANG) { //interpolate orientation linearly and let cartesian axes follow along
										
                                                        for(j=0;j<BEZIER_XYZ;j++) {//X,Y,Z on circle along with orientation
                                                            //P = C + R cos(t) U + R sin(t) V
                                                            if (u_tmp <= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM) {
                                                                u_tmp = u_tmp/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM;
                                                                gRobot[i]->Monitor.MountBasePosition[j] = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Center[j] + Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius * cos(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleLength * u_tmp / Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartVersor[j] + Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius * sin(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleLength * u_tmp / Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.CrossVersor[j];
                                                            } else {
                                                                u_tmp = (u_tmp-Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM)/(1-Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM);
                                                                gRobot[i]->Monitor.MountBasePosition[j] = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Center[j] + Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius * cos((Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleLength + (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Length - Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleLength) * u_tmp) / Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartVersor[j] + Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius * sin((Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleLength + (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Length - Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleLength) * u_tmp) / Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.CrossVersor[j];
                                                            }
                                                        }	                                                
                                                    } else { // interpolate cartesian axes along circle and let orientation follow along
                                                        for(j=0;j<BEZIER_XYZ;j++) {//X,Y,Z on circle
                                                            //P = C + R cos(t) U + R sin(t) V
                                                            gRobot[i]->Monitor.MountBasePosition[j] = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Center[j] + Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius * cos(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Length * u_tmp / Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartVersor[j] + Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius * sin(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Length * u_tmp / Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Radius) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.CrossVersor[j];
                                                        }
                                                    }
                                            
                                                    //tangential auto mode: calculate tangential angle along circle
                                                    if(gRobot[i]->Monitor.TangActive) {
                                                        //calculate tangent angle on XY plane
                                                        double tmpRadiant[3];
                                                        double tmpTangent[3];
                                                        PointsToVector(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Center,gRobot[i]->Monitor.MountBasePosition,tmpRadiant);
                                                        CrossProduct(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Normal,tmpRadiant,tmpTangent);
                                                        tangAngle = atan2d(tmpTangent[1],tmpTangent[0]);
                                                    }
                                                }
                                        
                                                //interpolate ORIENTATION
                                                if (gRobot[i]->Monitor.AxesNum != 6) {//orientation interpolated linearly
                                                    for(j=3;j<gRobot[i]->Monitor.AxesNum;j++) {
                                                        if (u <= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM) {
                                                            double u_tmp = u/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM;
                                                            gRobot[i]->Monitor.MountBasePosition[j] = (1-u_tmp) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartPointPath[j] + u_tmp * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddlePointPath[j];
                                                        } else {
                                                            double u_tmp = (u-Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM)/(1-Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM);
                                                            gRobot[i]->Monitor.MountBasePosition[j] = (1-u_tmp) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddlePointPath[j] + u_tmp * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.TargetPointPath[j];
                                                        }
                                                    }
                                                } else {//orientation uses quaternions slerp								
                                                    double tmpA,tmpB,tmpC;
                                                    Quat_Type tmpQuat;
                                                    if (u <= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM) {
                                                        double u_tmp = u/Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM;
                                                        Slerp(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleQuat,&tmpQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.QuatAngle1,u_tmp);
                                                    } else {
                                                        double u_tmp = (u-Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM)/(1-Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.uM);
                                                        Slerp(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.MiddleQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndQuat,&tmpQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.QuatAngle2,u_tmp);
                                                    }
                                                    QuatToEuler(tmpQuat,gRobot[i]->Monitor.MountBasePosition[3],gRobot[i]->Monitor.MountBasePosition[4],gRobot[i]->Monitor.MountBasePosition[5],&tmpA,&tmpB,&tmpC);
                                                    gRobot[i]->Monitor.MountBasePosition[3] = tmpA;
                                                    gRobot[i]->Monitor.MountBasePosition[4] = tmpB;
                                                    gRobot[i]->Monitor.MountBasePosition[5] = tmpC;
                                                }
                                        
                                                //tangential auto mode: force C axis to follow path tangent
                                                if(gRobot[i]->Monitor.TangActive) {
                                                    //TODO - this assumes 4 or 6 axis robot
                                                    gRobot[i]->Monitor.MountBasePosition[gRobot[i]->Monitor.AxesNum-1] = tangAngle + gRobot[i]->Monitor.TangOffset;
                                                }
																		
                                                //memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                                                AxesMoved = 1; //movement active -> add whole conveyor offset when tracking
                                                Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
                                                if (Trf_Status != STATUS_OK) {
                                                    gRobot[i]->Monitor.ActiveError = Trf_Status;
                                                    gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                    //resume old joints positions to prevent jumps
                                                    memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                }							
                                        
                                                //reduce path speed if speed of any joint axis is exceeded
                                                // MovementAllowed is set to 255 after test already failed once - no need to repeat again
                                                if (MovementAllowed != 255) {
                                                    double tmpRedFactor;
                                                    unsigned short BadAxes = DynamicLimitsViolated(OldAxesValues,gRobot[i]->Monitor.JointPosition,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits,TaskCycleTime,&tmpRedFactor);
                                                
                                                    if (BadAxes) {
                                                        RedFactor[i] = tmpRedFactor;
                                                        
                                                        if (RedFactor[i] > 1) { //should never happen!
                                                            RedFactor[i] = 1;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                                    
                                                        if (RedFactor[i] < 0) {//should never happen!
                                                            RedFactor[i] = 0;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                                    
                                                        //repeat movement with slower speed
                                                        MovementAllowed = 0;
                                                        memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                    }
                                                } else {
                                                    for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) { //manually truncate joint movements -> WARNING! TCP will leave programmed path!
                                                        double JointSpeed = (gRobot[i]->Monitor.JointPosition[j]-OldAxesValues[j]) / TaskCycleTime; //do not scale cycle time here!
                                                        if (JointSpeed != 0) {
                                                            if ((JointSpeed > 0)&&(gRobot[i]->Parameters.JointLimits[j].VelocityPos / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] + gRobot[i]->Parameters.JointLimits[j].VelocityPos * TaskCycleTime;
                                                            }
                                                            if ((JointSpeed < 0)&&(-gRobot[i]->Parameters.JointLimits[j].VelocityNeg / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] - gRobot[i]->Parameters.JointLimits[j].VelocityNeg * TaskCycleTime;
                                                            }
                                                        }
                                                    }
                                                }              
                                            } while (MovementAllowed == 0);
                                            
                                            if (fRSVG[i].Done) { // movement completed
                                                if ((StoppingError[i] == 0)&&(gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                                    //single step mode -> wait for continue command
                                                    gRobot[i]->Monitor.Halted = 1;
                                                    if (gRobot[i]->Commands.Continue) {
                                                        //TODO -> what if single step is activated during movement
                                                        gRobot[i]->Commands.Continue = 0;
                                                        gRobot[i]->Monitor.Halted = 0;
                                                        fRSVG[i].Enable = 0;
                                                        if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                            strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                        }
                                                        memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                        Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                        Buffer[i].EXEC_Index++;
                                                    }
                                                }
                                                else if ((StoppingError[i] == 0)||(fRSVG[i].Status == STATUS_ABORT)) {//movement was completed correctly or aborted but non completed yet -> move on to next block
                                                    fRSVG[i].Enable = 0;
                                                    double tmpEndSpeed = fRSVG[i].Speed / fRSVG[i].Override; //Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;	
                                                    double tmpEndAcc = fRSVG[i].Acceleration; //SVG output acceleration not affected by override
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                    }
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acceleration for next block to read
                                                    if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                        ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                                    }
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                } else if (StoppingError[i] > 0) {//movement stopped because of error
                                                    gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                                    gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                                    gRobot[i]->Monitor.State = ERROR_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                } else if (StoppingError[i] < 0) { //movement stopped from command
                                                    gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                }
                                            }
                                        }				
                                        break;
                                    }

                                case MOVE_SPLINE: {
                                        if (!fRSVG[i].Enable) { //svg not active yet -> movement not started
								
                                            gRobot[i]->Monitor.BlockLength = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
                                            gRobot[i]->Monitor.CompletedBlockLength = 0;
                                    
                                            gRobot[i]->Monitor.Tool = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Tool;
                                            gRobot[i]->Monitor.Frame = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Frame;
							
                                            /* do not start movement if block length is zero */
                                            if(gRobot[i]->Monitor.BlockLength < TRF_EPSILON) {
                                                gRobot[i]->Monitor.CompletedBlockLength = 0;
                                                fRSVG[i].Enable = 0;
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                double tmpEndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;
                                                double tmpEndAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acc for next block to read
                                                if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                    ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)
                                                }
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                                break;
                                            }
							
                                            memcpy(&fRSVG[i].DynamicLimits,&PathLimits,sizeof(fRSVG[i].DynamicLimits));
                                            memcpy(&fRSVG[i].DynamicValues,&PathLimits,sizeof(fRSVG[i].DynamicValues));

                                            fRSVG[i].DynamicValues.VelocityPos = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate;
									
                                            //calculate end speed of current block based on future path
                                            unsigned short lookAhead = 1;
                                            unsigned char endBuffer = 0;
                                            double v_max;
                                            do {//look for end of tangential path
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH) {
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                }
                                                if ((Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle < 0)||(Buffer[i].MotionPackage[lookAheadIndex].TransitionAngle > gRobot[i]->Parameters.MaxTransitionAngle)) {
                                                    //non-tangential transition found ---> stop here
                                                    endBuffer = 1;
                                                    lookAhead--; //do not consider this block!
                                                }	
                                                lookAhead++;
                                            } while ((lookAhead < BUFFER_LENGTH)&&(endBuffer == 0));

                                            double EndSpeed = 0;
                                            double TotalBlockLength = 0;
                                            double SameFeedBlockLength = 0;
                                            double SameFeedEndSpeed = 0;
                                            while (lookAhead > 1) { //go backwards from here and find max end speed of current block
                                                lookAhead--;
                                                unsigned short lookAheadIndex =	Buffer[i].EXEC_Index + lookAhead;
                                                if (lookAheadIndex >= BUFFER_LENGTH) {
                                                    lookAheadIndex -= BUFFER_LENGTH;
                                                }
                                                if (Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_MCODE || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_SETDO || Buffer[i].MotionPackage[lookAheadIndex].MovementType == MOVE_RESETDO) {
                                                    continue; //non-synch M-functions do not affect path speed
                                                }
                                                if (!MaxBlockSpeed(Buffer[i].MotionPackage[lookAheadIndex].BlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,EndSpeed,&v_max)) { //AccPos and JerkPos are used here because SVG over a block always moves in positive direction
                                                    TotalBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    //EndSpeed = min(v_max,Buffer[i].MotionPackage[lookAheadIndex].Feedrate); //this should not be used here because it reduces the start speed too much
                                                    EndSpeed = Buffer[i].MotionPackage[lookAheadIndex].Feedrate;
                                                    if (Buffer[i].MotionPackage[lookAheadIndex].Feedrate >= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate) {
                                                        SameFeedBlockLength += Buffer[i].MotionPackage[lookAheadIndex].BlockLength;
                                                    } else {
                                                        SameFeedBlockLength = 0;
                                                        SameFeedEndSpeed = EndSpeed;
                                                    }
                                                } else {
                                                    EndSpeed = 0;
                                                    TotalBlockLength = 0;
                                                    SameFeedBlockLength = 0;
                                                    SameFeedEndSpeed = 0;
                                                }										
                                            }

                                            TotalBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
                                            SameFeedBlockLength += Buffer[i].MotionPackage[Buffer[i].EXEC_Index].BlockLength;
											
                                            //end speed is zero if running in single step mode
                                            if (gRobot[i]->Parameters.SingleStep) {
                                                EndSpeed = 0;
                                            } else {										
                                                //end speed cannot be higher than feedrate!
                                                EndSpeed = min(EndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate);

                                                //end speed cannot be higher than what current dynamics allow until end of tangential path
                                                double tmpV, tmpA0, tmpA1,tmpDelta;
                                                double tmpMaxDyn = MaxMovementDynamics(TotalBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,0,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
										
                                                //however, if feedrate is not constant along path, we should only consider path until current feedrate drops
                                                tmpMaxDyn = MaxMovementDynamics(SameFeedBlockLength,fRSVG[i].DynamicValues.AccelerationPos,fRSVG[i].DynamicValues.JerkPos,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Feedrate,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed,SameFeedEndSpeed,Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc,&tmpV,&tmpA0,&tmpA1,&tmpDelta);
                                                if (tmpMaxDyn == STATUS_OK) {
                                                    EndSpeed = min(EndSpeed,tmpV);	
                                                } else {
                                                    EndSpeed = 0;	
                                                }
                                            }
									
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = EndSpeed;
									
                                            fRSVG[i].StartPosition = 0;
                                            fRSVG[i].TargetPosition = gRobot[i]->Monitor.BlockLength;
                                            fRSVG[i].StartSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;		//start with the end speed of previous block (was adjusted at exec time)
                                            fRSVG[i].StartAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;		//start with the end acceleration of previous block
                                            if (!StoppingError[i]) {
                                                fRSVG[i].EndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;			 // note that this end speed is not necessarily reached (will be adjusted at end of movement)		
                                                fRSVG[i].Start = 1;
                                            } else if (StoppingError[i]<0) { //movement is already stopping -> reduce end speed
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].Stop = 1;
                                            } else { // e-stopping because of active error -> continue e-stop
                                                fRSVG[i].EndSpeed = 0;	
                                                fRSVG[i].EStop = 1;										
                                            }
                                            fRSVG[i].Enable = 1;							
                                            OldSVGPos[i] = 0;
                                            fRSVG[i].Position = 0;
                                        } else {//movement started -> wait for movement to complete
							
                                            if (fRSVG[i].Status >= ERR_SPG) { // error in SVG call
                                                gRobot[i]->Monitor.ActiveError = fRSVG[i].Status;
                                                gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                            }
                                    
                                            //save old X,Y for tangential angle calculation
                                            double oldX = gRobot[i]->Monitor.MountBasePosition[0];
                                            double oldY = gRobot[i]->Monitor.MountBasePosition[1];
                                    
                                            // OptMot:
                                            // try to execute movement at given speed
                                            // check if joint dynamic limits are violated
                                            // if it fails then execute it again at lower speed
                                            unsigned short MovementAllowed = 1;
                                            memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                                            double tangAngle;

                                            do {
                                                                                
                                                if (MovementAllowed) {//first time we attempt movement - try full speed
                                                    gRobot[i]->Monitor.CompletedBlockLength = fRSVG[i].Position;
                                                } else {//previous movement attempt failed - repeat with new target position (calculated at limited speed);
                                                    gRobot[i]->Monitor.CompletedBlockLength = OldSVGPos[i] + (fRSVG[i].Position - OldSVGPos[i]) * (1 - RedFactor[i]);
                                                    MovementAllowed = 255; // do not repeat test again
                                                }
                                        
                                                //protect from small overshoots
                                                if (gRobot[i]->Monitor.CompletedBlockLength > gRobot[i]->Monitor.BlockLength) {
                                                    gRobot[i]->Monitor.CompletedBlockLength = gRobot[i]->Monitor.BlockLength;
                                                }
									
                                                //interpolate position along spline
                                                Point_Type tmpPoint;
                                                double u = gRobot[i]->Monitor.CompletedBlockLength / gRobot[i]->Monitor.BlockLength;
                                                EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Spline.CtrlPoint,u,&tmpPoint,BEZIER_XYZ,BEZIER_CUBIC);
                                                for(j=0;j<3;j++) {
                                                    gRobot[i]->Monitor.MountBasePosition[j] = tmpPoint.Axes[j];
                                                }											
                                        
                                                //interpolate orientation along spline
                                                if (gRobot[i]->Monitor.AxesNum != 6) {
                                                    for(j=3;j<gRobot[i]->Monitor.AxesNum;j++) {
                                                        gRobot[i]->Monitor.MountBasePosition[j] = (1-u) * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Spline.CtrlPoint[3].Axes[j] + u * Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Spline.CtrlPoint[0].Axes[j];
                                                    }
                                                } else {								
                                                    double tmpA,tmpB,tmpC;
                                                    Quat_Type tmpQuat;
                                                    Slerp(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.StartQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.EndQuat,&tmpQuat,Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.QuatAngle,u);
                                                    QuatToEuler(tmpQuat,gRobot[i]->Monitor.MountBasePosition[3],gRobot[i]->Monitor.MountBasePosition[4],gRobot[i]->Monitor.MountBasePosition[5],&tmpA,&tmpB,&tmpC);
                                                    gRobot[i]->Monitor.MountBasePosition[3] = tmpA;
                                                    gRobot[i]->Monitor.MountBasePosition[4] = tmpB;
                                                    gRobot[i]->Monitor.MountBasePosition[5] = tmpC;
                                                }
									
                                                //tangential auto mode: force C axis to follow path tangent
                                                if(gRobot[i]->Monitor.TangActive) {
                                                    //calculate tangent angle on XY plane
                                                    double tang_u = u + (1.0/gRobot[i]->Monitor.BlockLength/10.0); //move on spline by 0.1mm (note that increment is purely geometrical, independent of path speed)
                                                    if (tang_u<1) {
                                                        EvaluateBezier(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.Spline.CtrlPoint,tang_u,&tmpPoint,BEZIER_XYZ,BEZIER_CUBIC);
                                                        tangAngle = atan2d(tmpPoint.Axes[1]-oldY,tmpPoint.Axes[0]-oldX);
                                                    } else {//spline close to end - use tangent of next block
                                                        tangAngle = atan2d(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndPathVector[1],Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndPathVector[0]);
                                                    }
                                                    //TODO - this assumes 4 or 6 axis robot
                                                    gRobot[i]->Monitor.MountBasePosition[gRobot[i]->Monitor.AxesNum-1] = tangAngle + gRobot[i]->Monitor.TangOffset;
                                                }

                                                //memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
                                                AxesMoved = 1; //movement active -> add whole conveyor offset when tracking
                                                Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
                                                if (Trf_Status != STATUS_OK) {
                                                    gRobot[i]->Monitor.ActiveError = Trf_Status;
                                                    gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                    //resume old joints positions to prevent jumps
                                                    memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                }							
                                    
                                                //reduce path speed if speed of any joint axis is exceeded
                                                // MovementAllowed is set to 255 after test already failed once - no need to repeat again
                                                if (MovementAllowed != 255) {
                                                    double tmpRedFactor;
                                                    unsigned short BadAxes = DynamicLimitsViolated(OldAxesValues,gRobot[i]->Monitor.JointPosition,gRobot[i]->Monitor.AxesNum,gRobot[i]->Parameters.JointLimits,TaskCycleTime,&tmpRedFactor);
                                            
                                                    if (BadAxes) {
                                                        RedFactor[i] = tmpRedFactor;
                                                
                                                        if (RedFactor[i] > 1) {//should never happen!
                                                            RedFactor[i] = 1;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                            
                                                        if (RedFactor[i] < 0) {//should never happen!
                                                            RedFactor[i] = 0;
                                                            gRobot[i]->Monitor.ActiveError = ERR_OPTMOT;
                                                            gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                                        }
                                            
                                                        //repeat movement with slower speed
                                                        MovementAllowed = 0;
                                                        memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                                                    }
                                                } else {
                                                    for(j=0;j<gRobot[i]->Monitor.AxesNum;j++) { //manually truncate joint movements -> WARNING! TCP will leave programmed path!
                                                
                                                        double JointSpeed = (gRobot[i]->Monitor.JointPosition[j]-OldAxesValues[j]) / TaskCycleTime; //do not scale cycle time here!
                                                        if (JointSpeed != 0) {                                                    
                                                            if ((JointSpeed > 0)&&(gRobot[i]->Parameters.JointLimits[j].VelocityPos / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] + gRobot[i]->Parameters.JointLimits[j].VelocityPos * TaskCycleTime;
                                                            }
                                                            if ((JointSpeed < 0)&&(-gRobot[i]->Parameters.JointLimits[j].VelocityNeg / JointSpeed < 1)) {
                                                                gRobot[i]->Monitor.JointPosition[j] = OldAxesValues[j] - gRobot[i]->Parameters.JointLimits[j].VelocityNeg * TaskCycleTime;
                                                            }
                                                        }
                                                    }
                                                }
                                            } while (MovementAllowed == 0);
                                    
                                            if (fRSVG[i].Done) { // movement completed
                                                if ((StoppingError[i] == 0)&&(gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                                    //single step mode -> wait for continue command
                                                    gRobot[i]->Monitor.Halted = 1;
                                                    if (gRobot[i]->Commands.Continue) {
                                                        //TODO -> what if single step is activated during movement
                                                        gRobot[i]->Commands.Continue = 0;
                                                        gRobot[i]->Monitor.Halted = 0;
                                                        fRSVG[i].Enable = 0;
                                                        if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                            strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                        }
                                                        memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                        Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                        Buffer[i].EXEC_Index++;
                                                    }
                                                } else if ((StoppingError[i] == 0)||(fRSVG[i].Status == STATUS_ABORT)) {//movement was completed correctly or aborted but non completed yet -> move on to next block
                                                    fRSVG[i].Enable = 0;
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                    }
                                                    double tmpEndSpeed = fRSVG[i].Speed / fRSVG[i].Override; //Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed;
                                                    double tmpEndAcc = fRSVG[i].Acceleration; //SVG output acceleration not affected by override
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                                    Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acceleration for next block to read
                                                    if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                        ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                                    }
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                } else if (StoppingError[i] > 0) {//movement stopped because of error
                                                    gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                                    gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                                    gRobot[i]->Monitor.State = ERROR_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                } else if (StoppingError[i] < 0) { //movement stopped from command
                                                    gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                                    StoppingError[i] = 0;
                                                    StoppingLine[i] = 0;
                                                }
                                            }														
                                        }
                                        break;	
                                    }
                            
                                case MOVE_TRK: {
                                        gRobot[i]->Monitor.BlockLength = 0;
                                        gRobot[i]->Monitor.CompletedBlockLength = 0;
                                        if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].TrkIndex == 0) { // stop tracking conveyors
                                            gRobot[i]->Monitor.TrackActive = 0; //stop following conveyor's position
                                            gRobot[i]->Monitor.TrackSynch = 0; //used below to restart PP
                                        } else { // TrkIndex == 1 or 2 
                                            // start tracking conveyors
									
                                            gRobot[i]->Parameters.Conveyor[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].TrkIndex - 1].Position = 0;
									
                                            //set TrackWait only the first time here (use DelayTime as dirty trick flag)
                                            if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime == 0) {
                                                gRobot[i]->Monitor.TrackSynch = 1;
                                                Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime = -1;
                                            }
										
                                            if (!gRobot[i]->Monitor.TrackSynch) { //was reset by application -> PP can restart
                                                gRobot[i]->Monitor.TrackActive = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].TrkIndex; //start following conveyor's position
                                            }
                                        }
                                    
                                        if((!gRobot[i]->Monitor.TrackSynch)&&(StoppingError[i] == 0)) { //tracking has been allowed -> continue program execution
								
                                            if ((gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                                //single step mode -> wait for continue command
                                                gRobot[i]->Monitor.Halted = 1;
                                                if (gRobot[i]->Commands.Continue) {
                                                    //TODO -> what if single step is activated during movement
                                                    gRobot[i]->Commands.Continue = 0;
                                                    gRobot[i]->Monitor.Halted = 0;
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                    }
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                    Buffer[i].Synch = 1; //reset PP
                                                    Buffer[i].Planned = 0; //wait for PP to refill buffer
                                                }
                                            } else {
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                                Buffer[i].Synch = 1; //reset PP
                                                Buffer[i].Planned = 0; //wait for PP to refill buffer
                                            }
                                        } else if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;								
                                        break;
                                    }

                                case MOVE_SETDO:
                                case MOVE_RESETDO: {
                                        gRobot[i]->Monitor.BlockLength = 0;
                                        gRobot[i]->Monitor.CompletedBlockLength = 0;
								
                                        //(re)set DO nodes only the first time here (use DelayTime as dirty trick flag)
                                        if(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime == 0) {
                                            unsigned short value;
                                            if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].MovementType == MOVE_SETDO) {
                                                value = 1;
                                            } else {
                                                value = 0;
                                            }
                                            gRobot[i]->Monitor.DO_[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].IO_Index] = value;
                                        }
                                        Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime = -1;

                                        if ((StoppingError[i] == 0)&&(gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {//single step mode -> wait for continue command
                                            gRobot[i]->Monitor.Halted = 1;
                                            if (gRobot[i]->Commands.Continue) {
                                                //TODO -> what if single step is activated during movement
                                                gRobot[i]->Commands.Continue = 0;
                                                gRobot[i]->Monitor.Halted = 0;
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                            }
                                        } else if (StoppingError[i] == 0) { //move immediately to next block
                                            if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                            }
                                            double tmpEndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;
                                            double tmpEndAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;
                                            memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acc for next block to read	
                                            if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                            }
                                            Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                            Buffer[i].EXEC_Index++;
                                        } else if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;																
                                        break;
                                    }

                            
                                case MOVE_MCODE: {
                                        gRobot[i]->Monitor.BlockLength = 0;
                                        gRobot[i]->Monitor.CompletedBlockLength = 0;
								
                                        //set M functions only the first time here (use DelayTime as dirty trick flag)
                                        if(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime == 0) {
                                            M_count = 0;
                                            M_value = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].M_Index[M_count];
                                            while((M_value > 0)&&(M_count < MAX_MFUNC_INLINE)) {
                                                gRobot[i]->Monitor.M[M_value] = 1;
                                                M_count++;
                                                M_value = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].M_Index[M_count];
                                            }
                                        }
                                        Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime = -1;

                                        if ((StoppingError[i] == 0)&&(gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                            //single step mode -> wait for continue command
                                            gRobot[i]->Monitor.Halted = 1;
                                            if (gRobot[i]->Commands.Continue) {
                                                //TODO -> what if single step is activated during movement
                                                gRobot[i]->Commands.Continue = 0;
                                                gRobot[i]->Monitor.Halted = 0;
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                            }
                                        } else if (StoppingError[i] == 0) { //move immediately to next block
                                            if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                            }
                                            double tmpEndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;
                                            double tmpEndAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;
                                            memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acc for next block to read	
                                            if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                            }
                                            Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                            Buffer[i].EXEC_Index++;
                                        } else if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;																
                                        break;
                                    }
						
                                case MOVE_MCODE_SYNCH: {
                                        M_count = 0;
                                        M_value = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].M_Index[M_count];
                                        M_synch = 0; //flag to check if all the synch M funcs have been reset
                                        while((M_value > 0)&&(M_count < MAX_MFUNC_INLINE)) {
                                            //set M functions only the first time here (use DelayTime as dirty trick flag)
                                            if(Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime == 0) {
                                                gRobot[i]->Monitor.M[M_value] = 1;
                                            }
                                            //check if all the synch M funcs have been reset
                                            if ((gRobot[i]->Parameters.M_synch[M_value])&&(gRobot[i]->Monitor.M[M_value])) { // non-reset synch M func found!
                                                M_synch = 1;
                                            }
                                            M_count++;
                                            M_value = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].M_Index[M_count];
                                        }
                                        Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime = -1;

                                        if((!M_synch)&&(StoppingError[i] == 0)) { //all synch M-functions have been reset -> continue program execution
								
                                            if ((gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                                //single step mode -> wait for continue command
                                                gRobot[i]->Monitor.Halted = 1;
                                                if (gRobot[i]->Commands.Continue) {
                                                    //TODO -> what if single step is activated during movement
                                                    gRobot[i]->Commands.Continue = 0;
                                                    gRobot[i]->Monitor.Halted = 0;
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],""); 
                                                    }
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                    Buffer[i].Synch = 1; //reset PP
                                                    Buffer[i].Planned = 0; //wait for PP to refill buffer
                                                }
                                            } else{
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                                Buffer[i].Synch = 1; //reset PP
                                                Buffer[i].Planned = 0; //wait for PP to refill buffer
                                            }
                                        } else if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;
                                        break;	
                                    }

                                case MOVE_WAITDI: {
                                        gRobot[i]->Monitor.BlockLength = 0;
                                        gRobot[i]->Monitor.CompletedBlockLength = 0;
                                        if ((StoppingError[i] == 0)&&(gRobot[i]->Monitor.DI_[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].IO_Index] == 1)) {
                                            if ((gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                                //single step mode -> wait for continue command
                                                gRobot[i]->Monitor.Halted = 1;
                                                if (gRobot[i]->Commands.Continue) {
                                                    //TODO -> what if single step is activated during movement
                                                    gRobot[i]->Commands.Continue = 0;
                                                    gRobot[i]->Monitor.Halted = 0;
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                    }
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                }
                                            } else {
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;						
                                            }
                                        } else if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;
                                        break;
                                    }

                                case MOVE_DELAY: {
                                        gRobot[i]->Monitor.BlockLength = 0;
                                        if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index].IO_Index == 0) {   //set CompletedBlockLength to zero first time (use IO_Index as dirty trick)
                                            gRobot[i]->Monitor.CompletedBlockLength = 0;
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].IO_Index = 1;
                                        }
                                        gRobot[i]->Monitor.CompletedBlockLength += TaskCycleTime;
                                        if ((StoppingError[i] == 0)&&(gRobot[i]->Monitor.CompletedBlockLength >= Buffer[i].MotionPackage[Buffer[i].EXEC_Index].DelayTime)) {
                                            if ((gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {//single step mode -> wait for continue command
                                                gRobot[i]->Monitor.Halted = 1;
                                                if (gRobot[i]->Commands.Continue) {
                                                    //TODO -> what if single step is activated during movement
                                                    gRobot[i]->Commands.Continue = 0;
                                                    gRobot[i]->Monitor.Halted = 0;
                                                    if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                        strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                    }
                                                    memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                    Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                    Buffer[i].EXEC_Index++;
                                                }
                                            } else {
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;						
                                            }
                                        } else if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;
                                        break;
                                    }

                                case MOVE_TANG: {
                                        gRobot[i]->Monitor.BlockLength = 0;
                                        gRobot[i]->Monitor.CompletedBlockLength = 0;
                                        gRobot[i]->Monitor.TangActive = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].TangCmd;
                                        gRobot[i]->Monitor.TangOffset = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].Path.RotAngle;
                                
                                        if ((StoppingError[i] == 0)&&(gRobot[i]->Parameters.SingleStep)&&(Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_END)) {
                                            //single step mode -> wait for continue command
                                            gRobot[i]->Monitor.Halted = 1;
                                            if (gRobot[i]->Commands.Continue) {
                                                //TODO -> what if single step is activated during movement
                                                gRobot[i]->Commands.Continue = 0;
                                                gRobot[i]->Monitor.Halted = 0;
                                                if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                    strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                                }
                                                memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                                Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                                Buffer[i].EXEC_Index++;
                                            }
                                        } else if (StoppingError[i] == 0) { //move immediately to next block
                                            if (Robot_Program[i] == 0) { //remove block from ring buffer if in move_blocks mode
                                                strcpy(gRobot[i]->Parameters.Blocks[Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber],"");
                                            }
                                            double tmpEndSpeed = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndSpeed;
                                            double tmpEndAcc = Buffer[i].MotionPackage[Buffer[i].EXEC_Index_Prev].EndAcc;
                                            memset(&Buffer[i].MotionPackage[Buffer[i].EXEC_Index],0,sizeof(Buffer[i].MotionPackage[Buffer[i].EXEC_Index]));
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndSpeed = tmpEndSpeed; //leave end speed for next block to read
                                            Buffer[i].MotionPackage[Buffer[i].EXEC_Index].EndAcc = tmpEndAcc; //leave end acc for next block to read	
                                            if (Buffer[i].MotionPackage[Buffer[i].EXEC_Index+1].MovementType != MOVE_TRK) {
                                                ContinueExec = 1; //continue immediately to next block without waiting for a cyletime (to avoid speed dip) - except for TRK (to avoid axes jumps)										
                                            }
                                            Buffer[i].EXEC_Index_Prev = Buffer[i].EXEC_Index;
                                            Buffer[i].EXEC_Index++;
                                        } else if (StoppingError[i] > 0) { //movement stopped because of error
                                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                                            gRobot[i]->Monitor.State = ERROR_STATE;
                                        } else if (StoppingError[i] < 0) { //movement stopped from command
                                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                                        }
                                        StoppingError[i] = 0;
                                        StoppingLine[i] = 0;																
                                        break;
                                    }

                                case MOVE_END: {
                                        //end of program - back to standstill
                                        gRobot[i]->Monitor.State = STANDSTILL_STATE;	
                                        if (Robot_Program[i] == 0) { //remove last ("END") block from ring buffer if in move_blocks mode
                                            int LastLine = Buffer[i].MotionPackage[Buffer[i].EXEC_Index].LineNumber;
                                            if (LastLine >= RING_BUFFER_SIZE) {
                                                LastLine -= RING_BUFFER_SIZE;
                                            }
                                            strcpy(gRobot[i]->Parameters.Blocks[LastLine],"");
                                        }
                                        break;
                                    }

                                default: {
                                        gRobot[i]->Monitor.ActiveError = ERR_NOT_SUPPORTED;
                                        gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                                        break;
                                    }
                            }
				
                        }	while ((ContinueExec)&&(EXEC_LoopCount < MAX_LOOP_COUNT));	//&&(StoppingError[i] == 0)
				
                    } else if(StoppingError[i] != 0) {
                        // check if errors are found before EXEC starts working
                        if (StoppingError[i] > 0) {
                            //movement stopped because of error
                            gRobot[i]->Monitor.ActiveError = StoppingError[i];
                            gRobot[i]->Monitor.ErrorLine = StoppingLine[i];
                            gRobot[i]->Monitor.State = ERROR_STATE;
                        } else if (StoppingError[i] < 0) {
                            //movement stopped from command
                            gRobot[i]->Monitor.State = STANDSTILL_STATE;
                        }
                        StoppingError[i] = 0;
                        StoppingLine[i] = 0;
                    }
								
                    OldSVGPos[i] = fRSVG[i].Position;
                    break;	//end case MOVING
                    
                }
        
            case ERROR_STATE: {            
                    fRSVG[i].Enable = 0;                
                    gRobot[i]->Monitor.Moving = 0;
                    gRobot[i]->Monitor.Halted = 0;
                    HaltedByCmd[i] = 0;
                    gRobot[i]->Monitor.PathSpeed = 0;
                    gRobot[i]->Monitor.TrackSynch = 0;
                    gRobot[i]->Monitor.TangActive = 0;
                    gRobot[i]->Monitor.TangOffset = 0;
                    break;
                }
        }		
		
		
        /******************** update axis monitor ********************/
	
			
        gRobot[i]->Monitor.Handle = (unsigned long)&gRobot[i];
				
        //add conveyor offset if tracking is active
        if (gRobot[i]->Monitor.TrackActive > 0) {
            //modify path axes
            double ConveyorAngle = gRobot[i]->Parameters.Conveyor[gRobot[i]->Monitor.TrackActive - 1].Angle;
            double ConveyorDelta;
            if (AxesMoved) { //movement is active -> add whole conveyor offset to planned position
                ConveyorDelta = gRobot[i]->Parameters.Conveyor[gRobot[i]->Monitor.TrackActive - 1].Position;
            } else { //movement is not active -> only add incremental offset to current position
                ConveyorDelta = gRobot[i]->Parameters.Conveyor[gRobot[i]->Monitor.TrackActive - 1].Position - OldConveyorPos[i][gRobot[i]->Monitor.TrackActive - 1];
            }
            double OldPath[6];
            memcpy(&OldPath,&gRobot[i]->Monitor.MountBasePosition,sizeof(OldPath));
            gRobot[i]->Monitor.MountBasePosition[0] += 	(ConveyorDelta*cosd(ConveyorAngle)); //tracking along X axis
            gRobot[i]->Monitor.MountBasePosition[1] += 	(ConveyorDelta*sind(ConveyorAngle)); //tracking along Y axis			

            //modify joint axes
            short WorkspaceFlag = 0;
            memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
            if (Trf_Status == STATUS_OK) {//make sure joints are within limits
                for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {					
                    //check for joint axes limits
                    if ((gRobot[i]->Monitor.JointPosition[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(gRobot[i]->Monitor.JointPosition[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                        WorkspaceFlag = 1;
                    }
                }							
            }
            if ((Trf_Status != STATUS_OK)||(WorkspaceFlag)) {// ignore movement and reload old position
                gRobot[i]->Monitor.ActiveError = ERR_TRK1 + gRobot[i]->Monitor.TrackActive -1;
                gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                memcpy(&gRobot[i]->Monitor.MountBasePosition,&OldPath,sizeof(gRobot[i]->Monitor.MountBasePosition));
                memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                gRobot[i]->Monitor.State = STANDSTILL_STATE;
            }
        }
        OldConveyorPos[i][0] = gRobot[i]->Parameters.Conveyor[0].Position;
        OldConveyorPos[i][1] = gRobot[i]->Parameters.Conveyor[1].Position;


        //add external path corrections
        //note that correction can be added even at standstill and the robot will move!!!
        double OldPath[6];
        memcpy(&OldPath,&gRobot[i]->Monitor.MountBasePosition,sizeof(OldPath));
        double CorrectionDelta[6];
        short CorrectionActive = 0;
        for(k=0;k<6;k++) {
            //corrections along X,Y,Z,A,B,C axes
            if (AxesMoved) { //add absolute correction (because MountBasePosition was refreshed by EXEC)
                CorrectionDelta[k] = gRobot[i]->Parameters.PathCorrection[k];
            } else { //only add incremental correction (because MountBasePosition was not updated from last cycle)
                CorrectionDelta[k] = gRobot[i]->Parameters.PathCorrection[k] - OldPathCorrections[i][k];
            }
            if (CorrectionDelta[k] != 0)    CorrectionActive = 1;	//use this flag to save time by avoiding calling TRF if no correction was given

            gRobot[i]->Monitor.MountBasePosition[k] += 	CorrectionDelta[k];
            OldPathCorrections[i][k] = gRobot[i]->Parameters.PathCorrection[k];
        }

        if (CorrectionActive) { //no need to call TRF if no correction was given
            //update joint axes
            short WorkspaceFlag = 0;
            memcpy(&OldAxesValues,&gRobot[i]->Monitor.JointPosition,sizeof(OldAxesValues));
            Trf_Status = Transformations(&gRobot[i]->Parameters.Mechanics,TRF_INVERSE,OldAxesValues,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.JointPosition);
            if (Trf_Status == STATUS_OK) {//make sure joints are within limits
                for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {					
                    //check for joint axes limits
                    if ((gRobot[i]->Monitor.JointPosition[k] > gRobot[i]->Parameters.JointLimits[k].PositionPos)||(gRobot[i]->Monitor.JointPosition[k] < gRobot[i]->Parameters.JointLimits[k].PositionNeg)) {
                        WorkspaceFlag = 1;
                    }
                }							
            }
            if ((Trf_Status != STATUS_OK)||(WorkspaceFlag)) {// ignore movement and reload old position
                gRobot[i]->Monitor.ActiveError = ERR_TRK1 + gRobot[i]->Monitor.TrackActive -1;
                gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
                memcpy(&gRobot[i]->Monitor.MountBasePosition,&OldPath,sizeof(gRobot[i]->Monitor.MountBasePosition));
                memcpy(&gRobot[i]->Monitor.JointPosition,&OldAxesValues,sizeof(gRobot[i]->Monitor.JointPosition));
                gRobot[i]->Monitor.State = STANDSTILL_STATE;
            }
        }
		
        //calculate monitor pathposition based on current frame and tool
        //1. add tool
        if (gRobot[i]->Monitor.AxesNum < 5) {
            SubFrame2D(gRobot[i]->Parameters.Tool[gRobot[i]->Monitor.Tool].Axes,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.ToolBasePosition);
        } else {
            SubFrame3D(gRobot[i]->Parameters.Tool[gRobot[i]->Monitor.Tool].Axes,gRobot[i]->Monitor.MountBasePosition,gRobot[i]->Monitor.ToolBasePosition[3],gRobot[i]->Monitor.ToolBasePosition[4],gRobot[i]->Monitor.ToolBasePosition[5],gRobot[i]->Monitor.ToolBasePosition);
        }
        
        //2. add work frame (this is not base frame!!! - that is added individually in the transformations and is configured in the mechanical parameters)
        if (gRobot[i]->Monitor.AxesNum < 5) {
            AddFrame2D(gRobot[i]->Monitor.ToolBasePosition,gRobot[i]->Parameters.Frame[gRobot[i]->Monitor.Frame].Axes,gRobot[i]->Monitor.PathPosition);
        } else {
            AddFrame3D(gRobot[i]->Monitor.ToolBasePosition,gRobot[i]->Parameters.Frame[gRobot[i]->Monitor.Frame].Axes,gRobot[i]->Monitor.PathPosition[3],gRobot[i]->Monitor.PathPosition[4],gRobot[i]->Monitor.PathPosition[5],gRobot[i]->Monitor.PathPosition);
        }							
		
        //update path speed
        if ((gRobot[i]->Monitor.State != ERROR_STATE)&&(gRobot[i]->Monitor.State != STANDSTILL_STATE)) {
            gRobot[i]->Monitor.PathSpeed = fRSVG[i].Speed * RedFactor[i];
        }

        //calculate joints speeds
        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {					
            gRobot[i]->Monitor.JointSpeed[k] = (gRobot[i]->Monitor.JointPosition[k] - OldMonitor[i].JointPosition[k]) / TaskCycleTime;

            //abort movement if joint speed limits are violated (by 5%) while movement is active
            // 5% tolerance was added because of false triggers caused by numerical stability
            //provides protection for jumps (especially during tracking)
            //no monitoring during standstill because homing command SetJoints would trigger false violation
            if ((gRobot[i]->Monitor.State != STANDSTILL_STATE)&&(gRobot[i]->Monitor.State != ERROR_STATE)&&((gRobot[i]->Monitor.JointSpeed[k] > gRobot[i]->Parameters.JointLimits[k].VelocityPos * 1.05)||(gRobot[i]->Monitor.JointSpeed[k] < -gRobot[i]->Parameters.JointLimits[k].VelocityNeg * 1.05))) {
                gRobot[i]->Monitor.JointPosition[k] = OldMonitor[i].JointPosition[k];
                gRobot[i]->Monitor.ActiveError = ERR_LIMIT_VEL_J1+k;
                gRobot[i]->Monitor.ErrorLine = gRobot[i]->Monitor.LineNumber;
            }			

        }							

        /* write set positions to drives */ 
		
        double PreSetToDrive[6]; //do all operations in double before converting to integers
		
        /* filter set values */
        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {
            //activate filter when moving and leave it on after stopping a movement in order for set values to settle down 
            if ((gRobot[i]->Monitor.Moving)||(StoppedTime[i] < gRobot[i]->Parameters.FilterTime)) {
                Filter[i][k].Enable = 1;
            } else {
                Filter[i][k].Enable = 0;
            }
						
            Filter[i][k].Window = (unsigned short) round(gRobot[i]->Parameters.FilterTime / gRobot[i]->Parameters.CycleTime);
            Filter[i][k].InputValue = gRobot[i]->Monitor.JointPosition[k];
            GaussianFilter((struct Filter_Type*)&Filter[i][k]);
						
            PreSetToDrive[k] = Filter[i][k].OutputValue;
        }

        /* add mechanical coupling */
        if (gRobot[i]->Parameters.Mechanics.Coupling[1]!=0) PreSetToDrive[2] -= (PreSetToDrive[1]*gRobot[i]->Parameters.Mechanics.Coupling[1]);
        if (gRobot[i]->Parameters.Mechanics.Coupling[3]!=0) PreSetToDrive[4] -= (PreSetToDrive[3]*gRobot[i]->Parameters.Mechanics.Coupling[3]);
        if (gRobot[i]->Parameters.Mechanics.Coupling[4]!=0) PreSetToDrive[5] -= (PreSetToDrive[3]*gRobot[i]->Parameters.Mechanics.Coupling[4]);
        if (gRobot[i]->Parameters.Mechanics.Coupling[5]!=0) PreSetToDrive[5] -= (PreSetToDrive[4]*gRobot[i]->Parameters.Mechanics.Coupling[5]);
		
        /* scale units */
        for (k=0;k<gRobot[i]->Monitor.AxesNum;k++) {
            if (gRobot[i]->Parameters.UnitsRatio[k].AxisUnits != 0) {
                gRobot[i]->Monitor.SetToDrive[k] = gRobot[i]->Parameters.UnitsRatio[k].HomeOffset + round(PreSetToDrive[k] * (double) gRobot[i]->Parameters.UnitsRatio[k].Direction * (double) gRobot[i]->Parameters.UnitsRatio[k].MotorUnits / (double) gRobot[i]->Parameters.UnitsRatio[k].AxisUnits);
            } else {
                gRobot[i]->Monitor.SetToDrive[k] = 0;
            }
            //add License check (should be zero in normal cases)
            gRobot[i]->Monitor.SetToDrive[k] += License;
        }

        //reset AxesMoved flag
        AxesMoved = 0;
		
        //reset continue command
        gRobot[i]->Commands.Continue = 0;
				
        // save monitor structure in static local variable so that they can be reactivated the next cycle time preventing the user from modifying it
        OldMonitor[i] = gRobot[i]->Monitor;		
		
    }

    return STATUS_OK;

}

