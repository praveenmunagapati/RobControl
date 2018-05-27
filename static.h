#ifndef STATIC_H
#define STATIC_H


/* define static variables here */
/* static variables are not modified at the end of a function call, their values are saved and available at the next call */
static Robot_Monitor_Type OldMonitor[MAX_ROBOTS];
static Buffer_Type Buffer[MAX_ROBOTS];
static struct RSVG_Type fRSVG[MAX_ROBOTS];
static Robot_Jog_Type Jog[MAX_ROBOTS];
static double Override[MAX_ROBOTS];
//static double OverrideRampStep[MAX_ROBOTS];
static struct Filter_Type FilterOverride[MAX_ROBOTS];
static UnitsRatio_Type OldUnitsRatioJoints[MAX_ROBOTS][6];
static UnitsRatio_Type OldUnitsRatioAux[MAX_ROBOTS][6];
static double OldFilterTime[MAX_ROBOTS];
static double OldSingleStep[MAX_ROBOTS];
static double HaltedByCmd[MAX_ROBOTS];
static double StoppedTime[MAX_ROBOTS];
static double OldTransitionAngle[MAX_ROBOTS];
static double OldConveyorPos[MAX_ROBOTS][2];
static double OldSVGPos[MAX_ROBOTS];
static double OldPathCorrections[MAX_ROBOTS][6];
static struct Filter_Type FilterJoints[MAX_ROBOTS][6];
static struct Filter_Type FilterAux[MAX_ROBOTS][6];
//static Robot_Parameter_Workspace_Type CyclicWorkspace[MAX_ROBOTS];
//static Robot_Parameter_Workspace_Type PathPlannerWorkspace[MAX_ROBOTS];
static short StoppingError[MAX_ROBOTS];
static long StoppingLine[MAX_ROBOTS];
static char* Robot_Program[MAX_ROBOTS];
static unsigned long EvaluationTime;
static double RedFactor[MAX_ROBOTS];
static unsigned short CheckConstDone;

#endif
