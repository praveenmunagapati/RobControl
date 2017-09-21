#ifndef STATIC_H
#define STATIC_H


/* define static variables here */
/* static variables are not modified at the end of a function call, their values are saved and available at the next call */
static Robot_Monitor_Type OldMonitor[MAX_ROBOTS];
static Buffer_Type Buffer[MAX_ROBOTS];
static struct RSVG_Type fRSVG[MAX_ROBOTS];
static Robot_Jog_Type Jog[MAX_ROBOTS];
static float Override[MAX_ROBOTS];
//static float OverrideRampStep[MAX_ROBOTS];
static struct Filter_Type FilterOverride[MAX_ROBOTS];
static Robot_Parameter_UnitsRatio_Type OldUnitsRatio[MAX_ROBOTS][6];
static float OldFilterTime[MAX_ROBOTS];
static float OldSingleStep[MAX_ROBOTS];
static float HaltedByCmd[MAX_ROBOTS];
static float StoppedTime[MAX_ROBOTS];
static float OldTransitionAngle[MAX_ROBOTS];
static float OldConveyorPos[MAX_ROBOTS][2];
static float OldSVGPos[MAX_ROBOTS];
static float OldPathCorrections[MAX_ROBOTS][6];
static struct Filter_Type Filter[MAX_ROBOTS][6];
//static Robot_Parameter_Workspace_Type CyclicWorkspace[MAX_ROBOTS];
//static Robot_Parameter_Workspace_Type PathPlannerWorkspace[MAX_ROBOTS];
static short StoppingError[MAX_ROBOTS];
static long StoppingLine[MAX_ROBOTS];
static char* Robot_Program[MAX_ROBOTS];
static unsigned long EvaluationTime;
static float RedFactor[MAX_ROBOTS];
static unsigned short CheckConstDone;

#endif
