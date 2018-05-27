#ifndef _MISC_TYPES
#define _MISC_TYPES

#include "RobControl.h"
#include "Frame.h"
#include <string.h>

/* declare miscellaneous datatypes and functions */

typedef struct RefPoint_Type
{	
    double Axes[6];
    unsigned char Defined[6];
} RefPoint_Type;

typedef struct Edge_Type
{
    double Radius;
    double Length;
    Frame_Type CtrlPoint[7]; //#5 and #6 are the same as #0 and #4 in joint world
    double tangAngleStart;
    double tangAngleEnd;
    Quat_Type EdgeQuat; //used only for edges of PTP movements
    double QuatAngle;
} Edge_Type;

typedef struct Spline_Type
{
    double LengthA;
    double LengthB;
    Frame_Type CtrlPoint[4];
} Spline_Type;

typedef struct Path_Type
{	
	double StartPointPath[6];
	double StartPointJoint[6];
	double StartPointAux[6];
	double MiddlePointPath[6];
	double MiddlePointJoint[6];
	double TargetPointPath[6];
	double TargetPointJoint[6];
	double TargetPointAux[6];
	double Radius;
	double Center[3];
	double Length;
	double MiddleLength;
	double Normal[3];
	double StartVersor[3];
	double CrossVersor[3];
	double MiddleVersor[3];
	double EndVersor[3];
	double RotAngle;
	Quat_Type StartQuat;
	Quat_Type MiddleQuat;
	Quat_Type EndQuat;
	double QuatAngle;	//from start to end
	double QuatAngle1;	//from start to middle
	double QuatAngle2;	//from middle to end
	double uM; //position of middle point (0..1)
	Edge_Type StartEdge;
    Edge_Type EndEdge;
    Spline_Type Spline;
} Path_Type;

typedef struct Label_Type
{
    char Name[MAX_BLOCK_SIZE+1];
    unsigned short Found;
    unsigned long Index;
    int Counter;
} Label_Type;

typedef struct Goto_Type
{	
    unsigned long Line; //line where goto or sub were called
    int Counter; //number of required iterations
    unsigned long Index; //line where the target label was found
} Goto_Type;

typedef struct MotionPackage_Type
{
    unsigned char MovementType;
    unsigned short TargetPoint;
    unsigned short CenterPoint;
    unsigned short Frame;
    unsigned short Tool;
    double Round;
    double MaxSpeed;
    unsigned char M_Index[MAX_MFUNC+1];
    unsigned char IO_Index;
    double DelayTime;
    double Feedrate;
    unsigned char FeedrateType;
    unsigned short Sub;
    struct Path_Type Path;
    struct RefPoint_Type RefPoint;
    unsigned long LineNumber;
    char BlockString[MAX_BLOCK_SIZE+1];
    Label_Type Label;
    double BlockLength;
    double BlockLengthIdeal;
    unsigned char Planned;
    double StartJointVector[6];
    double EndJointVector[6];
    double StartPathVector[3];
    double EndPathVector[3];
    double TransitionAngle;
    double EndSpeed;
    double EndAcc;	
    unsigned short TrkIndex;
    unsigned short TangCmd;
} MotionPackage_Type;

typedef struct Buffer_Type
{	struct MotionPackage_Type MotionPackage[BUFFER_LENGTH+1];
    unsigned char EXEC_Index; //buffer block being executed
    unsigned char EXEC_Index_Prev; //last executed block
    unsigned char PP_Index; //buffer block being planned
    unsigned char PP_Index_Prev; //last planned block
    unsigned char IP_Index; //buffer block being interpreted
    unsigned long IP_PrgCount; //program line being interpreted
    unsigned char IP_SubLevel; //level of subprogram (0=main program; 1=first subprogram level; ...)
    unsigned long IP_ReturnIdx[10]; //saved line numbers from subprogram calls
    unsigned char Eof;
    unsigned char Planned;
    unsigned char Synch; //used by synch M-functions to restart the halted PP 
    unsigned char IP_TrkIndex; //keeps note of what tracking index is currently active (IP-synch)
    double ModalFeedrate; //save feedrate as modal
    unsigned char ModalFeedrateType; //save also feedrate type as modal
    Goto_Type GotoBuffer[MAX_SUBLEVEL];
    Goto_Type SubBuffer[MAX_SUBLEVEL];
} Buffer_Type;

typedef struct Filter_Type
{
    double InputValue;
    unsigned char Enable;
    unsigned short Window;
    double OutputValue;
    double Buffer[1000];
    unsigned short Index;
}Filter_Type;

unsigned short LineFromString(char* s, char* line, unsigned long linenumber);
double RoundToEpsilon(double Value);	
void MovingAverageFilter(struct Filter_Type* inst);
void GaussianFilter(struct Filter_Type* inst);
unsigned short CheckConst();

#endif
