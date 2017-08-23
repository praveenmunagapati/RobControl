#ifndef _MISC_TYPES
#define _MISC_TYPES

#include "RobControl.h"
#include "Frame.h"
#include <string.h>

/* declare miscellaneous datatypes and functions */

typedef struct RefPoint_Type
{	
	float Axes[6];
	unsigned char Defined[6];
} RefPoint_Type;

typedef struct Edge_Type
{
    float Radius;
    float Length;
    Point_Type CtrlPoint[5];
    float tangAngleStart;
    float tangAngleEnd;
} Edge_Type;

typedef struct Spline_Type
{
    float LengthA;
    float LengthB;
    Point_Type CtrlPoint[4];
} Spline_Type;

typedef struct Path_Type
{	
	float StartPointPath[6];
	float StartPointJoint[6];
	float MiddlePointPath[6];
	float MiddlePointJoint[6];
	float TargetPointPath[6];
	float TargetPointJoint[6];
	float Radius;
	float Center[3];
	float Length;
	float MiddleLength;
	float Normal[3];
	float StartVersor[3];
	float CrossVersor[3];
	float MiddleVersor[3];
	float EndVersor[3];
	float RotAngle;
	Quat_Type StartQuat;
	Quat_Type MiddleQuat;
	Quat_Type EndQuat;
	float QuatAngle;	//from start to end
	float QuatAngle1;	//from start to middle
	float QuatAngle2;	//from middle to end
	float uM; //position of middle point (0..1)
	Edge_Type StartEdge;
    Edge_Type EndEdge;
    Spline_Type Spline;
} Path_Type;

typedef struct Label_Type
{
	char Name[MAX_BLOCK_SIZE+1];
	unsigned short Found;
	unsigned long Index;
} Label_Type;

typedef struct MotionPackage_Type
{
    unsigned char MovementType;
    unsigned short TargetPoint;
    unsigned short CenterPoint;
    unsigned short Frame;
    unsigned short Tool;
    float Round;
    float MaxSpeed;
    unsigned char M_Index[MAX_MFUNC+1];
    float DelayTime;
    float Feedrate;
    unsigned char FeedrateType;
    unsigned short Sub;
    struct Path_Type Path;
    struct RefPoint_Type RefPoint;
    unsigned long LineNumber;
    char BlockString[MAX_BLOCK_SIZE+1];
    Label_Type Label;
    float BlockLength;
    float BlockLengthIdeal;
    unsigned char Planned;
    float StartJointVector[6];
    float EndJointVector[6];
    float StartPathVector[3];
    float EndPathVector[3];
    float TransitionAngle;
    float EndSpeed;
    float EndAcc;	
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
    float ModalFeedrate; //save feedrate as modal
    unsigned char ModalFeedrateType; //save also feedrate type as modal
} Buffer_Type;

typedef struct Filter_Type
{
    float InputValue;
    unsigned char Enable;
    unsigned short Window;
    float OutputValue;
    float Buffer[1000];
    unsigned short Index;
}Filter_Type;

unsigned short LineFromString(char* s, char* line, unsigned long linenumber);
float RoundToEpsilon(float Value);	
void MovingAverageFilter(struct Filter_Type* inst);
void GaussianFilter(struct Filter_Type* inst);
unsigned short CheckConst();

#endif
