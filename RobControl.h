#ifndef ROB_CONTROL_H
#define ROB_CONTROL_H

/* Declaration of global constants, datatypes and functions for the RobControl library */

#define ERR_TRK2 1251
#define ERR_TRK1 1250
#define ERR_SPG_LIMIT_JERK 1208
#define ERR_SPG_LIMIT_ACC 1207
#define ERR_SPG_LIMIT_VEL 1206
#define ERR_SPG_LIMIT_POS 1205
#define ERR_SPG_DYNCALC 1204
#define ERR_SPG_LIMITS_REACHED 1203
#define ERR_SPG_OVERRIDE 1202
#define ERR_SPG_CYCLETIME 1201
#define ERR_SPG 1200
#define ERR_LIMIT_Q6 1186
#define ERR_LIMIT_Q5 1185
#define ERR_LIMIT_Q4 1184
#define ERR_LIMIT_Q3 1183
#define ERR_LIMIT_Q2 1182
#define ERR_LIMIT_Q1 1181
#define ERR_LIMIT_C 1176
#define ERR_LIMIT_B 1175
#define ERR_LIMIT_A 1174
#define ERR_LIMIT_Z 1173
#define ERR_LIMIT_Y 1172
#define ERR_LIMIT_X 1171
#define ERR_PP_CIRCLE_MIDDLEPOINT 1152
#define ERR_PP_CIRCLE_LENGTH 1151
#define ERR_PP_CIRCLEPOINTS 1150
#define ERR_IP_SUBLEVEL 1116
#define ERR_IP_JUMP 1115
#define ERR_IP_TRK_INDEX 1114
#define ERR_IP_LABEL 1113
#define ERR_IP_NOBLOCKS 1112
#define ERR_IP_FEEDRATE 1111
#define ERR_IP_MFUNCINDEX 1110
#define ERR_IP_FRAMEINDEX 1109
#define ERR_IP_TOOLINDEX 1108
#define ERR_IP_POINTINDEX 1107
#define ERR_IP_MAXMFUNC 1106
#define ERR_IP_NOCENTER 1105
#define ERR_IP_NOPOINT 1104
#define ERR_IP_CONFLICT 1103
#define ERR_IP_SYNTAX 1102
#define ERR_IP_COMMENT 1101
#define ERR_IP_EMPTYSTRING 1100
#define ERR_FILE_NOMEMORY 1054
#define ERR_FILE_END 1053
#define ERR_FILE_EMPTY 1052
#define ERR_FILE_NOT_FOUND 1051
#define ERR_NOT_SUPPORTED 1050
#define ERR_TRF_AXESNUM 1026
#define ERR_TRF_POINTER 1025
#define ERR_TRF_POSE 1024
#define ERR_TRF_MECH_NOT_SUPPORTED 1023
#define ERR_TRF_WORKSPACE 1022
#define ERR_TRF_MECH 1021
#define ERR_TRF_MODE 1020
#define ERR_POINT_TYPE 1016
#define ERR_UNITS_SCALING 1015
#define ERR_WRONG_JOINT_LIMITS 1014
#define ERR_JOG_GOTOPOS 1013
#define ERR_JOG_PAR 1012
#define ERR_ROBOT_LICENSE 1011
#define ERR_MAX_ROBOTS 1010
#define ERR_CYCLETIME 1005
#define ERR_DISABLED 99
#define TRF_POSE_FRONT 0
#define TRF_DIRECT 0
#define TRF_INVERSE 1
#define TRF_POSE_RIGHT 0
#define TRF_POSE_LEFT 1
#define TRF_POSE_CONVEX 0
#define TRF_POSE_CONCAVE 2
#define TRF_POSE_BACK 4
#define RING_BUFFER_SIZE 100
#define BUFFER_LENGTH 20
#define MAX_PRG_SIZE 10000
#define MAX_ROBOTS 8
#define MAX_MFUNC_INLINE 10
#define MAX_ERR 20
#define MAX_TOOL 20
#define MAX_FRAME 20
#define MAX_LOOP_COUNT 100
#define MAX_BLOCK_SIZE 100
#define MAX_POINT 200
#define MAX_MFUNC 100
#define USER 10
#define ARM 6
#define PALLETIZER 4
#define DELTA 3
#define JOG_GOTO 2
#define JOG_TOOL 2
#define POINT_JOINTS 0
#define JOG_PATHS 1
#define JOG_NEGATIVE 1
#define SCARA 1
#define POINT_PATH 1
#define JOG_POSITIVE 0
#define JOG_JOINTS 0
#define CNC 0
#define STATUS_OK 0


typedef enum Robot_Monitor_State_Type
{
	STANDSTILL_STATE = 0,
	JOGGING = 10,
	MOVING = 20,
	ERROR_STATE = 255
} Robot_Monitor_State_Type;

typedef struct Robot_Command_Type
{
	unsigned short RunProgram;
	unsigned short RunBlocks;
	unsigned short Stop;
	unsigned short Halt;
	unsigned short Continue;
	unsigned short JogAxis;
	unsigned short SetJoints;
	unsigned short Reset;
} Robot_Command_Type;

typedef struct Robot_Parameter_JointLimits_Type
{
	float PositionPos;
	float PositionNeg;
	float VelocityPos;
	float VelocityNeg;
	float AccelerationPos;
	float AccelerationNeg;
	float JerkPos;
	float JerkNeg;
} Robot_Parameter_JointLimits_Type;

typedef struct Robot_Parameter_PathLimits_Type
{
	float Velocity;
	float Acceleration;
	float Jerk;
} Robot_Parameter_PathLimits_Type;

typedef struct Robot_Parameter_Workspace_Type
{
	float PositionMax[6];
	float PositionMin[6];
} Robot_Parameter_Workspace_Type;

typedef struct Robot_Parameter_UnitsRatio_Type
{
	unsigned long MotorUnits;
	unsigned long AxisUnits;
	signed char Direction;
	signed long HomeOffset;
} Robot_Parameter_UnitsRatio_Type;

typedef struct Coord_Type
{
	float X;
	float Y;
	float Z;
} Coord_Type;

typedef struct Link_Type
{
	struct Coord_Type Offset;
	struct Coord_Type Rotation;
} Link_Type;

typedef struct UserTrf_Type
{
	unsigned char AxesNum;
	unsigned long Direct;
	unsigned long Inverse;
} UserTrf_Type;

typedef struct Mech_Type
{
	unsigned char Type;
	struct Link_Type Links[6];
	float Coupling[6];
	struct UserTrf_Type UserTrf;
} Mech_Type;

typedef struct Point_Type
{
	float Axes[6];
	unsigned char Mode;
} Point_Type;

typedef struct Robot_Jog_Type
{
	unsigned char Mode;
	unsigned char AxisIndex;
	unsigned char Direction;
	float Feedrate;
	float GotoPos;
} Robot_Jog_Type;

typedef struct Robot_Parameter_Conveyor_Type
{
	float Angle;
	float Position;
} Robot_Parameter_Conveyor_Type;

typedef struct Robot_Parameter_Type
{
	struct Robot_Parameter_JointLimits_Type JointLimits[6];
	struct Robot_Parameter_PathLimits_Type PathLimits;
	struct Robot_Parameter_Workspace_Type WorkspaceLimits;
	struct Robot_Parameter_UnitsRatio_Type UnitsRatio[6];
	struct Mech_Type Mechanics;
	float Override;
	unsigned char* Program;
	char Blocks[101][101];
	signed long ActFromDrives[6];
	unsigned long StartLine;
	struct Point_Type Points[201];
	struct Point_Type Tool[21];
	struct Point_Type Frame[21];
	struct Robot_Jog_Type Jog;
	struct Robot_Parameter_Conveyor_Type Conveyor[2];
	float PathCorrection[6];
	unsigned short M_synch[101];
	float CycleTime;
	float FilterTime;
	float MaxTransitionAngle;
	unsigned short SingleStep;
} Robot_Parameter_Type;

typedef struct Robot_Monitor_Type
{
	unsigned long Handle;
	unsigned char AxesNum;
	unsigned short Moving;
	unsigned short Halted;
	float PathSpeed;
	float PathPosition[6];
	float JointPosition[6];
	float JointSpeed[6];
	signed long SetToDrive[6];
	float MountBasePosition[6];
	float ToolBasePosition[6];
	unsigned long LineNumber;
	char CurrentBlock[101];
	float BlockLength;
	float CompletedBlockLength;
	unsigned char TargetPoint;
	unsigned short Tool;
	unsigned short Frame;
	unsigned short M[101];
	unsigned char TrackActive;
	unsigned char TrackSynch;
	unsigned short ActiveError;
	unsigned short ErrorLine;
	enum Robot_Monitor_State_Type State;
} Robot_Monitor_Type;

typedef struct Robot_Type
{
	struct Robot_Command_Type Commands;
	struct Robot_Parameter_Type Parameters;
	struct Robot_Monitor_Type Monitor;
} Robot_Type;



unsigned short RobotControl(struct Robot_Type* Robots, unsigned char RobotsNumber);


#endif



