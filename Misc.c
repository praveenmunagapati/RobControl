#include "Misc.h"
#include "RobControl.h"
#include "Frame.h"
#include <string.h>

unsigned short LineFromString(char* s, char* line, unsigned long linenumber)
{ //extract line from string
		
	memset(line,0,MAX_BLOCK_SIZE);
	
	unsigned long i=1;
	unsigned short linesize;
	char* tmp = s;
	
	while (i<linenumber)
	{
		if ((*tmp == 13)&&(*(tmp+1) == 10)) //carriage return + line feed
		{
			i++;
			tmp+=2;
		}
		else
		{
			tmp++;	
		}
		if(*tmp == 0)	return ERR_FILE_END;	//end of string reached
	}
	
	linesize = 0;
	char* start = tmp;
	while(((*tmp != 13)||(*(tmp+1) != 10))&&(linesize<MAX_BLOCK_SIZE)) //reached carriage return + line feed or max block length
	{
		tmp++;
		linesize++;
	}
	memcpy(line,start,linesize);
	
	return STATUS_OK;
}

double RoundToEpsilon(double Value)
{
	int nValue = round(Value);
	if (fabs(Value - nValue) < TRF_EPSILON)
		return nValue;
	else
		return Value;
}

void MovingAverageFilter(struct Filter_Type* inst)
{
	double Sum = 0;
	int i;

	if (!inst->Enable)
	{
		for(i=0;i<100;i++)
		{//flood array with input value so that restart (Enable=1) does not jump
			inst->Buffer[i] = inst->InputValue;	
		}
		inst->Index = 0;
		inst->OutputValue = inst->InputValue;
		return;
	}
	
	//automatically limit window size
	if (inst->Window > 100)
		inst->Window = 100;
	if (inst->Window <= 0)
		inst->Window = 1;
	
	//insert new element in buffer
	inst->Index++;
	if (inst->Index>=inst->Window)
		inst->Index = 0;
	inst->Buffer[inst->Index] = inst->InputValue;
	
	for(i=0;i<inst->Window;i++)
	{
		Sum += inst->Buffer[i];	
	}
	
	inst->OutputValue = Sum / (double) inst->Window;
	
}

void GaussianFilter(struct Filter_Type* inst)
{
	double Sum = 0;
	double GSum = 0;
	int i,idx;
	
	if (!inst->Enable)
	{
		for(i=0;i<1000;i++)
		{//flood array with input value so that restart (Enable=1) does not jump
			inst->Buffer[i] = inst->InputValue;	
		}
		inst->Index = 0;
		inst->OutputValue = inst->InputValue;
		return;
	}
	
	//automatically limit window size
	if (inst->Window > 1000)
		inst->Window = 1000;
	if (inst->Window <= 0)
		inst->Window = 1;
	
	//insert new element in buffer
	inst->Index++;
	if (inst->Index>=inst->Window)
		inst->Index = 0;
	inst->Buffer[inst->Index] = inst->InputValue;

	for(i=0;i<inst->Window;i+=((inst->Window+99)/100))	//decrease resolution to speed up calculation
	{
		double tmpVal = (i-inst->Window/2.0)/(0.4*inst->Window/2.0);
		double Gauss = exp(-0.5 * tmpVal*tmpVal);		
		GSum += Gauss;
		
		//start counting signal from first element inserted in time
		idx = i+inst->Index+1;
		if (idx>=inst->Window)
		idx -= inst->Window;

		Sum += (inst->Buffer[idx] * Gauss);	
		
	}
	
	//normalize output
	inst->OutputValue = Sum/GSum;
		
	
}


unsigned short CheckConst()
{
    //check all constants
    unsigned short check = 1;
    
    check *=  (STATUS_OK == 0);
    check *=  (CNC == 0);
    check *=  (JOG_JOINTS == 0);
    check *=  (JOG_POSITIVE == 0);
    check *=  (POINT_PATH == 1);
    check *=  (SCARA == 1);
    check *=  (JOG_NEGATIVE == 1);
    check *=  (JOG_BASE == 1);
    check *=  (POINT_JOINTS == 0);
    check *=  (JOG_TOOL == 2);
	check *=  (JOG_GOTO == 2);
	check *=  (JOG_AUX == 3);
	check *=  (ZONE_DISABLED == 0);
    check *=  (ZONE_SAFE == 1);
    check *=  (ZONE_FORBIDDEN == 2);
    check *=  (DELTA == 3);
    check *=  (PALLETIZER == 4);
    check *=  (RTCP == 5);
    check *=  (ARM == 6);
    check *=  (USER == 10);
    check *=  (MAX_IO == 100);
    check *=  (MAX_MFUNC == 100);
    check *=  (MAX_POINT == 200);
    check *=  (MAX_BLOCK_SIZE == 100);
    check *=  (MAX_LOOP_COUNT == 100);
    check *=  (MAX_FRAME == 20);
    check *=  (MAX_TOOL == 20);
    check *=  (MAX_ERR == 20);
    check *=  (MAX_MFUNC_INLINE == 10);
    check *=  (MAX_ROBOTS == 8);
    check *=  (MAX_PRG_SIZE == 10000);
    check *=  (MAX_ZONE == 10);
    check *=  (BUFFER_LENGTH == 20);
    check *=  (RING_BUFFER_SIZE == 100);
    check *=  (TRF_POSE_BACK == 4);
    check *=  (TRF_POSE_CONCAVE == 2);
    check *=  (TRF_POSE_CONVEX == 0);
    check *=  (TRF_POSE_LEFT == 1);
    check *=  (TRF_POSE_RIGHT == 0);
    check *=  (TRF_INVERSE == 1);
    check *=  (TRF_DIRECT == 0);
    check *=  (TRF_POSE_FRONT == 0);
    check *=  (ERR_DISABLED == 99);
    check *=  (ERR_CYCLETIME == 1005);
    check *=  (ERR_CHECKSUM == 1006);
    check *=  (ERR_MAX_ROBOTS == 1010);
    check *=  (ERR_ROBOT_LICENSE == 1011);
    check *=  (ERR_JOG_PAR == 1012);
    check *=  (ERR_JOG_GOTOPOS == 1013);
    check *=  (ERR_WRONG_JOINT_LIMITS == 1014);
    check *=  (ERR_UNITS_SCALING == 1015);
    check *=  (ERR_POINT_TYPE == 1016);
	check *=  (ERR_WRONG_AUX_LIMITS == 1017);
    check *=  (ERR_TRF_MODE == 1020);
    check *=  (ERR_TRF_MECH == 1021);
    check *=  (ERR_TRF_WORKSPACE == 1022);
    check *=  (ERR_TRF_MECH_NOT_SUPPORTED == 1023);
    check *=  (ERR_TRF_POSE == 1024);
    check *=  (ERR_TRF_POINTER == 1025);
    check *=  (ERR_TRF_AXESNUM == 1026);
    check *=  (ERR_TRF_ROT == 1027);
    check *=  (ERR_CALIBRATION == 1030);
    check *=  (ERR_NOT_SUPPORTED == 1050);
    check *=  (ERR_FILE_NOT_FOUND == 1051);
    check *=  (ERR_FILE_EMPTY == 1052);
    check *=  (ERR_FILE_END == 1053);
    check *=  (ERR_FILE_NOMEMORY == 1054);
    check *=  (ERR_IP_EMPTYSTRING == 1100);
    check *=  (ERR_IP_COMMENT == 1101);
    check *=  (ERR_IP_SYNTAX == 1102);
    check *=  (ERR_IP_CONFLICT == 1103);
    check *=  (ERR_IP_NOPOINT == 1104);
    check *=  (ERR_IP_NOCENTER == 1105);
    check *=  (ERR_IP_MAXMFUNC == 1106);
    check *=  (ERR_IP_POINTINDEX == 1107);
    check *=  (ERR_IP_TOOLINDEX == 1108);
    check *=  (ERR_IP_FRAMEINDEX == 1109);
    check *=  (ERR_IP_MFUNCINDEX == 1110);
    check *=  (ERR_IP_FEEDRATE == 1111);
    check *=  (ERR_IP_NOBLOCKS == 1112);
    check *=  (ERR_IP_LABEL == 1113);
    check *=  (ERR_IP_TRK_INDEX == 1114);
    check *=  (ERR_IP_JUMP == 1115);
    check *=  (ERR_IP_SUBLEVEL == 1116);
    check *=  (ERR_IP_TANG == 1117);
    check *=  (ERR_IP_IO_INDEX == 1118);
    check *=  (ERR_PP_CIRCLEPOINTS == 1150);
    check *=  (ERR_PP_CIRCLE_LENGTH == 1151);
    check *=  (ERR_PP_CIRCLE_MIDDLEPOINT == 1152);
    check *=  (ERR_WORKSPACE_ZONE1 == 1160);
    check *=  (ERR_WORKSPACE_ZONE2 == 1161);
    check *=  (ERR_WORKSPACE_ZONE3 == 1162);
    check *=  (ERR_WORKSPACE_ZONE4 == 1163);
    check *=  (ERR_WORKSPACE_ZONE5 == 1164);
    check *=  (ERR_WORKSPACE_ZONE6 == 1165);
    check *=  (ERR_WORKSPACE_ZONE7 == 1166);
    check *=  (ERR_WORKSPACE_ZONE8 == 1167);
    check *=  (ERR_WORKSPACE_ZONE9 == 1168);
    check *=  (ERR_WORKSPACE_ZONE10 == 1169);
    check *=  (ERR_LIMIT_X == 1171);
    check *=  (ERR_LIMIT_Y == 1172);
    check *=  (ERR_LIMIT_Z == 1173);
    check *=  (ERR_LIMIT_A == 1174);
    check *=  (ERR_LIMIT_B == 1175);
    check *=  (ERR_LIMIT_C == 1176);
    check *=  (ERR_LIMIT_J1 == 1181);
    check *=  (ERR_LIMIT_J2 == 1182);
    check *=  (ERR_LIMIT_J3 == 1183);
    check *=  (ERR_LIMIT_J4 == 1184);
    check *=  (ERR_LIMIT_J5 == 1185);
    check *=  (ERR_LIMIT_J6 == 1186);
    check *=  (ERR_LIMIT_VEL_J1 == 1187);
    check *=  (ERR_LIMIT_VEL_J2 == 1188);
    check *=  (ERR_LIMIT_VEL_J3 == 1189);
    check *=  (ERR_LIMIT_VEL_J4 == 1190);
    check *=  (ERR_LIMIT_VEL_J5 == 1191);
    check *=  (ERR_LIMIT_VEL_J6 == 1192);
    check *=  (ERR_SPG == 1200);
    check *=  (ERR_SPG_CYCLETIME == 1201);
    check *=  (ERR_SPG_OVERRIDE == 1202);
    check *=  (ERR_SPG_LIMITS_REACHED == 1203);
    check *=  (ERR_SPG_DYNCALC == 1204);
    check *=  (ERR_SPG_LIMIT_POS == 1205);
    check *=  (ERR_SPG_LIMIT_VEL == 1206);
    check *=  (ERR_SPG_LIMIT_ACC == 1207);
    check *=  (ERR_SPG_LIMIT_JERK == 1208);
	check *=  (ERR_LIMIT_A1 == 1220);
	check *=  (ERR_LIMIT_A2 == 1221);
	check *=  (ERR_LIMIT_A3 == 1222);
	check *=  (ERR_LIMIT_A4 == 1223);
	check *=  (ERR_LIMIT_A5 == 1224);
	check *=  (ERR_LIMIT_A6 == 1225);
	check *=  (ERR_LIMIT_VEL_A1 == 1230);
	check *=  (ERR_LIMIT_VEL_A2 == 1231);
	check *=  (ERR_LIMIT_VEL_A3 == 1232);
	check *=  (ERR_LIMIT_VEL_A4 == 1233);
	check *=  (ERR_LIMIT_VEL_A5 == 1234);
	check *=  (ERR_LIMIT_VEL_A6 == 1235);
    check *=  (ERR_TRK1 == 1250);
    check *=  (ERR_TRK2 == 1251);
    check *=  (ERR_OPTMOT == 1260);

    return check;
    
}
