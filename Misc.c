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

float RoundToEpsilon(float Value)
{
	int nValue = round(Value);
	if (fabs(Value - nValue) < TRF_EPSILON)
		return nValue;
	else
		return Value;
}

void MovingAverageFilter(struct Filter_Type* inst)
{
	float Sum = 0;
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
	
	inst->OutputValue = Sum / (float) inst->Window;
	
}

void GaussianFilter(struct Filter_Type* inst)
{
	float Sum = 0;
	float GSum = 0;
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
		float tmpVal = (i-inst->Window/2.0f)/(0.4f*inst->Window/2.0f);
		float Gauss = exp(-0.5f * tmpVal*tmpVal);		
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

