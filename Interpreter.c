#include "interpreter.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "RobControl.h"
#include "Misc.h"


char *my_strcasestr(const char *arg1, const char *arg2)
{                  
    const char *a, *b;
                       
    for(;*arg1;*arg1++) {
                       
        a = arg1;
        b = arg2;
    
        while((*a++ | 32) == (*b++ | 32))
            if(!*b) 
                return (char*)arg1;
     
    }
     
    return NULL;
}

static float str2float(char* s)
{// converts string into float - does not support exponential notation - atoff does not work in AS C
	float result = 0;
	int i = 0;
	int sign = 1;
	short digits = 0; //turns true as soon as digits appear, after that no '-' or empty space can exist
	short decpoint = 0; //turns true when the decimal point is detected
	
	if (!strlen(s)) return(0.0f);

	int decimalPointLoc = strlen(s);

	for (;s[i]!='\0';i++)
	{
		if ((s[i] == ' ')&&(digits == 0)&&(sign == 1)) //any number of empty spaces allowed before sign and digits
		{// ignore empty spaces
			continue;
		}

		if ((s[i] == '-')&&(digits == 0)&&(sign == 1)) //only one negative sign allowed
		{// detect negative sign
			sign = -1;
			continue;
		}

		if ((s[i] == '.')&&(decpoint == 0)) //only one decimal point is allowed
		{// detect decimal point
			decpoint = 1;
			decimalPointLoc = i;
			continue;
		}

		if ((s[i]<'0')||(s[i]>'9')) break; //parsing the number is completed -> cannot use exponential notation!
		
		digits = 1; //digits detected -> no more negative sign or empty spaces allowed
		
		if (i < decimalPointLoc)
		{// integer part
			result *= 10.0f;
			result += (int)(s[i] - '0');
		}
		else
		{// fractional part
			result += (float)(s[i] - '0')*(pow(10,decimalPointLoc-i));
		}
		
	}
	return (result * sign);
}



/* converts a string block into a motion package */
unsigned short Interpreter(char* Block, MotionPackage_Type* Package)
{
	char *strMovement;
	char *strParameter;
	int i = 0;
	
	//save modal feedrate (if exists) before clearing package
	float ModalFeedrate = Package->Feedrate;
	float ModalFeedrateType = Package->FeedrateType;
	
	memset(Package,0,sizeof(MotionPackage_Type));
	Package->Feedrate = ModalFeedrate;
	Package->FeedrateType = ModalFeedrateType;
	
	/* look for comment sign */
	strMovement = strstr(Block,"//");
	if (strMovement != 0)
	{
		*strMovement = 0; //cut the Block string here
	}

	/* look for label sign */
	strMovement = strstr(Block,":");
	if (strMovement != 0)
	{
		return 0; //to prevent syntax error
	}

    /* look for GOTO jump */
    strMovement = my_strcasestr(Block,"GOTO ");
    if (strMovement != 0)
    {
        if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
        Package->MovementType = MOVE_GOTO;
        /* extract label string */
        strMovement+=5;
        while(*strMovement == ' ') strMovement++; //remove preceeding spaces
        char* end = strMovement + strlen(strMovement) - 1;
        while((end > strMovement) && (*end == ' ')) end--;
        *(end+1) = 0; //cut string here (to remove possible following empty spaces)
        strcpy(Package->Label.Name,strMovement);
        return 0; //to prevent conflicts with other commands
    }

    /* look for SUB jump */
    strMovement = my_strcasestr(Block,"SUB ");
    if (strMovement != 0)
    {
        if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
        Package->MovementType = MOVE_SUB;
        /* extract label string */
        strMovement+=4;
        while(*strMovement == ' ') strMovement++; //remove preceeding spaces
        char* end = strMovement + strlen(strMovement) - 1;
        while((end > strMovement) && (*end == ' ')) end--;
        *(end+1) = 0; //cut string here (to remove possible following empty spaces)
        strcpy(Package->Label.Name,strMovement);
        return 0; //to prevent conflicts with other commands
    }

    /* look for Move Line */
	strMovement = my_strcasestr(Block,"ML ");
	if (strMovement != 0)
	{
		if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
		Package->MovementType = MOVE_LINE;		
	}

	/* look for Move PTP */
	strMovement = my_strcasestr(Block,"MJ ");
	if (strMovement != 0)
	{
		if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
		Package->MovementType = MOVE_PTP;
	}

    /* look for Move Circle */
    strMovement = my_strcasestr(Block,"MC ");
    if (strMovement != 0)
    {
        if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
        Package->MovementType = MOVE_CIRCLE;		
    }

    /* look for Move Spline */
    strMovement = my_strcasestr(Block,"MS ");
    if (strMovement != 0)
    {
        if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
        Package->MovementType = MOVE_SPLINE;		
    }

    /* look for Move HOME */
	strMovement = my_strcasestr(Block,"HOME ");
	if (strMovement != 0)
	{
		if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
		Package->MovementType = MOVE_HOME;
	}

	/* look for target point */
	strParameter = my_strcasestr(Block,"P");
	if (strParameter != 0)
	{
		for(i=1;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if ((strParameter[i]<'0')||(strParameter[i]>'9'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoi to read them
		}
		Package->TargetPoint = atoi(strParameter+1);
		if (Package->TargetPoint >=MAX_POINT)
		{
			return ERR_IP_POINTINDEX;
		}
	}
	else if ((Package->MovementType == MOVE_LINE)||(Package->MovementType == MOVE_PTP)||(Package->MovementType == MOVE_CIRCLE)||(Package->MovementType == MOVE_SPLINE))
	{
		return ERR_IP_NOPOINT;
	}		

	/* look for center point */
	strParameter = my_strcasestr(Block,"Q");
	if (strParameter != 0)
	{
		for(i=1;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if ((strParameter[i]<'0')||(strParameter[i]>'9'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoi to read them
		}
		Package->CenterPoint = atoi(strParameter+1);
		if (Package->CenterPoint >=MAX_POINT)
		{
			return ERR_IP_POINTINDEX;
		}
	}
	else if (Package->MovementType == MOVE_CIRCLE)
	{
		return ERR_IP_NOCENTER;
	}

	/* look for reference positions (only valid for MJ movements) */
	strParameter = my_strcasestr(Block,"J1=");
	if (strParameter != 0)
	{
		if (Package->MovementType != MOVE_PTP) return ERR_IP_CONFLICT;
		for(i=3;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.')&&(strParameter[i]!='-'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->RefPoint.Defined[0] = 1;
		Package->RefPoint.Axes[0] = str2float(strParameter+3);
	}
	strParameter = my_strcasestr(Block,"J2=");
	if (strParameter != 0)
	{
		if (Package->MovementType != MOVE_PTP) return ERR_IP_CONFLICT;
		for(i=3;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.')&&(strParameter[i]!='-'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->RefPoint.Defined[1] = 1;
		Package->RefPoint.Axes[1] = str2float(strParameter+3);
	}
	strParameter = my_strcasestr(Block,"J3=");
	if (strParameter != 0)
	{
		if (Package->MovementType != MOVE_PTP) return ERR_IP_CONFLICT;
		for(i=3;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.')&&(strParameter[i]!='-'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->RefPoint.Defined[2] = 1;
		Package->RefPoint.Axes[2] = str2float(strParameter+3);
	}
	strParameter = my_strcasestr(Block,"J4=");
	if (strParameter != 0)
	{
		if (Package->MovementType != MOVE_PTP) return ERR_IP_CONFLICT;
		for(i=3;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.')&&(strParameter[i]!='-'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->RefPoint.Defined[3] = 1;
		Package->RefPoint.Axes[3] = str2float(strParameter+3);
	}
	strParameter = my_strcasestr(Block,"J5=");
	if (strParameter != 0)
	{
		if (Package->MovementType != MOVE_PTP) return ERR_IP_CONFLICT;
		for(i=3;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.')&&(strParameter[i]!='-'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->RefPoint.Defined[4] = 1;
		Package->RefPoint.Axes[4] = str2float(strParameter+3);
	}
	strParameter = my_strcasestr(Block,"J6=");
	if (strParameter != 0)
	{
		if (Package->MovementType != MOVE_PTP) return ERR_IP_CONFLICT;
		for(i=3;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.')&&(strParameter[i]!='-'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->RefPoint.Defined[5] = 1;
		Package->RefPoint.Axes[5] = str2float(strParameter+3);
	}

	
	/* look for rotation angle */
	strParameter = my_strcasestr(Block,"H");
	if ((strParameter != 0)&&(Package->MovementType != MOVE_HOME))
	{
		for(i=1;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.')&&(strParameter[i]!='-'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->Path.RotAngle = str2float(strParameter+1);
	}
	
	/* look for feedrate */
	strParameter = my_strcasestr(Block,"F");
	if (strParameter != 0)
	{
		unsigned char offset;
		if (strParameter[1]=='C')	//cartesian speed definition
		{
			Package->FeedrateType = FEED_CART;
			offset = 2;
		}
		else if (strParameter[1]=='A')	//angular speed definition
		{
			Package->FeedrateType = FEED_ANG;
			offset = 2;
		}
		else //default speed definition
		{
			Package->FeedrateType = FEED_DEFAULT;
			offset = 1;
		}
		
		for(i=offset;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&((strParameter[i]!='.')))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->Feedrate = str2float(strParameter+offset);	
		if (Package->Feedrate <=0)
		{
			return ERR_IP_FEEDRATE;
		}
	}
	else if ( ((Package->MovementType == MOVE_LINE)||(Package->MovementType == MOVE_PTP)||(Package->MovementType == MOVE_CIRCLE)||(Package->MovementType == MOVE_HOME)) &&(Package->Feedrate <= 0))
	{
		return ERR_IP_FEEDRATE;
	}		

	/* look for Delay time */
	strMovement = my_strcasestr(Block,"WAIT");
	if (strMovement != 0)
	{
		if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
		Package->MovementType = MOVE_DELAY;
		for(i=4;strMovement[i]!='\0';i++)
		{
			if (strMovement[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strMovement[i]<'0')||(strMovement[i]>'9'))&&((strMovement[i]!='.')))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->DelayTime = str2float(strMovement+4);
	}

	/* look for Tracking TRK */
	strMovement = my_strcasestr(Block,"TRK");
	if (strMovement != 0)
	{
		if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
		Package->MovementType = MOVE_TRK;
		for(i=3;strMovement[i]!='\0';i++)
		{
			if (strMovement[i]==' ')
				continue;	//ignore emtpy spaces
			if ((strMovement[i]<'0')||(strMovement[i]>'9'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoi to read them
		}
		Package->TrkIndex = atoi(strMovement+3);
		if ((Package->TrkIndex > 2)||(strMovement[i]=='\0')) //either wrong or no index given
		{
			return ERR_IP_TRK_INDEX;
		}
	}
	
	/* look for tool */
	strParameter = my_strcasestr(Block,"T");
	if ((strParameter != 0)&&(Package->MovementType != MOVE_DELAY)&&(Package->MovementType != MOVE_GOTO)&&(Package->MovementType != MOVE_TRK)) //to avoid conflicts with "WAIT", "GOTO", "TRK"
	{
		for(i=1;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if ((strParameter[i]<'0')||(strParameter[i]>'9'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoi to read them
		}
		Package->Tool = atoi(strParameter+1);
		if (Package->Tool >=MAX_TOOL)
		{
			return ERR_IP_TOOLINDEX;
		}
	}

	/* look for frame */
	strParameter = my_strcasestr(Block,"Z");
	if (strParameter != 0)
	{
		for(i=1;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if ((strParameter[i]<'0')||(strParameter[i]>'9'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoi to read them
		}
		Package->Frame = atoi(strParameter+1);
		if (Package->Frame >=MAX_FRAME)
		{
			return ERR_IP_FRAMEINDEX;
		}
	}

	/* look for round edge */
	Package->Round = -1; //round edge not defined
	strParameter = my_strcasestr(Block,"R");
	if ((strParameter != 0)&&(Package->MovementType != MOVE_TRK)) //to avoid conflicts with "TRK"
	{
		for(i=1;strParameter[i]!='\0';i++)
		{
			if (strParameter[i]==' ')
				continue;	//ignore emtpy spaces
			if (((strParameter[i]<'0')||(strParameter[i]>'9'))&&(strParameter[i]!='.'))
				return ERR_IP_SYNTAX;	//only digits are allowed
			else
				break;	//digits found -> use atoff to read them
		}
		Package->Round = str2float(strParameter+1);
	}

	/* look for End of program */
	strMovement = my_strcasestr(Block,"END");
	if (strMovement != 0)
	{
		if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
		Package->MovementType = MOVE_END;		
	}	
	
	/* look for M functions - max 10 can be on a single line */
	int M_count = 0;
	strMovement = my_strcasestr(Block,"M");
	while (strMovement != 0)
	{
		int index = atoi(strMovement+1); //check that it is an M-function
		if(index > 0) //new M function found - no negative or zero values accepted
		{
			if ((Package->MovementType != MOVE_UNDEF)&&(Package->MovementType != MOVE_MCODE)) return ERR_IP_CONFLICT;
			if (M_count >= MAX_MFUNC_INLINE) return ERR_IP_MAXMFUNC;
			if (index>=MAX_MFUNC) return ERR_IP_MFUNCINDEX;
			Package->MovementType = MOVE_MCODE;
			Package->M_Index[M_count] = index;
			M_count++;
		}
		//else return ERR_IP_MFUNCINDEX; //cannot use this line because atoi(ML) returns 0!
		strMovement++;
		strMovement = my_strcasestr(strMovement,"M");		
	}

	return STATUS_OK;
	
}


