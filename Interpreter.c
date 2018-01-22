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

static double str2double(char* s)
{// converts string into double - does not support exponential notation - atoff does not work in AS C
	double result = 0;
	int i = 0;
	int sign = 1;
	short digits = 0; //turns true as soon as digits appear, after that no '-' or empty space can exist
	short decpoint = 0; //turns true when the decimal point is detected
	
	if (!strlen(s)) return(0.0);

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
			result *= 10.0;
			result += (int)(s[i] - '0');
		}
		else
		{// fractional part
			result += (double)(s[i] - '0')*(pow(10,decimalPointLoc-i));
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
	double ModalFeedrate = Package->Feedrate;
	double ModalFeedrateType = Package->FeedrateType;
	
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
        
        for(i=1;strMovement[i]!='\0' && strMovement[i]!=' ';i++) {}
        char *tmpStr = strMovement;
        tmpStr[i]=0;
        strcpy(Package->Label.Name,tmpStr);
  
        char *tmpIndex = strMovement + i;

        for(i=1;tmpIndex[i]!='\0';i++)
        {
            if (tmpIndex[i]==' ')
                continue;	//ignore emtpy spaces
            if ((tmpIndex[i]<'1')||(tmpIndex[i]>'9'))
                return ERR_IP_SYNTAX;	//only digits are allowed
            else
                break;	//digits found -> use atoi to read them
        }
        Package->Label.Counter = atoi(tmpIndex+1);

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
        
        for(i=1;strMovement[i]!='\0' && strMovement[i]!=' ';i++) {}
        char *tmpStr = strMovement;
        tmpStr[i]=0;
        strcpy(Package->Label.Name,tmpStr);
  
        char *tmpIndex = strMovement + i;

        for(i=1;tmpIndex[i]!='\0';i++)
        {
            if (tmpIndex[i]==' ')
                continue;	//ignore emtpy spaces
            if ((tmpIndex[i]<'1')||(tmpIndex[i]>'9'))
                return ERR_IP_SYNTAX;	//only digits are allowed
            else
                break;	//digits found -> use atoi to read them
        }
        Package->Label.Counter = atoi(tmpIndex+1);

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
        if (strParameter[i]==0)
        {
            return ERR_IP_POINTINDEX;
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
        if (strParameter[i]==0)
        {
            return ERR_IP_POINTINDEX;
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
        if (strParameter[i]==0)
        {//end of line - no position programmed after Ji=
            return ERR_IP_SYNTAX;
        }
		Package->RefPoint.Defined[0] = 1;
		Package->RefPoint.Axes[0] = str2double(strParameter+3);
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
        if (strParameter[i]==0)
        {//end of line - no position programmed after Ji=
            return ERR_IP_SYNTAX;
        }
        Package->RefPoint.Defined[1] = 1;
		Package->RefPoint.Axes[1] = str2double(strParameter+3);
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
        if (strParameter[i]==0)
        {//end of line - no position programmed after Ji=
            return ERR_IP_SYNTAX;
        }
        Package->RefPoint.Defined[2] = 1;
		Package->RefPoint.Axes[2] = str2double(strParameter+3);
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
        if (strParameter[i]==0)
        {//end of line - no position programmed after Ji=
            return ERR_IP_SYNTAX;
        }
        Package->RefPoint.Defined[3] = 1;
		Package->RefPoint.Axes[3] = str2double(strParameter+3);
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
        if (strParameter[i]==0)
        {//end of line - no position programmed after Ji=
            return ERR_IP_SYNTAX;
        }
        Package->RefPoint.Defined[4] = 1;
		Package->RefPoint.Axes[4] = str2double(strParameter+3);
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
        if (strParameter[i]==0)
        {//end of line - no position programmed after Ji=
            return ERR_IP_SYNTAX;
        }
        Package->RefPoint.Defined[5] = 1;
		Package->RefPoint.Axes[5] = str2double(strParameter+3);
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
		Package->Feedrate = str2double(strParameter+offset);	
		if (Package->Feedrate <=0)
		{
			return ERR_IP_FEEDRATE;
		}
	}
	else if ( ((Package->MovementType == MOVE_LINE)||(Package->MovementType == MOVE_PTP)||(Package->MovementType == MOVE_CIRCLE)||(Package->MovementType == MOVE_HOME)) &&(Package->Feedrate <= 0))
	{
		return ERR_IP_FEEDRATE;
	}		

	/* look for Delay time or for Signal input */
	strMovement = my_strcasestr(Block,"WAIT");
	if (strMovement != 0)
	{
		if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
		
		for(i=4;strMovement[i]!='\0';i++)
		{
			if (strMovement[i]==' ')
				continue;	//ignore emtpy spaces
            if ((strMovement[i]>='0' && strMovement[i]<='9')|| strMovement[i]=='.')
            {//digits found -> wait for time
                Package->MovementType = MOVE_DELAY;
                break;
            }
            else if (strMovement[i]=='D' && strMovement[i+1]=='I')
            {//DI found -> wait for signal
                Package->MovementType = MOVE_WAITDI;
                break;                
            }
			else
				return ERR_IP_SYNTAX;
		}
        
        if (Package->MovementType == MOVE_DELAY)
        {
            if (strMovement[i]==0) return ERR_IP_SYNTAX; //end of line - no delay time programmed
            Package->DelayTime = str2double(strMovement+i-1);
        }
        else
        {
            if (strMovement[i+2]==0) return ERR_IP_SYNTAX; //end of line - no delay time programmed
            Package->IO_Index = atoi(strMovement+i+2);
        }
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
	
    /* look for Tangential TANG */
    strMovement = my_strcasestr(Block,"TANG");
    if (strMovement != 0)
    {
        if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
        Package->MovementType = MOVE_TANG;
        for(i=4;strMovement[i]!='\0';i++)
        {
            if (strMovement[i]==' ')
                continue;	//ignore emtpy spaces
            if ((strMovement[i]<'0')||(strMovement[i]>'1'))
                return ERR_IP_SYNTAX;	//only 0-1 are allowed
            else
                break;	//0-1 found -> use atoi to read them
        }
        Package->TangCmd = atoi(strMovement+4);
        if (strMovement[i]=='\0') //tangential command not complete
        {
            return ERR_IP_TANG;
        }
    }

    /* look for RESET */
    strMovement = my_strcasestr(Block,"RESET");
    if (strMovement != 0)
    {
        if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
        Package->MovementType = MOVE_RESETDO;
        /*
        for(i=8;strMovement[i]!='\0';i++)
        {
            if (strMovement[i]==' ')
                continue;	//ignore emtpy spaces
            if ((strMovement[i]<'0')||(strMovement[i]>'9'))
                return ERR_IP_SYNTAX;	//only digits are allowed
            else
                break;	//digits found -> use atoi to read them
        }
        Package->IO_Index = atoi(strMovement+8);
        if (strMovement[i]=='\0') //IO index not provided
        {
            return ERR_IP_IO_INDEX;
        }
        if (Package->IO_Index > MAX_IO) return ERR_IP_IO_INDEX;
        */
    }

    /* look for SET */
    strMovement = my_strcasestr(Block,"SET");
    if ((strMovement != 0)&&(Package->MovementType != MOVE_RESETDO)) //avoid conflict with reset_do
    {
        if (Package->MovementType != MOVE_UNDEF) return ERR_IP_CONFLICT;
        Package->MovementType = MOVE_SETDO;
        /*
        for(i=6;strMovement[i]!='\0';i++)
        {
            if (strMovement[i]==' ')
                continue;	//ignore emtpy spaces
            if ((strMovement[i]<'0')||(strMovement[i]>'9'))
                return ERR_IP_SYNTAX;	//only digits are allowed
            else
                break;	//digits found -> use atoi to read them
        }
        Package->IO_Index = atoi(strMovement+6);
        if (strMovement[i]=='\0') //IO index not provided
        {
            return ERR_IP_IO_INDEX;
        }
        if (Package->IO_Index > MAX_IO) return ERR_IP_IO_INDEX;
        */
    }
  
    /* look for target DO point */
    strParameter = my_strcasestr(Block,"DO");
    if (strParameter != 0)
    {
        for(i=2;strParameter[i]!='\0';i++)
        {
            if (strParameter[i]==' ')
                continue;	//ignore emtpy spaces
            if ((strParameter[i]<'0')||(strParameter[i]>'9'))
                return ERR_IP_SYNTAX;	//only digits are allowed
            else
                break;	//digits found -> use atoi to read them
        }
        if (strParameter[i]==0)
        {
            return ERR_IP_IO_INDEX;
        }
        Package->IO_Index = atoi(strParameter+2);
        if (Package->IO_Index > MAX_IO) return ERR_IP_IO_INDEX;
    }
    else if ((Package->MovementType == MOVE_RESETDO)||(Package->MovementType == MOVE_SETDO))
    {
        return ERR_IP_IO_INDEX;
    }
    
    /* look for rotation angle (or tangential offset) */
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
        if (strParameter[i]==0)
        {//end of line - no value programmed after H
            return ERR_IP_SYNTAX;
        }
        Package->Path.RotAngle = str2double(strParameter+1);
    }
	
    /* look for tool */
	strParameter = my_strcasestr(Block,"T");
	if ((strParameter != 0)&&(Package->MovementType != MOVE_DELAY)&&(Package->MovementType != MOVE_GOTO)&&(Package->MovementType != MOVE_TRK)&&(Package->MovementType != MOVE_TANG)&&(Package->MovementType != MOVE_RESETDO)&&(Package->MovementType != MOVE_SETDO)&&(Package->MovementType != MOVE_WAITDI)) //to avoid conflicts with "T" in other commands
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
        if (strParameter[i]==0)
        {//end of line - no value programmed after T
            return ERR_IP_TOOLINDEX;
        }
		Package->Tool = atoi(strParameter+1);
		if (Package->Tool >=MAX_TOOL)
		{
			return ERR_IP_TOOLINDEX;
		}
        if (Package->MovementType == MOVE_UNDEF)
        {
            Package->MovementType = MOVE_TOOL;
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
        if (strParameter[i]==0)
        {//end of line - no value programmed after Z
            return ERR_IP_FRAMEINDEX;
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
	if ((strParameter != 0)&&(Package->MovementType != MOVE_TRK)&&(Package->MovementType != MOVE_RESETDO)) //to avoid conflicts with "R" in other commands
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
        if (strParameter[i]==0)
        {//end of line - no value programmed after R
            return ERR_IP_SYNTAX;
        }
		Package->Round = str2double(strParameter+1);
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


