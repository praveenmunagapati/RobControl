#include "Transformations.h"
#include <math.h>
#include "Robots.h"
#include "Misc.h"

unsigned short Transformations(struct Mech_Type* Mechanics, unsigned char Mode, double JointAxes[6], double PathAxes[6], double Axes[6])
{
	
    unsigned short Status;
    unsigned short (*UserDirect)(Link_Type[6],double[6],double[6],double[6]) = Mechanics->UserTrf.Direct;
    unsigned short (*UserInverse)(Link_Type[6],double[6],double[6],double[6]) = Mechanics->UserTrf.Inverse;
	
	if (Mode == TRF_DIRECT)	//direct transformations -> calculate path axes from joint axes
	{
				
		switch (Mechanics->Type)
		{
			case CNC:
				Axes[0] = JointAxes[0];
				Axes[1] = JointAxes[1];
				Axes[2] = JointAxes[2];
				//reset unused output values
				Axes[3] = 0;
				Axes[4] = 0;
				Axes[5] = 0;
				Status = STATUS_OK;
				break;
		
			case SCARA:
				Status = ScaraDirect(Mechanics->Links,JointAxes,PathAxes,Axes);
				//reset unused output values
				Axes[4] = 0;
				Axes[5] = 0;
				break;
		
			case DELTA:
				Status = DeltaDirect(Mechanics->Links,JointAxes,PathAxes,Axes);
				//reset unused output values
				Axes[4] = 0;
				Axes[5] = 0;
				break;
		
			case PALLETIZER:
				Status = PalletDirect(Mechanics->Links,JointAxes,PathAxes,Axes);
				//reset unused output values
				Axes[4] = 0;
				Axes[5] = 0;
				break;
		
            case RTCP:
                Status = RTCP_Direct(Mechanics->Links,JointAxes,PathAxes,Axes);
                break;
            
            case ARM:
				Status = ArmDirect(Mechanics->Links,JointAxes,PathAxes,Axes);
				break;

			case USER:
				if (UserDirect == 0)
				{
					Status = ERR_TRF_POINTER;
					break;
				}
				Axes[0] = 0;
				Axes[1] = 0;
				Axes[2] = 0;
				Axes[3] = 0;
				Axes[4] = 0;
				Axes[5] = 0;
				Status = UserDirect(Mechanics->Links,JointAxes,PathAxes,Axes);
				break;

			default :
				Status = ERR_TRF_MECH_NOT_SUPPORTED;
					
		}
		
		/* removing small values near zero helps avoid useless errors at the edge of workspace */
		int i=0;
		for(i=0;i<6;i++)
		{
			Axes[i] = RoundToEpsilon(Axes[i]);
		}

	}


	
	
	else if (Mode == TRF_INVERSE)	//inverse transformations -> calculate joint axes from path axes
	{
		
		switch (Mechanics->Type) {
            
			case CNC:
				Axes[0] = PathAxes[0];
				Axes[1] = PathAxes[1];
				Axes[2] = PathAxes[2];
				//reset unused output values
				Axes[3] = 0;
				Axes[4] = 0;
				Axes[5] = 0;
				Status = STATUS_OK;
				break;
		
			case SCARA:
				Status = ScaraInverse(Mechanics->Links,PathAxes,JointAxes,Axes);
				//reset unused output values
				Axes[4] = 0;
				Axes[5] = 0;
				break;
		
			case DELTA:
				Status = DeltaInverse(Mechanics->Links,PathAxes,JointAxes,Axes);
				//reset unused output values
				Axes[4] = 0;
				Axes[5] = 0;
				break;
		
			case PALLETIZER:
				Status = PalletInverse(Mechanics->Links,PathAxes,JointAxes,Axes);
				//reset unused output values
				Axes[4] = 0;
				Axes[5] = 0;
				break;

            case RTCP:
                Status = RTCP_Inverse(Mechanics->Links,PathAxes,JointAxes,Axes);
                break;
            
			case ARM:
				Status = ArmInverse(Mechanics->Links,PathAxes,JointAxes,Axes);
				break;

			case USER:
				if (UserInverse == 0)
				{
					Status = ERR_TRF_POINTER;
					break;
				}
				Axes[0] = 0;
				Axes[1] = 0;
				Axes[2] = 0;
				Axes[3] = 0;
				Axes[4] = 0;
				Axes[5] = 0;
				Status = UserInverse(Mechanics->Links,PathAxes,JointAxes,Axes);
				break;

			default :
				Status = ERR_TRF_MECH_NOT_SUPPORTED;
			
		}

		int i=0;
		for(i=0;i<6;i++) {
			Axes[i] = RoundToEpsilon(Axes[i]);
		}
        
	} else {//this mode does not exist
		Status = ERR_TRF_MODE;	
	}

    return Status;

}
