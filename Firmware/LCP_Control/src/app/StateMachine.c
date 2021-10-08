#include "StateMachine.h"

#include "sensors.h"


static System_t system;


void STATE_initialize(SystemState_t state)
{

}

void STATE_MainState(SystemState_t state)
{
    switch(state)
    {
        case SYSST_Predeployment_mode:
            break;
        case SYSST_AutoBallast_mode:
            break;
        case SYSST_SimpleProfiler_mode:
            break;
        case SYSST_Moored_mode:
        case SYSST_AirDeploy_mode:
        case SYSST_Popup_mode:
        default:
            /** These modes are not currently implemented */
            break;
    }
}




 void STATE_Predeploy(void)
 {
     switch(system.predeploy.state)
     {
        case PDS_Idle:
            break;
        case PDS_SystemCheck:
            break;
        default:
            break;
     }
 }  

 void STATE_AutoBallast(void)
 {
     switch(system.ballast.state)
     {
        case ABS_DiveToExpected:
            break;
        case ABS_Sample:
            break;
        case ABS_Validate:
            break;
        default:
            break;
     }
 }

 void STATE_Moored(void)
 {
     switch (system.moored.state)
     {
     case MOOR_MoveToParkDepth_mode:
         /* code */
         break;
     
     case MOOR_Park_mode:
         /* code */
         break;
         
     case MOOR_MoveToSampleDepth_mode:
         /* code */
         break;
         
     case MOOR_Sample_mode:
         /* code */
         break;
         
     case MOOR_Surface_mode:
         /* code */
         break;
         
     case MOOR_TX_mode:
         /* code */
         break;
         
     case MOOR_RX_mode:
         /* code */
         break;
     default:
         break;
     }
 }


 void STATE_Profiler(void)
 {
     switch(system.profiler.state)
     {
        case SPS_Idle:
        break;

        case SPS_MoveToParkDepth_mode:
        break;

        case SPS_Park_mode:
        break;
        case SPS_MoveToSampleDepth_mode:
        break;
        case SPS_Sample_mode:
        break;
        case SPS_Surface_mode:
        break;
        case SPS_TX_mode:
        break;
        case SPS_RX_mode:
        break;
        default:
        break;
     }
 }