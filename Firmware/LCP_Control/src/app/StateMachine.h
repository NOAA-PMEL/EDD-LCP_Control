#ifndef STATEMACHINE_H
#define STATEMACHINE_H


typedef enum eSystemState_t{
    SYSST_Predeployment_mode,
    SYSST_AutoBallast_mode,
    SYSST_SimpleProfiler_mode,
    SYSST_Moored_mode,
    SYSST_AirDeploy_mode,
    SYSST_Popup_mode,
}SystemState_t;


typedef enum ePredeploymentState_t
{
    PDS_Idle,
    PDS_SystemCheck
}PredeploymentState_t;

typedef enum eAutoBallastState_t
{
    ABS_DiveToExpected,
    ABS_Sample,
    ABS_Validate
}AutoBallastState_t;
typedef enum eMooredState_t 
{
    MOOR_MoveToParkDepth_mode,
    MOOR_Park_mode,
    MOOR_MoveToSampleDepth_mode,
    MOOR_Sample_mode,
    MOOR_Surface_mode,
    MOOR_TX_mode,
    MOOR_RX_mode,
}MooredState_t;


typedef enum ePopupState_t
{

}PopupState_t;

// typedef enum 

#endif // STATEMACHINE_H
