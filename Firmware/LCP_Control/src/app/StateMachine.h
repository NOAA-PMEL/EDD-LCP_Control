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
    PUS_Idle,
}PopupState_t;


typedef eAirDeployState_t
{
    ADS_Idle,
}AirDeployState_t;

typedef eSimpleProfilerState_t
{
    SPS_Idle,
    SPS_MoveToParkDepth_mode,
    SPS_Park_mode,
    SPS_MoveToSampleDepth_mode,
    SPS_Sample_mode,
    SPS_Surface_mode,
    SPS_TX_mode,
    SPS_RX_mode,
}SimpleProfilerState_t;


typedef struct eSystem_t
{
    SystemState_t system;
    struct {
        PredeploymentState_t state;
    }predeploy;

    struct {
        AirDeployState_t state;
    }airdeploy;
    struct {
        SimpleProfilerState_t state;
    }profiler;
    struct {
        AutoBallastState_t state;
    }ballast;
    
    struct {
        MooredState_t state;
    }moored;
    struct {
        PopupState_t state;
    }popup;

}System_t;


#endif // STATEMACHINE_H
