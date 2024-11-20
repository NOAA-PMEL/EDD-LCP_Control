#ifndef STATEMACHINE_H
#define STATEMACHINE_H

/**********************************************************************************
 * Includes
 *********************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "control.h"
#include "sensors.h"
#include "data.h"
#include "config.h"

/**********************************************************************************
 * Configuration Constants
 *********************************************************************************/

/**********************************************************************************
 * MACROS
 *********************************************************************************/
#define DATA_PROFILE_SAMPLE_FREQ                ( 1 )       /**< 1 Hz Sample Frequency */
#define DATA_PROFILE_VELOCITY_MIN               ( 0.1 )     /**< Lowest minimum profile velocity (m/s)*/
#define DATA_PROFILE_DEPTH_MAX                  ( 220 )     /**< Maximum depth (m) */
#define DATA_PROFILE_OVERAGE_MAX                ( 5 )       /**< Percent extra in buffer */
//#define DATA_PROFILE_SAMPLES_MAX              ( ( (DATA_PROFILE_DEPTH_MAX / DATA_PROFILE_VELOCITY_MIN) / DATA_PROFILE_SAMPLE_FREQ ) * (100 + DATA_PROFILE_OVERAGE_MAX) / 100)
#define DATA_PROFILE_SAMPLES_MAX                ( 230 * SYSTEM_PROFILE_NUMBER ) //( 2310 )

#define DATA_PARK_SAMPLE_FREQ                   ( 1.0 / 60.0 )  /**< Sample every Minute */
#define DATA_PARK_PARK_MAX_DURATION_HOURS       ( 24 )
#define DATA_NUM_SEC_IN_HOUR                    ( 60 * 60 )
#define DATA_NUM_SEC_IN_DAY                     ( DATA_NUM_SEC_IN_HOUR * DATA_PARK_PARK_MAX_DURATION_HOURS )
#define DATA_NUM_SAMPLES_IN_DAY                 ( DATA_PARK_SAMPLE_FREQ * DATA_NUM_SEC_IN_DAY)
#define DATA_PARK_SAMPLES_MAX                   ( 80 * SYSTEM_PROFILE_NUMBER ) //( 1440 )
//#define DATA_PARK_SAMPLES_MAX                   ( ((uint32_t)( 3600 ) * DATA_PARK_SAMPLE_FREQ) * DATA_PARK_PARK_MAX_DURATION_HOURS )

//#define DATA_PROFILE_MAX_LEN                    ( 40000 )
#define DATA_PROFILE_MAX_LEN                    ( (DATA_PROFILE_SAMPLES_MAX * SYSTEM_PROFILE_NUMBER) + \
                                                    (DATA_PARK_SAMPLES_MAX * SYSTEM_PROFILE_NUMBER) )

/**********************************************************************************
 * Typdefs
 *********************************************************************************/
typedef enum eSystemState_t
{
    SYSST_Predeployment_mode,
    SYSST_AutoBallast_mode,
    SYSST_SimpleProfiler_mode,
    SYSST_Moored_mode,
    SYSST_AirDeploy_mode,
    SYSST_Popup_mode,
}SystemMode_t;

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
    MOOR_Idle,
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
    //PUS_DoSomething
    PUS_Surface_float
}PopupState_t;

typedef enum eAirDeployState_t
{
    ADS_Idle,
    ADS_DoSomething
}AirDeployState_t;

typedef enum eSimpleProfilerState_t
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
    SystemMode_t mode;
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

/**********************************************************************************
 * Function Prototypes
 *********************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void STATE_initialize(SystemMode_t mode);
void STATE_MainState(SystemMode_t mode);
void STATE_Predeploy(void);
void STATE_AutoBallast(void);
void STATE_Moored(void);
void STATE_Profiler(void);
void STATE_Popup(void);

/**********************************************************************************
 * Unit Test Variables & Static Prototpyes
 *********************************************************************************/
#ifdef TEST
#ifdef DOXYGEN_IGNORE_THIS

#endif // DOXYGEN_IGNORE_THIS
#endif

#ifdef __cplusplus
} // extern "C"
#endif
#endif // STATEMACHINE_H
