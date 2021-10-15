#include "StateMachine.h"

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************



//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "config.h"
#include "sensors.h"


//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************

static uint16_t park_time[DATA_PARK_SAMPLES_MAX];
static uint16_t park_temp[DATA_PARK_SAMPLES_MAX];
static uint16_t park_depth[DATA_PARK_SAMPLES_MAX];
static uint16_t prof_time[DATA_PROFILE_SAMPLES_MAX];
static uint16_t prof_temp[DATA_PROFILE_SAMPLES_MAX];
static uint16_t prof_depth[DATA_PROFILE_SAMPLES_MAX];

static Data_t park;     /**< Park mode Data */
static Data_t prof;     /**< Profile mode data */

static System_t system = {
    .system = SYSST_Predeployment_mode,
    .predeploy.state = PDS_Idle,
    .airdeploy.state = ADS_Idle,
    .profiler.state = SDS_Idle,
    .ballast.state = ABS_DiveToExpected,
    .moored.state = MOOR_Idle,
    .popup.state = PUS_Idle,
};

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
static bool module_sps_idle(void);
static bool module_sps_move_to_park(void);
static bool module_sps_park(void);
static bool module_sps_move_to_profile(void);
static bool module_sps_profile(void);
static bool module_sps_move_to_surface(void);
static bool module_sps_tx(void);
static bool module_sps_rx(void);



//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void STATE_initialize(SystemState_t state)
{
    /** Configure the Data buffer for the state */
    switch(state)
    {
        case SYSST_SimpleProfiler_mode:
            DATA_setbuffer(&park, park_time, park_depth, park_temp, DATA_PARK_SAMPLES_MAX);
            DATA_setbuffer(&prof, prof_time, prof_depth, prof_temp, DATA_PROFILE_SAMPLES_MAX);
        break;
        default:
        break;
    }

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
    bool success = false;

    switch(system.profiler.state)
    {
    case SPS_Idle:
        succes = module_sps_idles();
        break;
    case SPS_MoveToParkDepth_mode:
        succes = module_sps_move_to_park();
        break;
    case SPS_Park_mode:
        succes = module_sps_park();
        break;
    case SPS_MoveToSampleDepth_mode:
        succes = module_sps_move_to_profile();
        break;
    case SPS_Sample_mode:
        succes = module_sps_profile();
        break;
    case SPS_Surface_mode:
        succes = module_sps_surface();
        break;
    case SPS_TX_mode:
        succes = module_sps_tx();
        break;
    case SPS_RX_mode:
        succes = module_sps_rx();
        break;
    default:
        break;
    }

    if(success)
    {
        switch(system.profiler.state)
        {
            case SPS_Idle:
            case SPS_MoveToParkDepth_mode:
            case SPS_Park_mode:
            case SPS_MoveToSampleDepth_mode:
            case SPS_Sample_mode:
            case SPS_Surface_mode:
            case SPS_TX_mode:
                system.profiler.state++;
                break;
            case SPS_RX_mode:
            default:
                system.profiler.state = SPS_Idle;
                break
        }
    }
}







static bool module_sps_idle(void)
{
    /** Turn all sensors and systems OFF */
    SENS_sensor_gps_off();
    SENS_sensor_depth_off();
    SENS_sensor_temperature_off();

}

static bool module_sps_move_to_park(void)
{
    /** Calculate volume to move to */

    /** Start sampling depth @ 1Hz */

    /** Set volume */

    /**  Monitor depth until we get there */
    bool depthFlag = false;
    while(!depthFlag)
    {

    }

}


static bool module_sps_park(void)
{
    /** Start 1/60Hz sampling of sensors for XXX minutes */
    /** Save data in Park Data strucutre */

}

static bool module_sps_move_to_profile(void)
{
    /** Calculate volume to move to */
    float volume = module_calculate_buoyancy_from_descent_rate(system.SysInfo.rate.fall.setpoint);
    
    /** Set volume */

    /** Monitor Depth, when depth reached, mode is complete */

}

static bool module_sps_profile(void)
{
    /** Calculate volume to move to (surface) */

    /** Start 1 Hz data (Depth & Temperature) */

    /** Set volume */

    /** Start recording samples */

    /** Monitor depth & rate.  If rate fails, fix it.  If depth reaches surface, done */

}

static bool module_sps_move_to_surface(void)
{
    /** Extend the piston fully out */

    /** Turn on the GPS */

    /** Monitor piston position */

    /** Capture current GPS Location  */
    
    /** GPS OFF */

}

static bool module_sps_tx(void)
{
    /** Turn on the Iridium Modem */

    /** Monitor for lock or timeout */

    /** Transmit all data */


}
static bool module_sps_rx(void)
{

    /** Check for messages */

    /** If message, receive it */

    /** Iridium OFF */

}





