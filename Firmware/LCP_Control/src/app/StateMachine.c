/** @file StateMachine.c
 *  @brief LCP StateMachine that runs the states in a loop for said number of profiles
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date September 30, 2020
 *  @version 0.0.1
 *
 *  @co-author Basharat Martin, basharat.martin@noaa.gov
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */

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
#include <string.h>
#include <math.h>
#include "artemis_debug.h"
#include "artemis_rtc.h"
#include "StateMachine.h"
#include "config.h"
#include "piston.h"
#include "i9603n.h"

/* add datalogger */
#include "datalogger.h"

//#define TEST

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************

// --- Static declarations for "in-use" Park data ---
static pData current_park_profile_info; // Holds metadata (start/stop times, lat/lon, length) for the current park cycle
static float park_pressure_measurements[DATA_PARK_SAMPLES_MAX]; // Static array for park pressure
static float park_temp_measurements[DATA_PARK_SAMPLES_MAX];     // Static array for park temperature

// The main static structure for current park data
static Data_t current_park_data = {                             
    .data.pressure = park_pressure_measurements,                // Point to the static pressure array
    .data.temperature = park_temp_measurements,               // Point to the static temperature array
    .p = &current_park_profile_info                             // Point to the static metadata struct
};

// --- Static declarations for "in-use" Profile data ---
static pData current_profile_profile_info; // Holds metadata for the current profile cycle
static float profile_pressure_measurements[DATA_PROFILE_SAMPLES_MAX]; // Static array for profile pressure
static float profile_temp_measurements[DATA_PROFILE_SAMPLES_MAX];     // Static array for profile temperature

// The main static structure for current profile data
static Data_t current_profile_data = {                          
    .data.pressure = profile_pressure_measurements,             // Point to the static pressure array
    .data.temperature = profile_temp_measurements,            // Point to the static temperature array
    .p = &current_profile_profile_info                          // Point to the static metadata struct
};

static sData sPark;         /**< Park mode measurement - StateMachine */
static sData sProf;         /**< Profile mode measurement - StateMachine */

static System_t system =
{
    .mode = SYSST_Predeployment_mode,
    .predeploy.state = PDS_SystemCheck,
    .airdeploy.state = ADS_Idle,
    .profiler.state = SPS_Idle,
    .ballast.state = ABS_DiveToExpected,
    .moored.state = MOOR_Idle,
    .popup.state = PUS_Surface_float,
};

//*****************************************************************************
//
// FreeRTOS Functions, global variables etc
//
//*****************************************************************************

typedef enum {
    MODE_PRE_DEPLOY,
    MODE_PROFILE,
    MODE_POPUP,
    MODE_IDLE,
    MODE_DONE,
    MODE_CRIT_TO_PARK,
    MODE_CRUSH_TO_PROFILE
} Event_e;

static Event_e gEvent;
static QueueHandle_t gEventQueue;
static QueueHandle_t spsEventQueue;
static QueueHandle_t pdsEventQueue;
static QueueHandle_t pusEventQueue;
static void SendEvent(QueueHandle_t eventQueue, Event_e *event);
static void ReceiveEvent(QueueHandle_t eventQueue, Event_e *event);

/* profiling functions */
void module_sps_idle(void);
void module_sps_move_to_park(void);
void module_sps_park(void);
void module_sps_move_to_profile(void);
void module_sps_profile(void);
void module_sps_move_to_surface(void);
void module_sps_tx(void);
void module_sps_rx(void);

/* predeploy functions */
void module_pds_idle(void);
void module_pds_systemcheck(void);

/* popup functions */
void module_pus_idle(void);
void module_pus_surface_float(void);

// memory monitor function
void monitor_memory_usage(void);

//*****************************************************************************
//
// Global Variables, buffers etc
//
//*****************************************************************************

static uint8_t irid_park[IRID_DATA_OUT];    /* max 340 bytes */
static uint8_t irid_prof[IRID_DATA_OUT];    /* max 340 bytes */

//static uint8_t temp_park[312];              /* needed for creating a page */
//static uint8_t temp_prof[312];              /* needed for creating a page */

/* linearize profile and park measurements for iridium */
static uint16_t prepare_transmit_page(uint8_t *iridium_buffer, QueuedDataEntry_t *item, 
                                        uint16_t samples_already_processed, uint8_t page_num, 
                                        uint16_t *samples_packed_in_page);
static uint16_t create_park_page(uint8_t *pPark, uint8_t *readlength);
static uint16_t create_profile_page(uint8_t *pProf, uint8_t *readlength);

/* global variables */
static volatile bool sensors_check = false;
static volatile bool iridium_init = false;

static volatile uint8_t prof_number = 0; // limited to 255 profiles
static volatile uint8_t park_number = 0; // limited to 255 parks
static volatile uint8_t m_prof_number = 0;
static volatile uint16_t m_prof_length = 0;
static volatile uint8_t m_park_number = 0;
static volatile uint16_t m_park_length = 0;
static volatile uint8_t pistonzero_number = 0;
static volatile uint8_t pistonfull_number = 0;
///* for extended measurements */
//static uint8_t m_prof_number_ext = 0;
//static uint8_t m_prof_length_ext = 0;
//static uint8_t m_park_number_ext = 0;
//static uint8_t m_park_length_ext = 0;

//static uint16_t parkPage = 0;
//static uint16_t profPage = 0;
static volatile float systemcheck_Lat = 0.0;
static volatile float systemcheck_Lon = 0.0;

static volatile float park_piston_length = 0.0;
static volatile float to_prof_piston_length = 0.0;
static volatile float prof_piston_length = 0.0;
static volatile bool critical_park_state = false;
static volatile bool crush_depth = false;

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************

void STATE_initialize(SystemMode_t mode)
{
    /* initialize all the sensors */
    sensors_check = SENS_initialize();

    /* Create a global event for passing messages within LCP modes */
    gEventQueue = xQueueCreate(3, sizeof(Event_e));

    /** Configure the Data buffer for the mode */
    switch(mode)
    {
        case SYSST_Predeployment_mode:
            // No specific data buffer setup needed here for park/profile
            break;

        case SYSST_AutoBallast_mode:
            // No specific data buffer setup needed here for park/profile
            break;

        case SYSST_SimpleProfiler_mode:
            // --- Initialize static data structures ---
            // Set the maximum sample capacity for the static buffers
            current_park_data.cbuf.length = DATA_PARK_SAMPLES_MAX;
            current_profile_data.cbuf.length = DATA_PROFILE_SAMPLES_MAX;

            // Reset the counters and state for the static buffers
            DATA_reset(&current_park_data);
            DATA_reset(&current_profile_data);
            ARTEMIS_DEBUG_PRINTF("STATE_initialize: Static Park and Profile data structures reset.\n");

            break;

        case SYSST_Moored_mode:
            // Add specific setup if this mode uses the static buffers
            break;

        case SYSST_AirDeploy_mode:
            // Add specific setup if this mode uses the static buffers
            break;

        case SYSST_Popup_mode:
            // No specific data buffer setup needed here for park/profile
            break;

        default:
            break;
    }

    /* Initialize the memory management system */
    MEM_init_transmission_queue();
}

void STATE_MainState(SystemMode_t mode)
{
    switch(mode)
    {
        case SYSST_Predeployment_mode:
            break;
        case SYSST_AutoBallast_mode:
            break;
        case SYSST_SimpleProfiler_mode:
            break;
        case SYSST_Moored_mode:
            break;
        case SYSST_AirDeploy_mode:
            break;
        case SYSST_Popup_mode:
            break;
        default:
            /** These modes are not currently implemented */
            break;
    }
}

void STATE_Popup(void)
{
    /* wait for a global event */
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("PUS :: Popup global event wait\n");
        ReceiveEvent(gEventQueue, &gEvent);
        ARTEMIS_DEBUG_PRINTF("PUS :: Popup global event received\n");
        if (gEvent == MODE_POPUP)
        {
            break;
        }
    }

    /* create a local task event queue */
    pusEventQueue = xQueueCreate(1, sizeof(Event_e));
    Event_e pusEvent;

    for (;;)
    {
        switch(system.popup.state)
        {
            case PUS_Idle:
                configASSERT(xTaskCreate((TaskFunction_t) module_pus_idle,
                                        "popup_idle", 256, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(pusEventQueue, &pusEvent);
                break;
            case PUS_Surface_float:
                configASSERT(xTaskCreate((TaskFunction_t) module_pus_surface_float,
                                        "popup_surface_float", 256, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(pusEventQueue, &pusEvent);
                break;
            default:
                break;
        }

        if(pusEvent == MODE_DONE)
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: Popup Surface done, going to Idle\n");
            system.popup.state = PUS_Idle;
            vTaskDelay(xDelay2000ms);
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: Popup mode, ERROR: something went wrong\n");
            vTaskDelay(portMAX_DELAY);
        }
    }
}

void module_pus_idle(void)
{
    /* turn off all the sensor */
    SENS_sensor_depth_off();
    SENS_sensor_temperature_off();
    SENS_sensor_gps_off();
    i9603n_off();
    /* uninitialize sensors */
    SENS_uninitialize();

    bool run = true;
    while (run)
    {
        /* go into deep sleep */
        ARTEMIS_DEBUG_PRINTF("\n\nPUS :: Idle, going to deep sleep\n");
        ARTEMIS_DEBUG_PRINTF("PUS :: Idle, << DONE HERE >>\n\n");
        vTaskDelay(xDelay1000ms);

        /* check Heap size */
        uint32_t size = xPortGetFreeHeapSize();
        ARTEMIS_DEBUG_PRINTF("\nPUS :: Idle, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

        /* turn off datalogger */
        datalogger_power_off();
        datalogger_deinit(4);

        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        vTaskDelay(portMAX_DELAY);
        run = false;
    }
    vTaskDelete(NULL);
}

void module_pus_surface_float(void)
{
    uint32_t piston_period = xDelay1000ms;
    uint32_t piston_timer = 0;
    bool piston_move = false;
    float Volume = 0.0;
    float Length = 0.0;
    float Density = 0.0;
    float length_update = 0.0;

    /* get piston current length */
    PIS_Get_Length(&Length);
    length_update = Length;
    Volume = CTRL_calculate_volume_from_length(Length);
    Density = CTRL_calculate_lcp_density(Volume);
    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");

    TaskHandle_t xPiston = NULL;
    eTaskState eStatus;
    PIS_set_piston_rate(1);

    /* Monitor the Depth if it is less than 50m */
    float s_rate = 1.0;
    uint32_t period = xDelay1000ms/s_rate;
    SENS_set_depth_rate(1);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth = NULL;
    SENS_task_sample_depth_continuous(&xDepth);
    bool run = true;
    float Depth = 0, Rate = 0;
    float Pressure = 0;
    rtc_time time;

    /** check for the depth rate up to XX measurements */
    uint8_t rate_count = 0;
    float rate_avg = 0;

    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);
        ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Pressure = %0.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Depth    = %0.4f m, rate = %0.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Epoch    = %ld\n", epoch);
        if ( Depth <= CRITICAL_PISTON_POSITON_DEPTH )
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Pressure reached = %0.4f bar\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Depth reached    = %0.4f m\n", Depth);
            run = false;
            piston_move = false;
            SENS_task_delete(xDepth);
            SENS_sensor_depth_off();
        }
        else
        {
            /* check the Depth rate */

            /* collect up to XX measurements
             * rate_count is the number of depth rate average */
            rate_avg += Rate;
            rate_count++;
            if (rate_count >= 5)
            {
                float averaged_rate = (float) (rate_avg / (float)rate_count);
                /* check if rate is positive, negative or stable */
                if (averaged_rate < 0.0)
                {
                    /* do nothing for now, */
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Depth Rate is negative, do nothing\n");
                }
                else if (averaged_rate > 0.0 && !piston_move)
                {
                    /* decrease piston position by PARK_POSITION_INCREMENT inches */
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Depth Rate is positive, increase %fin\n", PARK_POSITION_INCREMENT);
                    length_update += PARK_POSITION_INCREMENT;

                    /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                    if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                    {
                        if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                        {
                            ARTEMIS_DEBUG_PRINTF("\n<< PUS :: surface_float, Depth = %.4f is @critial piston position >>\n", Depth);
                            length_update = CRUSH_DEPTH_PISTON_POSITION;
                        }
                    }

                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
                    vTaskDelay(xDelay5000ms);
                }
                else if (averaged_rate == 0.0 && !piston_move)
                {
                    /* decrease piston position by PARK_POSITION_INCREMENT inches */
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Depth Rate is stable, increase %fin\n", PARK_POSITION_INCREMENT2);
                    length_update += PARK_POSITION_INCREMENT;

                    /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                    if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                    {
                        if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                        {
                            ARTEMIS_DEBUG_PRINTF("\n<< PUS :: surface_float, Depth=%.4f is @critial piston position >>\n", Depth);
                            length_update = CRUSH_DEPTH_PISTON_POSITION;
                        }
                    }

                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
                    vTaskDelay(xDelay5000ms);
                }
                /* reset the rate counter and rate_avg*/
                rate_count = 0;
                rate_avg = 0.0;
            }
        }

        while (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->active\n");

                /* piston time for up to 120 seconds */
                piston_timer += piston_period;
                if (piston_timer >= 120000)
                {
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    PIS_Reset();
                    piston_timer = 0;
                }
            }
            // else if (eStatus==eReady)
            // {
            //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
            //     piston_move = false;
            //     piston_timer = 0;
            // }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->suspended\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                //piston_move = false;
                piston_timer = 0;
            }
            else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
            {
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->finished\n");
                piston_move = false;
                piston_timer = 0;
            }
        }
        vTaskDelay(period);
    }

    /* it is safe to fully extend the piston if need be */
    vTaskDelay(piston_period);
    PIS_task_move_full(&xPiston);
    piston_move = true;
    vTaskDelay(xDelay5000ms);

    Event_e pusEvent;
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->active\n");

            /* piston time for up to 180 seconds */
            piston_timer += piston_period;
            if (piston_timer >= 180000)
            {
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                PIS_Reset();
                piston_timer = 0;
                pusEvent = MODE_DONE;
            }
        }
        // else if (eStatus==eReady)
        // {
        //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
        //     piston_move = false;
        //     piston_timer = 0;
        // }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            vTaskDelay(xDelay5000ms);
            //piston_move = false;
            piston_timer = 0;
        }
        else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
        {
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
            pusEvent = MODE_DONE;
        }
        vTaskDelay(piston_period);
    }
    vTaskDelay(xDelay5000ms);

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nPUS :: surface_float, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    SendEvent(pusEventQueue, &pusEvent);
    vTaskDelete(NULL);
}

void STATE_Predeploy(void)
{
    pdsEventQueue = xQueueCreate(1, sizeof(Event_e));
    Event_e pdsEvent;

    for (;;)
    {
        switch(system.predeploy.state)
        {
            case PDS_Idle:
                configASSERT(xTaskCreate((TaskFunction_t) module_pds_idle,
                                        "predeploy_idle", 256, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(pdsEventQueue, &pdsEvent);
                break;
            case PDS_SystemCheck:
                configASSERT(xTaskCreate((TaskFunction_t) module_pds_systemcheck,
                                        "predeploy_systemcheck", 2048, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(pdsEventQueue, &pdsEvent);
                break;
            default:
                break;
        }
        if (pdsEvent == MODE_PRE_DEPLOY)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Transitionng to Idle State\n");
            system.predeploy.state = PDS_Idle;
            vTaskDelay(xDelay1000ms);
        }
        else if (pdsEvent == MODE_PROFILE)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Switching to Profile Mode (SPS)\n");
            vTaskDelay(xDelay1000ms);
            gEvent = pdsEvent;
            SendEvent(gEventQueue, &gEvent);
            vQueueDelete(pdsEventQueue);
            vTaskDelete(NULL);
        }
    }
}

void module_pds_idle(void)
{
    /** Extend the piston fully out */
    ARTEMIS_DEBUG_PRINTF("\nPDS :: Idle, piston move to park density=%.3f kg/m³\n", PARK_DENSITY);

    float Density = PARK_DENSITY;
    float Volume = CTRL_set_lcp_density(Density);
    float Length = CTRL_calculate_length_from_volume(Volume);
    ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Setting -> density=%.3f kg/m³, volume=%.3fin³, length=%.4fin\n\n", Density, Volume, Length);

    uint32_t piston_period = xDelay1000ms;
    uint32_t piston_timer = 0;
    bool piston_move = true;
    TaskHandle_t xPiston = NULL;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_set_volume(Volume);
    PIS_task_move_volume(&xPiston);
    vTaskDelay(xDelay5000ms);

    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->active\n");
            /* piston time for up to 180 seconds */
            piston_timer += piston_period;
            if (piston_timer >= 180000)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston time-out, task->finished\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                PIS_Reset();
                piston_timer = 0;
            }
            // else if (eStatus==eReady)
            // {
            //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
            //     piston_move = false;
            //     piston_timer = 0;
            // }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            vTaskDelay(xDelay5000ms);
            //piston_move = false;
            piston_timer = 0;
        }
        else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
        {
            PIS_Get_Volume(&Volume);
            Length = CTRL_calculate_length_from_volume(Volume);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, density=%.3f kg/m³, volume=%.3fin³, length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
        }
        vTaskDelay(piston_period);
    }
    vTaskDelay(xDelay5000ms);

#if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
    /* go faster @0.5Hz*/
    float s_rate = 2.0;
#else
    /** Start sampling depth @ 1.0/30.0 Hz */
    float s_rate = BALLAST_DEPTH_SAMPLE_RATE;
#endif

    uint32_t period = xDelay1000ms/s_rate;
    SENS_set_depth_rate(s_rate);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth = NULL;
    SENS_task_sample_depth_continuous(&xDepth);

    /** Monitor Depth */
    bool run = true;
    float Depth = 0, Rate = 0;
    float Pressure = 0;
    rtc_time time;

    Event_e pdsEvent;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);
        ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Pressure = %0.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Depth    = %0.4f m, rate = %0.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Epoch    = %ld\n", epoch);
        if ( Depth>BALLAST_DEPTH )
        {
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, deliberately stopping the Piston\n");
                /* stop the piston */
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                PIS_stop();
                vTaskDelay(piston_period);
                piston_move = false;
                piston_timer = 0;
            }

            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Pressure reached = %0.4f bar\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Depth reached    = %0.4f m\n", Depth);
            run = false;
            SENS_task_delete(xDepth);
            SENS_sensor_depth_off();
            pdsEvent = MODE_PROFILE;
        }
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nPDS :: Idle, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    SendEvent(pdsEventQueue, &pdsEvent);
    vTaskDelete(NULL);
}

void module_pds_systemcheck(void)
{
    Event_e pdsEvent;
    if (sensors_check == true)
    {
        /* Turn off all the sensors, except GPS */
        SENS_sensor_depth_off();
        SENS_sensor_temperature_off();
        SENS_sensor_gps_on();

        /* GPS task handle */
        uint8_t s_rate = 1;
        uint32_t period = xDelay1000ms/s_rate;
        SENS_set_gps_rate(s_rate);
        TaskHandle_t xGps = NULL;

        /* Start GPS task */
        SENS_task_gps(&xGps);

        SensorGps_t gps;
        eTaskState eStatus;
        bool run = true;
        uint8_t fix = 0;

        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();

        while (run)
        {
            eStatus = eTaskGetState(xGps);
            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                SENS_get_gps(&gps);
                if (gps.fix == true)
                {
                    //ARTEMIS_DEBUG_PRINTF("GPS : TimeStampe, %u.%u.%u, %u:%u:%u\n", gps.month, gps.day, gps.year, gps.hour, gps.min, gps.sec);
                    ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS : fixed, latitude=%0.7f, longitude=%0.7f, altitude=%0.7f\n", gps.latitude, gps.longitude, gps.altitude);
                    fix++;
                    if (fix > 9)
                    {
                        /* update latitude and longitude globally */
                        systemcheck_Lat = gps.latitude;
                        systemcheck_Lon = gps.longitude;
                        /* Calibrate the GPS UTC time into RTC */
                        ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, RTC : <GPS Time Set>\n");
                        artemis_rtc_gps_calibration(&gps);
                        fix = 0;
                    }
                }
                else
                {
                    ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task->active : No fix\n");
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task->suspended\n");
                vTaskDelete(xGps);
            }
            else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
            {
                /* check, if it got at least two to three fixes */
                if (fix >= 2)
                {
                    /* update latitude and longitude globally */
                    systemcheck_Lat = gps.latitude;
                    systemcheck_Lon = gps.longitude;
                    /* Calibrate the GPS UTC time into RTC */
                    ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, RTC : <GPS Time Set>\n");
                    artemis_rtc_gps_calibration(&gps);
                    fix = 0;
                }
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task->finished\n");
                run = false;
                SENS_sensor_gps_off();

                /* store data in the SDcard */
                datalogger_predeploy_mode(&gps, true);
                pdsEvent = MODE_PRE_DEPLOY;
            }
            vTaskDelayUntil(&xLastWakeTime, period);
        }
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, ->>> ERROR !!\n");
        /* turn on the RED LED */
        am_hal_gpio_output_clear(AM_BSP_GPIO_LED_RED);
        /* put a forever delay */
        vTaskDelay(portMAX_DELAY);
    }

    rtc_time time;
    bool utc = artemis_rtc_get_time(&time);
    if (utc)
    {
        ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, RTC : TimeStamp, %02d.%02d.20%02d, %02d:%02d:%02d (UTC)\n",
                        time.month, time.day, time.year, time.hour, time.min, time.sec);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, RTC : TimeStamp, %02d.%02d.20%02d, %02d:%02d:%02d (local)\n",
                        time.month, time.day, time.year, time.hour, time.min, time.sec);
    }

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nPDS :: systemcheck, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    /* task done, move to Idle state of the PreDeploy_mode */
    SendEvent(pdsEventQueue, &pdsEvent);
    vTaskDelete(NULL);
}

void STATE_Profiler(void)
{
#ifdef TEST
    /* do nothing */
#else
    /* wait for a global event */
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: Profile global event wait\n");
        ReceiveEvent(gEventQueue, &gEvent);
        ARTEMIS_DEBUG_PRINTF("SPS :: Profile global event received\n");
        if (gEvent == MODE_PROFILE)
        {
            //ARTEMIS_DEBUG_PRINTF("\n\nStarting Profile %u\n\n", prof_number+1);
            break;
        }
    }
#endif

    /* create a local task event queue */
    spsEventQueue = xQueueCreate(2, sizeof(Event_e));
    Event_e spsEvent;
    for (;;)
    {
        switch(system.profiler.state)
        {
            case SPS_Idle:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_idle,
                                        "sps_idle", 512, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_MoveToParkDepth_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_move_to_park,
                                        "sps_move_to_park", 1024, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_Park_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_park,
                                        "sps_park", 1024, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_MoveToSampleDepth_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_move_to_profile,
                                        "sps_move_to_profile", 1024, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_Sample_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_profile,
                                        "sps_profile", 2048, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_Surface_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_move_to_surface,
                                        "sps_move_to_surface", 1024, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_TX_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_tx,
                                        "sps_txt", 1024, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            //case SPS_RX_mode:
            //    success = module_sps_rx();
            //    break;
            default:
                break;
        }

        /* check the received event */
        if(spsEvent == MODE_DONE)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: Transitionng to next state\n");
            system.profiler.state++;
            vTaskDelay(xDelay2000ms);
        }
        else if (spsEvent == MODE_IDLE)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: Profiling done, going to Idle\n");
            system.profiler.state = SPS_Idle;
            vTaskDelay(xDelay2000ms);
        }
        else if (spsEvent == MODE_POPUP)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: Switching to Popup Mode\n");
            vTaskDelay(xDelay2000ms);
            gEvent = spsEvent;
            SendEvent(gEventQueue, &gEvent);
            vQueueDelete(spsEventQueue);
            vTaskDelay(xDelay500ms);
            vTaskDelete(NULL);
        }

        /** add a special MODE in case of getting stuck in Profile state for Critical Depth Piston Position */
        else if (spsEvent == MODE_CRIT_TO_PARK)
        {
            ARTEMIS_DEBUG_PRINTF("\nSPS :: Switching to move_to_park State !! <<< Critical Depth Piston Position >>>\n\n");
            system.profiler.state = SPS_MoveToParkDepth_mode;
            vTaskDelay(xDelay2000ms);
        }
        /** add a special MODE in case of hitting the Crush Depth then surface and adjust piston length settings */
        else if (spsEvent == MODE_CRUSH_TO_PROFILE)
        {
            ARTEMIS_DEBUG_PRINTF("\nSPS :: Switching to Profile State !! <<< CRUSH DEPTH EXCEEDED >>>\n\n");
            system.profiler.state = SPS_Sample_mode;
            vTaskDelay(xDelay2000ms);
        }
    }
}

void module_sps_idle(void)
{
    /** Turn all sensors and systems OFF */
    SENS_sensor_gps_off();
    SENS_sensor_depth_off();
    SENS_sensor_temperature_off();

    /* wait here for 1 second for now */
    uint8_t wait_time = 1;
    uint32_t xDelay = (xDelay1000ms * wait_time);

    Event_e spsEvent;
    bool run = true;
    while (run)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: Idle, %u sec wait\n", wait_time);
        vTaskDelay(xDelay);
        spsEvent = MODE_DONE;
        run = false;
    }

    ARTEMIS_DEBUG_PRINTF("SPS :: Idle, Task->finished\n");
    ARTEMIS_DEBUG_PRINTF("\n<<< Starting Profile %u >>>\n\n", prof_number+1);

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("SPS :: Idle, FreeRTOS HEAP SIZE = %u Bytes\n", size);

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_move_to_park(void)
{
    float Volume = 0.0;
    float Length = 0.0;
    float Density = 0.0;
    float length_update = 0.0;
    float zlengthadjust = 0.0;
    float zlengthdrift  = 0.0;
    uint32_t piston_period = xDelay1000ms;
    uint32_t piston_timer = 0;
    bool piston_move = true;

    TaskHandle_t xPiston = NULL;
    eTaskState eStatus;
    PIS_set_piston_rate(1);

    /* set crush depth to false */
    crush_depth = false;

#ifdef TEST
    /* do nothing */
#else

    /*Check if it is time to zero the piston encoder counts, if yes, then zero cal the piston*/
    if( pistonzero_number >= PISTON_ZEROCAL_COUNTER )
    {

        piston_timer = 0;
        piston_move = true;

        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_park, Setting -> Piston encoder value to zero, %u profiles reached since last cal >>\n\n", pistonzero_number);
        PIS_task_move_zero(&xPiston); /*This is the piston zero reset command*/
        vTaskDelay(xDelay5000ms);

            /* check on piston movement */
        while (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {   
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->active\n");
                /* piston time for up to 180 seconds */
                piston_timer += piston_period;
                if (piston_timer >= 180000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero time-out, task->finished\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    PIS_Reset();
                    piston_timer = 0;
                }
            }
            // else if (eStatus==eReady)
            // {
            //         ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->Ready\n");
            //         piston_move = false;
            //         piston_timer = 0;
            // }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->suspended\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                piston_timer = 0;
            }
            else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
            {
                vTaskDelay(piston_period);
                PIS_Get_Length(&zlengthdrift);
                vTaskDelay(piston_period);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Length=%.4fin\n", zlengthdrift);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Task->Finished\n");
                piston_move = false;
                piston_timer = 0;
            }
            vTaskDelay(piston_period);
        }
        vTaskDelay(xDelay5000ms);
        zlengthadjust = 0.0 - zlengthdrift;

        if( (zlengthadjust<=0.25) && (zlengthadjust>=-0.25))
        {
            /*Adjust saved position settings retain correct buoyancy states*/
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, New Piston Zero Cal Length=0in\n");
            park_piston_length = park_piston_length + zlengthadjust;
            if(park_piston_length >= CRUSH_DEPTH_PISTON_POSITION && (PARK_DEPTH + PARK_DEPTH_ERR) >= CRITICAL_PISTON_POSITON_DEPTH)
            {
                park_piston_length = CRUSH_DEPTH_PISTON_POSITION;
            }
            to_prof_piston_length= to_prof_piston_length + zlengthadjust;
            if(to_prof_piston_length >= CRUSH_DEPTH_PISTON_POSITION && (PROFILE_DEPTH + PROFILE_DEPTH_ERR) >= CRITICAL_PISTON_POSITON_DEPTH)
            {
                to_prof_piston_length = CRUSH_DEPTH_PISTON_POSITION;
            }
            prof_piston_length = prof_piston_length + zlengthadjust;
            if(prof_piston_length >= CRUSH_DEPTH_PISTON_POSITION && (PROFILE_DEPTH + PROFILE_DEPTH_ERR) >= CRITICAL_PISTON_POSITON_DEPTH)
            {
                prof_piston_length = CRUSH_DEPTH_PISTON_POSITION;
            }
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Setting -> Piston Length Adjustment %.4fin, Park Length=%.4fin, To Prof Length=%.4fin, Prof Length=%.4fin\n", zlengthadjust, park_piston_length, to_prof_piston_length, prof_piston_length);
        }
        pistonzero_number = 0;
    }
    
     /** Set park_piston_length */
    /* for now , pressure and temperature variables are zeros for compressibility and thermal expansion */
    
    if (park_piston_length == 0.0)
    {
        CTRL_set_lcp_density(PARK_DENSITY);
        park_piston_length = CTRL_calculate_piston_position(0.0, 0.0);
        length_update = park_piston_length;
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_park, Setting -> first time park_piston_length=%.4fin >>\n", park_piston_length);

        /*check if length update will result in a piston position less than zero, set length to PISTON_POSITION_MINIMUM*/
        if (length_update <= PISTON_POSITION_MINIMUM)
        {
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_park, length_update=%.4fin < piston position minimum >>\n", length_update);
        length_update = PISTON_POSITION_MINIMUM;
        }
    }
    else
    {
        /* assigned to local_length, assumed that length was set previously */
        length_update = park_piston_length;
        ARTEMIS_DEBUG_PRINTF("\n<<SPS :: move_to_park, Setting -> adjusted park_piston_length=%.4fin >>\n", park_piston_length);
    }

    /* check the length_update if it is greater than CRUSH_DEPTH_PISTON_POSITION (5.25in) */
    if (length_update > CRUSH_DEPTH_PISTON_POSITION && PARK_DEPTH >= CRITICAL_PISTON_POSITON_DEPTH)
    {
        length_update = CRUSH_DEPTH_PISTON_POSITION;
        ARTEMIS_DEBUG_PRINTF("\n<<SPS :: move_to_park, Hit critical depth piston length, Setting -> adjusted park_piston_length=%.4fin >>\n", CRUSH_DEPTH_PISTON_POSITION);
    }

    Volume = CTRL_calculate_volume_from_length(length_update);
    Density = CTRL_calculate_lcp_density(Volume);
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Setting -> Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n\n", Density, Volume, length_update);

    piston_timer = 0;
    piston_move = true;

    PIS_set_length(length_update);
    PIS_task_move_length(&xPiston);
    vTaskDelay(xDelay5000ms);

    /* check on piston movement */
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->active\n");
            /* piston time for up to 180 seconds */
            piston_timer += piston_period;
            if (piston_timer >= 180000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston time-out, task->finished\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                PIS_Reset();
                piston_timer = 0;
            }
        }
        // else if (eStatus==eReady)
        // {
        //         ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
        //         piston_move = false;
        //         piston_timer = 0;
        // }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            vTaskDelay(xDelay5000ms);
            piston_timer = 0;
        }
        else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
        {
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
        }
        vTaskDelay(piston_period);
    }
    vTaskDelay(xDelay5000ms); 

#endif

#ifdef TEST
    /** Start sampling depth @ 9Hz */
    float s_rate = 9.0;
    uint32_t period = xDelay1000ms/s_rate;
    bool run = false;
    Event_e spsEvent;
    spsEvent = MODE_DONE;
#else
    /** Start sampling depth @ 2Hz or settable */
    float s_rate = MOVE_TO_PARK_SAMPLE_RATE;
    Event_e spsEvent;
    uint32_t period = xDelay1000ms/s_rate;

    SENS_set_depth_rate(s_rate);
    TaskHandle_t xDepth = NULL;

    if (period >= xDelay10000ms)
    {
        /*do not turn on the sensors yet*/
    }
    else
    {
        /* turn on the sensor */
        SENS_sensor_depth_on();
        vTaskDelay(xDelay10ms);
        /* start sampling the sensor */
        SENS_task_sample_depth_continuous(&xDepth);
    }

    bool run = true;
    /** check for the depth rate up to XX measurements */
    uint8_t rate_count = 0;
    float rate_avg = 0;

#endif

    /**  Monitor depth until we get there */
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    uint32_t to_park_state_time = 0;
    uint8_t to_park_pistonmin_try = 0;

    /* variables for checking if the LCP hit the bottom for PISTON_MOVEMENT_ON_BOTTOM (inches) movement */
    //PIS_Get_Length(&Length);
    //vTaskDelay(piston_period);
    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_park, << Setting Length %.4fin to piston_on_bottom_length variable >>\n\n", length_update);
    float piston_on_bottom_length = length_update;
    float length_update_last_adjusted = length_update;

    while (run)
    {
        if (period >= xDelay10000ms)
        {
            /* turn on the sensors */
            SENS_sensor_depth_on();
            vTaskDelay(xDelay10ms);
            /* start sampling the sensor */
            SENS_task_sample_depth_continuous(&xDepth);

            SENS_get_depth(&Depth, &Pressure, &Rate);
            vTaskDelay(xDelay10ms);
            /* turn off the sensors */
            SENS_sensor_depth_off();
        }
        else
        {
            SENS_get_depth(&Depth, &Pressure, &Rate);
        }

        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure  = %.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth     = %.4f m, rate = %.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));

#ifdef TEST
    /* do nothing */
#else

        /* collect up to XX measurements
         * rate_count is the number of depth rate average */
        rate_avg += Rate;
	    rate_count++;
        if (rate_count >= 3)
        {
            float averaged_rate = (float) (rate_avg / (float)rate_count);
            /* check if rate is positive, negative or stable */
            if (averaged_rate >= SYSTEM_FALL_RATE_MIN && !crush_depth)
            {
                /* do nothing for now, */
                //ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth Rate is positive and >= %.4fm/%.1fs\n", SYSTEM_FALL_RATE_MIN, (float)(1/s_rate));
                //ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth Rate is positive\n");

                /* keep updating the piston_on_bottom_length to length_update */
                piston_on_bottom_length = length_update;
            }
            else if (averaged_rate < SYSTEM_FALL_RATE_MIN && !piston_move && !crush_depth)
            {
                /* decrease piston position by PARK_POSITION_INCREMENT inches */
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth Rate is negative, decrease %fin\n", PARK_POSITION_INCREMENT);
                length_update -= PARK_POSITION_INCREMENT;

                /* check if LCP hits the bottom already */
                if ( (piston_on_bottom_length - length_update) <= PISTON_MOVEMENT_ON_BOTTOM)
                {
                    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_park, LCP is not hitting the bottom\n\n");
                }
                else if ( (piston_on_bottom_length - length_update) > PISTON_MOVEMENT_ON_BOTTOM)
                {
                    ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_park, LCP presumably hitting the bottom >>\n\n");

                    /* set previous adjusted length update */
                    park_piston_length = length_update_last_adjusted;
                    /* move to the next state */
                    to_park_state_time = 0;
                    /* reset the rate counter and rate_avg*/
                    rate_count = 0;
                    rate_avg = 0.0;

                    /* stop here, and delete the task and turn off pressure sensor, move to next state */
                    if (period >= xDelay10000ms)
                    {
                        /* sensor task is already deleted */
                    }
                    else
                    {
                        SENS_task_delete(xDepth);
                        SENS_sensor_depth_off();
                    }

                    spsEvent = MODE_DONE;
                    run = false;
                    /* I guess break the loop*/
                    break;
                }

                /* piston minimum position check */
                if (length_update <= PISTON_POSITION_MINIMUM)
                {
                    length_update = PISTON_POSITION_MINIMUM;
                    to_park_pistonmin_try++;
                    /* start the timer for TO_PARK_STATE_TIMER (20 mins) */
                    to_park_state_time += (period * rate_count);
                    ARTEMIS_DEBUG_PRINTF("<< SPS :: move_to_park, critical piston minimum position time = %.2f seconds >>\n", (float)to_park_state_time/xDelay1000ms);
                    if (to_park_state_time >= (TO_PARK_STATE_TIMER*xDelay1000ms))
                    {
                        ARTEMIS_DEBUG_PRINTF("<< SPS :: move_to_park, critical piston minimum position time out = %.2f mins >>\n", (float)to_park_state_time/(60.0*xDelay1000ms));
                        /* set previous adjusted length update */
                        park_piston_length = length_update_last_adjusted;
                        /* move to park state */
                        to_park_state_time = 0;
                        /* stop here, and delete the task and turn off pressure sensor, move to next state */
                        if (period >= xDelay10000ms)
                        {
                            /* sensor task is already deleted */
                        }
                        else
                        {
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        run = false;
                        spsEvent = MODE_DONE;
                    }
                }

                /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                else if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_park, Depth = %.4f is @critial piston position >>\n", Depth);
                        length_update = CRUSH_DEPTH_PISTON_POSITION;
                    }
                }

                /* check if it needs to move the piston or not */
                PIS_Get_Length(&Length);
                vTaskDelay(piston_period);

                if (length_update <= PISTON_POSITION_MINIMUM && Length <= PISTON_POSITION_MINIMUM)
                {
                    /* do not even send a piston command, do nothing  */
                }
                else if (length_update <= PISTON_POSITION_MINIMUM && to_park_pistonmin_try >=2 )
                {
                    /* do not even send a piston command, do nothing  */
                }
                else if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        /* do not even send a piston command, do nothing  */
                    }
                    else
                    {
                        park_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
                        vTaskDelay(xDelay5000ms);
                    }
                }
                else
                {
                    park_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
                    vTaskDelay(xDelay5000ms);
                }
            }
            /* reset the rate counter and rate_avg*/
            rate_count = 0;
            rate_avg = 0.0;
        }

        /* check on depth to reach */
        if (Depth >= PARK_DEPTH && !crush_depth)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure Reached = %0.4f bar\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth Reached    = %0.4f m, rate = %0.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Reach PARK Depth\n");

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, deliberately stopping the Piston\n");
                /* stop the piston */
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                PIS_stop();
                vTaskDelay(piston_period);
                piston_move = false;
                piston_timer = 0;
            }

            /* piston task delay 1000ms */
            if (period >= xDelay10000ms)
            {
                /* sensor task is already deleted */
            }
            else
            {
                /* stop here, and delete the task and turn off pressure sensor, move to next state */
                SENS_task_delete(xDepth);
                SENS_sensor_depth_off();
            }

            run = false;
            spsEvent = MODE_DONE;
        }

        /* check on piston movement */
        if (piston_move)
        {
            do
            {
                eStatus = eTaskGetState( xPiston );
                if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->active\n");
                    /* keep piston time for up to 15 seconds unless crush_depth activated use piston up to 120 seconds */
                    piston_timer += piston_period;

                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(xDelay5000ms);
                            PIS_Reset();
                            piston_timer = 0;

                            /* piston task delay 1000ms */
                            if (period >= xDelay10000ms)
                            {
                                /* sensor task is already deleted */
                            }
                            else
                            {
                                /* stop here, in case of emergency blow */
                                SENS_task_delete(xDepth);
                                SENS_sensor_depth_off();
                            }

                            spsEvent = MODE_CRUSH_TO_PROFILE;
                            vTaskDelay(piston_period);
                            run = false;
                            break;
                        }
                    }
                    else
                    {
                        if (piston_timer >= 30000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(xDelay5000ms);
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                }
                // else if (eStatus==eReady)
                // {
                //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
                //     piston_move = false;
                //     piston_timer = 0;

                //     if (crush_depth)
                //     {
                //         /* piston task delay 1000ms */
                //         if (period >= xDelay10000ms)
                //         {
                //             /* sensor task is already deleted */
                //         }
                //         else
                //         {
                //             /* stop here, in case of emergency blow */
                //             SENS_task_delete(xDepth);
                //             SENS_sensor_depth_off();
                //         }

                //         spsEvent = MODE_CRUSH_TO_PROFILE;
                //         vTaskDelay(piston_period);
                //         run = false;
                //         break;
                //     }
                // }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    piston_timer = 0;

                    if (crush_depth)
                    {
                        /* piston task delay 1000ms */
                        if (period >= xDelay10000ms)
                        {
                            /* sensor task is already deleted */
                        }
                        else
                        {
                            /* stop here, in case of emergency blow */
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        spsEvent = MODE_CRUSH_TO_PROFILE;
                        vTaskDelay(piston_period);
                        run = false;
                        break;
                    }
                }
                else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
                {
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->finished\n");
                    piston_move = false;
                    piston_timer = 0;

                    if (crush_depth)
                    {
                        /* piston task delay 1000ms */
                        if (period >= xDelay10000ms)
                        {
                            /* sensor task is already deleted */
                        }
                        else
                        {
                            /* stop here, in case of emergency blow */
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        spsEvent = MODE_CRUSH_TO_PROFILE;
                        vTaskDelay(piston_period);
                        run = false;
                        break;
                    }
                }

                /* piston task delay 1000ms */
                if (period >= xDelay10000ms)
                {
                    vTaskDelay(piston_period);
                }

            } while (piston_move && period >= xDelay10000ms);
        }

        /* keep checking for crush depth */
        if (Depth >= CRUSH_DEPTH && !crush_depth)
        {
            /*add a CRUSH DEPTH FOS to the saved move-to-park piston length for next profile*/
            park_piston_length = length_update + 0.5;
            
            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* stop the piston */
                PIS_stop();
                piston_timer = 0;
            }

            vTaskDelay(piston_period);
            PIS_set_length(CRUSH_DEPTH_PISTON_POSITION);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: move_to_park, <<< CRUSH DEPTH activated >>>\n\n\n");
            vTaskDelay(xDelay5000ms);
        }

#endif
        vTaskDelay(period);
    }

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Task->finished\n\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_park(void)
{
    /* set crush depth to false */
    crush_depth = false;

    // --- Prepare the static park data structure for this cycle ---
    ARTEMIS_DEBUG_PRINTF("SPS :: park, Resetting static park data structure for park number %d\n", park_number);
    DATA_reset(&current_park_data);
    current_park_data.pNumber = park_number; // Explicitly set the profile number for this cycle
    // --- End data preparation ---

#ifdef TEST
    /** Sample at 9Hz */
    float s_rate = 9.0;
#else

    float s_rate = 0;
    uint32_t park_time = 0;

    if (park_number == 0)
    {
        /** Start s_rate sampling of sensors for PARK_TIME_FIRST minutes */
        s_rate = PARK_RATE_FAST;
        park_time = (xDelay1000ms * PARK_TIME_FIRST);
        ARTEMIS_DEBUG_PRINTF("\nSPS :: park, < PARK_TIME_FIRST = %.2f mins >\n\n", (float)(PARK_TIME_FIRST/60));
    }
    else
    {
        /** Start s_rate sampling of sensors for PARK_TIME_FIRST minutes */
        s_rate = PARK_RATE;
        park_time = (xDelay1000ms * PARK_TIME);
        ARTEMIS_DEBUG_PRINTF("\nSPS :: park, < PARK_TIME = %.2f mins >\n\n", (float)(PARK_TIME/60));
    }

    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);

#endif

    /* local variable to calculate the waiting time */
    uint32_t wait_time = 0;

    uint32_t park_period = xDelay1000ms/s_rate;
    TaskHandle_t xDepth = NULL;
    TaskHandle_t xTemp  = NULL;
    vTaskDelay(xDelay100ms);

    if (park_period >= xDelay10000ms)
    {
        /*do not turn on the sensors yet*/
    }
    else
    {
        /* turn on the sensors */
        SENS_sensor_temperature_on();
        SENS_sensor_depth_on();
    }

#ifdef TEST
    SENS_task_sample_depth_continuous(&xDepth);
    float Temperature = -5.0;
    uint16_t read = 0;
#else

    if (park_period >= xDelay10000ms)
    {
        /*do not start sensors task yet*/
    }
    else
    {
        /* start the sensors task */
        SENS_task_park_sensors(&xDepth, &xTemp);
    }

    float Temperature = 0.0;
    /** check for the depth rate up to XX measurements */
    uint8_t rate_count = 0;
    float rate_avg = 0;
#endif

    /** Monitor Depth and Temperature and store these */
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    Event_e spsEvent;
    rtc_time time;

    char *filename = datalogger_park_create_file(park_number);
    vTaskDelay(xDelay100ms);

    /** Set piston variables */
    eTaskState eStatus;
    TaskHandle_t xPiston = NULL;
    PIS_set_piston_rate(1);
    uint32_t piston_timer = 0;
    uint32_t piston_period = xDelay1000ms;
    float Volume = 0.0;
    float Density = 0.0;
    float Length = 0.0;
    bool piston_move = false;
    float length_update = park_piston_length;
    uint8_t park_pistonmin_try = 0;

    /* average 10 pressure, temperature values and store */
    float samples_p[10] = {0};
    float samples_t[10] = {0};
    uint8_t samples = 0;
    bool start_time = true;

    bool run = true;
    while (run)
    {
        if (park_period >= xDelay10000ms)
        {
            /* turn on the sensors */
            //SENS_task_temperature_on(&xTemp);
            datalogger_power_on();
            vTaskDelay(xDelay500ms);
            SENS_sensor_temperature_on();
            SENS_sensor_depth_on();
            vTaskDelay(xDelay10ms);

            /* start the sensors task */
            SENS_task_park_sensors(&xDepth, &xTemp);
            SENS_get_depth(&Depth, &Pressure, &Rate);
            SENS_get_temperature(&Temperature);

            vTaskDelay(xDelay100ms);
            /* turn off the sensors */
            SENS_sensor_temperature_off();
            SENS_sensor_depth_off();
            //datalogger_power_off();
        }
        else
        {
            SENS_get_depth(&Depth, &Pressure, &Rate);
#ifndef TEST
            SENS_get_temperature(&Temperature);
#endif
        }

        ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure    = %0.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth       = %0.4f m, rate = %0.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Temperature = %0.4f °C\n", Temperature);
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Epoch       = %ld\n", epoch);

        /* store first sample with start time */
        if (start_time)
        {
            DATA_add(&current_park_data, epoch, Pressure, Temperature, park_number); // changed to use current_park_data
            datalogger_park_mode(filename, Pressure, Temperature, &time);
            start_time = false;
#ifdef TEST
            read++;
#endif
        }

        /* Average Data, and store samples */
        samples_p[samples] = Pressure;
        samples_t[samples] = Temperature;
        samples++;

#ifdef TEST
        if (samples > 1)
#else
        if (samples > 9)
#endif
        {
            datalogger_power_on();
            vTaskDelay(xDelay500ms);
            float var, avg_p, avg_t;
            float std = std_div(samples_p, samples, &var, &avg_p);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            std = std_div(samples_t, samples, &var, &avg_t);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            /* store averages data locally */
            DATA_add(&current_park_data, epoch, avg_p, avg_t, park_number); // changed to use current_park_data
            samples = 0;

#ifdef TEST
            read++;
            ARTEMIS_DEBUG_PRINTF("SPS :: park, sending measurements = %u\n", read);
#else
            datalogger_park_mode(filename, avg_p, avg_t, &time);
            // Power off datalogger after logging data until next collection
            //datalogger_power_off();
            
#endif
            /* just for testing */
        }

#ifdef TEST
        /* delete this in real test */
        if (read >= 50)
        {
            run = false;
            SENS_task_delete(xDepth);
            SENS_sensor_depth_off();
            spsEvent = MODE_DONE;
        }
        /* delete */
#else

        if (Depth >= PARK_DEPTH-PARK_DEPTH_ERR && Depth <= PARK_DEPTH+PARK_DEPTH_ERR)
        {
            /* do nothing */
            //ARTEMIS_DEBUG_PRINTF("SPS :: park, within PARK_DEPTH_ERR range, do nothing\n");
            //datalogger_power_off();
        }
        else
        {
            // When leaving the target depth range, turn datalogger back on
            datalogger_power_on();
            vTaskDelay(xDelay500ms);
            
            /* collect up to PARK_DEPTH_RATE_COUNTER measurements */
            rate_avg += Rate;
            rate_count++;
            if (rate_count >= PARK_DEPTH_RATE_COUNTER)
            {
                float averaged_rate = (float) (rate_avg / rate_count);
                /* check if rate is positive, negative or stable */

                if (Depth >= PARK_DEPTH+PARK_DEPTH_ERR)
                {
                    if (averaged_rate >= 0.0 && !piston_move && !crush_depth)
                    {
                        /* increase piston position by PARK_POSITION_INCREMENT inches */
                        length_update += PARK_POSITION_INCREMENT;
                        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth Rate Positive, averaged_rate=%f, increase %fin, length_update=%.4fin\n", averaged_rate, PARK_POSITION_INCREMENT, length_update);

                        /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                        if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                        {
                            if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                            {
                                ARTEMIS_DEBUG_PRINTF("\n<< SPS :: park, Depth=%.4f is @critial piston position >>\n", Depth);
                                length_update = CRUSH_DEPTH_PISTON_POSITION;
                            }
                        }

                        park_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
                        vTaskDelay(xDelay5000ms);
                    }
                    else if (averaged_rate < 0.0 && !piston_move && !crush_depth)
                    {
                        /* increase piston position by PARK_POSITION_INCREMENT 2 inches */
                        length_update += PARK_POSITION_INCREMENT2;
                        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth Rate Positive, averaged_rate=%f, increase %fin, length_update=%.4fin\n", averaged_rate, PARK_POSITION_INCREMENT, length_update);

                        /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                        if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                        {
                            if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                            {
                                ARTEMIS_DEBUG_PRINTF("\n<< SPS :: park, Depth=%.4f is @critial piston position >>\n", Depth);
                                length_update = CRUSH_DEPTH_PISTON_POSITION;
                            }
                        }

                        park_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
                        vTaskDelay(xDelay5000ms);
                    }
                }
                else if (Depth < PARK_DEPTH-PARK_DEPTH_ERR)
                {
                    if (averaged_rate <= 0.0 && !piston_move && !crush_depth)
                    {
                        /* decrease piston position by PARK_POSITION_INCREMENT inches */
                        length_update -= PARK_POSITION_INCREMENT;
                        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth Rate Negative, averaged_rate=%f, decrease %fin, length_update=%.4fin\n", averaged_rate, PARK_POSITION_INCREMENT, length_update);

                        /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                        if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                        {
                            if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                            {
                                ARTEMIS_DEBUG_PRINTF("\n<< SPS :: park, Depth=%.4f is @critial piston position >>\n", Depth);
                                length_update = CRUSH_DEPTH_PISTON_POSITION;
                            }
                        }

                        /*check if length update will result in a piston position less than zero, set length to PISTON_POSITION_MINIMUM*/
                        if (length_update <= PISTON_POSITION_MINIMUM)
                        {
                            ARTEMIS_DEBUG_PRINTF("\n<< SPS :: park, length_update=%.4fin < piston position minimum >>\n", length_update);
                            length_update = PISTON_POSITION_MINIMUM;
                            park_pistonmin_try++;
                        }

                        /* check if it needs to move the piston or not */
                        PIS_Get_Length(&Length);
                        vTaskDelay(piston_period);

                        if (length_update <= PISTON_POSITION_MINIMUM && Length <= PISTON_POSITION_MINIMUM)
                        {
                        /* do not even send a piston command, do nothing  */
                        }
                        else if (length_update <= PISTON_POSITION_MINIMUM && park_pistonmin_try >=2 )
                        {
                            /* do not even send a piston command, do nothing  */
                        }
                        else
                        {
                        park_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
                        vTaskDelay(xDelay5000ms);
                        }
                    }
                    else if (averaged_rate > 0.0 && !piston_move && !crush_depth)
                    {
                        /* decrease piston position by PARK_POSITION_INCREMENT 2 inches */
                        length_update -= PARK_POSITION_INCREMENT2;
                        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth Rate Positive, averaged_rate=%f, decrease %fin, length_update=%.4fin\n", averaged_rate, PARK_POSITION_INCREMENT2, length_update);

                        /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                        if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                        {
                            if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                            {
                                ARTEMIS_DEBUG_PRINTF("\n<< SPS :: park, Depth=%.4f is @critial piston position >>\n", Depth);
                                length_update = CRUSH_DEPTH_PISTON_POSITION;
                            }
                        }

                        /*check if length update will result in a piston position less than zero, set length to PISTON_POSITION_MINIMUM*/
                        if (length_update <= PISTON_POSITION_MINIMUM)
                        {
                            ARTEMIS_DEBUG_PRINTF("\n<< SPS :: park, length_update=%.4fin < piston position minimum >>\n", length_update);
                            length_update = PISTON_POSITION_MINIMUM;
                        }

                        /* check if it needs to move the piston or not */
                        PIS_Get_Length(&Length);
                        vTaskDelay(piston_period);

                        if (length_update <= PISTON_POSITION_MINIMUM && Length <= PISTON_POSITION_MINIMUM)
                        {
                        /* do not even send a piston command, do nothing  */
                        }
                        else if (length_update <= PISTON_POSITION_MINIMUM && park_pistonmin_try >=2 )
                        {
                            /* do not even send a piston command, do nothing  */
                        }
                        else
                        {
                        park_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
                        vTaskDelay(xDelay5000ms);
                        }
                    }
                }
                /* reset the rate counter and rate_avg variables */
                rate_count = 0;
                rate_avg = 0.0;
            }
        }

        /* check on piston movement */
        if (piston_move)
        {
            do
            {
                eStatus = eTaskGetState( xPiston );
                if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->active\n");
                    /* keep piston time for up to 15 seconds unless crush_depth activated use piston up to 120 seconds */
                    piston_timer += piston_period;
                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(xDelay5000ms);
                            PIS_Reset();
                            piston_timer = 0;

                            /* stop here, in case of emergency blow */
                            if (park_period >= xDelay10000ms)
                            {
                                /* do nothing, tasks are already deleted and sensors are turned off */
                            }
                            else
                            {
                                SENS_task_delete(xTemp);
                                SENS_sensor_temperature_off();
                                SENS_task_delete(xDepth);
                                SENS_sensor_depth_off();
                            }

                            spsEvent = MODE_CRUSH_TO_PROFILE;
                            vTaskDelay(piston_period);
                            run = false;
                            break;
                        }
                    }
                    else
                    {
                        if (piston_timer >= 30000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(xDelay5000ms);
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                }
                // else if (eStatus==eReady)
                // {
                //     ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->Ready\n");
                //     piston_move = false;
                //     piston_timer = 0;
                    
                //     if (crush_depth)
                //     {
                //         /* stop here, in case of emergency blow */
                //         if (park_period >= xDelay10000ms)
                //         {
                //             /* do nothing, tasks are already deleted and sensors are turned off */
                //         }
                //         else
                //         {
                //             SENS_task_delete(xTemp);
                //             SENS_sensor_temperature_off();
                //             SENS_task_delete(xDepth);
                //             SENS_sensor_depth_off();
                //         }

                //         spsEvent = MODE_CRUSH_TO_PROFILE;
                //         vTaskDelay(piston_period);
                //         run = false;
                //         break;
                //     }
                // }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->suspended\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    piston_timer = 0;

                    if (crush_depth)
                    {
                        /* stop here, in case of emergency blow */
                        if (park_period >= xDelay10000ms)
                        {
                            /* do nothing, tasks are already deleted and sensors are turned off */
                        }
                        else
                        {
                            SENS_task_delete(xTemp);
                            SENS_sensor_temperature_off();
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        spsEvent = MODE_CRUSH_TO_PROFILE;
                        vTaskDelay(piston_period);
                        run = false;
                        break;
                    }
                }
                else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
                {
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->finished\n");
                    piston_move = false;
                    piston_timer = 0;

                    if (crush_depth)
                    {
                        /* stop here, in case of emergency blow */
                        if (park_period >= xDelay10000ms)
                        {
                            /* do nothing, tasks are already deleted and sensors are turned off */
                        }
                        else
                        {
                            SENS_task_delete(xTemp);
                            SENS_sensor_temperature_off();
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        spsEvent = MODE_CRUSH_TO_PROFILE;
                        vTaskDelay(piston_period);
                        run = false;
                        break;
                    }
                }

                /* piston task delay 1000ms */
                if (park_period >= xDelay10000ms)
                {
                    vTaskDelay(piston_period);
                }

            } while (piston_move && park_period >= xDelay10000ms);
        }

        /* emergency blow , extend piston to full */
        if (Depth >= CRUSH_DEPTH && !crush_depth)
        {
            /*add a CRUSH DEPTH FOS to the saved park piston length for next profile*/
            park_piston_length = length_update + 0.5;
            
            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* stop the piston */
                PIS_stop();
                piston_timer = 0;
            }

            vTaskDelay(piston_period);
            PIS_set_length(CRUSH_DEPTH_PISTON_POSITION);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: park, <<< CRUSH DEPTH activated >>>\n\n\n");
            vTaskDelay(xDelay5000ms);
        }

        /* check on Maximum park depth = ? */
        if (Depth >= PARK_DEPTH_MAX && !crush_depth)
        {
            // Ensure datalogger is on for maximum depth event
            datalogger_power_on();
            vTaskDelay(xDelay500ms);
            
            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* stop the piston */
                PIS_stop();
                vTaskDelay(piston_period);
                piston_timer = 0;
            }

            if (park_period >= xDelay10000ms)
            {
                /* do nothing, tasks are already deleted and sensors are turned off */
            }
            else
            {
                SENS_task_delete(xTemp);
                SENS_sensor_temperature_off();
                SENS_task_delete(xDepth);
                SENS_sensor_depth_off();
            }

            ARTEMIS_DEBUG_PRINTF("\n\nSPS :: park, << Reached maximum Park Depth >>\n\n");
            vTaskDelay(piston_period);
            spsEvent = MODE_DONE;
            wait_time = 0;
            run = false;
            break;
        }
#endif
        /* park depth timer */
        wait_time += park_period;
        if (wait_time >= park_time && !crush_depth)
        {
            ARTEMIS_DEBUG_PRINTF("\n\nSPS :: park, << Timer out %f mins >>\n\n", (float) (wait_time/(60.0*xDelay1000ms)));
            
            // Ensure datalogger is powered on before exiting
            datalogger_power_on();
            vTaskDelay(xDelay500ms);

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* stop the piston */
                PIS_stop();
                vTaskDelay(piston_period);
                piston_timer = 0;
            }

            if (park_period >= xDelay10000ms)
            {
                /* do nothing, tasks are already deleted and sensors are turned off */
            }
            else
            {
                SENS_task_delete(xTemp);
                SENS_sensor_temperature_off();
                SENS_task_delete(xDepth);
                SENS_sensor_depth_off();
            }

            spsEvent = MODE_DONE;
            run = false;
            vTaskDelay(piston_period);
            break;
        }
        /* task delay time */
        vTaskDelay(park_period);
    }
    
    // Log memory status after collection
    MEM_log_memory_status("SPS :: park end");
    
    // Final cleanup and exit handling
    park_number++;

    /* check Heap size */
    monitor_memory_usage();

    ARTEMIS_DEBUG_PRINTF("SPS :: park, Task->finished\n\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_move_to_profile(void)
{
    crush_depth = false;
    float Volume = 0.0;
    float Length = 0.0;
    float Density = 0.0;
    float length_update = 0.0;

#ifdef TEST
    /* do nothing */
#else

    /** Set prof_piston_length */
    /* for now , pressure and temperature variables are zeros for compressibility and thermal expansion */
    if (to_prof_piston_length == 0.0)
    {
        CTRL_set_lcp_density(TO_PROFILE_DENSITY);
        to_prof_piston_length = CTRL_calculate_piston_position(0.0, 0.0);

        /* check if to_prof_piston_length is greater than park_piston_length */
        if (to_prof_piston_length > park_piston_length)
        {
            to_prof_piston_length = park_piston_length;
            ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_profile, Setting -> park_piston_length (%.4fin) to to_prof_piston_length >>\n", park_piston_length);
        }

        length_update = to_prof_piston_length;
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_profile, Setting -> first time to_prof_piston_length=%.4fin >>\n", to_prof_piston_length);

        /*check if length update will result in a piston position less than zero, set length to PISTON_POSITION_MINIMUM*/
        if (length_update <= PISTON_POSITION_MINIMUM)
        {
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_profile, length_update=%.4fin < piston position minimum >>\n", length_update);
        length_update = PISTON_POSITION_MINIMUM;
        }
    }
    else
    {
        /* check if to_prof_piston_length is greater than park_piston_length */
        if (to_prof_piston_length > park_piston_length)
        {
            to_prof_piston_length = park_piston_length;
            ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_profile, Setting -> park_piston_length (%.4fin) to to_prof_piston_length >>\n", park_piston_length);
        }

        /* assigned to local_length, assumed that length was set previously */
        length_update = to_prof_piston_length;
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_profile, Setting -> adjusted to_prof_piston_length=%.4fin >>\n", to_prof_piston_length);
    }

    Volume = CTRL_calculate_volume_from_length(length_update);
    Density = CTRL_calculate_lcp_density(Volume);
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Setting -> Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n\n", Density, Volume, length_update);

    /** move to length */
    uint32_t piston_period = xDelay1000ms;
    uint32_t piston_timer = 0;
    bool piston_move = true;
    TaskHandle_t xPiston = NULL;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_set_length(length_update);
    PIS_task_move_length(&xPiston);
    vTaskDelay(xDelay5000ms);

    /* check on piston movement */
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->active\n");
            /* piston time for up to 60 seconds */
            piston_timer += piston_period;
            if (piston_timer >= 60000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out, task->finished\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                PIS_Reset();
                //piston_move = false;
                piston_timer = 0;
            }
        }
        // else if (eStatus==eReady)
        // {
        //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
        //     piston_move = false;
        //     piston_timer = 0;
        // }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            vTaskDelay(xDelay5000ms);
            //piston_move = false;
            piston_timer = 0;
        }
        else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
        {
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
        }
        vTaskDelay(piston_period);
    }
    vTaskDelay(xDelay5000ms);

    /** Start Sampling @2Hz or user settable in the config.h file */
    float s_rate = MOVE_TO_PROFILE_SAMPLE_RATE;
    uint32_t period = xDelay1000ms/s_rate;
    SENS_set_depth_rate(s_rate);
    TaskHandle_t xDepth = NULL;

    if (period >= xDelay10000ms)
    {
        /*do not turn on the sensors yet*/
    }
    else
    {
        /* turn on the sensor */
        SENS_sensor_depth_on();
        vTaskDelay(xDelay10ms);
        /* start sampling the sensor */
        SENS_task_sample_depth_continuous(&xDepth);
    }

#endif

#ifdef TEST
    bool run = false;
    Event_e spsEvent;
    spsEvent = MODE_DONE;
#else

    /** Monitor Depth, when depth reached, mode is complete */
    Event_e spsEvent;

    float Depth = 0, Rate = 0;
    float Pressure = 0;
    /** check for the depth rate up to XX measurements */
    uint8_t rate_count = 0;
    float rate_avg = 0;

    /* time for critical piston minimum position */
    uint32_t to_profile_state_time = 0;
    uint8_t to_profile_pistonmin_try = 0;

    /* variables for checking if the LCP hit the bottom for PISTON_MOVEMENT_ON_BOTTOM (inches) movement */
    //PIS_Get_Length(&Length);
    //vTaskDelay(piston_period);
    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_profile, << Setting Length %.4fin to piston_on_bottom_length variable >>\n\n", length_update);
    float piston_on_bottom_length = length_update;
    float length_update_last_adjusted = length_update;

    bool run = true;
    while (run)
    {
        if (period >= xDelay10000ms)
        {
            /* turn on the sensors */
            SENS_sensor_depth_on();
            vTaskDelay(xDelay10ms);
            /* start sampling the sensor */
            SENS_task_sample_depth_continuous(&xDepth);

            SENS_get_depth(&Depth, &Pressure, &Rate);
            vTaskDelay(xDelay10ms);
            /* turn off the sensors */
            SENS_sensor_depth_off();
        }
        else
        {
            SENS_get_depth(&Depth, &Pressure, &Rate);
        }

        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure = %0.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth    = %0.4f m, rate = %0.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));

        /* collect up to XX measurements */
        rate_avg += Rate;
	    rate_count++;
	    if (rate_count >= 3)
	    {
            float averaged_rate = (float) (rate_avg / (float)(rate_count));

            /* check if rate is positive, negative or stable */
            if (averaged_rate >= SYSTEM_FALL_RATE_MIN)
            {
                /* do nothing for now, */
                //ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth Rate is positive and >= %.4fm/%.1fs\n", SYSTEM_FALL_RATE_MIN, (float)(1/s_rate));
                //ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth Rate is positive\n");

                /* keep updating the piston_on_bottom_length to length_update */
                piston_on_bottom_length = length_update;
            }
            else if (averaged_rate < SYSTEM_FALL_RATE_MIN && !piston_move && !crush_depth)
            {
                /* decrease piston position by PARK_POSITION_INCREMENT inches */
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth averaged_rate = %f, decrease %fin\n", averaged_rate, PARK_POSITION_INCREMENT);
                length_update -= PARK_POSITION_INCREMENT;

                /* check if LCP hits the bottom already */
                if ( (piston_on_bottom_length-length_update) <= PISTON_MOVEMENT_ON_BOTTOM)
                {
                    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_profile, LCP is not hitting the bottom\n\n");
                }
                else if ( (piston_on_bottom_length-length_update) > PISTON_MOVEMENT_ON_BOTTOM)
                {
                    ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_profile, LCP presumably hitting the bottom >>\n\n");

                    /* set previous adjusted length update */
                    to_prof_piston_length = length_update_last_adjusted;
                    /* move to the next state */
                    to_profile_state_time = 0;
                    /* reset the rate counter and rate_avg*/
                    rate_count = 0;
                    rate_avg = 0.0;

                    /* stop here, and delete the task and turn off pressure sensor, move to next state */
                    if (period >= xDelay10000ms)
                    {
                        /* sensor task is already deleted */
                    }
                    else
                    {
                        SENS_task_delete(xDepth);
                        SENS_sensor_depth_off();
                    }

                    spsEvent = MODE_DONE;
                    run = false;
                    /* I guess break the loop*/
                    break;
                }

                if (length_update <= PISTON_POSITION_MINIMUM)
                {
                    length_update = PISTON_POSITION_MINIMUM;
                    to_profile_pistonmin_try++;

                    if (Depth < PARK_DEPTH-PARK_DEPTH_ERR)
                    {
                        to_profile_state_time = TO_PROFILE_STATE_TIMER*xDelay1000ms;
                    }

                    /* start the timer for TO_PROFILE_STATE_TIMER (20 mins) */
                    to_profile_state_time += (period * rate_count);
                    ARTEMIS_DEBUG_PRINTF("<< SPS :: move_to_profile, critical piston minimum position time = %.2f seconds >>\n", (float)to_profile_state_time/xDelay1000ms);
                    
                    if (to_profile_state_time >= TO_PROFILE_STATE_TIMER*xDelay1000ms)
                    {
                        ARTEMIS_DEBUG_PRINTF("<< SPS :: move_to_profile, critical piston minimum position time out = %.2f mins >>\n", (float)to_profile_state_time/(60.0*xDelay1000ms));
                        /* set previous adjusted length update */
                        to_prof_piston_length = length_update_last_adjusted;
                        /* move to profile state */
                        to_profile_state_time = 0;

                        /* stop here, and delete the task and turn off pressure sensor, move to next state */
                        if (period >= xDelay10000ms)
                        {
                            /* sensor task is already deleted */
                        }
                        else
                        {
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        run = false;
                        spsEvent = MODE_DONE;
                    }
                }
                /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                else if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_profile, Depth=%.4f m is @critial piston position >>\n", Depth);
                        length_update = CRUSH_DEPTH_PISTON_POSITION;
                    }
                }

                /* check if it needs to move the piston or not */
                PIS_Get_Length(&Length);
                vTaskDelay(piston_period);

                if (length_update <= PISTON_POSITION_MINIMUM && Length <= PISTON_POSITION_MINIMUM)
                {
                    /* do not even send a piston command, do nothing  */
                }
                else if (length_update <= PISTON_POSITION_MINIMUM && to_profile_pistonmin_try >=2 )
                {
                    /* do not even send a piston command, do nothing  */
                }
                else if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        /* do not even send a piston command, do nothing  */
                    }
                    else
                    {
                        to_prof_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
                        vTaskDelay(xDelay5000ms);
                    }
                }
                else
                {
                    to_prof_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
                    vTaskDelay(xDelay5000ms);
                }
            }
            /* reset the rate counter and rate_avg*/
            rate_count = 0;
            rate_avg = 0.0;
        }

        /* check on depth to reach */
        if (Depth >= PROFILE_DEPTH-PROFILE_DEPTH_ERR && !crush_depth)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure Reached = %0.4f bar\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth Reached    = %0.4f m, rate = %0.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Reach Profile Depth\n");

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* stop the piston */
                PIS_stop();
                piston_move = false;
                piston_timer = 0;
                vTaskDelay(period);
            }

            /* piston task delay 1000ms */
            if (period >= xDelay10000ms)
            {
                /* sensor task is already deleted */
            }
            else
            {
                /* stop here, and delete the task and turn off pressure sensor, move to next state */
                SENS_task_delete(xDepth);
                SENS_sensor_depth_off();
            }

            run = false;
            spsEvent = MODE_DONE;
        }

        /* check on piston movement */
        if (piston_move)
        {
            do
            {
                eStatus = eTaskGetState( xPiston );
                if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->active\n");
                    /* keep piston time for up to 15 seconds unless crush_depth activated use piston up to 120 seconds */
                    piston_timer += piston_period;
                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(xDelay5000ms);
                            PIS_Reset();
                            piston_timer = 0;
                            
                            /* piston task delay 1000ms */
                            if (period >= xDelay10000ms)
                            {
                                /* sensor task is already deleted */
                            }
                            else
                            {
                            /* stop here, in case of emergency blow */
                                SENS_task_delete(xDepth);
                                SENS_sensor_depth_off();
                            }

                            /* stop here, in case of emergency blow */
                            spsEvent = MODE_CRUSH_TO_PROFILE;
                            vTaskDelay(piston_period);
                            run = false;
                            break;
                        }
                    }
                    else
                    {
                        if (piston_timer >= 30000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(xDelay5000ms);
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                }
                // else if (eStatus==eReady)
                // {
                //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->ready\n");
                //     piston_move = false;
                //     piston_timer = 0;
                    
                //     if (crush_depth)
                //     {
                //         /* piston task delay 1000ms */
                //         if (period >= xDelay10000ms)
                //         {
                //             /* sensor task is already deleted */
                //         }
                //         else
                //         {
                //             /* stop here, in case of emergency blow */
                //             SENS_task_delete(xDepth);
                //             SENS_sensor_depth_off();
                //         }

                //         /* stop here, in case of emergency blow */
                //         spsEvent = MODE_CRUSH_TO_PROFILE;
                //         vTaskDelay(piston_period);
                //         run = false;
                //         break;
                //     }
                // }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->suspended\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    piston_timer = 0;

                    if (crush_depth)
                    {
                        /* piston task delay 1000ms */
                        if (period >= xDelay10000ms)
                        {
                            /* sensor task is already deleted */
                        }
                        else
                        {
                            /* stop here, in case of emergency blow */
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        /* stop here, in case of emergency blow */
                        spsEvent = MODE_CRUSH_TO_PROFILE;
                        vTaskDelay(piston_period);
                        run = false;
                        break;
                    }
                }
                else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
                {
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->finished\n");
                    piston_move = false;
                    piston_timer = 0;

                    if (crush_depth)
                    {
                        /* piston task delay 1000ms */
                        if (period >= xDelay10000ms)
                        {
                            /* sensor task is already deleted */
                        }
                        else
                        {
                            /* stop here, in case of emergency blow */
                            SENS_task_delete(xDepth);
                            SENS_sensor_depth_off();
                        }

                        /* stop here, in case of emergency blow */
                        spsEvent = MODE_CRUSH_TO_PROFILE;
                        vTaskDelay(piston_period);
                        run = false;
                        break;
                    }
                }

                /* piston task delay 1000ms */
                if (period >= xDelay10000ms)
                {
                    vTaskDelay(piston_period);
                }

            } while (piston_move && period >= xDelay10000ms);
        }

        /* keep checking for crush depth */
        if (Depth >= CRUSH_DEPTH && !crush_depth)
        {
            
            // Ensure datalogger is on for emergency events
            datalogger_power_on();
            vTaskDelay(xDelay500ms);
            
            /*add a CRUSH DEPTH FOS to the saved move-to-profile piston length for next profile*/
            to_prof_piston_length = length_update + 0.5;

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* stop the piston */
                PIS_stop();
            }

            vTaskDelay(piston_period);
            PIS_set_length(CRUSH_DEPTH_PISTON_POSITION);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            piston_timer = 0;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\nSPS :: move_to_profile, <<< CRUSH DEPTH activated >>>\n\n");
            vTaskDelay(xDelay5000ms);
        }
        vTaskDelay(period);
    }

#endif

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_profile, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Task->finished\n\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_profile(void)
{
    // --- Prepare the static profile data structure for this cycle ---
    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Resetting static profile data structure for profile number %d\n", prof_number);
    DATA_reset(&current_profile_data);
    current_profile_data.pNumber = prof_number; // Explicitly set the profile number for this cycle
    // --- End static profile data structure preparation ---
    
    float Volume = 0.0;
    float Density = 0.0;
    float Length = 0.0;
    float length_update = 0.0;
    bool surface = false;
    uint32_t surface_timer = 0;
    bool surface_piston = false;

#ifdef TEST
    /** Start Depth and Temperature Sensor @ 9Hz */
    float s_rate = 9.0;
    uint32_t period = xDelay1000ms/s_rate;
    SENS_set_depth_rate(s_rate);
    SENS_sensor_gps_off();
    TaskHandle_t xDepth = NULL;
    SENS_sensor_depth_on();
#else
    /** Start Depth and Temperature Sensor @ 1Hz */
    float s_rate = PROFILE_RATE;
    uint32_t period = xDelay1000ms/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);
    SENS_sensor_gps_off();
    SENS_sensor_depth_on();
    SENS_sensor_temperature_on();
    TaskHandle_t xDepth = NULL;
    TaskHandle_t xTemp  = NULL;
    vTaskDelay(xDelay100ms);
#endif

#ifdef TEST
    /* do nothing */
#else

    /** Calculate length for profiling at 0.1m/s upward , do length adjustment */
    if (prof_piston_length == 0.0)
    {
        float v_rate = SYSTEM_RISE_RATE_SETPOINT;
        Volume = module_ctrl_set_buoyancy_from_rate(v_rate, false);
        Density = CTRL_calculate_lcp_density(Volume);
        prof_piston_length = CTRL_calculate_length_from_volume(Volume);
        length_update = prof_piston_length;
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Setting -> first time prof_piston_length=%.4f >>\n", prof_piston_length);

        /*check if length update will result in a piston position less than zero, set length to PISTON_POSITION_MINIMUM*/
        if (length_update <= PISTON_POSITION_MINIMUM)
        {
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, length_update=%.4fin < piston position minimum >>\n", length_update);
        length_update = PISTON_POSITION_MINIMUM;
        }

    }
    else
    {
        length_update = prof_piston_length;
        Volume = CTRL_calculate_volume_from_length(length_update);
        Density = CTRL_calculate_lcp_density(Volume);
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Setting -> adjusted prof_piston_length=%.4f >>\n", prof_piston_length);
    }
    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Setting -> Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n\n", Density, Volume, length_update);

    /* time for critical depth piston position */
    uint32_t crit_depth_piston_pos_time = 0;
    float length_update_last_adjusted = length_update;
    float length_update_surf_adjusted = length_update;
    
    /*If entering profile state because exceeded Crush_Depth*/
    if (crush_depth)
    {
        length_update = CRUSH_DEPTH_PISTON_POSITION;
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Setting -> ENTERING PROFILE CRUSH DEPTH Activation Length=%.4fin\n\n", length_update);
    }

    /** Set volume or length */
    uint32_t piston_period = xDelay1000ms;
    bool piston_move = true;
    uint32_t piston_timer = 0;

    eTaskState eStatus;
    TaskHandle_t xPiston = NULL;
    PIS_set_piston_rate(1);

    /* check the length_update if it is greater than CRUSH_DEPTH_PISTON_POSITION (5.25in) */
    if (length_update > CRUSH_DEPTH_PISTON_POSITION && (PROFILE_DEPTH-PROFILE_DEPTH_ERR) >= CRITICAL_PISTON_POSITON_DEPTH)
    {
        length_update = CRUSH_DEPTH_PISTON_POSITION;
    }

    PIS_set_length(length_update);
    PIS_task_move_length(&xPiston);
    vTaskDelay(xDelay5000ms);

    /* check on piston movement */
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->active\n");
            /* piston time for up to 120 seconds */
            piston_timer += piston_period;
            if (piston_timer >= 120000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                PIS_Reset();
                piston_timer = 0;
            }
        }
        // else if (eStatus==eReady)
        // {
        //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
        //     piston_move = false;
        //     piston_timer = 0;
        // }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            vTaskDelay(xDelay5000ms);
            piston_timer = 0;
        }
        else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->finished\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            piston_move = false;
            piston_timer = 0;
        }
        vTaskDelay(piston_period);
    }
    vTaskDelay(xDelay5000ms);
#endif

#ifdef TEST
    SENS_task_sample_depth_continuous(&xDepth);
    float Temperature = 20.0;
    uint16_t read = 0;
#else
    SENS_task_profile_sensors(&xDepth, &xTemp);
    float Temperature = 0.0;
#endif

    /** Start recording samples */
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    Event_e spsEvent;
    rtc_time time;

    /** Monitor depth & rate.  If rate fails, fix it.  If depth reaches surface, done */
    char *filename = datalogger_profile_create_file(prof_number);
    vTaskDelay(xDelay100ms);

    /* average 10 pressure, temperature values and store */
    float samples_p = 0.0;
    float samples_t = 0.0;
    uint16_t samples = 0;

    ///* average 10 pressure, temperature values and store */
    //float samples_p[10] = {0};
    //float samples_t[10] = {0};
    //uint8_t samples = 0;
    bool start_time = true;
    bool run = true;
    bool start_fall = true;

    /** check for the depth rate up to ~1min - 60 measurements */
    uint8_t rate_count = 0;
    float rate_avg = 0;

    /* variable to align the measurements for 1m bin */
    float bin_pressure = 0.0;

    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);
#ifdef TEST
        /* do nothing */
#else
        SENS_get_temperature(&Temperature);
#endif
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure    = %0.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Depth       = %0.4f m, rate = %0.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature = %0.4f °C\n", Temperature);

        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Epoch       = %ld \n", epoch);

        ///* collect all values in the datalogger */
        //datalogger_profile_mode(filename, Depth, Temperature, &time);

        /* store first sample with start time */
        if (start_time == true)
        {
            DATA_add(&current_profile_data, epoch, Pressure, Temperature, prof_number); // Changed to use the static profile data structure
            datalogger_profile_mode(filename, Pressure, Temperature, &time);
            start_time = false;
            bin_pressure = ceil(Pressure * 10) / 10;
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, profile start bin = %0.4f bar\n", bin_pressure);
#ifdef TEST
            read++;
#endif
        }

        /* Average Data, and store samples */
        //samples_p[samples] = Pressure;
        //samples_t[samples] = Temperature;
        samples_p += Pressure;
        samples_t += Temperature;
        samples++;

#ifdef TEST
        if (samples > 1)
        {
            /* update the bin_pressure variable */
            bin_pressure = bin_pressure - 0.1;

            float avg_p = (float) samples_p / samples;
            float avg_t = (float) samples_t / samples;

            /* Note : commented out the buffer for temperature and pressure , used for calculating the variance and the std_div */

            //float var, avg_p, avg_t;
            //float std = std_div(samples_p, samples, &var, &avg_p);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            //std = std_div(samples_t, samples, &var, &avg_t);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature and Pressure -> number of samples = %u\n", samples);

            /* store averaged data locally */
            //DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
            DATA_add(&current_profile_data, epoch, avg_p, avg_t, prof_number); // Changed to use the static profile data structure
            samples = 0;
            samples_p = 0;
            samples_t = 0;
            read++;
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, sending measurements = %u\n", read);
        }
#else
        //if (samples > 9)
        /* Note : This routine bins the data into 1m averages that can move up and down if the LCP sinks durring the profile.  WARNING MAY RESULT IN > 230 SBD MEASUREMENTS */
        if (Pressure <= (bin_pressure - 0.1))
        {
            /* update the bin_pressure variable */
            bin_pressure = bin_pressure - 0.1;

            float avg_p = (float) samples_p / samples;
            float avg_t = (float) samples_t / samples;

            /* Note : commented out the buffer for temperature and pressure , used for calculating the variance and the std_div */

            //float var, avg_p, avg_t;
            //float std = std_div(samples_p, samples, &var, &avg_p);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            //std = std_div(samples_t, samples, &var, &avg_t);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature and Pressure -> number of samples = %u\n", samples);

            /* store averaged data locally */
            DATA_add(&current_profile_data, epoch, avg_p, avg_t, prof_number); // Changed to use the static profile data structure
            samples = 0;
            samples_p = 0;
            samples_t = 0;

            datalogger_profile_mode(filename, avg_p, avg_t, &time);
        }
        if (Pressure >= bin_pressure)
        {
            /* update the bin_pressure variable */
            bin_pressure = bin_pressure + 0.1;

            float avg_p = (float) samples_p / samples;
            float avg_t = (float) samples_t / samples;

            /* Note : commented out the buffer for temperature and pressure , used for calculating the variance and the std_div */

            //float var, avg_p, avg_t;
            //float std = std_div(samples_p, samples, &var, &avg_p);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            //std = std_div(samples_t, samples, &var, &avg_t);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature and Pressure -> number of samples = %u\n", samples);

            /* store averaged data locally */
            DATA_add(&current_profile_data, epoch, avg_p, avg_t, prof_number); // Changed to use the static profile data structure
            samples = 0;
            samples_p = 0;
            samples_t = 0;

            datalogger_profile_mode(filename, avg_p, avg_t, &time);
        }
/*

       //Note : This routine bins the data into 1m averages that can only move up, if the LCP sinks during the profile the data is ignored.
        if (Pressure <= (bin_pressure - 0.1))
        {
            // update the bin_pressure variable
            bin_pressure = bin_pressure - 0.1;

            float avg_p = (float) samples_p / samples;
            float avg_t = (float) samples_t / samples;

            // Note : commented out the buffer for temperature and pressure , used for calculating the variance and the std_div

            //float var, avg_p, avg_t;
            //float std = std_div(samples_p, samples, &var, &avg_p);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            //std = std_div(samples_t, samples, &var, &avg_t);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature and Pressure -> number of samples = %u\n", samples);

            // store averaged data locally
            //DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
            DATA_add(&current_profile_data, epoch, avg_p, avg_t, prof_number); // Changed to use the static profile data structure
            samples = 0;
            samples_p = 0;
            samples_t = 0;
            start_fall = true;

            datalogger_profile_mode(filename, avg_p, avg_t, &time);
        }
        if (Pressure >= bin_pressure && start_fall == true)
        {
            float avg_p = (float) samples_p / samples;
            float avg_t = (float) samples_t / samples;

            // Note : commented out the buffer for temperature and pressure , used for calculating the variance and the std_div

            //float var, avg_p, avg_t;
            //float std = std_div(samples_p, samples, &var, &avg_p);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            //std = std_div(samples_t, samples, &var, &avg_t);
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature and Pressure -> number of samples = %u\n", samples);

            //store averaged data locally
            //DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
            DATA_add(&current_profile_data, epoch, avg_p, avg_t, prof_number); // Changed to use the static profile data structure
            samples = 0;
            samples_p = 0;
            samples_t = 0;
            start_fall = false;

            datalogger_profile_mode(filename, avg_p, avg_t, &time);
        }
        if (Pressure >= bin_pressure && start_fall == false)
        {
            samples = 0;
            samples_p = 0;
            samples_t = 0;
        }
*/
#endif

#ifdef TEST
        /* delete this */
        if (read >= 50)
        {
            run = false;
            SENS_task_delete(xDepth);
            vTaskDelay(xDelay500ms);
            SENS_sensor_depth_off();
            spsEvent = MODE_DONE;
        }
        /* delete this end */
#else

        /* collect up to PROFILE_DEPTH_RATE_COUNTER measurements */
        rate_avg += Rate;
	    rate_count++;
	    if (rate_count >= PROFILE_DEPTH_RATE_COUNTER)
	    {
            float averaged_rate = (float) (rate_avg/rate_count);
            if ( averaged_rate >= (-1*SYSTEM_RISE_RATE_MIN) && !piston_move)
            {
                /* increase piston position by PARK_POSITION_INCREMENT inches */
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Depth Rate = %.4f, increase %fin\n", averaged_rate, PARK_POSITION_INCREMENT);
                length_update += PARK_POSITION_INCREMENT;

                /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Depth=%.4f is @critical piston position >>\n", Depth);
                        length_update = CRUSH_DEPTH_PISTON_POSITION;
                        if(!crush_depth)
                        {
                            /* set a timer for 5 mins ? , period = xDelay1000ms , it can change in case of TEST */
                            crit_depth_piston_pos_time += (period * rate_count);
                            ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Critical Depth Piston Position Timer = %f seconds >>\n", (float) crit_depth_piston_pos_time/period);
                            if (crit_depth_piston_pos_time >= period*SYSTEM_CDPP_TIMER)
                            {
                                /* set to previous adjusted length update */
                                prof_piston_length = length_update_last_adjusted;
                                /* send it to -> Critical Park State */
                                ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Critical Depth Piston Position Time out %f mins >>\n", (float) SYSTEM_CDPP_TIMER/60);
                                critical_park_state = true;
                                crit_depth_piston_pos_time = 0;
                                spsEvent = MODE_CRIT_TO_PARK;

                                /* turn off the sensors */
                                SENS_task_delete(xTemp);
                                SENS_sensor_temperature_off();
                                SENS_task_delete(xDepth);
                                SENS_sensor_depth_off();
                                piston_move = false;
                                run = false;
                                /* break here */
                                break;
                            }
                        }
                    }
                    else if (Depth < CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        if (length_update >= PISTON_POSITION_MAXIMUM)
                        {
                            ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Depth=%.4f is @Maximum piston position >>\n", Depth);
                            length_update = PISTON_POSITION_MAXIMUM;

                            /* set a timer for 5 mins ? , period = xDelay1000ms , it can change in case of TEST */
                            crit_depth_piston_pos_time += (period * rate_count);
                            ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Critical Depth Piston Position Timer = %f seconds >>\n", (float) crit_depth_piston_pos_time/period);
                            if (crit_depth_piston_pos_time >= period*SYSTEM_CDPP_TIMER)
                            {
                                /* set to previous adjusted length update */
                                prof_piston_length = length_update_last_adjusted;
                                /* send it to -> Critical Park State */
                                ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Critical Depth Piston Position Time out %f mins >>\n", (float) SYSTEM_CDPP_TIMER/60);
                                critical_park_state = true;
                                crit_depth_piston_pos_time = 0;
                                spsEvent = MODE_CRIT_TO_PARK;

                                /* turn off the sensors */
                                SENS_task_delete(xTemp);
                                SENS_sensor_temperature_off();
                                SENS_task_delete(xDepth);
                                SENS_sensor_depth_off();
                                piston_move = false;
                                run = false;
                                /* break here */
                                break;
                            }
                        }
                    }
                }

                /* check if it needs to move the piston or not */
                if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    PIS_Get_Length(&Length);
                    vTaskDelay(piston_period);

                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        if (Length >= (CRUSH_DEPTH_PISTON_POSITION - 0.01))
                        {
                            /* do not send the piston command */
                        }
                         else
                        {
                            prof_piston_length = length_update;
                            PIS_set_length(length_update);
                            PIS_task_move_length(&xPiston);
                            piston_move = true;
                            vTaskDelay(xDelay5000ms);
                        }
                    }
                    else if (Depth < CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        if (length_update >= PISTON_POSITION_MAXIMUM && Length >= (PISTON_POSITION_MAXIMUM -.01))
                        {
                            /* do not send the piston command */
                        }
                        else
                        {
                            prof_piston_length = length_update;
                            PIS_set_length(length_update);
                            PIS_task_move_length(&xPiston);
                            piston_move = true;
                            vTaskDelay(xDelay5000ms);
                        }
                    }
                }
                else
                {
                    prof_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
                    vTaskDelay(xDelay5000ms);
                }
            }
            else if ( averaged_rate < (-1*SYSTEM_RISE_RATE_MAX) && !piston_move && !crush_depth)
            {
                /* decrease piston position by PARK_POSITION_INCREMENT2 inches */
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Depth Rate = %.4f, decrease %fin\n", averaged_rate, PARK_POSITION_INCREMENT2);
                length_update -= PARK_POSITION_INCREMENT2;

                /* check critcal depth for piston, do not increase piston beyond 5.25in when depth greater than 50m */
                if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Depth=%.4f is @critial piston position >>\n", Depth);
                        length_update = CRUSH_DEPTH_PISTON_POSITION;

                        ///* set a timer for 5 mins ? , period = xDelay1000ms , it can change in case of TEST */
                        //crit_depth_piston_pos_time += period;
                        //ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Critical Depth Piston Position Timer = %f seconds >>\n", (float) crit_depth_piston_pos_time/period);
                        //if (crit_depth_piston_pos_time >= period*SYSTEM_CDPP_TIMER)
                        //{
                        //    /* set to previous adjusted length update */
                        //    prof_piston_length = length_update_last_adjusted;
                        //
                        //    /* send it to -> Critical Park State */
                        //    ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, Critical Depth Piston Position Time out %f mins >>\n", (float) SYSTEM_CDPP_TIMER/60);
                        //    critical_park_state = true;
                        //    crit_depth_piston_pos_time = 0;
                        //    spsEvent = MODE_CRIT_TO_PARK;

                        //    /* turn off the sensors */
                        //    SENS_task_delete(xTemp);
                        //    SENS_sensor_temperature_off();
                        //    SENS_task_delete(xDepth);
                        //    SENS_sensor_depth_off();
                        //    piston_move = false;
                        //    run = false;
                        //    /* break here */
                        //    break;
                        //}
                    }
                }

                /*check if length update will result in a piston position less than zero, set length to PISTON_POSITION_MINIMUM*/
                if (length_update <= PISTON_POSITION_MINIMUM)
                {
                    ARTEMIS_DEBUG_PRINTF("\n<< SPS :: profile, length_update=%.4fin < piston position minimum >>\n", length_update);
                    length_update = PISTON_POSITION_MINIMUM;
                }

                /* check if it needs to move the piston or not */
                if (length_update >= CRUSH_DEPTH_PISTON_POSITION)
                {
                    PIS_Get_Length(&Length);
                    vTaskDelay(piston_period);

                    if (Depth >= CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        if (Length >= (CRUSH_DEPTH_PISTON_POSITION - 0.01))
                        {
                            /* do not send the piston command */
                        }
                         else
                        {
                            prof_piston_length = length_update;
                            PIS_set_length(length_update);
                            PIS_task_move_length(&xPiston);
                            piston_move = true;
                            vTaskDelay(xDelay5000ms);
                        }
                    }
                    else if (Depth < CRITICAL_PISTON_POSITON_DEPTH)
                    {
                        if (length_update >= PISTON_POSITION_MAXIMUM && Length >= (PISTON_POSITION_MAXIMUM -.01))
                        {
                            /* do not send the piston command */
                        }
                        else
                        {
                            prof_piston_length = length_update;
                            PIS_set_length(length_update);
                            PIS_task_move_length(&xPiston);
                            piston_move = true;
                            vTaskDelay(xDelay5000ms);
                        }
                    }
                }
                else
                {
                    prof_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
                    vTaskDelay(xDelay5000ms);
                }
            }
            /* reset the rate counter and rate_avg*/
            rate_count = 0;
            rate_avg = 0.0;
        }

        /* check on piston movement */
        if (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->active\n");
                /* keep piston time for up to 15 seconds unless crush_depth activated or at the surface use piston up to 180 seconds */
                piston_timer += period;

                if ( (crush_depth) || (Depth <= BALLAST_DEPTH_PROFILE) )
                {
                    if (piston_timer >= 180000)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston CRUSH or SURFACE time-out, task->finished\n");
                        PIS_task_delete(xPiston);
                        vTaskDelay(xDelay5000ms);
                        PIS_Reset();
                        piston_timer = 0;
                    }
                }
                else
                {
                    if (piston_timer >= 30000)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");
                        PIS_task_delete(xPiston);
                        vTaskDelay(xDelay5000ms);
                        PIS_Reset();
                        piston_timer = 0;
                    }
                }
            }
            // else if (eStatus==eReady)
            // {
            //     ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->ready\n");
            //     //PIS_task_delete(xPiston);
            //     piston_move = false;
            //     piston_timer = 0;
            // }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->suspended\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                piston_timer = 0;
            }
            else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
            {
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->finished\n");
                piston_move = false;
                piston_timer = 0;
            }
        }

        /* emergency blow , extend piston to full */
        if (Depth >= CRUSH_DEPTH && !crush_depth)
        {
            /*add a CRUSH DEPTH FOS to the saved profile piston length for next profile*/
            length_update_last_adjusted = length_update + 0.5;
            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* try to stop first*/
                PIS_stop();
                piston_timer = 0;
            }
            length_update = CRUSH_DEPTH_PISTON_POSITION;
            vTaskDelay(piston_period);
            PIS_set_length(CRUSH_DEPTH_PISTON_POSITION);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: profile, <<< CRUSH DEPTH activated >>>\n\n\n");
            vTaskDelay(xDelay5000ms);
        }

        if (Depth <= BALLAST_DEPTH_PROFILE && !surface)
        {
            if (!surface_piston)
            {
                length_update_surf_adjusted = length_update;

                /* check if piston is still moving then reset it and stop */
                if (piston_move)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: profile, deliberately stopping the Piston\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    /* try to stop first*/
                    PIS_stop();
                    piston_move = false;   
                } 
                /* check Heap size */
                uint32_t sizeA = xPortGetFreeHeapSize();
                ARTEMIS_DEBUG_PRINTF("\nSPS :: profile, FreeRTOS HEAP SIZE A = %u Bytes\n\n", sizeA);
                
                /* Move piston all the way to the surface setting */
                // #if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
                //     /* set piston to 10.5in */
                //     length_update = 10.5;
                // #else
                length_update = PISTON_MOVE_TO_SURFACE;
                //#endif
                vTaskDelay(piston_period);
                PIS_set_length(PISTON_MOVE_TO_SURFACE);
                PIS_task_move_length(&xPiston);
                piston_move = true;
                surface_piston = true;
                vTaskDelay(xDelay5000ms);
            }

             /*start a 210 second timer to keep recording until LCP reaches the surface*/
            surface_timer += period;
            if (surface_timer >= 210000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, surface time-out, task->finished\n");
                surface = true;
                surface_timer = 0;
            }
        }

        if (Depth <= BALLAST_DEPTH_PROFILE && surface)
        {
            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                /* try to stop first*/
                PIS_stop();
                piston_move = false;   
            }

            if (crush_depth)
            {
                /* set to previous adjusted length update */
                prof_piston_length = length_update_last_adjusted;
            }
            else
            {
                /* set to profile length update before surface move */
                prof_piston_length = length_update_surf_adjusted;
            }
          
            crush_depth = false;
            surface = false;
            SENS_task_delete(xTemp);
            SENS_sensor_temperature_off();
            SENS_task_delete(xDepth);
            SENS_sensor_depth_off();
            run = false;
            spsEvent = MODE_DONE;
        }
#endif
        vTaskDelay(period);
    }
    
    // Log memory status after collection
    MEM_log_memory_status("SPS :: profile end");

    prof_number++;
    pistonzero_number++;
    pistonfull_number++;

    // Log memory usage
    monitor_memory_usage();

    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Task->finished\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_move_to_surface(void)
{
    uint32_t piston_period = xDelay1000ms;
    uint32_t piston_timer = 0;
    bool piston_move = false;
    float Volume = 0.0;
    float Density = 0.0;
    float Length = 0.0;
    float lengthadjust = 0.0;
    float lengthdrift  = 0.0;

    eTaskState eStatus;
    TaskHandle_t xPiston = NULL;
    PIS_set_piston_rate(1);

    #if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
        /* set piston to 10.5in */
        PIS_set_length(10.5);
    #else
        PIS_set_length(PISTON_MOVE_TO_SURFACE);
    #endif

    /*Check if it is time to reset the piston encoder counts to max at full piston extent, if yes, then run reset encoder to max value at full*/
    if( pistonfull_number >= PISTON_FULLCAL_COUNTER )
    {
        Length = PISTON_POSITION_ATFULLRESET;
        piston_timer = 0;
        piston_move = true;

        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_surface, Piston move to full, %u profiles reached since last encoder full reset >>\n\n", pistonfull_number);
        PIS_task_move_full(&xPiston); /*This is the move piston full command*/
        vTaskDelay(xDelay5000ms); 

        /* check on piston movement */
        while (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {   
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full task->active\n");
                /* piston time for up to 180 seconds */
                piston_timer += piston_period;
                if (piston_timer >= 180000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full time-out, task->finished\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    PIS_Reset();
                    piston_timer = 0;
                }
            }
            // else if (eStatus==eReady)
            // {
            //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->Ready\n");
            //     piston_move = false;
            //     piston_timer = 0;
            // }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full task->suspended\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                piston_timer = 0;
            }
            else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
            {
                vTaskDelay(piston_period);
                PIS_Get_Length(&Length);
                vTaskDelay(piston_period);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full Length=%.4fin\n", Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full Task->Finished\n");
                piston_move = false;
                piston_timer = 0;
            }
            vTaskDelay(piston_period);
        }
        vTaskDelay(xDelay5000ms);
        
        if (piston_move)

        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, move to full deliberately stopping the Piston\n");
            /* stop the piston */
            PIS_task_delete(xPiston);
            vTaskDelay(xDelay5000ms);
            PIS_stop();
            vTaskDelay(piston_period);
            piston_move = false;
            piston_timer = 0;
        }

        lengthdrift = Length;   
        piston_timer = 0;
        piston_move = true;
        PIS_set_piston_rate(1);
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_surface, Setting -> Piston encoder value to max, %u profiles reached since last encoder full reset >>\n\n", pistonfull_number);
        PIS_task_reset_full(&xPiston); /*This is the reset piston encoder to full command*/
        vTaskDelay(xDelay5000ms);

        /* check on piston movement */
        while (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {   
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->active\n");
                /* piston time for up to 30 seconds */
                piston_timer += piston_period;
                if (piston_timer >= 30000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full time-out, task->finished\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    PIS_Reset();
                    piston_timer = 0;
                }
            }
            // else if (eStatus==eReady)
            // {
            //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->Ready\n");
            //     piston_move = false;
            //     piston_timer = 0;
            // }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->suspended\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                piston_timer = 0;
            }
            else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
            {
                vTaskDelay(piston_period);
                PIS_Get_Length(&Length);
                vTaskDelay(piston_period);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston encoder reset to full Length=%.4fin\n", Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full Task->Finished\n");
                piston_move = false;
                piston_timer = 0;
            }
            vTaskDelay(piston_period);
        }
        vTaskDelay(xDelay5000ms);
        lengthadjust = Length - lengthdrift;

        if( (lengthadjust<=0.25) && (lengthadjust>=-0.25))
        {
            /*Adjust saved position settings retain correct buoyancy states*/
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, New Piston Full Cal Length=%.4fin\n", Length);
            park_piston_length = park_piston_length + lengthadjust;
            if(park_piston_length >= CRUSH_DEPTH_PISTON_POSITION && (PARK_DEPTH + PARK_DEPTH_ERR) >= CRITICAL_PISTON_POSITON_DEPTH)
            {
                park_piston_length = CRUSH_DEPTH_PISTON_POSITION;
            }
            to_prof_piston_length= to_prof_piston_length + lengthadjust;
            if(to_prof_piston_length >= CRUSH_DEPTH_PISTON_POSITION && (PROFILE_DEPTH + PROFILE_DEPTH_ERR) >= CRITICAL_PISTON_POSITON_DEPTH)
            {
                to_prof_piston_length = CRUSH_DEPTH_PISTON_POSITION;
            }
            prof_piston_length = prof_piston_length + lengthadjust;
            if(prof_piston_length >= CRUSH_DEPTH_PISTON_POSITION && (PROFILE_DEPTH + PROFILE_DEPTH_ERR) >= CRITICAL_PISTON_POSITON_DEPTH)
            {
                prof_piston_length = CRUSH_DEPTH_PISTON_POSITION;
            }
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Setting -> Piston Length Adjustment %.4fin, Park Length=%.4fin, To Prof Length=%.4fin, Prof Length=%.4fin\n", lengthadjust, park_piston_length, to_prof_piston_length, prof_piston_length);
        }
        pistonfull_number = 0;
    }
    else
    {
        PIS_task_move_length(&xPiston);
        piston_move = true;
        vTaskDelay(xDelay5000ms);
        

        while (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston task->active\n");
                /* piston time for up to 180 seconds */
                piston_timer += piston_period;
                if (piston_timer >= 180000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston time-out, task->finished\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    PIS_task_delete(xPiston);
                    vTaskDelay(xDelay5000ms);
                    PIS_Reset();
                    piston_timer = 0;
                }
            }
            // else if (eStatus==eReady)
            // {
            //     ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston task->Ready\n");
            //     piston_move = false;
            //     piston_timer = 0;
            // }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston task->suspended\n");
                PIS_task_delete(xPiston);
                vTaskDelay(xDelay5000ms);
                //piston_move = false;
                piston_timer = 0;
            }
            else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
            {
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston task->finished\n");
                piston_move = false;
                piston_timer = 0;
            }
            vTaskDelay(piston_period);
        }
    }
    vTaskDelay(xDelay5000ms);

    if (piston_move)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS deliberately stopping the Piston\n");
        /* stop the piston */
        PIS_task_delete(xPiston);
        vTaskDelay(xDelay5000ms);
        PIS_stop();
        vTaskDelay(piston_period);
        piston_move = false;
        piston_timer = 0;
    }

    /** Turn on the GPS */
    uint8_t s_rate = 1;
    uint32_t period = xDelay1000ms/s_rate;
    SENS_set_gps_rate(s_rate);

#ifdef TEST
    /* do nothing */
    SENS_sensor_gps_off();
    Event_e spsEvent;
    spsEvent = MODE_DONE;
#else
    SENS_sensor_gps_on();
    TaskHandle_t xGps = NULL;
    SENS_task_gps(&xGps);
    SensorGps_t gps;
    bool run = true;
    uint8_t fix = 0;
    Event_e spsEvent;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (run)
    {
        eStatus = eTaskGetState( xGps );
        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            /* ask for gps data */
            SENS_get_gps(&gps);
            if (gps.fix == true)
            {
                //ARTEMIS_DEBUG_PRINTF("GPS : TimeStampe, %u.%u.%u, %u:%u:%u\n", gps.month, gps.day, gps.year, gps.hour, gps.min, gps.sec);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->active : fixed, latitude=%0.7f, longitude=%0.7f, altitude=%0.7f\n", gps.latitude, gps.longitude, gps.altitude);
                fix++;
                if (fix > 9)
                {
                    /* update latitude and longitude for park and profile modes */
                    DATA_add_gps(&current_park_data, gps.latitude, gps.longitude); // Change to current_park_data
                    DATA_add_gps(&current_profile_data, gps.latitude, gps.longitude); // Change to current_profile_data

                    /* Calibrate the GPS UTC time into RTC */
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, RTC : <GPS Time Set>\n");
                    artemis_rtc_gps_calibration(&gps);
                    fix = 0;
                }
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->active : No fix\n");
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->suspended\n");
        }
        else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
        {
            /* check, if it got at least two to three fixes */
            if (fix >= 2)
            {
                /* update latitude and longitude for park and profile modes */
                if (park != NULL) {
                    DATA_add_gps(&current_park_data, gps.latitude, gps.longitude); // Change to current_park_data
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added GPS to park data\n");
                }
                
                if (prof != NULL) {
                    DATA_add_gps(&current_profile_data, gps.latitude, gps.longitude); // Change to current_profile_data
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added GPS to profile data\n");
                }

                /* Calibrate the GPS UTC time into RTC */
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, RTC : <GPS Time Set>\n");
                artemis_rtc_gps_calibration(&gps);
                fix = 0;
            }

            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->finished\n");
            run = false;
            vTaskDelay(xDelay100ms);
            /** GPS OFF */
            SENS_sensor_gps_off();
            spsEvent = MODE_DONE;
        }
        vTaskDelayUntil(&xLastWakeTime, period);
    }
#endif
    /* wait for 1 seconds here */
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, task->finished\n");
    vTaskDelay(xDelay1000ms);

    rtc_time time;
    bool utc = artemis_rtc_get_time(&time);
    if (utc)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, RTC : TimeStamp, %02d.%02d.20%02d, %02d:%02d:%02d (UTC)\n",
                        time.month, time.day, time.year, time.hour, time.min, time.sec);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, RTC : TimeStamp, %02d.%02d.20%02d, %02d:%02d:%02d (local)\n",
                        time.month, time.day, time.year, time.hour, time.min, time.sec);
    }

    // Add Park Data to the transmission queue/flash and reset the static buffer
    // Check if any park data was actually written in this cycle
    if (current_park_data.cbuf.written > 0) 
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Attempting to add park data (Profile %u, %u samples)\n", 
                             park_number - 1, current_park_data.cbuf.written); // Use park_number if it tracks completed profiles

        // Attempt to add park data to the queue (or flash)
        if (!MEM_queue_add(&current_park_data, true)) { // true indicates park data
            // MEM_queue_add failed (queue and flash full?) - Data not stored.
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Failed to add park data to queue/flash.\n");
        } else {
             ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added/Stored park data.\n");
        }
        // Always reset the static park data structure after attempting to add its data
        DATA_reset(&current_park_data); 
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Reset static park data buffer.\n");
    } else {
         ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, No park data written in this cycle to add.\n");
    }

    // Add Profile Data to the transmission queue/flash and reset the static buffer
    // Check if any profile data was actually written in this cycle
    if (current_profile_data.cbuf.written > 0) 
    {
         ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Attempting to add profile data (Profile %u, %u samples)\n", 
                              prof_number - 1, current_profile_data.cbuf.written); // Use prof_number if it tracks completed profiles

        // Attempt to add profile data to the queue (or flash)
        if (!MEM_queue_add(&current_profile_data, false)) { 
            // MEM_queue_add failed (queue and flash full?) - Data not stored.
             ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Failed to add profile data to queue/flash.\n");
        } else {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added/Stored profile data.\n");
        }
        // Always reset the static profile data structure after attempting to add its data
        DATA_reset(&current_profile_data); 
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Reset static profile data buffer.\n");
    } else {
         ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, No profile data written in this cycle to add.\n");
    }

    /* check Heap size */
    monitor_memory_usage();

    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Task->finished\n\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_tx(void)
{
    Event_e spsEvent;
    QueuedDataEntry_t *current_item = NULL;
    bool fatal_error_occurred = false;      // Flag to exit main loop on HW fail
    bool queue_processed = false;           // Flag to track if we processed at least one item

    MEM_log_memory_status("SPS :: tx start");

    /** Initialize the Iridium Modem */
    // ... (Keep existing Iridium init/power-on logic - verified OK) ...
     if (!iridium_init)
     {
         i9603n_initialize();
         iridium_init = true;
         vTaskDelay(xDelay1000ms);
 #ifdef TEST
         datalogger_test_sbd_messages_init();
 #endif
     }

    /* turn on the iridium module and wait for it be charged */
     uint8_t wait = 0;
     uint8_t tries = 0;
     while (wait < 2 && tries < 2)
     {
         bool retVal = i9603n_on();
         if (!retVal)
         {
             wait++;
             if (wait >= 2)
             {
                 tries++;
                 i9603n_off();
                 vTaskDelay(xDelay1000ms);
                 wait = 0;
             }
         }
         else
         {
             ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium looks fine\n");
             tries = 0;
             break;
         }
     }

     if (tries >= 2)
     {
         i9603n_off();
 #if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
         datalogger_read_test_profile(true);
 #endif
         spsEvent = MODE_IDLE;
         ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium not charged, trying again later.\n");
         SendEvent(spsEventQueue, &spsEvent);
         ARTEMIS_DEBUG_PRINTF("SPS :: tx, Task->finished abruptly, NOT transmitting.\n");
         vTaskDelete(NULL);
         return; // Exit function
     }
     vTaskDelay(xDelay1000ms);


    // --- Main Transmission Loop ---
    while (MEM_queue_get_count() > 0 && !fatal_error_occurred)
    {
        queue_processed = true; // Indicate we started processing the queue
        current_item = MEM_queue_get_next(); // Get item at head
        
        if (current_item == NULL) {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Queue empty unexpectedly, exiting TX loop.\n");
            // This case should ideally not be reached if MEM_queue_get_count() > 0,
            // but handles potential race conditions or errors.
            break; 
        }

        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Processing %s %u (Attempts: %u, Samples: %u)\n", 
                             current_item->is_park_data ? "Park" : "Profile", 
                             current_item->profile_number, 
                             current_item->attempt_count,
                             current_item->num_samples);

        uint16_t samples_processed_this_item = 0;
        uint8_t current_page_num = 0;
        bool item_fully_transmitted = false;
        uint8_t max_transmit_tries = current_item->is_park_data ? PARK_TRANSMIT_TRIES : PROF_TRANSMIT_TRIES;

        // --- Paging Loop (for current_item) ---
        while (!item_fully_transmitted && !fatal_error_occurred) 
        {
            TaskHandle_t xIridium = NULL;
            TaskHandle_t xSatellite = NULL;
            eTaskState eStatus;
            uint32_t visibility_period = xDelay1000ms; // Default
            bool page_transmission_success = false;
            uint16_t samples_packed_this_page = 0;

            // Buffer for the current page's packed data
            uint8_t current_page_buffer[IRID_DATA_OUT]; // Defined in StateMachine.c? Ensure size.
            memset(current_page_buffer, 0, IRID_DATA_OUT); // Clear buffer

            // Use the new helper to prepare and pack the page
            uint16_t txbytes = prepare_transmit_page(current_page_buffer, 
                                                     current_item, 
                                                     samples_processed_this_item, 
                                                     current_page_num, 
                                                     &samples_packed_this_page);

            // Check if packing produced data or if all samples are done
            if (txbytes == 0 || samples_packed_this_page == 0) {
                 if (samples_processed_this_item >= current_item->num_samples) {
                     ARTEMIS_DEBUG_PRINTF("SPS :: tx, All samples processed for item %u. Finalizing.\n", current_item->profile_number);
                     // Mark transmitted should happen upon *successful* transmission of the last page
                     // This path indicates successful completion from previous iteration.
                     item_fully_transmitted = true; 
                 } else {
                      // Error during packing or unexpected state
                      ARTEMIS_DEBUG_PRINTF("SPS :: tx, ERROR packing page %u for item %u! Discarding item.\n", current_page_num, current_item->profile_number);
                      MEM_queue_mark_transmitted(); // Discard problematic item
                      item_fully_transmitted = true; // Stop processing this item
                 }
                 break; // Exit paging loop for this item
            }

            // --- Transmission Attempt Loop (for current page) ---
            bool retry_page = true; 
            while(retry_page && !fatal_error_occurred) 
            {
                retry_page = false; // Assume success for this attempt unless failure occurs

                /* Satellite visibility check */
                // ... (Keep existing satellite check logic - verified OK) ...
                bool run_satellite = true;
                uint8_t satellite_tries = 0;
                task_Iridium_satellite_visibility(&xSatellite);
                while(run_satellite && !fatal_error_occurred) 
                {
                    eStatus = eTaskGetState(xSatellite);
                    if ((eStatus==eRunning) || (eStatus==eReady) || (eStatus==eBlocked)) {
                         // Waiting...
                    } else if ((eStatus==eDeleted) || (eStatus==eInvalid)) {
                         ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite check finished for page %u.\n", current_page_num);
                         satellite_tries++;
                         bool visible = GET_Iridium_satellite(); 
                         if (visible) {
                              ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite VISIBLE.\n");
                              run_satellite = false;
                              SET_Iridium_delay_rate(0.5); 
                         } else {
                              if (satellite_tries >= SATELLITE_VISIBILITY_TRIES) {
                                   run_satellite = false;
                                   SET_Iridium_delay_rate(0.1);
                                   ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite NOT visible after %u tries.\n", SATELLITE_VISIBILITY_TRIES);
                              } else {
                                   visibility_period = 20;
                                   ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite NOT Visible, waiting %u sec...\n", visibility_period);
                                   i9603n_sleep();
                                   vTaskDelay(xDelay1000ms * visibility_period);
                                   i9603n_wakeup();
                                   vTaskDelete(xSatellite); 
                                   xSatellite = NULL;
                                   task_Iridium_satellite_visibility(&xSatellite); 
                              }
                         }
                    } else if (eStatus==eSuspended) { 
                         ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite check task suspended, retrying...\n");
                         vTaskDelete(xSatellite); 
                         xSatellite = NULL;
                         task_Iridium_satellite_visibility(&xSatellite); 
                         vTaskDelay(xDelay1000ms); 
                    }
                    vTaskDelay(xDelay1000ms); 
                } // End satellite check loop

                /* Send data to modem buffer */
                bool send_buffer_success = i9603n_send_data(current_page_buffer, txbytes);
                if (!send_buffer_success) {
                     ARTEMIS_DEBUG_PRINTF("SPS :: tx, Failed to send Page %u data to modem buffer. Retrying page.\n", current_page_num);
                     retry_page = true; 
                     vTaskDelay(xDelay2000ms); // Wait before retrying send
                     continue; // Retry this page attempt
                }
                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u data (%u bytes) sent to modem buffer.\n", current_page_num, txbytes);
                vTaskDelay(xDelay1000ms);

                /* Start Iridium transfer task and monitor */
                task_Iridium_transfer(&xIridium); 
                bool transfer_run = true;
                while(transfer_run && !fatal_error_occurred)
                {
                    eStatus = eTaskGetState(xIridium);
                    if ((eStatus==eRunning) || (eStatus==eReady) || (eStatus==eBlocked)){
                         // Waiting...
                    } else if ((eStatus==eDeleted) || (eStatus==eInvalid)) {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task finished.\n");
                        vTaskDelay(xDelay500ms);
                        uint8_t recv[6] = {0};
                        bool status_ok = GET_Iridium_status(recv); 
                        uint16_t wait_time = 10; // Default wait
                        
                        transfer_run = false; // Exit transfer monitor loop
                        
                        if (status_ok) {
                            if (recv[0] <= 4) { // Success code
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit SUCCESSFUL.\n", current_page_num);
                                page_transmission_success = true; // Mark page as successfully sent
                            } else { // Failure code
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit FAILED (Iridium Status: %u).\n", current_page_num, recv[0]);
                                page_transmission_success = false; 
                                retry_page = true; // Signal to retry this page after handling error code
                                
                                // Handle specific codes like traffic management 
                                if (recv[0] == 38) {
                                     vTaskDelay(xDelay2000ms);
                                     uint16_t traffic_buf[8] = {0};
                                     uint8_t traffic_len = i9603n_traffic_mgmt_time(traffic_buf);
                                     i9603n_sleep();
                                     if(traffic_len > 0 && traffic_buf[0] == 0) wait_time = traffic_buf[1];
                                     if(wait_time < 10) wait_time = 10;
                                     ARTEMIS_DEBUG_PRINTF("SPS :: tx, Traffic management wait: %u sec\n", wait_time);
                                     vTaskDelay(xDelay1000ms * wait_time);
                                     i9603n_wakeup();
                                     vTaskDelay(xDelay1000ms);
                                } else { // Other failure codes
                                     i9603n_sleep();
                                     vTaskDelay(xDelay1000ms * wait_time); // General wait
                                     i9603n_wakeup();
                                     vTaskDelay(xDelay1000ms);
                                }
                            }
                        } else { // Failed to get Iridium status
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, ERROR getting Iridium transmit status!\n");
                            page_transmission_success = false;
                            retry_page = true; // Retry page if status check failed
                        }
                    } else if (eStatus == eSuspended) {
                         ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task suspended?\n");
                         vTaskDelay(xDelay1000ms);
                    }
                    vTaskDelay(xDelay1000ms); // Check status periodically
                } // end transfer_run loop

                // --- Handle outcome of this transmission attempt ---
                if (page_transmission_success) {
                     samples_processed_this_item += samples_packed_this_page; // Update processed count
                     MEM_queue_reset_attempts(); // Reset attempts for the item on page success
                     
                     if (samples_processed_this_item >= current_item->num_samples) {
                          ARTEMIS_DEBUG_PRINTF("SPS :: tx, Item %u fully transmitted.\n", current_item->profile_number);
                          MEM_queue_mark_transmitted(); // Mark item done in queue
                          item_fully_transmitted = true; // Exit paging loop
                     } else {
                          ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u for item %u transmitted, more pages needed (%u/%u samples).\n", 
                                                current_page_num, current_item->profile_number, samples_processed_this_item, current_item->num_samples);
                          current_page_num++; // Go to next page
                     }
                     retry_page = false; // Don't retry this page

                } else if (retry_page) { 
                     // Page transmission failed, increment attempt count for the *item*
                     MEM_queue_increment_attempt(); 
                     // Re-fetch item pointer in case queue shifted (unlikely if single-threaded access)
                     current_item = MEM_queue_get_next(); 
                     if (current_item == NULL) { // Check if item was removed somehow
                         ARTEMIS_DEBUG_PRINTF("SPS :: tx, ERROR: Current item became NULL after failed attempt!\n");
                         fatal_error_occurred = true; // Treat as fatal
                         retry_page = false;
                         item_fully_transmitted = true; // Exit loops
                         break;
                     }

                     ARTEMIS_DEBUG_PRINTF("SPS :: tx, Item %u attempt count now %u.\n", 
                                           current_item->profile_number, current_item->attempt_count);

                     if (MEM_queue_max_attempts_reached(max_transmit_tries)) {
                          ARTEMIS_DEBUG_PRINTF("SPS :: tx, MAX ATTEMPTS (%u) reached for item %u. Discarding.\n", 
                                                max_transmit_tries, current_item->profile_number);
                          MEM_queue_mark_transmitted(); // Discard item
                          item_fully_transmitted = true; // Exit paging loop for this item
                          retry_page = false; // Don't retry page
                     } else {
                          ARTEMIS_DEBUG_PRINTF("SPS :: tx, Retrying page %u for item %u.\n", current_page_num, current_item->profile_number);
                          // retry_page remains true, transmission attempt loop will iterate
                     }
                } else {
                    // This case (page !success and !retry) shouldn't be reached easily
                    // but implies a non-retryable HW error potentially
                     ARTEMIS_DEBUG_PRINTF("SPS :: tx, Unhandled non-retryable failure for page %u.\n", current_page_num);
                     fatal_error_occurred = true; // Treat as fatal
                     retry_page = false;
                     item_fully_transmitted = true; // Exit loops
                }

                vTaskDelay(xDelay1000ms); // Small delay between page attempts or before moving to next page

            } // End Transmission Attempt Loop (retry_page)

        } // End Paging Loop (item_fully_transmitted)

        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Finished processing item %u.\n", (current_item ? current_item->profile_number : 0)); // Safely print profile number
        vTaskDelay(xDelay1000ms); // Delay before processing next item

    } // End Main Transmission Loop (while queue not empty)


    // --- Cleanup ---
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Transmission loop finished. Queue count: %u\n", MEM_queue_get_count());
    
    // Only turn off if it was initialized
    if(iridium_init) {
       i9603n_off(); 
       iridium_init = false; // Maybe reset flag? Depends on overall design.
    }
    vTaskDelay(xDelay1000ms);

    /* check Heap size */
    monitor_memory_usage(); // Assuming this function exists

    // Determine the next state to transition to
    if (prof_number >= SYSTEM_PROFILE_NUMBER) { // Assuming prof_number is still tracked globally
        spsEvent = MODE_POPUP;
        ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, << %u Profiles have been reached >>\n\n", prof_number);
    } else {
#if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
        datalogger_read_test_profile(true); // Assuming this exists
#endif
        spsEvent = MODE_IDLE;
    }
    
    MEM_log_memory_status("SPS :: tx end"); // Log final status
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Task->finished\n\n");
    SendEvent(spsEventQueue, &spsEvent); // Assuming this exists
    vTaskDelete(NULL);
}

// *** NEW STATIC HELPER FUNCTION (Add this before module_sps_tx) ***
/**
 * @brief Prepares and packs one page of data from a QueuedDataEntry item for Iridium transmission.
 * * @param iridium_buffer Pointer to the output buffer for the packed Iridium message.
 * @param item Pointer to the QueuedDataEntry item containing the data.
 * @param samples_already_processed The number of samples from this item already processed/packed in previous pages.
 * @param page_num The current page number being generated for this item.
 * @param[out] samples_packed_in_page Pointer to store the number of samples included in this generated page.
 * @return uint16_t The total number of bytes packed into iridium_buffer (including header), or 0 on error/completion.
 */
 static uint16_t prepare_transmit_page(uint8_t *iridium_buffer, 
    QueuedDataEntry_t *item, 
    uint16_t samples_already_processed, 
    uint8_t page_num,
    uint16_t *samples_packed_in_page) 
{
// --- Input Validation ---
if (!iridium_buffer || !item || !samples_packed_in_page) {
ARTEMIS_DEBUG_PRINTF("SPS :: prepare_transmit_page: ERROR - Invalid arguments.\n");
if (samples_packed_in_page) *samples_packed_in_page = 0;
return 0;
}

*samples_packed_in_page = 0; // Default output
uint16_t total_samples = item->num_samples;

// --- Calculate Samples for this Page ---
if (samples_already_processed >= total_samples) {
ARTEMIS_DEBUG_PRINTF("SPS :: prepare_transmit_page: No more samples needed for item %u.\n", item->profile_number);
return 0; // All samples already processed
}
uint16_t remaining_samples = total_samples - samples_already_processed;
uint16_t samples_this_page = (remaining_samples > MEASUREMENT_MAX) ? MEASUREMENT_MAX : remaining_samples;

if (samples_this_page == 0) {
ARTEMIS_DEBUG_PRINTF("SPS :: prepare_transmit_page: Calculated zero samples for page %u of item %u.\n", page_num, item->profile_number);
return 0; 
}

// --- Prepare sData for pack_measurements_irid ---
sData current_sData; // Local struct for page info
current_sData.profNumber = item->profile_number; 
current_sData.modeType = item->is_park_data ? LCP_PARK_MODE : LCP_PROFILE_MODE; 
current_sData.pageNumber = page_num;
current_sData.mLength = (uint8_t)samples_this_page; // Length for *this page*

ARTEMIS_DEBUG_PRINTF("SPS :: prepare_transmit_page: Preparing Page %u for %s %u: %u samples (processed %u / total %u)\n",
current_sData.pageNumber, item->is_park_data ? "Park" : "Profile", item->profile_number,
samples_this_page, samples_already_processed, total_samples);

// --- Create Temporary Data_t View ---
// This view points to the data within the QueuedDataEntry_t item 
// and sets the read offset for pack_measurements_irid.
Data_t temp_data_view;
memset(&temp_data_view, 0, sizeof(Data_t)); // Important: Initialize to zero
temp_data_view.data.pressure = item->pressure_measurements;     
temp_data_view.data.temperature = item->temp_measurements;  
temp_data_view.p = &item->profile_metadata; // Point to the stored metadata             
temp_data_view.cbuf.read = samples_already_processed; // !!! Start reading from this offset !!!
temp_data_view.cbuf.written = item->num_samples;    // Total samples available in the item
temp_data_view.cbuf.length = DATA_MAX_SAMPLES;      // Max capacity of arrays in item
temp_data_view.pNumber = item->profile_number;      // Use item's number       
temp_data_view.wLength = item->num_samples;         // Use item's sample count
// rLength is managed internally by DATA_get_converted called by pack_measurements_irid

// --- Call Existing Packing Function ---
// Uses the temporary view to read the correct segment of samples.
uint16_t nrBytes = pack_measurements_irid(&temp_data_view, temp_data_view.p, &current_sData, iridium_buffer);

if (nrBytes > 0) {
*samples_packed_in_page = samples_this_page; 
ARTEMIS_DEBUG_PRINTF("SPS :: prepare_transmit_page: Packed %u bytes for page %u (%u samples).\n", nrBytes, current_sData.pageNumber, *samples_packed_in_page);
} else {
ARTEMIS_DEBUG_PRINTF("SPS :: prepare_transmit_page: ERROR - pack_measurements_irid returned 0 bytes for page %u.\n", current_sData.pageNumber);
*samples_packed_in_page = 0;
}

return nrBytes;
}

// old helper function, not used anymore
static uint16_t create_park_page(uint8_t *ptrPark, uint8_t *readlength) 
{
    /* check if we have read and transmitted all the measurements*/
    if (m_park_number >= park_number)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, No more measurements available m_park_number=%u, park_number=%u\n", m_park_number, park_number);
        *readlength = 0;
        return 0;
    }

    /* set modeType */
    sPark.modeType = LCP_PARK_MODE;
    /* check if we are sending the first page of any profile number */
    if (m_park_length == 0)
    {
        // Instead of reading from pPark array, get the actual length from the Data_t structure
        m_park_length = park->cbuf.written;
        sPark.profNumber = m_park_number;
        sPark.pageNumber = 0;
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, profile_number=%u, pageNumber=%u, measurement_length=%u\n", m_park_number, sPark.pageNumber, m_park_length);

        /* check the length of measurements must not exceed 312 bytes > 312*8/(12+8) = 124 */
        if (m_park_length > MEASUREMENT_MAX)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, WARNING : Park, profile_number=%u exceeding (%u) length of measurements, creating pages!\n", sPark.profNumber, MEASUREMENT_MAX);
            sPark.mLength = MEASUREMENT_MAX;
            m_park_length = park->cbuf.written - sPark.mLength;
        }
        else
        {
            sPark.mLength = m_park_length;
            m_park_length = 0;
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, profile_number=%u length of measurements=%u fitting in one page!\n", sPark.profNumber, sPark.mLength);
        }
    }
    else
    {
        if (m_park_length > MEASUREMENT_MAX)
        {
            sPark.mLength = MEASUREMENT_MAX;
            m_park_length = m_park_length - sPark.mLength;
        }
        else
        {
            sPark.mLength = m_park_length;
            m_park_length = 0;
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, profile_number=%u measurement_length=%u\n", sPark.profNumber, sPark.mLength);
        }
    }

    // Get the actual Data_t structure from the global park pointer
    //uint16_t nrBytes = pack_measurements_irid(park, NULL, &sPark, ptrPark);
    uint16_t nrBytes = pack_measurements_irid(park, park->p, &sPark, ptrPark);
    *readlength = sPark.mLength;
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park : profile_number=%u, pageNumber=%u, m_park_number=%u, total_bytes=%u\n", sPark.profNumber, sPark.pageNumber, m_park_number, nrBytes);

    return nrBytes;
}

// old helper function, not used anymore
static uint16_t create_profile_page(uint8_t *ptrProf, uint8_t *readlength)
{
    /* check if we have read and transmitted all the measurements*/
    if (m_prof_number >= prof_number)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, No more measurements available m_prof_number=%u, prof_number=%u\n", m_prof_number, prof_number);
        *readlength = 0;
        return 0;
    }

    /* set modeType */
    sProf.modeType = LCP_PROFILE_MODE;
    /* check if we are sending the first page of any profile number */
    if (m_prof_length == 0)
    {
        // Instead of reading from pProf array, get the actual length from the Data_t structure
        m_prof_length = prof->cbuf.written;
        sProf.profNumber = m_prof_number;
        sProf.pageNumber = 0;
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, profile_number=%u, pageNumber=%u, measurement_length=%u\n", m_prof_number, sProf.pageNumber, m_prof_length);

        /* check the length of measurements must not exceed 312 bytes > 312*8/(12+8) = 124 */
        if (m_prof_length > MEASUREMENT_MAX)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, WARNING : Profile, profile_number=%u exceeding (%u) length of measurements, creating pages!\n", sProf.profNumber, MEASUREMENT_MAX);
            sProf.mLength = MEASUREMENT_MAX;
            m_prof_length = prof->cbuf.written - sProf.mLength;
        }
        else
        {
            sProf.mLength = m_prof_length;
            m_prof_length = 0;
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, profile_number=%u length of measurements=%u fitting in one page!\n", sProf.profNumber, sProf.mLength);
        }
    }
    else
    {
        if (m_prof_length > MEASUREMENT_MAX)
        {
            sProf.mLength = MEASUREMENT_MAX;
            m_prof_length = m_prof_length - sProf.mLength;
        }
        else
        {
            sProf.mLength = m_prof_length;
            m_prof_length = 0;
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, profile_number=%u measurement_length=%u\n", sProf.profNumber, sProf.mLength);
        }
    }

    // Get the actual Data_t structure from the global prof pointer
    //uint16_t nrBytes = pack_measurements_irid(prof, NULL, &sProf, ptrProf);
    uint16_t nrBytes = pack_measurements_irid(prof, prof->p, &sProf, ptrProf);
    *readlength = sProf.mLength;
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile : profile_number=%u, pageNumber=%u, m_prof_number=%u, total_bytes=%u\n", sProf.profNumber, sProf.pageNumber, m_prof_number, nrBytes);

    return nrBytes;
}

void module_sps_rx(void)
{
    /** Check for messages */

    /** If message, receive it */

    /** Iridium OFF */
}

static void SendEvent(QueueHandle_t eventQueue, Event_e *event)
{
    xQueueSend(eventQueue, event, portMAX_DELAY);
}

static void ReceiveEvent(QueueHandle_t eventQueue, Event_e *event)
{
    xQueueReceive(eventQueue, event, portMAX_DELAY);
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

void monitor_memory_usage(void) {
    uint32_t total_heap = configTOTAL_HEAP_SIZE;
    uint32_t free_heap = xPortGetFreeHeapSize();
    uint32_t min_ever_free = xPortGetMinimumEverFreeHeapSize();
    uint32_t used_heap = total_heap - free_heap;
    
    ARTEMIS_DEBUG_PRINTF("Memory Usage: %lu/%lu bytes (%.1f%%)\n", 
                         used_heap, total_heap, 
                         (float)used_heap*100.0f/total_heap);
    ARTEMIS_DEBUG_PRINTF("Minimum Ever Free: %lu bytes\n", min_ever_free);
}