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
#include <stdint.h>
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

static uint16_t gpsTimer = 0;

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
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->state = %d\n", eStatus);
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->active\n");

                /* piston time for up to 120 seconds */
                if (piston_timer >= 120000)
                {
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_timer = 0;
                    piston_move = false;
                }
            }
            else if (eStatus==eSuspended) // NOTE: in its current state, the program won't reach this.
            {
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->suspended\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay10000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;                
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
            else if (piston_timer >= 180000) 
            {
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out and task state invalid.\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
        }
        vTaskDelay(period);
        piston_timer += piston_period;
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
        ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->state = %d\n", eStatus);
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->active\n");
            /* piston time for up to 180 seconds */
            if (piston_timer >= 180000)
            {
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
                pusEvent = MODE_DONE;
            }
        }
        else if (eStatus==eSuspended) // NOTE: in its current state, the program won't reach this. 
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->suspended\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay10000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_move = false;
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
        else if (piston_timer >= 180000) 
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out and task state invalid.\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Density=%.3fkg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->finished\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_timer = 0;
            piston_move = false;
            pusEvent = MODE_DONE;
        }
        vTaskDelay(piston_period);
        piston_timer += piston_period;
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
        ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->state = %d\n", eStatus);
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->active\n");
            /* piston time for up to 180 seconds */
            
            if (piston_timer >= 180000)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston time-out, task->finished\n");
                // Time-out: delete the task, reset the piston board and set piston_move to false
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston time-out, task->finished\n");

                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay10000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->suspended\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay10000ms); // Wait for the task to exit the loop and delete itself
            piston_move = false;
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
        else if (piston_timer >= 180000) 
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston time-out and task state invalid.\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, density=%.3f kg/m³, volume=%.3fin³, length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->finished\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_move = false;
            piston_timer = 0;
        }
        vTaskDelay(piston_period);
        piston_timer += piston_period;
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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay10000ms); // Wait for the task to exit the loop and delete itself
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
                gpsTimer = 0;
                SENS_sensor_gps_off();

                /* store data in the SDcard */
                datalogger_predeploy_mode(&gps, true);
                pdsEvent = MODE_PRE_DEPLOY;
            } 
            else if (gpsTimer >= GPS_TIMER * 60 * period + (5 * period)) 
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task TIMEOUT. Forcefully ending task...\n");
                killGPS();
                vTaskDelay(xDelay5000ms);

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
                gpsTimer = 0;
                SENS_sensor_gps_off();

                /* store data in the SDcard */
                datalogger_predeploy_mode(&gps, true);
                pdsEvent = MODE_PRE_DEPLOY;
            }
            vTaskDelayUntil(&xLastWakeTime, period);
            gpsTimer += period;
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
                                        "sps_txt", 2048, NULL,
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
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->state = %d\n", eStatus);
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {   
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->active\n");
                /* piston time for up to 180 seconds */
                if (piston_timer >= 180000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero time-out, task->finished\n");
                    PIS_task_delete();
                    PIS_Get_Length(&zlengthdrift);
                    vTaskDelay(xDelay10000ms); // Wait for the task to exit the loop and delete itself
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Length=%.4fin\n", zlengthdrift);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Task->Finished\n");
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_timer = 0;
                    piston_move = false;
                }
            }
            else if (eStatus==eSuspended) // NOTE: in its current state, the program won't reach this.
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->suspended\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
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
            else if (piston_timer >= 180000) 
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero time-out and task state invalid.\n");
                PIS_Get_Length(&zlengthdrift);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Length=%.4fin\n", zlengthdrift);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Task->Finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
            vTaskDelay(piston_period);
            piston_timer += piston_period;
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
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->state = %d\n", eStatus);
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->active\n");
            /* piston time for up to 180 seconds */
            if (piston_timer >= 180000)
            {
                // Time-out: delete the task, reset the piston board and set piston_move to false
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston time-out, task->finished\n");

                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;

            }
        }
        else if (eStatus==eSuspended) // NOTE: in its current state, the program won't reach this.
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_timer = 0;
            piston_move = false;
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
        else if (piston_timer >= 180000) 
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston time-out and task state invalid.\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->finished\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_move = false;
            piston_timer = 0;
        }
        vTaskDelay(piston_period);
        piston_timer += piston_period;
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
                    /* check if piston is still moving then reset it and stop */
                    if (piston_move)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                        PIS_task_delete(); // Signal to exit loop
                        vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                        PIS_stop();
                        piston_move = false;
                        piston_timer = 0;
                        vTaskDelay(period);
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

                        /* check if piston is still moving then reset it and stop */
                        if (piston_move)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            PIS_stop();
                            piston_move = false;
                            piston_timer = 0;
                            vTaskDelay(period);
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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
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
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->state = %d\n", eStatus);
                if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->active\n");
                    /* keep piston time for up to 15 seconds unless crush_depth activated use piston up to 120 seconds */

                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            PIS_Reset();
                            vTaskDelay(xDelay1000ms);
                            piston_timer = 0;
                            piston_move = false;

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
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            PIS_Reset();
                            vTaskDelay(xDelay1000ms);
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Board Done Resetting...\n");
                            piston_timer = 0;
                            piston_move = false;
                        }
                    }
                }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
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
                else if ( (eStatus==eDeleted) || (eStatus==eInvalid) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->deleted or invalid. Getting Length...\n");
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
                else if (piston_timer >= 180000) {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston time-out and task state invalid.\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_move = false;
                }


                if (period >= xDelay10000ms)
                {
                    vTaskDelay(piston_period);
                }
                piston_timer += piston_period;

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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_stop();
                piston_timer = 0;
                piston_move = false;
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
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->state = %d\n", eStatus);
                if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->active\n");
                    /* keep piston time for up to 15 seconds unless crush_depth activated use piston up to 120 seconds */
                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            PIS_Reset();
                            vTaskDelay(xDelay1000ms);
                            piston_timer = 0;
                            piston_move = false;

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
                            PIS_Get_Length(&Length);
                            Volume = CTRL_calculate_volume_from_length(Length);
                            Density = CTRL_calculate_lcp_density(Volume);
                            ARTEMIS_DEBUG_PRINTF("SPS :: park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                            ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->finished\n");
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston Board Resetting...\n");
                            PIS_Reset();
                            vTaskDelay(xDelay1000ms);
                            piston_timer = 0;
                            piston_move = false;
                        }
                    }
                }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->suspended\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston Board Resetting...\n");
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_timer = 0;
                    piston_move = false;

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
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->deleted or invalid. Getting Length...\n");
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
                else if (piston_timer >= 180000) {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston time-out and task state invalid.\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston Board Resetting...\n");
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_move = false;
                }                  
                vTaskDelay(piston_period);
                piston_timer += piston_period;

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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_stop();
                piston_timer = 0;
                piston_move = false;
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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_stop();
                vTaskDelay(piston_period);
                piston_timer = 0;
                piston_move = false;
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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_stop();
                vTaskDelay(piston_period);
                piston_timer = 0;
                piston_move = false;
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
    vTaskDelay(xDelay10000ms);
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
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->state = %d\n", eStatus);
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->active\n");
            /* piston time for up to 60 seconds */
            if (piston_timer >= 60000)
            {
                // Time-out: delete the task, reset the piston board and set piston_move to false
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out, task->finished\n");

                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->suspended\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            piston_move = false;
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
        else if (piston_timer >= 180000) 
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out and task invalid.\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out, task->finished\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_timer = 0;
            piston_move = false;
        }
        vTaskDelay(piston_period);
        piston_timer += piston_period;
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

                    /* check if piston is still moving then reset it and stop */
                    if (piston_move)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                        PIS_task_delete(); // Signal to exit loop
                        vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                        PIS_stop();
                        piston_move = false;
                        piston_timer = 0;
                        vTaskDelay(period);
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

                         /* check if piston is still moving then reset it and stop */
                        if (piston_move)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            PIS_stop();
                            piston_move = false;
                            piston_timer = 0;
                            vTaskDelay(period);
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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
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
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->state = %d\n", eStatus);
                if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->active\n");
                    /* keep piston time for up to 15 seconds unless crush_depth activated use piston up to 120 seconds */
                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            PIS_Reset();
                            vTaskDelay(xDelay1000ms);
                            piston_timer = 0;
                            piston_move = false;
                            
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
                            PIS_task_delete(); // Signal to exit loop
                            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston Board Resetting...\n");
                            PIS_Reset();
                            vTaskDelay(xDelay1000ms);
                            piston_timer = 0;
                            piston_move = false;
                        }
                    }
                }
                else if (eStatus==eSuspended) // NOTE: In its current state, this is not possible to reach.
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->suspended\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    piston_timer = 0;
                    piston_move = false;

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
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->deleted or invalid. Getting Length...\n");
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
                else if (piston_timer >= 30000) 
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out and task invalid.\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out, task->finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston Board Resetting...\n");
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_timer = 0;
                    piston_move = false;
                }

                /* piston task delay 1000ms */
                if (period >= xDelay10000ms)
                {
                    vTaskDelay(piston_period);
                }
                piston_timer += piston_period;

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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_stop();
                piston_timer = 0;
                piston_move = false;
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
    vTaskDelay(xDelay10000ms);
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
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->state=%d\n", eStatus);

        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->active\n");
            /* piston time for up to 120 seconds */
            if (piston_timer >= 120000)
            {
                // Time-out: delete the task, reset the piston board and set piston_move to false
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");

                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->suspended\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_timer = 0;
            piston_move = false;
        }
        else if ( (eStatus==eDeleted) || (eStatus==eInvalid)  )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->finished\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            piston_move = false;
            piston_timer = 0;
        }
        else if (piston_timer >= 180000)
        {
            
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out and no Piston eStatus, task->finished\n");
            // Time-out: delete the task, reset the piston board and set piston_move to false
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
            PIS_Reset();
            vTaskDelay(xDelay1000ms);
            piston_timer = 0;
            piston_move = false;
        }
        // Delay for the piston period and then increment the timer by the period
        vTaskDelay(piston_period);
        piston_timer += piston_period;
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
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->state=%d\n", eStatus);
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->active\n");
                /* keep piston time for up to 15 seconds unless crush_depth activated or at the surface use piston up to 180 seconds */
                if ( (crush_depth) || (Depth <= BALLAST_DEPTH_PROFILE) )
                {
                    if (piston_timer >= 180000)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston CRUSH or SURFACE time-out, task->finished\n");
                        PIS_Get_Length(&Length);
                        Volume = CTRL_calculate_volume_from_length(Length);
                        Density = CTRL_calculate_lcp_density(Volume);
                        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->finished\n");
                        PIS_task_delete(); // Signal to exit loop
                        vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                        PIS_Reset();
                        vTaskDelay(xDelay1000ms);
                        piston_timer = 0;
                        piston_move = false;
                    }
                }
                else
                {
                    if (piston_timer >= 30000)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");
                        PIS_task_delete(); // Signal to exit loop
                        vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                        PIS_Reset();
                        vTaskDelay(xDelay1000ms);
                        piston_timer = 0;
                        piston_move = false;
                    }
                }
            }
            else if (eStatus==eSuspended) // NOTE: In its current state, the program will never reach here.
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->suspended\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                piston_timer = 0;
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_move = false;
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
            else if (piston_timer >= 180000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out and no Piston eStatus, task->finished\n");
                // Time-out: delete the task, reset the piston board and set piston_move to false
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_stop();
                piston_timer = 0;
                piston_move = false;
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
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    PIS_stop();
                    piston_timer = 0;
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
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_stop();
                piston_timer = 0;
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
        piston_timer += period;
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
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston state = %d\n", eStatus);
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {   
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full task->active\n");
                /* piston time for up to 180 seconds */
                if (piston_timer >= 180000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full time-out, task->finished\n");
                    PIS_Get_Length(&Length);
                    vTaskDelay(piston_period);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full Length=%.4fin\n", Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full Task->Finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_timer = 0;
                    piston_move = false;
                }
            }
            else if (eStatus==eSuspended) // NOTE: In its current state, this condition will never be true.
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full task->suspended\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
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
            else if (piston_timer >= 180000) 
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full time-out and task invalid.\n");
                PIS_Get_Length(&Length);
                vTaskDelay(piston_period);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full Length=%.4fin\n", Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move to full Task->Finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
            vTaskDelay(piston_period);
            piston_timer += piston_period;
        }
        vTaskDelay(xDelay5000ms);
        
        if (piston_move)

        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, move to full deliberately stopping the Piston\n");
            /* stop the piston */
            PIS_task_delete(); // Signal to exit loop
            vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
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
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston state = %d\n", eStatus);
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {   
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->active\n");
                /* piston time for up to 30 seconds */
                
                if (piston_timer >= 30000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full time-out, task->finished\n");
                    PIS_Get_Length(&Length);
                    vTaskDelay(piston_period);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston encoder reset to full Length=%.4fin\n", Length);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full Task->Finished\n");
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_timer = 0;
                    piston_move = false;
                }
            }
            else if (eStatus==eSuspended) // NOTE: In its current state, this condition will never be true.
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->suspended\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                piston_timer = 0;
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_move = false;
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
            else if (piston_timer >= 1800000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full time-out and task invalid.\n");
                PIS_Get_Length(&Length);
                vTaskDelay(piston_period);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston encoder reset to full Length=%.4fin\n", Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full Task->Finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
            vTaskDelay(piston_period);
            piston_timer += piston_period;
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
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston state = %d\n", eStatus);
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston task->active\n");
                /* piston time for up to 180 seconds */
                if (piston_timer >= 180000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston time-out, task->finished\n");
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                    PIS_task_delete(); // Signal to exit loop
                    vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);
                    piston_timer = 0;
                    piston_move = false;
                }
            }
            else if (eStatus==eSuspended) // NOTE: In its current state, this condition will never be true.
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston task->suspended\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
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
            else if (piston_timer >= 1800000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston time-out and task invalid.\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface B, Piston task->finished\n");
                PIS_task_delete(); // Signal to exit loop
                vTaskDelay(xDelay5000ms); // Wait for the task to exit the loop and delete itself
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
                piston_timer = 0;
                piston_move = false;
            }
            vTaskDelay(piston_period);
            piston_timer += piston_period;
        }
    }
    vTaskDelay(xDelay5000ms);

    if (piston_move)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS deliberately stopping the Piston\n");
        /* stop the piston */
        PIS_task_delete(); // Signal to exit loop
        vTaskDelay(xDelay10000ms); // Wait for the task to exit the loop and delete itself
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
                // Check if any park data samples were actually collected in this cycle
                if (current_park_data.cbuf.written > 0) { 
                    DATA_add_gps(&current_park_data, gps.latitude, gps.longitude); 
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added GPS to park data (%u samples exist)\n", current_park_data.cbuf.written);
                }
                
                // Check if any profile data samples were actually collected in this cycle
                if (current_profile_data.cbuf.written > 0) { 
                    DATA_add_gps(&current_profile_data, gps.latitude, gps.longitude); 
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added GPS to profile data (%u samples exist)\n", current_profile_data.cbuf.written);
                }

                /* Calibrate the GPS UTC time into RTC */
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, RTC : <GPS Time Set>\n");
                artemis_rtc_gps_calibration(&gps);
                fix = 0;
            }

            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->finished\n");
            run = false;
            gpsTimer = 0;
            vTaskDelay(xDelay100ms);
            /** GPS OFF */
            SENS_sensor_gps_off();
            spsEvent = MODE_DONE;
        }
        else if (gpsTimer >= GPS_TIMER * 60 * period + (5 * period)) 
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task TIMEOUT. Forcefully ending task...\n");
            killGPS();
            vTaskDelay(xDelay5000ms);

            /* check, if it got at least two to three fixes */
            if (fix >= 2)
            {
                /* update latitude and longitude for park and profile modes */
                // Check if any park data samples were actually collected in this cycle
                if (current_park_data.cbuf.written > 0) { 
                    DATA_add_gps(&current_park_data, gps.latitude, gps.longitude); 
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added GPS to park data (%u samples exist)\n", current_park_data.cbuf.written);
                }
                
                // Check if any profile data samples were actually collected in this cycle
                if (current_profile_data.cbuf.written > 0) { 
                    DATA_add_gps(&current_profile_data, gps.latitude, gps.longitude); 
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Added GPS to profile data (%u samples exist)\n", current_profile_data.cbuf.written);
                }

                /* Calibrate the GPS UTC time into RTC */
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, RTC : <GPS Time Set>\n");
                artemis_rtc_gps_calibration(&gps);
                fix = 0;
            }
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->finished\n");
            run = false;
            gpsTimer = 0;
            vTaskDelay(xDelay100ms);
            /** GPS OFF */
            SENS_sensor_gps_off();
            spsEvent = MODE_DONE;
        }
        vTaskDelayUntil(&xLastWakeTime, period);
        gpsTimer += period;
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

/**
 * @brief State function for transmitting queued data via Iridium.
 * Attempts items sequentially. Continues ONLY on full item success.
 * Exits state on ANY persistent item failure in the current cycle.
 * Resets attempt counter ONLY if max attempts were reached for the failed item before exiting.
 */
void module_sps_tx(void)
{
    Event_e spsEvent = MODE_IDLE; // Default next state
    QueuedDataEntry_t *current_item = NULL;
    bool fatal_error_occurred = false;
    bool stop_processing_queue_this_cycle = false; // Flag to break main loop on ANY persistent failure for the current item
    bool max_attempts_stop_reason = false;      // Specific flag indicating max attempts caused the stop
    bool item_processed_successfully = false;   // Flag set when an item is fully transmitted and removed
    bool queue_processed_at_least_once = false; // Renamed original queue_processed for clarity

    MEM_log_memory_status("SPS :: tx start");

    // --- Initialize Iridium (Only if not already initialized) ---
    if (!iridium_init) {
        i9603n_initialize(); // Initializes UART etc.
        iridium_init = true;
        vTaskDelay(xDelay1000ms);
        #ifdef TEST
        // datalogger_test_sbd_messages_init(); // Assuming this function exists
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, TEST mode - SBD message init called.\n");
        #endif
    }

    // --- Power On Iridium ---
    uint8_t tries = 0;
    bool iridium_ready = false;
    while (tries < 2 && !fatal_error_occurred) {
        if (i9603n_on()) {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium powered on.\n");
            vTaskDelay(xDelay1000ms); // Allow modem time to boot
            iridium_ready = true;
            break;
        } else {
            tries++;
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium power on attempt %u failed. Retrying...\n", tries);
            i9603n_off(); // Ensure it's fully off before retry
            vTaskDelay(xDelay2000ms);
        }
    }

    if (!iridium_ready) {
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium failed to power on after %u tries. Exiting TX state.\n", tries);
        #if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
        // datalogger_read_test_profile(true); // Assuming this exists
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, TEST mode - Reading test profile due to power fail.\n");
        #endif
        goto cleanup_and_exit; // Use goto for cleaner exit
    }


    // --- Main Transmission Loop ---
    // Processes items sequentially. Exits if queue is empty or any item fails persistently.
    while (MEM_queue_get_count() > 0 && !fatal_error_occurred && !stop_processing_queue_this_cycle)
    {
        queue_processed_at_least_once = true; // Mark that we entered the loop
        current_item = MEM_queue_get_next(); // Get item at head

        if (current_item == NULL) {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Queue count > 0 but get_next is NULL! Exiting TX loop.\n");
            fatal_error_occurred = true; // Inconsistent state
            break; // Exit main loop
        }

        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Processing %s %u (Attempts: %u, Samples: %u)\n",
                             current_item->is_park_data ? "Park" : "Profile",
                             current_item->profile_number,
                             current_item->attempt_count,
                             current_item->num_samples);

        uint16_t samples_processed_this_item = 0;
        uint8_t current_page_num = 0;
        bool item_fully_transmitted = false; // Tracks if all pages were SENT successfully for this item
        uint8_t max_transmit_tries = current_item->is_park_data ? PARK_TRANSMIT_TRIES : PROF_TRANSMIT_TRIES;
        bool exit_paging_loop = false;       // Flag to break out of page processing for this item upon failure
        item_processed_successfully = false; // Reset for each item attempt

        // --- Paging Loop (for current_item) ---
        // Processes pages sequentially unless item finishes, a fatal error occurs, or a page fails persistently.
        while (!item_fully_transmitted && !fatal_error_occurred && !exit_paging_loop)
        {
            TaskHandle_t xIridium = NULL;
            TaskHandle_t xSatellite = NULL;
            eTaskState eStatus;
            bool page_transmission_success = false; // Success for the *current* page transmission cycle
            uint16_t samples_packed_this_page = 0;
            // Buffer for the current page's packed data - declared inside loop
            uint8_t current_page_buffer[IRID_DATA_OUT];

            // --- Pack Page ---
            memset(current_page_buffer, 0, IRID_DATA_OUT);
            uint16_t txbytes = prepare_transmit_page(current_page_buffer,
                                                     current_item,
                                                     samples_processed_this_item,
                                                     current_page_num,
                                                     &samples_packed_this_page);

            if (txbytes == 0 || samples_packed_this_page == 0) 
            {
                if (samples_processed_this_item >= current_item->num_samples) 
                {
                    // Should have been caught by item_fully_transmitted, but defensive check
                    item_fully_transmitted = true;
                    item_processed_successfully = true; // Assume previous pages were successful
                } 
                else 
                {
                    // Error during packing
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, ERROR packing page %u for item %u! Discarding item.\n", current_page_num, current_item->profile_number);
                    MEM_queue_mark_transmitted(); // Discard unprocessable item
                    item_fully_transmitted = true; // Treat as 'processed' (by discarding)
                    item_processed_successfully = false; // Explicitly mark as not successful
                    exit_paging_loop = true;       // Ensure loop exits
                }
                break; // Exit paging loop
            }

            // --- Transmission Attempt Loop (retries CURRENT PAGE on temporary failure) ---
            bool retry_page_internally = true; // Flag to control retries for THIS page
            while (retry_page_internally && !fatal_error_occurred)
            {
                retry_page_internally = false; // Assume success for this inner attempt unless flag is set true again

                // --- Satellite Check ---
                bool run_satellite = true;
                uint8_t satellite_tries = 0;
                task_Iridium_satellite_visibility(&xSatellite); // This task self-deletes
                page_transmission_success = false; // Assume check will fail unless it explicitly succeeds
                while(run_satellite && !fatal_error_occurred) 
                {
                    eStatus = eTaskGetState(xSatellite);
                    if (eStatus == eRunning || eStatus == eReady || eStatus == eBlocked) 
                    { 
                        vTaskDelay(xDelay1000ms); /* Waiting... */ 
                    }
                    else if (eStatus == eDeleted || eStatus == eInvalid) 
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite check finished for page %u.\n", current_page_num);
                        satellite_tries++;
                        bool visible = GET_Iridium_satellite();
                        if (visible) 
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite VISIBLE.\n");
                            page_transmission_success = true; // Visibility OK for this attempt
                            run_satellite = false; // Exit check loop
                            SET_Iridium_delay_rate(0.5);
                        } 
                        else 
                        { // Not visible
                            if (satellite_tries >= SATELLITE_VISIBILITY_TRIES) 
                            {
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite NOT visible after %u tries.\n", satellite_tries);
                                page_transmission_success = false; // Visibility FAILED persistently for this attempt cycle
                                run_satellite = false; // Exit check loop
                                SET_Iridium_delay_rate(0.1); // Keep reduced rate if needed later
                            } 
                            else 
                            { 
                                /* Wait and restart check */ 
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite NOT Visible, waiting 20 sec...\n"); 
                                i9603n_sleep(); 
                                vTaskDelay(xDelay20000ms); 
                                i9603n_wakeup(); 
                                xSatellite=NULL; 
                                task_Iridium_satellite_visibility(&xSatellite);
                            }
                        }
                    } 
                    else if (eStatus == eSuspended) 
                    { 
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite check task suspended, retrying...\n"); 
                        vTaskDelete(xSatellite); 
                        xSatellite = NULL; 
                        task_Iridium_satellite_visibility(&xSatellite); 
                        vTaskDelay(xDelay1000ms); 
                    }
                    else // BAD STATE
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite check task in unexpected state: %d. Exiting check loop...\n", eStatus);
                        
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite check finished for page %u.\n", current_page_num);
                        satellite_tries++;
                        bool visible = GET_Iridium_satellite();
                        if (visible) 
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite VISIBLE.\n");
                            page_transmission_success = true; // Visibility OK for this attempt
                            run_satellite = false; // Exit check loop
                            SET_Iridium_delay_rate(0.5);
                        } 
                        else 
                        { // Not visible
                            if (satellite_tries >= SATELLITE_VISIBILITY_TRIES) 
                            {
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite NOT visible after %u tries.\n", satellite_tries);
                                page_transmission_success = false; // Visibility FAILED persistently for this attempt cycle
                                run_satellite = false; // Exit check loop
                                SET_Iridium_delay_rate(0.1); // Keep reduced rate if needed later
                            } 
                            else 
                            { 
                                /* Wait and restart check */ 
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite NOT Visible, waiting 20 sec...\n"); 
                                i9603n_sleep(); 
                                vTaskDelay(xDelay20000ms); 
                                i9603n_wakeup(); 
                                xSatellite=NULL; 
                                task_Iridium_satellite_visibility(&xSatellite);
                            }
                        }
                    } // Added delay for non-waiting, non-finished states
                } // End satellite check loop

                // If satellite check failed permanently for this attempt cycle, exit attempt loop
                if (!page_transmission_success) 
                {
                     ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite visibility failed for this attempt cycle.\n");
                     break; // Exit attempt loop, page_transmission_success remains false
                }
                if(fatal_error_occurred) { 
                    break; 
                } // Exit if fatal error during check

                // --- Send data to modem buffer ---
                if (!i9603n_send_data(current_page_buffer, txbytes)) 
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Failed send Page %u to buffer. Retrying attempt...\n", current_page_num);
                    page_transmission_success = false; // Mark as failed for this attempt
                    retry_page_internally = true;      // Signal inner loop to retry
                    vTaskDelay(xDelay2000ms);
                    continue; // Retry this inner attempt loop
                }
                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u data (%u bytes) sent to modem buffer.\n", current_page_num, txbytes);
                vTaskDelay(xDelay1000ms);

                // --- Initiate Transfer and Monitor ---
                task_Iridium_transfer(&xIridium); // This task also self-deletes
                bool transfer_run = true;
                page_transmission_success = false; // Reset before this transfer attempt
                vTaskDelay(xDelay1000ms); // Allow time for task to start
                while(transfer_run && !fatal_error_occurred) 
                {
                    eStatus = eTaskGetState(xIridium);
                    if (eStatus == eRunning || eStatus == eReady || eStatus == eBlocked) 
                    { 
                        vTaskDelay(xDelay1000ms); /* Waiting */ 
                    }
                    else if (eStatus == eDeleted || eStatus == eInvalid) 
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task finished.\n"); 
                        vTaskDelay(xDelay500ms);
                        uint8_t recv[6] = {0}; 
                        bool status_ok = GET_Iridium_status(recv); 
                        transfer_run = false;
                        if (status_ok) 
                        {
                            if (recv[0] <= 4) 
                            { // Success code
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit SUCCESSFUL.\n", current_page_num);
                                page_transmission_success = true; // Page success!
                                // Let loop exit naturally
                            } 
                            else 
                            { // Failure code
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit FAILED (Iridium Status: %u).\n", current_page_num, recv[0]);
                                page_transmission_success = false; // Page failed
                                retry_page_internally = true; // Signal inner loop to retry page after delay
                                // Handle delays based on error codes
                                uint16_t wait_time = 10; // Default wait time
                                if (recv[0] == 38) 
                                { // Traffic management
                                    uint16_t traffic_buf[8] = {0}; uint8_t traffic_len = i9603n_traffic_mgmt_time(traffic_buf);
                                    if(traffic_len > 0 && traffic_buf[0] == 0) 
                                    {
                                        wait_time = traffic_buf[1];
                                    }
                                    if(wait_time < 10) wait_time = 10;
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Traffic management wait: %u sec\n", wait_time);
                                } // Add other specific error code delays here if needed
                                i9603n_sleep(); 
                                vTaskDelay(pdMS_TO_TICKS(wait_time * 1000)); 
                                i9603n_wakeup(); 
                                vTaskDelay(xDelay1000ms);
                                continue; // Continue attempt loop to retry page
                            }
                        } 
                        else 
                        { // Failed to get Iridium status
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, ERROR getting Iridium transmit status! Retrying attempt...\n");
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task finished.\n"); 
                            vTaskDelay(xDelay500ms);
                            uint8_t recv[6] = {0}; 
                            bool status_ok = GET_Iridium_status(recv); 
                            transfer_run = false;
                            if (status_ok) 
                            {
                                if (recv[0] <= 4) 
                                { // Success code
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit SUCCESSFUL.\n", current_page_num);
                                    page_transmission_success = true; // Page success!
                                    // Let loop exit naturally
                                } 
                                else 
                                { // Failure code
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit FAILED (Iridium Status: %u).\n", current_page_num, recv[0]);
                                    page_transmission_success = false; // Page failed
                                    retry_page_internally = true; // Signal inner loop to retry page after delay
                                    // Handle delays based on error codes
                                    uint16_t wait_time = 10; // Default wait time
                                    if (recv[0] == 38) 
                                    { // Traffic management
                                        uint16_t traffic_buf[8] = {0}; 
                                        uint8_t traffic_len = i9603n_traffic_mgmt_time(traffic_buf);
                                        if(traffic_len > 0 && traffic_buf[0] == 0) 
                                        {
                                            wait_time = traffic_buf[1];
                                        }
                                        if(wait_time < 10) wait_time = 10;
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Traffic management wait: %u sec\n", wait_time);
                                    } // Add other specific error code delays here if needed
                                    i9603n_sleep(); 
                                    vTaskDelay(pdMS_TO_TICKS(wait_time * 1000)); 
                                    i9603n_wakeup(); 
                                    vTaskDelay(xDelay1000ms);
                                    continue; // Continue attempt loop to retry page
                                }
                            }
                        } 
                    }
                    else if (eStatus == eSuspended) 
                    { 
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task suspended?\n"); 
                        vTaskDelay(xDelay1000ms); 
                    }
                    else // Handle unexpected task states
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task in unexpected state: %d. Exiting TX loop...\n", eStatus);
                        vTaskDelay(xDelay1000ms);
                        
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task finished.\n"); 
                        vTaskDelay(xDelay500ms);
                        uint8_t recv[6] = {0}; 
                        bool status_ok = GET_Iridium_status(recv); 
                        transfer_run = false;
                        if (status_ok) 
                        {
                            if (recv[0] <= 4) 
                            { // Success code
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit SUCCESSFUL.\n", current_page_num);
                                page_transmission_success = true; // Page success!
                                // Let loop exit naturally
                            } 
                            else 
                            { // Failure code
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit FAILED (Iridium Status: %u).\n", current_page_num, recv[0]);
                                page_transmission_success = false; // Page failed
                                retry_page_internally = true; // Signal inner loop to retry page after delay
                                // Handle delays based on error codes
                                uint16_t wait_time = 10; // Default wait time
                                if (recv[0] == 38) 
                                { // Traffic management
                                    uint16_t traffic_buf[8] = {0}; 
                                    uint8_t traffic_len = i9603n_traffic_mgmt_time(traffic_buf);
                                    if(traffic_len > 0 && traffic_buf[0] == 0) 
                                    {
                                        wait_time = traffic_buf[1];
                                    }
                                    if(wait_time < 10) wait_time = 10;
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Traffic management wait: %u sec\n", wait_time);
                                } // Add other specific error code delays here if needed
                                i9603n_sleep(); 
                                vTaskDelay(pdMS_TO_TICKS(wait_time * 1000)); 
                                i9603n_wakeup(); 
                                vTaskDelay(xDelay1000ms);
                                continue; // Continue attempt loop to retry page
                            }
                        } 
                        else 
                        { 
                            // Failed to get Iridium status
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, ERROR getting Iridium transmit status! Retrying attempt...\n");
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium transfer task finished.\n"); 
                            vTaskDelay(xDelay500ms);
                            uint8_t recv[6] = {0}; 
                            bool status_ok = GET_Iridium_status(recv); 
                            transfer_run = false;
                            if (status_ok) 
                            {
                                if (recv[0] <= 4) 
                                { // Success code
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit SUCCESSFUL.\n", current_page_num);
                                    page_transmission_success = true; // Page success!
                                    // Let loop exit naturally
                                } 
                                else 
                                { // Failure code
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u transmit FAILED (Iridium Status: %u).\n", current_page_num, recv[0]);
                                    page_transmission_success = false; // Page failed
                                    retry_page_internally = true; // Signal inner loop to retry page after delay
                                    // Handle delays based on error codes
                                    uint16_t wait_time = 10; // Default wait time
                                    if (recv[0] == 38) 
                                    { // Traffic management
                                        uint16_t traffic_buf[8] = {0}; 
                                        uint8_t traffic_len = i9603n_traffic_mgmt_time(traffic_buf);
                                        if(traffic_len > 0 && traffic_buf[0] == 0) 
                                        {
                                            wait_time = traffic_buf[1];
                                        }
                                        if(wait_time < 10) wait_time = 10;
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Traffic management wait: %u sec\n", wait_time);
                                    } // Add other specific error code delays here if needed
                                    i9603n_sleep(); 
                                    vTaskDelay(pdMS_TO_TICKS(wait_time * 1000)); 
                                    i9603n_wakeup(); 
                                    vTaskDelay(xDelay1000ms);
                                    continue; // Continue attempt loop to retry page
                                }
                            }
                        } 
                    }
                } // end transfer_run loop

            } // End Transmission Attempt Loop (retry_page_internally)

            // --- Handle outcome for the CURRENT PAGE ---
            if (page_transmission_success) 
            {
                // Page was sent successfully after internal retries (if any)
                MEM_queue_reset_attempts(); // Reset item's overall attempts on page success
                samples_processed_this_item += samples_packed_this_page;

                if (samples_processed_this_item >= current_item->num_samples) 
                {
                    // This was the last page for the item
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Item %u fully transmitted.\n", current_item->profile_number);
                    MEM_queue_mark_transmitted();      // Remove successful item from RAM queue
                    item_fully_transmitted = true;     // Mark item processing as done for paging loop
                    item_processed_successfully = true;// Flag overall success for this item
                    exit_paging_loop = true;         // Exit paging loop (main loop will continue to next item)
                } 
                else 
                {
                    // More pages remain for this item
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u sent, moving to next page for item %u.\n", current_page_num, current_item->profile_number);
                    current_page_num++;
                    // Continue paging loop for the next page
                }
            } 
            else 
            {
                // Failed to transmit this page after all internal retries for this attempt cycle
                MEM_queue_increment_attempt();
                // Re-fetch item pointer as the queue implementation might use it directly
                // (or ensure MEM_queue_increment_attempt works correctly on the handle)
                current_item = MEM_queue_get_next();
                if (current_item == NULL) 
                { // Should not happen if count > 0 but defensive check
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, ERROR: Head item became NULL after failed page attempt!\n");
                    fatal_error_occurred = true;
                    break; // Exit paging loop immediately
                }

                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Item %u attempt count now %u.\n", current_item->profile_number, current_item->attempt_count);

                if (MEM_queue_max_attempts_reached(max_transmit_tries)) 
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, MAX ATTEMPTS (%u) reached for item %u. Stopping ALL transmissions for this cycle.\n",
                                        max_transmit_tries, current_item->profile_number);
                    max_attempts_stop_reason = true;          // Set specific flag for cleanup logic
                    stop_processing_queue_this_cycle = true; // Signal outer loop to stop
                } 
                else 
                {
                    // Failed, but more attempts remain. Stop processing for THIS CYCLE.
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Page %u failed for item %u, more attempts remain. Stopping TX cycle.\n",
                                        current_page_num, current_item->profile_number);
                    stop_processing_queue_this_cycle = true; // Signal outer loop to stop
                }
                // Exit the paging loop in both failure cases (max attempts or not)
                // because we stop processing the queue for this cycle upon any persistent page failure.
                exit_paging_loop = true;
            }
            vTaskDelay(xDelay500ms); // Small delay between pages or before exiting item loop

        } // End Paging Loop

        // If processing stopped for ANY reason other than successful transmission of the item, break the main loop
        if (!item_processed_successfully || stop_processing_queue_this_cycle || fatal_error_occurred) 
        {
            break; // Exit the main 'while (MEM_queue_get_count...' loop
        }

        // If we get here, the item was successful, and the main loop continues to the next item.
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Successfully processed item %u. Checking for next item...\n", current_item->profile_number);
        vTaskDelay(xDelay1000ms); // Delay before processing next item

    } // End Main Transmission Loop


// --- Cleanup and Exit Logic ---
cleanup_and_exit:
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Entering cleanup and exit. Max attempts stop reason: %d\n", max_attempts_stop_reason);

    // Reset attempts ONLY if we stopped processing BECAUSE the head item reached max attempts
    if (max_attempts_stop_reason) 
    {
        // Get the pointer again in case the queue was modified unexpectedly
        current_item = MEM_queue_get_next();
        if (current_item != NULL) 
        {
           ARTEMIS_DEBUG_PRINTF("SPS :: tx, Resetting attempts for failed item %u before exiting.\n", current_item->profile_number);
           MEM_queue_reset_attempts();
        } else 
        {
            // This case should ideally not happen if max_attempts_stop_reason is true
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, WARNING: Failed item pointer was NULL during cleanup reset attempt.\n");
        }
    }

    // Power off and De-initialize Iridium
    if (iridium_init) // Check if initialized at all during this run
    { 
        if (iridium_ready) // Only turn off if power on succeeded
        { 
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Powering off Iridium.\n");
            i9603n_off();
        } 
        else 
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium power on failed, ensuring power is off.\n");
            i9603n_off(); // Ensure power is off even if startup failed
        }
        // Always uninitialize the driver state if init flag was set
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Uninitializing Iridium driver.\n");
        i9603n_uninitialize();
        iridium_init = false;
    }
    vTaskDelay(xDelay1000ms); // Allow time for power off

    monitor_memory_usage();

    // Determine next state
    // Base decision only on RAM queue and profile number completion
    if (prof_number >= SYSTEM_PROFILE_NUMBER && MEM_queue_get_count() == 0) 
    {
        spsEvent = MODE_POPUP;
        ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, << All profiles processed and RAM queue empty >>\n\n");
    } 
    else 
    {
        // If we exited due to failure (max attempts or otherwise), or success but more items remain, go to IDLE
        spsEvent = MODE_IDLE;
        #if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
        // This datalogger call might be better placed in the transition logic *to* IDLE state
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, TEST mode - Reading test profile on exit to IDLE.\n");
        // datalogger_read_test_profile(true); // Assuming this exists
        #endif
    }

    MEM_log_memory_status("SPS :: tx end");

    // Reset the test pressure profile for the next cycle
    #if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
        /* reset test profile */
        datalogger_read_test_profile(true);
    #endif

    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Task->finished, transitioning to %s\n\n",
                         (spsEvent == MODE_IDLE) ? "MODE_IDLE" : "MODE_POPUP");

    // Send the event to the queue (void function)
    SendEvent(spsEventQueue, &spsEvent);

    vTaskDelete(NULL); // Delete self task
}

// *** NEW COMBINED STATIC HELPER FUNCTION ***
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