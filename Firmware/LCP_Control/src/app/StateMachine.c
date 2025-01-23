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

/* park numbers and measurements */
static pData pPark[SYSTEM_PROFILE_NUMBER];
static float park_temp[DATA_PARK_SAMPLES_MAX];
static float park_pressure[DATA_PARK_SAMPLES_MAX];
/* profile numbers and measurements */
static pData pProf[SYSTEM_PROFILE_NUMBER];
static float prof_temp[DATA_PROFILE_SAMPLES_MAX];
static float prof_pressure[DATA_PROFILE_SAMPLES_MAX];

static Data_t park;         /**< Park mode Data */
static Data_t prof;         /**< Profile mode data */

static sData sPark;         /**< Park mode measurement - StateMachine */
static sData sProf;         /**< Profile mode measurement - StateMachine */
//static sData sPark_ext;     /**< Park mode measurement extended - StateMachine */
//static sData sProf_ext;     /**< Profile mode measurement extended - StateMachine */

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
static uint16_t create_park_page(uint8_t *pPark, uint8_t *readlength);
static uint16_t create_profile_page(uint8_t *pProf, uint8_t *readlength);

/* global variables */
static volatile bool sensors_check = false;
static volatile bool iridium_init = false;

static volatile uint8_t prof_number = 0;
static volatile uint8_t park_number = 0;
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
            break;

        case SYSST_AutoBallast_mode:
            break;

        case SYSST_SimpleProfiler_mode:
            DATA_setbuffer(&park, pPark, park_pressure, park_temp, DATA_PARK_SAMPLES_MAX);
            DATA_setbuffer(&prof, pProf, prof_pressure, prof_temp, DATA_PROFILE_SAMPLES_MAX);
            break;

        case SYSST_Moored_mode:
            break;

        case SYSST_AirDeploy_mode:
            break;

        case SYSST_Popup_mode:
            break;

        default:
            break;
    }
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
                }
                /* reset the rate counter and rate_avg*/
                rate_count = 0;
                rate_avg = 0.0;
            }
        }

        while (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
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
                    vTaskDelay(piston_period);
                    PIS_Reset();
                    piston_move = false;
                    piston_timer = 0;
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->suspended\n");
                PIS_task_delete(xPiston);
                //piston_move = false;
                piston_timer = 0;
            }
            else if (eStatus==eDeleted)
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
    vTaskDelay(piston_period);

    Event_e pusEvent;
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
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
                vTaskDelay(piston_period);
                PIS_Reset();
                piston_move = false;
                piston_timer = 0;
                pusEvent = MODE_DONE;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            //piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
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

    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->active\n");
            /* piston time for up to 120 seconds */
            piston_timer += piston_period;
            if (piston_timer >= 120000)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston time-out, task->finished\n");
                PIS_task_delete(xPiston);
                vTaskDelay(piston_period);
                PIS_Reset();
                piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            //piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
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
            else if (eStatus==eDeleted)
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
                                        "sps_profile", 1048, NULL,
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

    /* set crush depth to false */
    crush_depth = false;

#ifdef TEST
    /* do nothing */
#else

    /*Check if it is time to zero the piston encoder counts, if yes, then zero cal the piston*/
    if( pistonzero_number >= PISTON_ZEROCAL_COUNTER )
    {

        piston_timer = 0;
        uint8_t pistoncal = 0;
        piston_move = true;

        eTaskState eStatus;
        TaskHandle_t xPiston = NULL;
        PIS_set_piston_rate(1);
        PIS_task_move_zero(&xPiston); /*This is the piston zero reset command*/
        vTaskDelay(piston_period);
    
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_park, Setting -> Piston encoder value to zero, %u profiles reached since last cal >>\n\n", pistonzero_number);

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
                    vTaskDelay(piston_period);
                    PIS_Reset();
                    piston_timer = 0;
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->suspended\n");
                PIS_task_delete(xPiston);
                piston_timer = 0;
            }
            else if (eStatus==eDeleted)
            {
                vTaskDelay(piston_period);
                PIS_Get_Length(&zlengthdrift);
                vTaskDelay(piston_period);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Length=%.4fin, Try%u\n", zlengthdrift, pistoncal);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Task->Finished\n");
                piston_move = false;
                piston_timer = 0;
            }
            vTaskDelay(piston_period);
        }
        zlengthadjust = 0.0 - zlengthdrift;

        if( (zlengthadjust>=0.25) || (zlengthadjust<=-0.25))
        {
            if ( pistoncal < 1)
            {
                pistoncal++;
                piston_timer = 0;
                piston_move = true;

                eTaskState eStatus;
                TaskHandle_t xPiston = NULL;
                PIS_set_piston_rate(1);
                PIS_task_move_zero(&xPiston); /*This is the piston zero reset command*/
                vTaskDelay(piston_period);

                while (piston_move)
                {
                    eStatus = eTaskGetState( xPiston );
                    if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                    {   
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->active\n");
                        /* piston time for up to 60 seconds */
                        piston_timer += piston_period;
                        if (piston_timer >= 60000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(piston_period);
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                    else if (eStatus==eSuspended)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston zero task->suspended\n");
                        PIS_task_delete(xPiston);
                        piston_timer = 0;
                    }
                    else if (eStatus==eDeleted)
                    {
                        vTaskDelay(piston_period);
                        PIS_Get_Length(&zlengthdrift);
                        vTaskDelay(piston_period);
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Length=%.4fin, Try%u\n", zlengthdrift, pistoncal);
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Zero Cal Task->Finished\n");
                        piston_move = false;
                        piston_timer = 0;
                    }
                    vTaskDelay(piston_period);
                }
            }
            zlengthadjust = 0.0 - zlengthdrift;
        }
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

    eTaskState eStatus;
    TaskHandle_t xPiston = NULL;
    PIS_set_piston_rate(1);
    PIS_set_length(length_update);
    PIS_task_move_length(&xPiston);
    vTaskDelay(piston_period);

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
                vTaskDelay(piston_period);
                PIS_Reset();
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
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
                    }
                }
                else
                {
                    park_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
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
                if ( (eStatus==eRunning) || (eStatus==eBlocked) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->active\n");
                    /* keep piston time for up to 30 seconds unless crush_depth activated use piston up to 120 seconds */
                    piston_timer += piston_period;

                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(piston_period);
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
                            vTaskDelay(piston_period);
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                }
                else if (eStatus==eReady)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->Ready\n");
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
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended\n");
                    PIS_task_delete(xPiston);
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
                else if (eStatus==eDeleted)
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
            DATA_add(&park, epoch, Pressure, Temperature, park_number);
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
            float var, avg_p, avg_t;
            float std = std_div(samples_p, samples, &var, &avg_p);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            std = std_div(samples_t, samples, &var, &avg_t);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            /* store averages data locally */
            DATA_add(&park, epoch, avg_p, avg_t, park_number);
            samples = 0;

#ifdef TEST
            read++;
            ARTEMIS_DEBUG_PRINTF("SPS :: park, sending measurements = %u\n", read);
#else
            datalogger_park_mode(filename, avg_p, avg_t, &time);
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
        }
        else
        {
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
                    }
                    else if (averaged_rate < 0.0 && !piston_move && !crush_depth)
                    {
                        /* increase piston position by PARK_POSITION_INCREMENT inches */
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
                        }

                        /* check if it needs to move the piston or not */
                        PIS_Get_Length(&Length);
                        vTaskDelay(piston_period);

                        if (length_update <= PISTON_POSITION_MINIMUM && Length <= PISTON_POSITION_MINIMUM)
                        {
                        /* do not even send a piston command, do nothing  */
                        }
                        else
                        {
                        park_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
                        }
                    }
                    else if (averaged_rate > 0.0 && !piston_move && !crush_depth)
                    {
                        /* decrease piston position by PARK_POSITION_INCREMENT inches */
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
                        else
                        {
                        park_piston_length = length_update;
                        PIS_set_length(length_update);
                        PIS_task_move_length(&xPiston);
                        piston_move = true;
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
                if ( (eStatus==eRunning) || (eStatus==eBlocked) )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->active\n");
                    /* keep piston time for up to 30 seconds unless crush_depth activated use piston up to 120 seconds */
                    piston_timer += piston_period;
                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(piston_period);
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
                            vTaskDelay(piston_period);
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                }
                else if (eStatus==eReady)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->Ready\n");
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
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->suspended\n");
                    PIS_task_delete(xPiston);
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
                else if (eStatus==eDeleted)
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
        }

        /* check on Maximum park depth = ? */
        if (Depth >= PARK_DEPTH_MAX && !crush_depth)
        {
            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
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

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
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

    park_number++;

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nSPS :: park, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

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
    vTaskDelay(piston_period);

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
                vTaskDelay(piston_period);
                PIS_Reset();
                //piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            //piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
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
                    }
                }
                else
                {
                    to_prof_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
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
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Reach Porfile Depth\n");

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
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
                if ( (eStatus==eRunning) || (eStatus==eBlocked)  )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_porfile, Piston task->active\n");
                    /* keep piston time for up to 30 seconds unless crush_depth activated use piston up to 120 seconds */
                    piston_timer += piston_period;
                    if (crush_depth)
                    {
                        if (piston_timer >= 120000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston CRUSH_DEPTH time-out, task->finished\n");
                            PIS_task_delete(xPiston);
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
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                }
                else if (eStatus==eReady)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->ready\n");
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
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_porfile, Piston task->suspended\n");
                    PIS_task_delete(xPiston);
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
                else if (eStatus==eDeleted)
                {
                    PIS_Get_Length(&Length);
                    Volume = CTRL_calculate_volume_from_length(Length);
                    Density = CTRL_calculate_lcp_density(Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_porfile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
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
            /*add a CRUSH DEPTH FOS to the saved move-to-profile piston length for next profile*/
            to_prof_piston_length = length_update + 0.5;

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
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
    float Volume = 0.0;
    float Density = 0.0;
    float Length = 0.0;
    float length_update = 0.0;

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
    vTaskDelay(piston_period);

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
                PIS_Reset();
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
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
            DATA_add(&prof, epoch, Pressure, Temperature, prof_number);
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
            DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
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
            DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
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
            DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
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
            DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
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
            DATA_add(&prof, epoch, avg_p, avg_t, prof_number);
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
                        }
                    }
                }
                else
                {
                    prof_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
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
                        }
                    }
                }
                else
                {
                    prof_piston_length = length_update;
                    PIS_set_length(length_update);
                    PIS_task_move_length(&xPiston);
                    piston_move = true;
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
            if ( (eStatus==eRunning) || (eStatus==eBlocked) )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: porfile, Piston task->active\n");
                /* keep piston time for up to 30 seconds unless crush_depth activated use piston up to 120 seconds */
                piston_timer += period;

                if (crush_depth)
                {
                    if (piston_timer >= 120000)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston CRUSH_DEPTH time-out, task->finished\n");
                        PIS_task_delete(xPiston);
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
                        PIS_Reset();
                        piston_timer = 0;
                    }
                }
            }
            else if (eStatus==eReady)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->ready\n");
                //PIS_task_delete(xPiston);
                piston_move = false;
                piston_timer = 0;
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: porfile, Piston task->suspended\n");
                PIS_task_delete(xPiston);
                piston_timer = 0;
            }
            else if (eStatus==eDeleted)
            {
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: porfile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
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
        }

        if (Depth <= BALLAST_DEPTH_PROFILE)
        {
            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, deliberately stopping the Piston\n");
                PIS_task_delete(xPiston);
                /* try to stop first*/
                PIS_stop();
                piston_move = false;
                piston_timer = 0;
            }
            if (crush_depth)
            {
                /* set to previous adjusted length update */
                prof_piston_length = length_update_last_adjusted;
            }
            
            crush_depth = false;
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

    prof_number++;
    pistonzero_number++;
    pistonfull_number++;

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nSPS :: profile, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Task->finished\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_move_to_surface(void)
{
    /** Extend the piston fully out */
    uint32_t piston_period = xDelay1000ms;
    uint32_t piston_timer = 0;
    bool piston_move = true;
    float Volume = 0.0;
    float Density = 0.0;
    float Length = 0.0;
    float lengthadjust = 0.0;
    float lengthdrift  = 0.0;

    TaskHandle_t xPiston = NULL;
    eTaskState eStatus;
    PIS_set_piston_rate(1);

#if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
    /* set piston to 10.5in */
    PIS_set_length(10.5);
#else
    PIS_set_length(PISTON_MOVE_TO_SURFACE);
#endif

    PIS_task_move_length(&xPiston);
    vTaskDelay(piston_period);

    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->active\n");
            /* piston time for up to 120 seconds */
            piston_timer += piston_period;
            if (piston_timer >= 120000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston time-out, task->finished\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
                PIS_task_delete(xPiston);
                vTaskDelay(piston_period);
                PIS_Reset();
                piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->suspended\n");
            PIS_task_delete(xPiston);
            //piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
        {
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4fin\n", Density, Volume, Length);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
        }
        vTaskDelay(piston_period);
    }
    /*Check if it is time to reset the piston encoder counts to max at full piston extent, if yes, then run reset encoder to max value at full*/
    if( pistonfull_number >= PISTON_FULLCAL_COUNTER )
    {
        piston_timer = 0;
        uint8_t pistoncal = 0;
        piston_move = true;

        eTaskState eStatus;
        TaskHandle_t xPiston = NULL;
        PIS_set_piston_rate(1);
        PIS_task_reset_full(&xPiston); /*This is the reset piston encoder to full command*/
        vTaskDelay(piston_period);
    
        ARTEMIS_DEBUG_PRINTF("\n<< SPS :: move_to_surface, Setting -> Piston encoder value to max, %u profiles reached since last encoder full reset >>\n\n", pistonfull_number);

        /* check on piston movement */
        while (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
            {   
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->active\n");
                /* piston time for up to 60 seconds */
                piston_timer += piston_period;
                if (piston_timer >= 60000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full time-out, task->finished\n");
                    PIS_task_delete(xPiston);
                    vTaskDelay(piston_period);
                    PIS_Reset();
                    piston_timer = 0;
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->suspended\n");
                PIS_task_delete(xPiston);
                piston_timer = 0;
            }
            else if (eStatus==eDeleted)
            {
                vTaskDelay(piston_period);
                PIS_Get_Length(&lengthdrift);
                vTaskDelay(piston_period);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston encoder reset to full Length=%.4fin, Try%u\n", lengthdrift, pistoncal);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full Task->Finished\n");
                piston_move = false;
                piston_timer = 0;
            }
            vTaskDelay(piston_period);
        }
        lengthadjust = PISTON_POSITION_ATFULLRESET - lengthdrift;

        if( (lengthadjust>=0.25) || (lengthadjust<=-0.25))
        {
            if ( pistoncal < 1)
            {
                pistoncal++;
                piston_timer = 0;
                piston_move = true;

                eTaskState eStatus;
                TaskHandle_t xPiston = NULL;
                PIS_set_piston_rate(1);
                PIS_task_reset_full(&xPiston); /*This is the reset piston encoder to full command*/
                vTaskDelay(piston_period);
                
                while (piston_move)
                {
                    eStatus = eTaskGetState( xPiston );
                    if ( (eStatus==eRunning) || (eStatus==eBlocked) || (eStatus==eReady) )
                    {   
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->active\n");
                        /* piston time for up to 60 seconds */
                        piston_timer += piston_period;
                        if (piston_timer >= 60000)
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full time-out, task->finished\n");
                            PIS_task_delete(xPiston);
                            vTaskDelay(piston_period);
                            PIS_Reset();
                            piston_timer = 0;
                        }
                    }
                    else if (eStatus==eSuspended)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full task->suspended\n");
                        PIS_task_delete(xPiston);
                        piston_timer = 0;
                    }
                    else if (eStatus==eDeleted)
                    {
                        vTaskDelay(piston_period);
                        PIS_Get_Length(&lengthdrift);
                        vTaskDelay(piston_period);
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston encoder reset to full Length=%.4fin, Try%u\n", lengthdrift, pistoncal);
                        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston reset to full Task->Finished\n");
                        piston_move = false;
                        piston_timer = 0;
                    }
                    vTaskDelay(piston_period);
                }
            }
            lengthadjust = PISTON_POSITION_ATFULLRESET - lengthdrift;
        }
        if( (lengthadjust<=0.25) && (lengthadjust>=-0.25))
        {
            lengthadjust = PISTON_POSITION_ATFULLRESET - lengthdrift;
            /*Adjust saved position settings retain correct buoyancy states*/
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, New Piston Full Cal Length=%.4fin\n", PISTON_POSITION_ATFULLRESET);
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
                    DATA_add_gps(&park, gps.latitude, gps.longitude, park_number-1);
                    DATA_add_gps(&prof, gps.latitude, gps.longitude, prof_number-1);

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
        else if (eStatus==eDeleted)
        {
            /* check, if it got at least two to three fixes */
            if (fix >= 2)
            {
                /* update latitude and longitude for park and profile modes */
                DATA_add_gps(&park, gps.latitude, gps.longitude, park_number-1);
                DATA_add_gps(&prof, gps.latitude, gps.longitude, prof_number-1);

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

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_surface, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Task->finished\n\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_tx(void)
{
    Event_e spsEvent;
    /** Initialize the Iridium Modem */
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
        /* reset test profile */
        datalogger_read_test_profile(true);
#endif
        spsEvent = MODE_IDLE;
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium not charged, try again\n");
        SendEvent(spsEventQueue, &spsEvent);
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Task->finished abruptly, NOT transmitting today\n");
        vTaskDelete(NULL);
    }
    vTaskDelay(xDelay1000ms);

    /* move to transmitting bytes */
    TaskHandle_t xIridium = NULL;
    TaskHandle_t xSatellite = NULL;
    eTaskState eStatus;
    uint32_t visibility_period = xDelay1000ms;

    /* run task to start looking for satellites visibility */
    task_Iridium_satellite_visibility(&xSatellite);

    bool run = true;
    bool send_park = true;
    bool send_prof = true;
    bool run_satellite = true;
    uint8_t park_tries = 0;
    uint8_t prof_tries = 0;
    uint8_t satellite_tries = 0;

    while ( run )
    {
        /* start off with sending Park measurements pages */
        while (send_park)
        {
            park_tries++;
            /* run to see the satellites visibility */
            while(run_satellite)
            {
                eStatus = eTaskGetState( xSatellite );
                if ( (eStatus==eRunning) ||
                     (eStatus==eReady)   ||
                     (eStatus==eBlocked)  )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, Satellite, task->active\n");
                }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, Satellite, task->suspended\n");
                }
                else if (eStatus==eDeleted)
                {
                    satellite_tries++;
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, Satellite task->finished\n");
                    bool visible = GET_Iridium_satellite();
                    if (visible)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, Satellite <Visible>\n");
                        run_satellite = false;
                        satellite_tries = 0;
                        /* set iridium initiate transfer delay to 2 seconds */
                        SET_Iridium_delay_rate(0.5);
                    }
                    else
                    {
                        if (satellite_tries >= SATELLITE_VISIBILITY_TRIES)
                        {
                            run_satellite = false;
                            satellite_tries = 0;
                            /* set iridium initiate transfer delay to 10 seconds */
                            SET_Iridium_delay_rate(0.1);
                            vTaskDelay(xDelay500ms);
                        }
                        else
                        {
                            /* wait for 20 seconds */
                            visibility_period = 20;
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, Satellite <NOT Visible>, waiting for %u seconds\n\n", visibility_period);
                            i9603n_sleep();
                            vTaskDelay(xDelay1000ms * visibility_period);
                            i9603n_wakeup();
                            task_Iridium_satellite_visibility(&xSatellite);
                        }
                    }
                }
                vTaskDelay(xDelay1000ms);
            }

            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, Measurements written=%u, read=%u\n\n", park.cbuf.written, park.cbuf.read);
            uint8_t *ptrPark = &irid_park[0];
            uint8_t nr_park = 0;
            uint16_t txpark = create_park_page(ptrPark, &nr_park);

            if ( txpark > 0 )
            {
                /* we do have a page to send */
                /* send the bytes to the originated buffer */
                bool ret = i9603n_send_data(irid_park, txpark);
                if (ret)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park measurements=%u, bytes=%u are being transmitted\n", nr_park, txpark);
                }
                else
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park returned false\n");
                }
                vTaskDelay(xDelay1000ms);

                /* start iridium transfer task */
                task_Iridium_transfer(&xIridium);

                /* park run */
                bool park_run = true;
                while (park_run)
                {
                    /* Iridum task state checking */
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park checking task eState\n");
                    eStatus = eTaskGetState( xIridium );

                    if ( (eStatus==eRunning) ||
                         (eStatus==eReady)   ||
                         (eStatus==eBlocked)  )
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park task->active\n");
                    }
                    else if (eStatus==eSuspended)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park task->suspended\n");
                    }
                    else if (eStatus==eDeleted)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park task->finished\n");
                        vTaskDelay(xDelay500ms);

                        /* check if the page are transmitted successfully, tranmission number */
                        uint8_t recv[6] = {0};
                        bool ret = GET_Iridium_status (recv);
                        uint16_t wait_time = 0;
                        if (ret)
                        {
                            if (recv[0] <= 4)
                            {
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park transmit <Successful>\n\n");
                                park_run = false;
                                /* reset m_park_length and m_park_number */
                                if (m_park_length == 0)
                                {
                                    m_park_number++;
                                    sPark.pageNumber = 0;
                                    sPark.mLength = 0;

                                    ///* TODO: check extended measurements */
                                    //if (m_park_length_ext == 0)
                                    //{
                                    //    m_park_number_ext = 0;
                                    //    sPark_ext.pageNumber = 0;
                                    //    sPark_ext.mLength = 0;
                                    //}
                                }
                                else
                                {
                                    sPark.pageNumber++;
                                }
                            }
                            else if (recv[0] == 38)
                            {
                                /* wait for 2 seconds and look for traffic management valid time to wait */
                                vTaskDelay(xDelay2000ms);
                                uint16_t buf[8] = {0};
                                uint8_t len = i9603n_traffic_mgmt_time(buf);

                                /* put module into sleep mode */
                                i9603n_sleep();
                                if (len > 0)
                                {
                                    for (uint8_t i=0; i<len; i++)
                                    {
                                        ARTEMIS_DEBUG_PRINTF("%u ", buf[i]);
                                    }
                                    ARTEMIS_DEBUG_PRINTF("\n");

                                    if (buf[0] == 0)
                                    {
                                        wait_time = buf[1];
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, traffic management time is valid, %u seconds\n", wait_time);
                                        if (wait_time < 10 )
                                        {
                                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, waiting for 10 seconds instead\n");
                                            vTaskDelay(xDelay10000ms);
                                        }
                                        else
                                        {
                                            vTaskDelay(xDelay1000ms * wait_time);
                                        }
                                    }
                                    else
                                    {
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, traffic management time is not valid\n");
                                        wait_time = 10;
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, wait for %u seconds\n", wait_time);
                                        vTaskDelay(xDelay1000ms * wait_time);
                                    }
                                }
                                park_run = false;
                                /* wakeup module */
                                i9603n_wakeup();
                                vTaskDelay(xDelay1000ms);

                                /* reset the read length */
                                park.cbuf.read = park.cbuf.read - nr_park;
                                m_park_length += nr_park;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park after reset read=%u, m_park_length=%u, nr_park=%u\n\n", park.cbuf.read, m_park_length, nr_park);
                            }
                            else
                            {
                                /* put module into sleep mode */
                                i9603n_sleep();
                                wait_time = 10;
                                /* handle different error codes */
                                if (recv[0] == 18)
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, waiting for %u seconds\n", wait_time);
                                    vTaskDelay(xDelay1000ms * wait_time);
                                }
                                else if (recv[0] == 37)
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, waiting for %u seconds\n", wait_time);
                                    vTaskDelay(xDelay1000ms * wait_time);
                                }
                                else
                                {
                                    /* add more error codes handling, wait for 2 seconds for now */
                                    wait_time = 2;
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, waiting for %u seconds\n", wait_time);
                                    vTaskDelay(xDelay1000ms * wait_time);
                                }
                                i9603n_wakeup();
                                vTaskDelay(xDelay1000ms);

                                /* reset the read length */
                                park.cbuf.read = park.cbuf.read - nr_park;
                                m_park_length += nr_park;
                                park_run = false;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park after reset read=%u, m_park_length=%u, nr_park=%u\n\n", park.cbuf.read, m_park_length, nr_park);

                                if (park_tries >= PARK_TRANSMIT_TRIES)
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park transmit <NOT Successful>\n\n");
                                    send_park = false;
                                }
                                else
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park transmit <NOT Successful>\n\n");
                                    /* turn on checking satellite visibility */
                                    task_Iridium_satellite_visibility(&xSatellite);
                                    run_satellite = true;
                                    satellite_tries = 0;
                                }
                            }
                        }
                        else
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park ERROR :: getting transmit status\n");
                        }
                    }
                    vTaskDelay(xDelay1000ms);
                }
            }
            else
            {
                send_park = false;
            }
            vTaskDelay(xDelay1000ms);
        }

        /* turn on checking satellite visibility */
        task_Iridium_satellite_visibility(&xSatellite);
        run_satellite = true;
        satellite_tries = 0;

        /* start off with sending Profile measurements pages */
        while (send_prof)
        {
            prof_tries++;
            /* run to see the satellites visibility */
            while(run_satellite)
            {
                eStatus = eTaskGetState( xSatellite );
                if ( (eStatus==eRunning) ||
                     (eStatus==eReady)   ||
                     (eStatus==eBlocked)  )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, Satellite, task->active\n");
                }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, Satellite, task->suspended\n");
                }
                else if (eStatus==eDeleted)
                {
                    satellite_tries++;
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, Satellite, task->finished\n");
                    bool visible = GET_Iridium_satellite();
                    if (visible)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, Satellite, <Visible>\n");
                        run_satellite = false;
                        satellite_tries = 0;
                        /* set iridium initiate transfer delay to 2 seconds */
                        SET_Iridium_delay_rate(0.5);
                    }
                    else
                    {
                        if (satellite_tries >= SATELLITE_VISIBILITY_TRIES)
                        {
                            run_satellite = false;
                            satellite_tries = 0;
                            /* set iridium initiate transfer delay to 10 seconds */
                            SET_Iridium_delay_rate(0.1);
                            vTaskDelay(xDelay500ms);
                        }
                        else
                        {
                            /* wait for 20 seconds */
                            visibility_period = 20;
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, Satellite, <NOT Visible>, waiting for %u seconds\n\n", visibility_period);
                            i9603n_sleep();
                            vTaskDelay(xDelay1000ms * visibility_period);
                            i9603n_wakeup();
                            task_Iridium_satellite_visibility(&xSatellite);
                        }
                    }
                }
                vTaskDelay(xDelay1000ms);
            }

            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, Measurements written=%u, read=%u\n", prof.cbuf.written, prof.cbuf.read);
            uint8_t *ptrProf = &irid_prof[0];
            uint8_t nr_prof = 0;
            uint16_t txprof = create_profile_page(ptrProf, &nr_prof);

            if ( txprof > 0 )
            {
                /* we do have a page to send */
                /* send the bytes to the originated buffer */
                bool ret = i9603n_send_data(irid_prof, txprof);
                if (ret)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile measurements=%u, bytes=%u are being transmitted\n", nr_prof, txprof);
                }
                else
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile returned false\n");
                }
                vTaskDelay(xDelay1000ms);

                /* start iridium transfer task */
                task_Iridium_transfer(&xIridium);

                /* prof run */
                bool prof_run = true;
                while (prof_run)
                {
                    /* Iridum task state checking */
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Prof checking task eState\n");
                    eStatus = eTaskGetState( xIridium );

                    if ( (eStatus==eRunning) ||
                         (eStatus==eReady)   ||
                         (eStatus==eBlocked)  )
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile task->active\n");
                    }
                    else if (eStatus==eSuspended)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile task->suspended\n");
                    }
                    else if (eStatus==eDeleted)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile task->finished\n");
                        vTaskDelay(xDelay1000ms);

                        /* check if the page are transmitted successfully, tranmission number */
                        uint8_t recv[6] = {0};
                        bool ret = GET_Iridium_status (recv);
                        uint16_t wait_time = 10;

                        if (ret)
                        {
                            if (recv[0] <= 4)
                            {
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile transmit <Successful>\n\n");
                                prof_run = false;

                                /* reset m_park_length and m_park_number */
                                if (m_prof_length == 0)
                                {
                                    m_prof_number++;
                                    sProf.pageNumber = 0;
                                    sProf.mLength = 0;
                                }
                                else
                                {
                                    sProf.pageNumber++;
                                }
                            }
                            else if (recv[0] == 38)
                            {
                                /* wait for 2 seconds and look for traffic management valid time to wait */
                                vTaskDelay(xDelay2000ms);
                                uint16_t buf[8] = {0};
                                uint8_t len = i9603n_traffic_mgmt_time(buf);

                                /* put module into sleep mode */
                                i9603n_sleep();
                                if (len > 0)
                                {
                                    for (uint8_t i=0; i<len; i++)
                                    {
                                        ARTEMIS_DEBUG_PRINTF("%u ", buf[i]);
                                    }
                                    ARTEMIS_DEBUG_PRINTF("\n");

                                    if (buf[0] == 0)
                                    {
                                        wait_time = buf[1];
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, traffic management time is valid, %u seconds\n", wait_time);
                                        if (wait_time < 10 )
                                        {
                                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, waiting for 10 seconds instead\n");
                                            vTaskDelay(xDelay10000ms);
                                        }
                                        else
                                        {
                                            vTaskDelay(xDelay1000ms * wait_time);
                                        }
                                    }
                                    else
                                    {
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, traffic management time is not valid\n");
                                        wait_time = 10;
                                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, wait for %u seconds\n", wait_time);
                                        vTaskDelay(xDelay1000ms * wait_time);
                                    }
                                }
                                prof_run = false;
                                /* wakeup mode */
                                i9603n_wakeup();

                                /* reset the read length */
                                prof.cbuf.read = prof.cbuf.read - nr_prof;
                                m_prof_length += nr_prof;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile after reset read=%u, m_prof_length=%u, nr_prof=%u\n\n", prof.cbuf.read, m_prof_length, nr_prof);
                            }
                            else
                            {
                                /* put module into sleep mode */
                                i9603n_sleep();
                                wait_time = 10;
                                /* handle different error codes */
                                if (recv[0] == 18)
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, waiting for %u seconds\n", wait_time);
                                    vTaskDelay(xDelay1000ms * wait_time);
                                }
                                else if (recv[0] == 37)
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, waiting for %u seconds\n", wait_time);
                                    vTaskDelay(xDelay1000ms * wait_time);
                                }
                                else
                                {
                                    /* add more error codes handling, wait for 2 seconds for now */
                                    wait_time = 2;
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, waiting for %u seconds\n", wait_time);
                                    vTaskDelay(xDelay1000ms * wait_time);
                                }
                                /* wakeup mode */
                                i9603n_wakeup();

                                /* reset the read length */
                                prof.cbuf.read = prof.cbuf.read - nr_prof;
                                m_prof_length += nr_prof;
                                prof_run = false;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile after reset read=%u, m_prof_length=%u, nr_prof=%u\n\n", prof.cbuf.read, m_prof_length, nr_prof);
                                if (prof_tries >= PROF_TRANSMIT_TRIES)
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile transmit <NOT Successful>\n\n");
                                    send_prof = false;
                                }
                                else
                                {
                                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile transmit <NOT Successful>\n\n");
                                    /* turn on checking satellite visibility */
                                    task_Iridium_satellite_visibility(&xSatellite);
                                    run_satellite = true;
                                    satellite_tries = 0;
                                }
                            }
                        }
                        else
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile ERROR :: getting transmit status\n");
                        }
                    }
                    vTaskDelay(xDelay1000ms);
                }
            }
            else
            {
                send_prof = false;
            }
            vTaskDelay(xDelay1000ms);
        }

        if (send_park || send_prof)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, task->not_done, continue\n");
        }
        else
        {
            /* if we are here then transmission either was successful or failed */
            run = false;
            /* check if we have reached the maximum profiling numbers */
            if (prof_number >= SYSTEM_PROFILE_NUMBER)
            {
                spsEvent = MODE_POPUP;
                ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, << %u Profiles have been reached >>\n\n", prof_number);
            }
            else
            {

#if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
                /* reset test profile */
                datalogger_read_test_profile(true);
#endif
                spsEvent = MODE_IDLE;
            }
        }
        /* check after every two seconds */
        vTaskDelay(xDelay2000ms);
    }
    i9603n_off();
    vTaskDelay(xDelay1000ms);

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, FreeRTOS HEAP SIZE = %u Bytes\n\n", size);

    ///* stack size */
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //ARTEMIS_DEBUG_PRINTF("\n\nSPS :: tx, STACK Number = %u\n\n\n", uxHighWaterMark);
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Task->finished\n\n");
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

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
        m_park_length = pPark[m_park_number].pLength;
        sPark.profNumber = m_park_number;
        sPark.pageNumber = 0;
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, profile_number=%u, pageNumber=%u, measurement_length=%u\n", m_park_number, sPark.pageNumber, m_park_length);

        /* check the length of measurements must not exceed 312 bytes > 312*8/(12+8) = 124 */
        if (m_park_length > MEASUREMENT_MAX)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, WARNING : Park, profile_number=%u exceeding (%u) length of measurements, creating pages!\n", sPark.profNumber, MEASUREMENT_MAX);
            sPark.mLength = MEASUREMENT_MAX;
            m_park_length = pPark[m_park_number].pLength - sPark.mLength;
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

    uint16_t nrBytes = pack_measurements_irid(&park, pPark, &sPark, ptrPark);
    *readlength = sPark.mLength;
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park : profile_number=%u, pageNumber=%u, m_park_number=%u, total_bytes=%u\n", sPark.profNumber, sPark.pageNumber, m_park_number, nrBytes);

    /* TODO: for extended measurements, in the future */
    ///* add extension header and payload for the next profiles */
    //if (park_number > m_park_number)
    //{
    //    ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, park_number(%u) > m_park_number(%u)\n", park_number, m_park_number);

    //    /* check if nrBytes are less than (HEADER_extension plus at least 5 measurements) */
    //    uint16_t nrBytes_ext = IRID_HEADER_LENGTH_EXT + ((5*MEASUREMENT_BITS + 7)/8);
    //    ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, for 5 measurements nrBytes_ext (%u)\n", nrBytes_ext);
    //    if (nrBytes_ext <= (IRID_DATA_OUT - nrBytes))
    //    {
    //        /* set modeType to the extended measurements */
    //        sPark_ext.modeType = LCP_PARK_MODE;
    //        /* check how many bytes we have */
    //        uint16_t remaining_bytes = IRID_DATA_OUT - nrBytes;
    //        /* subtract the header_ext length from remaining_bytes */
    //        remaining_bytes -= IRID_HEADER_LENGTH_EXT;

    //        /* calculate the length of measurement for remaining bytes */
    //        uint8_t remaining_length = remaining_bytes * 8 / MEASUREMENT_BITS ;
    //        ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, remaining_bytes(%u), remaining_length(%u)\n", remaining_bytes, remaining_length);

    //        /* increase the measurement park_number_extension */
    //        m_park_number_ext++;
    //        /* proceed, check the length of next profiles */
    //        uint8_t mlength  = pPark[m_park_number+m_park_number_ext].pLength;

    //        if (mlength == 0)
    //        {
    //            /* decrease it here and break */
    //            ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, BREAK, profile(%u) mlength(%u), measurements are not available\n", m_park_number_ext, mlength);
    //            m_park_number_ext--;
    //            m_park_length_ext = 0;
    //            break;
    //        }
    //        else
    //        {
    //            if (mlength > remaining_length)
    //            {
    //                ARTEMIS_DEBUG_PRINTF("DATA :: WARNING : Park : Extended, ProfNumber(%u), creating pages !\n", m_park_number+m_park_number_ext);
    //                /* create extended measurements */
    //                sPark_ext.mLength = remaining_length;
    //                m_park_length_ext = mlength - sPark_ext.mLength;
    //                sPark_ext.profNumber = m_park_number + m_park_number_ext;
    //                sPark_ext.pageNumber = 0;
    //                ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, profNumber(%u), pageNumber(%u), mlength(%u)\n", m_park_number_ext, sPark_ext.pageNumber, m_park_length_ext);
    //
    //                uint8_t ptrPark_ext [remaining_bytes];
    //                nrBytes_ext = pack_measurements_irid_ext(&park, pPark, &sPark_ext, ptrPark_ext);

    //                /* update the measurement pointer ptrPark */
    //                for (uint16_t i=0; i<nrBytes_ext; i++)
    //                {
    //                    ptrPark[nrBytes+i] = ptrPark_ext[i];
    //                }

    //                ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, nrBytes_ext(%u)\n", nrBytes_ext);
    //                /* update the actual number of bytes here (nrBytes) */
    //                nrBytes += nrBytes_ext;
    //                /* update the readlength */
    //                *readlength += sPark_ext.mLength;
    //                ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, Total nrBytes(%u), readlength(%u)\n", nrBytes, *readlength);
    //                /* increase the page for next sbd message for the same profile number */
    //                sPark_ext.pageNumber++;
    //            }
    //            else
    //            {
    //                sPark_ext.profNumber = m_park_number + m_park_number_ext;
    //                sPark_ext.pageNumber = 0;
    //                sPark_ext.mLength = mlength;
    //                m_park_length_ext = 0;
    //                ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, profNumber(%u), pageNumber(%u), mlength(%u)\n", m_park_number_ext, sPark_ext.pageNumber, m_park_length_ext);

    //                uint8_t ptrPark_ext [remaining_bytes];
    //                nrBytes_ext = pack_measurements_irid_ext(&park, pPark, &sPark_ext, ptrPark_ext);

    //                /* update the measurement pointer ptrPark */
    //                for (uint16_t i=0; i<nrBytes_ext; i++)
    //                {
    //                    ptrPark[nrBytes+i] = ptrPark_ext[i];
    //                }

    //                /* update the actual number of bytes here (nrBytes) */
    //                ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, nrBytes_ext(%u)\n", nrBytes_ext);
    //                nrBytes += nrBytes_ext;
    //                /* update the readlength */
    //                *readlength += sPark_ext.mLength;
    //                ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, Total nrBytes(%u), readlength(%u)\n", nrBytes, *readlength);
    //            }
    //        }
    //    }
    //    else
    //    {
    //        ARTEMIS_DEBUG_PRINTF("DATA :: Park : Extended, BREAK, IRID_DATA_OUT (%u), nrBytes(%u), nrBytes_ext(%u)\n", IRID_DATA_OUT, nrBytes, nrBytes_ext);
    //        break;
    //    }
    //}

    return nrBytes;
}

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
        m_prof_length = pProf[m_prof_number].pLength;
        sProf.profNumber = m_prof_number;
        sProf.pageNumber = 0;
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, profile_number=%u, pageNumber=%u, measurement_length=%u\n", m_prof_number, sProf.pageNumber, m_prof_length);

        /* check the length of measurements must not exceed 312 bytes > 312*8/(12+8) = 124 */
        if (m_prof_length > MEASUREMENT_MAX)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, WARNING : Profile, profile_number=%u exceeding (%u) length of measurements, creating pages!\n", sProf.profNumber, MEASUREMENT_MAX);
            sProf.mLength = MEASUREMENT_MAX;
            m_prof_length = pProf[m_prof_number].pLength - sProf.mLength;
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

    uint16_t nrBytes = pack_measurements_irid(&prof, pProf, &sProf, ptrProf);
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
