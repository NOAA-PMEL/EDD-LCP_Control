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
#include <string.h>
#include <math.h>

#include "config.h"
#include "sensors.h"
#include "artemis_debug.h"
#include "artemis_rtc.h"
#include "piston.h"
#include "control.h"
#include "i9603n.h"
#include "data.h"

/* add datalogger */
#include "datalogger.h"

//#define TEST

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************

static uint32_t park_time[DATA_PARK_SAMPLES_MAX];
static float park_temp[DATA_PARK_SAMPLES_MAX];
static float park_depth[DATA_PARK_SAMPLES_MAX];
static uint32_t prof_time[DATA_PROFILE_SAMPLES_MAX];
static float prof_temp[DATA_PROFILE_SAMPLES_MAX];
static float prof_depth[DATA_PROFILE_SAMPLES_MAX];

static Data_t park;     /**< Park mode Data */
static Data_t prof;     /**< Profile mode data */

static System_t system = {
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
// FreeRTOS Functions, global variable and stuff
//
//*****************************************************************************

typedef enum {
    MODE_PRE_DEPLOY,
    MODE_PROFILE,
    MODE_POPUP,
    MODE_IDLE,
    MODE_DONE
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

static uint8_t irid_prof[IRID_DATA_OUT];    /* max 340 bytes */
static uint8_t irid_park[IRID_DATA_OUT];    /* max 340 bytes */

static uint8_t temp_park[312];    /* needed for creating a page */
static uint8_t temp_prof[312];    /* needed for creating a page */

/* linearize profile and park measurements for iridium */
static uint16_t create_park_page(uint8_t *pPark, uint16_t *readlength);
static uint16_t create_profile_page(uint8_t *pProf, uint16_t *readlength);

/* global variables */
static bool sensors_check = false;
static uint16_t prof_number = 0;
static uint16_t park_number = 0;
static uint16_t parkPage = 0;
static uint16_t profPage = 0;
static float Lat = 0.0;
static float Lon = 0.0;
static bool iridium_init = false;
static float park_piston_length = 0.0;
static bool park_length_update = false;
static float prof_piston_length = 0.0;
static float prof_length_update = false;
static bool crush_depth = false;

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
            DATA_setbuffer(&park, park_time, park_depth, park_temp, DATA_PARK_SAMPLES_MAX);
            DATA_setbuffer(&prof, prof_time, prof_depth, prof_temp, DATA_PROFILE_SAMPLES_MAX);
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

    /* wait for its global event */
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("PUS :: Popup global event wait...\n");
        ReceiveEvent(gEventQueue, &gEvent);
        ARTEMIS_DEBUG_PRINTF("PUS :: Popup global event received\n");
        if (gEvent == MODE_POPUP)
        {
            break;
        }
    }

    /* create a local task event queue */
    pusEventQueue = xQueueCreate(2, sizeof(Event_e));
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
            ARTEMIS_DEBUG_PRINTF("PUS :: Popup Surface done, going to Idle ...\n");
            system.popup.state = PUS_Idle;
            vTaskDelay(pdMS_TO_TICKS(5000UL));
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

    bool run = true;
    while (run)
    {
        /* go into deep sleep */
        ARTEMIS_DEBUG_PRINTF("\n\nPUS :: Idle, going to deep sleep\n");
        ARTEMIS_DEBUG_PRINTF("PUS :: Idle, << DONE HERE >>\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000UL));

        /* turn off datalogger */
        datalogger_power_off();

        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        vTaskDelay(portMAX_DELAY);
        run = false;
    }
    vTaskDelete(NULL);
}

void module_pus_surface_float(void)
{
    uint32_t piston_period = pdMS_TO_TICKS(1000UL)/1.0;
    uint32_t piston_timer = 0;
    bool piston_move = true;

    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_set_length(5.0);

    vTaskDelay(piston_period);
    //PIS_task_move_full(&xPiston);
    PIS_task_move_length(&xPiston);
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

            piston_timer += piston_period;
            if (piston_timer >= 300000)
            {
                ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston time-out, task->finished\n");
                vTaskDelete ( xPiston );
                vTaskDelay(piston_period);
                PIS_Reset();
                vTaskDelay(piston_period);
                piston_move = false;
                piston_timer = 0;
                pusEvent = MODE_DONE;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->suspended, who did this?\n");
            vTaskDelete ( xPiston );
            piston_move = false;
            piston_timer = 0;
            pusEvent = MODE_DONE;
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
            pusEvent = MODE_DONE;
        }

        vTaskDelay(piston_period);
    }

    SendEvent(pusEventQueue, &pusEvent);
    vTaskDelete(NULL);
}

void STATE_Predeploy(void)
{
    pdsEventQueue = xQueueCreate(2, sizeof(Event_e));
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
                                        "predeploy_systemcheck", 4096, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(pdsEventQueue, &pdsEvent);
                break;
            default:
                break;
        }

        if (pdsEvent == MODE_PRE_DEPLOY)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Transitionng to Idle State...\n");
            system.predeploy.state = PDS_Idle;
            vTaskDelay(pdMS_TO_TICKS(3000UL));
        }
        else if (pdsEvent == MODE_PROFILE)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Switching to Profile Mode (SPS)...\n");
            vTaskDelay(pdMS_TO_TICKS(3000UL));

            gEvent = pdsEvent;
            SendEvent(gEventQueue, &gEvent);
            vQueueDelete( pdsEventQueue );
            vTaskDelete(NULL);
            //vTaskDelay(portMAX_DELAY);
        }
    }
}

void module_pds_idle(void)
{
    /** Extend the piston fully out */
    ARTEMIS_DEBUG_PRINTF("PDS :: Idle, piston to move to certain density\n");

    float Density = PARK_DENSITY;
    float Volume = CTRL_set_lcp_density(Density);

    uint32_t piston_period = pdMS_TO_TICKS(1000UL)/1.0;
    uint32_t piston_timer = 0;
    bool piston_move = true;
    vTaskDelay(piston_period);

    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    ARTEMIS_DEBUG_PRINTF("PDS :: Idle, density=%.4f kg/m³, volume=%.4fin³\n", Density, Volume);
    PIS_set_volume(Volume);
    PIS_task_move_volume(&xPiston);

    vTaskDelay(piston_period * 2);

    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->active\n");

            piston_timer += piston_period;
            if (piston_timer >= 300000)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston time-out, task->finished\n");
                vTaskDelete ( xPiston );
                vTaskDelay(piston_period);
                PIS_Reset();
                vTaskDelay(piston_period);
                piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->suspended, who did this?\n");
            vTaskDelete ( xPiston );
            piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
        }

        vTaskDelay(piston_period);
    }

    /** Start sampling depth @ 1.0/30.0 Hz */
    float s_rate = 1.0/1.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth;
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
        ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Pressure = %f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Depth    = %0.4f, rate = %0.4f\n", Depth, Rate);
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Epoch    = %ld\n", epoch);

        if ( Depth > BALLAST_DEPTH )
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Pressure reached = %f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Depth reached    = %0.4f\n", Depth);
            //ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Start Dive to Park @ 180m, increment piston to Park Actuator position\n");
            run = false;
            vTaskDelete(xDepth);
            SENS_sensor_depth_off();
            pdsEvent = MODE_PROFILE;
        }
        vTaskDelayUntil(&xLastWakeTime, period);
    }

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
        uint8_t s_rate = 1.0;
        uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
        SENS_set_gps_rate(s_rate);
        TaskHandle_t xGps;

        /* Start GPS task */
        SENS_task_gps(&xGps);

        SensorGps_t gps;
        eTaskState eStatus;
        bool run = true;
        uint8_t fix = 0;

        vTaskDelay(pdMS_TO_TICKS(100UL));

        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();

        while (run)
        {

            eStatus = eTaskGetState( xGps );

            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                SENS_get_gps(&gps);

                if (gps.fix == true)
                {
                    //ARTEMIS_DEBUG_PRINTF("GPS : TimeStampe, %u.%u.%u, %u:%u:%u\n", gps.month, gps.day, gps.year, gps.hour, gps.min, gps.sec);
                    ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.7f\n", gps.latitude, gps.longitude, gps.altitude);
                    fix++;

                    if (fix > 9)
                    {
                        /* update latitude and longitude globally */
                        Lat = gps.latitude;
                        Lon = gps.longitude;

                        /* Calibrate the GPS UTC time into RTC */
                        ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, RTC : <GPS Time Set>\n");
                        artemis_rtc_gps_calibration(&gps);
                        am_hal_gpio_output_clear(AM_BSP_GPIO_LED_GREEN);
                        fix = 0;

                        //rtc_time time;
                        //artemis_rtc_get_time(&time);
                        //ARTEMIS_DEBUG_PRINTF("RTC : TimeStampe, %u.%u.%u, %u:%u:%u\n", time.month, time.day, time.year, time.hour, time.min, time.sec);
                    }
                }
                else
                {
                    ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task->active : No fix\n");
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task->suspended, who did this ?\n");
            }
            else if (eStatus==eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task->finished\n");
                run = false;
                vTaskDelay(pdMS_TO_TICKS(1000UL));
                SENS_sensor_gps_off();

                /* store data in the SDcard */
                datalogger_predeploy_mode(&gps, true);

                //vTaskDelay(portMAX_DELAY);
                pdsEvent = MODE_PRE_DEPLOY;
            }

            vTaskDelayUntil(&xLastWakeTime, period);
        }
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, ->>> ERROR !!\n");
        /* Visualize it via RED LED*/
        am_hal_gpio_output_clear(AM_BSP_GPIO_LED_RED);
        vTaskDelay(portMAX_DELAY);
    }

    /* task done, move to Idle state of the PreDeploy_mode */
    SendEvent(pdsEventQueue, &pdsEvent);
    vTaskDelete(NULL);
}

void STATE_Profiler(void)
{

#ifdef TEST
    /* do nothing */
#else

    /* wait for its global event */
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: Profile global event wait...\n");
        ReceiveEvent(gEventQueue, &gEvent);
        ARTEMIS_DEBUG_PRINTF("SPS :: Profile global event received\n");
        if (gEvent == MODE_PROFILE)
        {
            ARTEMIS_DEBUG_PRINTF("\n\nStarting Profile %u\n\n", prof_number+1);
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
                                        "sps_idle", 1024, NULL,
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
                                        "move_to_profile", 1024, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_Sample_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_profile,
                                        "profile", 2048, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_Surface_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_move_to_surface,
                                        "move_to_surface", 1024, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_TX_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_tx,
                                        "task_iridium", 8192, NULL,
                                        tskIDLE_PRIORITY + 3UL,
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
            ARTEMIS_DEBUG_PRINTF("SPS :: Transitionng to next state...\n");
            system.profiler.state++;
            vTaskDelay(pdMS_TO_TICKS(3000UL));
        }
        else if (spsEvent == MODE_IDLE)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: Profiling done, going to Idle...\n");
            ARTEMIS_DEBUG_PRINTF("\n\nStarting Profile %u\n\n", prof_number+1);
            system.profiler.state = SPS_Idle;
            vTaskDelay(pdMS_TO_TICKS(3000UL));
        }
        else if (spsEvent == MODE_POPUP)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: Switching to Popup Mode...\n");
            vTaskDelay(pdMS_TO_TICKS(3000UL));

            gEvent = spsEvent;
            SendEvent(gEventQueue, &gEvent);
            vQueueDelete( spsEventQueue );
            vTaskDelete(NULL);
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
    uint32_t wait_time = 1 * 1000;
    TickType_t xDelay = wait_time / portTICK_PERIOD_MS;

    Event_e spsEvent;
    bool run = true;
    while (run)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: Idle, 1 sec wait...\n");
        vTaskDelay(xDelay);
        spsEvent = MODE_DONE;
        run = false;
    }

    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: Idle, Task->finished\n");
    vTaskDelete(NULL);
}

void module_sps_move_to_park(void)
{
    float Volume = 0.0;
    float Length = 0.0;
    float Density = 0.0;
    float length_update = 0.0;

#ifdef TEST
    /* do nothing */
#else

    /** Set park_piston_length */
    /* for now , pressure and temperature variables are zeros for compressibility and thermal expansion */
    if (park_piston_length == 0.0)
    {
        CTRL_set_lcp_density(PARK_DENSITY);
        park_piston_length = CTRL_calculate_piston_position(0.0, 0.0);
        length_update = park_piston_length;
    }
    else
    {
        /* assigned to local_length, assumed that length was set previously */
        length_update = park_piston_length;
    }

    Volume = CTRL_calculate_volume_from_length(park_piston_length);
    Density = CTRL_calculate_lcp_density(Volume);
    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_park, Set Length, Volume, and Density\n");
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.4f kg/m³, Volume=%.4fin³, Length=%.4f\n\n\n", Density, Volume, length_update);

    uint32_t piston_period = pdMS_TO_TICKS(1000UL)/1.0;
    uint32_t piston_timer = 0;
    bool piston_move = true;

    eTaskState eStatus;
    TaskHandle_t xPiston;
    PIS_set_piston_rate(1);
    PIS_set_length(length_update);
    vTaskDelay(piston_period);
    PIS_task_move_length(&xPiston);
    vTaskDelay(piston_period * 2);

    /* check on piston movement */
    /* piston_time out is 10 mins */
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );

        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->active\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);

            piston_timer += piston_period;
            if (piston_timer >= 300000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston time-out, task->finished\n");
                vTaskDelete ( xPiston );
                vTaskDelay(piston_period);
                PIS_Reset();
                vTaskDelay(piston_period);
                piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended, who did this?\n");
            vTaskDelete ( xPiston );
            piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->finished\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);
            piston_move = false;
            piston_timer = 0;
        }

        vTaskDelay(piston_period);
    }

#endif

#ifdef TEST
    /** Start sampling depth @ 9Hz */
    float s_rate = 9.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    bool run = false;
    Event_e spsEvent;
    spsEvent = MODE_DONE;
#else
    /** Start sampling depth @ 2Hz */
    float s_rate = 2.0;
    Event_e spsEvent;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth;
    SENS_task_sample_depth_continuous(&xDepth);
    bool run = true;
#endif

    /**  Monitor depth until we get there */
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    vTaskDelay(period);

    while (run)
    {
        /* check on depth sensor */
        SENS_get_depth(&Depth, &Pressure, &Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure  = %.4f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth     = %.4f, rate = %.4f\n", Depth, Rate);

#ifdef TEST
    /* do nothing */
#else
        /* check if rate is positive, negative or stable */
        if (Rate > 0.0)
        {
            /* do nothing for now, */
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure Rate is positive\n");
        }
        else if (Rate < 0.0 && !piston_move)
        {
            /* decrease piston position by 0.05 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure Rate is negative, decrease 0.05 in\n");
            length_update -= 0.05;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
        }
        else if (Rate == 0.0 && !piston_move)
        {
            /* decrease piston position by 0.015 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure Rate is stable, decrease 0.015 in\n");
            length_update -= 0.015;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
        }

        /* check on depth to reach */
        if (Depth >= PARK_DEPTH-PARK_DEPTH_ERR && Depth <= PARK_DEPTH+PARK_DEPTH_ERR && !piston_move)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure Reached = %0.4f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Reach PARK Depth\n");

            /* stop here, and delete the task and turn off pressure sensor, move to next state */
            run = false;
            vTaskDelete(xDepth);
            SENS_sensor_depth_off();
            spsEvent = MODE_DONE;
            vTaskDelay(pdMS_TO_TICKS(500UL));
        }

        /* check on piston movement */
        if (piston_move)
        {
            eStatus = eTaskGetState( xPiston );

            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->active\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);

                piston_timer += period;
                if (piston_timer >= 300000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston time-out, task->finished\n");
                    vTaskDelete ( xPiston );
                    vTaskDelay(period);
                    PIS_Reset();
                    vTaskDelay(period);
                    piston_move = false;
                    piston_timer = 0;
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->suspended, who did this?\n");
                vTaskDelete( xPiston );
                piston_move = false;
                piston_timer = 0;
                vTaskDelete ( xPiston );
            }
            else if (eStatus==eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task->finished\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);
                piston_move = false;

                if (crush_depth)
                {
                    /* stop here, in case of emergency blow */
                    run = false;
                    vTaskDelete(xDepth);
                    SENS_sensor_depth_off();
                    spsEvent = MODE_POPUP;
                    vTaskDelay(pdMS_TO_TICKS(500UL));
                }
            }
        }

        /* keep checking for crush depth */
        if (Depth >= CRUSH_DEPTH && !piston_move)
        {
            /* reset the piston, this will stop the movement already */
            PIS_Reset();
            vTaskDelay(pdMS_TO_TICKS(1500UL));
            PIS_task_move_full(&xPiston);
            piston_move = true;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: move_to_park, <<< CRUSH DEPTH activated >>> \n\n\n");
        }

#endif
        vTaskDelay(period);
        //vTaskDelayUntil(&xLastWakeTime, period);
    }

    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Task->finished\n");
    vTaskDelete(NULL);
}

void module_sps_park(void)
{
    /** Start 1/60Hz sampling of sensors for PARK_TIME seconds */
    /** Save data in Park Data strucutre */
    uint32_t park_time = pdMS_TO_TICKS(1000UL) * PARK_TIME;
    ARTEMIS_DEBUG_PRINTF("\nSPS :: park, < PARK_TIME=%f mins>\n\n", (float) (park_time/(1000*60)));
    uint32_t wait_time = 0;

#ifdef TEST
    /** Sample at 9Hz */
    float s_rate = 9.0;
#else
    /** Sample at 1/60th Hz */
    float s_rate = PARK_RATE;
#endif

    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);
    SENS_sensor_depth_on();
    SENS_sensor_temperature_on();
    TaskHandle_t xDepth, xTemp;
    vTaskDelay(pdMS_TO_TICKS(100UL));

#ifdef TEST
    SENS_task_sample_depth_continuous(&xDepth);
    float Temperature = -5.0;
    uint16_t read = 0;
#else
    SENS_task_park_sensors(&xDepth, &xTemp);
    float Temperature = 0.0;
#endif

    /** Monitor Depth and Temperature and store these */
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    Event_e spsEvent;
    rtc_time time;

    char *filename = datalogger_park_create_file(park_number);
    vTaskDelay(pdMS_TO_TICKS(100UL));

    /** Set piston variables */
    eTaskState eStatus;
    TaskHandle_t xPiston;
    PIS_set_piston_rate(1);
    uint32_t piston_timer = 0;
    float Volume = 0.0;
    float Density = 0.0;
    float Length = 0.0;
    bool piston_move = false;
    uint8_t stable_count = 0;
    float length_update = park_piston_length;

    /* average 10 pressure, temperature values and store */
    float samples_p[10] = {0};
    float samples_t[10] = {0};
    uint8_t samples = 0;
    bool start_time = true;

    bool run = true;
    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);
#ifndef TEST
        SENS_get_temperature(&Temperature);
#endif
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure    = %0.4f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth       = %0.4f, rate = %0.4f\n", Depth, Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Temperature = %0.4f\n", Temperature);
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Epoch       = %ld\n", epoch);

        /* store first sample with start time */
        if (start_time)
        {
            DATA_add(&park, epoch, Pressure, Temperature);
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
            DATA_add(&park, epoch, avg_p, avg_t);
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
        if (read > 149)
        {
            run = false;
            vTaskDelete(xDepth);
            vTaskDelay(pdMS_TO_TICKS(100UL));
            SENS_sensor_depth_off();
            spsEvent = MODE_DONE;
        }
        /* delete */
#else
        /* check if rate is positive, negative or stable*/
        if (Rate > 0.0 && !piston_move && !park_length_update)
        {
            /* increase piston position by 0.015 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Rate is positive, increase 0.015 in\n");
            length_update += 0.015;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            vTaskDelay(pdMS_TO_TICKS(500UL));
        }
        else if (Rate < 0.0 && !piston_move && !park_length_update)
        {
            /* decrease piston position by 0.015 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Rate is negative, decrease 0.015 in\n");
            length_update -= 0.015;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            vTaskDelay(pdMS_TO_TICKS(500UL));
        }
        else if (Rate == 0.0 && !park_length_update)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Rate is stable\n");

            /* get at least 2 stable rates */
            stable_count++;
            if (stable_count > 2)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Rate is stable, stable_count=%u\n", stable_count);
                park_length_update = true;
                park_piston_length = length_update;
                stable_count = 0;
                ARTEMIS_DEBUG_PRINTF("SPS :: park, PARK DEPTH length updated\n");
            }
        }

        /* check on piston movement */
        if (piston_move)
        {
            eStatus = eTaskGetState( xPiston );

            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->active\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);

                piston_timer += period;
                if (piston_timer >= 300000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston time-out, task->finished\n");
                    vTaskDelete ( xPiston );
                    vTaskDelay(period);
                    PIS_Reset();
                    vTaskDelay(period);
                    piston_move = false;
                    piston_timer = 0;
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->suspended, who did this?\n");
                vTaskDelete( xPiston );
                piston_move = false;
                piston_timer = 0;
            }
            else if (eStatus==eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task->finished\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);
                piston_move = false;
                piston_timer = 0;

                if (crush_depth)
                {
                    /* stop here, in case of emergency blow */
                    run = false;
                    vTaskDelete(xDepth);
                    SENS_sensor_depth_off();
                    spsEvent = MODE_POPUP;
                    vTaskDelay(pdMS_TO_TICKS(500UL));
                }
            }
        }

        /* emergency blow , extend piston to full */
        if (Depth >= CRUSH_DEPTH && !piston_move)
        {
            /* reset the piston, this will stop the movement already */
            PIS_Reset();
            vTaskDelay(pdMS_TO_TICKS(1500UL));
            PIS_task_move_full(&xPiston);
            piston_move = true;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: park, <<< CRUSH DEPTH activated >>> \n\n\n");
        }

        /* check on Maximum park depth = ? */
        if (Depth >= PARK_DEPTH_MAX && !piston_move)
        {
            park_length_update = true;
            run = false;
            vTaskDelete(xTemp);
            vTaskDelete(xDepth);
            vTaskDelay(pdMS_TO_TICKS(100UL));
            wait_time = 0;

            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            spsEvent = MODE_DONE;
            ARTEMIS_DEBUG_PRINTF("\n\nSPS :: park, << Reached maximum Park Depth >> \n\n");
        }
#endif
        /* park depth timer */
        wait_time += period;
        if (wait_time >= park_time && !piston_move)
        {
            ARTEMIS_DEBUG_PRINTF("\n\nSPS :: park, << Timer out %f mins>>\n\n", (float) (wait_time/(60.0 * period)));
            run = false;
            vTaskDelete(xTemp);
            vTaskDelete(xDepth);
            vTaskDelay(pdMS_TO_TICKS(100UL));

            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            spsEvent = MODE_DONE;
        }

        /* task delay time */
        vTaskDelay(period);
    }

    park_number++;
    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: park, Task->finished\n");
    vTaskDelete(NULL);
}

void module_sps_move_to_profile(void)
{
    float Volume = 0.0;
    float Length = 0.0;
    float Density = 0.0;
    float length_update = 0.0;
    uint32_t piston_period = pdMS_TO_TICKS(1000UL);
    uint32_t piston_timer = 0;
    bool piston_move = true;


#ifdef TEST
    /* do nothing */
#else

    /** Set prof_piston_length */
    /* for now , pressure and temperature variables are zeros for compressibility and thermal expansion */
    if (prof_piston_length == 0.0)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, << Setting Profile Piston Length the first time >>\n");
        CTRL_set_lcp_density(TO_PROFILE_DENSITY);
        prof_piston_length = CTRL_calculate_piston_position(0.0, 0.0);
        length_update = prof_piston_length;
    }
    else
    {
        /* assigned to local_length, assumed that length was set previously */
        length_update = prof_piston_length;
    }

    Volume = CTRL_calculate_volume_from_length(prof_piston_length);
    Density = CTRL_calculate_lcp_density(Volume);
    ARTEMIS_DEBUG_PRINTF("\nSPS :: move_to_profile, Set Length, Volume, and Density\n");
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.4f kg/m³, Volume=%.4fin³, Length=%.4f\n\n\n", Density, Volume, prof_piston_length);

    /** move to length */
    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_set_length(length_update);

    vTaskDelay(piston_period);
    PIS_task_move_length(&xPiston);
    vTaskDelay(piston_period * 2);

    /* check on piston movement */
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );

        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->active\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);

            piston_timer += piston_period;
            if (piston_timer >= 300000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out, task->finished\n");
                vTaskDelete ( xPiston );
                vTaskDelay(piston_period);
                PIS_Reset();
                vTaskDelay(piston_period);
                piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->suspended, who did this?\n");
            vTaskDelete( xPiston);
            piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->finished\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);
            piston_move = false;
            piston_timer = 0;
        }

        vTaskDelay(piston_period);
    }


    /** Sample at 2 Hz */
    float s_rate = 2.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    SENS_set_depth_rate(s_rate);

    SENS_sensor_depth_on();
    TaskHandle_t xDepth;
    SENS_task_sample_depth_continuous(&xDepth);

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
    vTaskDelay(period);

    bool run = true;
    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure = %f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth    = %0.4f, rate = %0.4f\n", Depth, Rate);

        /* check if rate is positive, negative or stable */
        if (Rate > 0.0)
        {
            /* do nothing for now, */
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure Rate is positive\n");
        }
        else if (Rate < 0.0 && !piston_move)
        {
            /* decrease piston position by 0.05 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure Rate is negative, decrease 0.05 in\n");
            length_update -= 0.05;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
        }
        else if (Rate == 0.0 && !piston_move)
        {
            /* decrease piston position by 0.015 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure Rate is stable, decrease 0.015 in\n");
            length_update -= 0.015;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
        }

        /* check on depth to reach */
        //if (Depth >= PROFILE_DEPTH-PROFILE_DEPTH_ERR && Depth <= PROFILE_DEPTH+PROFILE_DEPTH_ERR && !piston_move)
        if (Depth >= PROFILE_DEPTH-PROFILE_DEPTH_ERR && Depth <= PROFILE_DEPTH+PROFILE_DEPTH_ERR)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure Reached = %0.4f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Reach Porfile Depth\n");

            /* check if piston is still moving then reset it and stop */
            if (piston_move)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, deliberately stop the Piston, and resetting ...\n");
                vTaskDelete ( xPiston );
                vTaskDelay(period);
                PIS_Reset();
                vTaskDelay(period);
                piston_move = false;
                piston_timer = 0;
            }

            /* stop here, and delete the task and turn off pressure sensor, move to next state */
            run = false;
            vTaskDelete(xDepth);
            vTaskDelay(pdMS_TO_TICKS(500UL));
            SENS_sensor_depth_off();
            spsEvent = MODE_DONE;
            vTaskDelay(pdMS_TO_TICKS(500UL));
        }

        /* check on piston movement */
        if (piston_move)
        {
            eStatus = eTaskGetState( xPiston );

            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_porfile, Piston task->active\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_porfile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);

                piston_timer += period;
                if (piston_timer >= 300000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston time-out, task->finished\n");
                    vTaskDelete ( xPiston );
                    vTaskDelay(period);
                    PIS_Reset();
                    vTaskDelay(period);
                    piston_move = false;
                    piston_timer = 0;
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_porfile, Piston task->suspended, who did this?\n");
                vTaskDelete( xPiston );
                piston_move = false;
                piston_timer = 0;
            }
            else if (eStatus==eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task->finished\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);
                piston_move = false;
                piston_timer = 0;

                if (crush_depth)
                {
                    /* stop here, in case of emergency blow */
                    run = false;
                    vTaskDelete(xDepth);
                    SENS_sensor_depth_off();
                    spsEvent = MODE_POPUP;
                    vTaskDelay(pdMS_TO_TICKS(500UL));
                }
            }
        }

        /* keep checking for crush depth */
        if (Depth >= CRUSH_DEPTH && !piston_move)
        {
            /* reset the piston, this will stop the movement already */
            PIS_Reset();
            vTaskDelay(pdMS_TO_TICKS(1500UL));
            PIS_task_move_full(&xPiston);
            piston_move = true;
            piston_timer = 0;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: move_to_profile, <<< CRUSH DEPTH activated >>> \n\n\n");
        }

        vTaskDelay(period);
    }

#endif

    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Task->finished\n");
    vTaskDelete(NULL);
}

void module_sps_profile(void)
{
    /* turn on temperature sensor first , in order to stop its sampling !!! */

#ifdef TEST
    /** Start Depth and Temperature Sensor @ 9Hz */
    float s_rate = 9.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    SENS_set_depth_rate(s_rate);
    SENS_sensor_gps_off();
    SENS_sensor_depth_on();

#else
    /** Start Depth and Temperature Sensor @ 1Hz */
    float s_rate = 1.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);
    SENS_sensor_depth_on();
    SENS_sensor_temperature_on();
    SENS_sensor_gps_off();

    vTaskDelay(period);

#endif

    float Volume = 0.0;
    float Density = 0.0;
    float Length = 0.0;
    float length_update = 0.0;

#ifdef TEST
    /* do nothing */
#else

    /** Calculate length for profiling at 0.1m/s upward */
    ARTEMIS_DEBUG_PRINTF("\n\nSPS :: profile, Setting volume for profiling\n");
    float v_rate = SYSTEM_RISE_RATE_SETPOINT;
    Volume = module_ctrl_set_buoyancy_from_rate(v_rate, false);
    Density = CTRL_calculate_lcp_density(Volume);
    length_update = CTRL_calculate_length_from_volume(Volume);
    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.4f kg/m³, Volume=%.3fin³, Length=%.4f\n\n\n", Density, Volume, length_update);

    /** Set volume or length */
    uint32_t piston_period = pdMS_TO_TICKS(1000UL);
    uint8_t profile_length_count = 0;
    bool piston_move = true;
    uint32_t piston_timer = 0;

    eTaskState eStatus;
    TaskHandle_t xPiston;
    PIS_set_piston_rate(1);
    PIS_set_length(length_update);

    vTaskDelay(piston_period);
    PIS_task_move_length(&xPiston);
    vTaskDelay(piston_period * 2);

    /* check on piston movement */
    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );

        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->active\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);

            piston_timer += piston_period;
            if (piston_timer >= 600000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");
                vTaskDelete ( xPiston );
                vTaskDelay(period);
                PIS_Reset();
                vTaskDelay(period);
                piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->suspended, who did this?\n");
            vTaskDelete ( xPiston );
            piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->finished\n");
            PIS_Get_Length(&Length);
            Volume = CTRL_calculate_volume_from_length(Length);
            Density = CTRL_calculate_lcp_density(Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);
            piston_move = false;
            piston_timer = 0;
        }

        vTaskDelay(piston_period);
    }

#endif

    vTaskDelay(pdMS_TO_TICKS(100UL));
    TaskHandle_t xDepth, xTemp;

#ifdef TEST
    SENS_task_sample_depth_continuous(&xDepth);
    float Temperature = 40.0;
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
    vTaskDelay(pdMS_TO_TICKS(500UL));

    /* average 10 pressure, temperature values and store */
    float samples_p[10] = {0};
    float samples_t[10] = {0};
    uint8_t samples = 0;
    bool start_time = true;


    bool run = true;
    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);

#ifdef TEST
        /* do nothing */
#else
        SENS_get_temperature(&Temperature);
#endif

        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure    = %f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Depth       = %0.4f, rate = %0.4f\n", Depth, Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature = %0.4f\n", Temperature);

        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Epoch       = %ld \n", epoch);

        ///* collect all values in the datalogger */
        //datalogger_profile_mode(filename, Depth, Temperature, &time);

        /* store first sample with start time */
        if (start_time == true)
        {
            DATA_add(&prof, epoch, Pressure, Temperature);
            datalogger_profile_mode(filename, Pressure, Temperature, &time);
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
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            std = std_div(samples_t, samples, &var, &avg_t);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            /* store average data locally */
            DATA_add(&prof, epoch, avg_p, avg_t);
            samples = 0;

#ifdef TEST
            read++;
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, sending measurements = %u\n", read);
#else
            datalogger_profile_mode(filename, avg_p, avg_t, &time);
#endif
        }

#ifdef TEST
        /* delete this */
        if (read > 149)
        {
            run = false;
            vTaskDelete(xDepth);
            //vTaskDelete(xTemp);
            SENS_sensor_depth_off();
            //SENS_sensor_temperature_off();
            spsEvent = MODE_DONE;
        }
        /* delete this end */
#else

        /* check if rate is positive, negative or stable*/
        if (Rate > 0.0 && !piston_move && !prof_length_update)
        {
            /* increase piston position by 0.015 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Rate is positive, increase 0.015 in\n");
            length_update += 0.015;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            prof_length_update = false;
            vTaskDelay(pdMS_TO_TICKS(500UL));
        }
        else if (Rate == 0.0 && !piston_move && !prof_length_update)
        {
            /* increase piston position by 0.015 inches */
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Rate is stable, increase 0.015 in\n");
            length_update += 0.015;
            PIS_set_length(length_update);
            PIS_task_move_length(&xPiston);
            piston_move = true;
            prof_length_update = false;
            vTaskDelay(pdMS_TO_TICKS(500UL));
        }
        else if (Rate < 0.0 && !prof_length_update)
        {
            /* pressure rate is negative , which is good maybe update the prof_piston_length ? */
            /* after 5 counts */
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Rate is negative, update the piston_length\n");
            profile_length_count++;
            if (profile_length_count > 2)
            {
                prof_length_update = true;
                prof_piston_length = length_update;
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, updated the prof_piston_length\n");
                profile_length_count = 0;
            }
        }

        /* check on piston movement */
        if (piston_move)
        {
            eStatus = eTaskGetState( xPiston );

            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: porfile, Piston task->active\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: porfile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);

                piston_timer += period;
                if (piston_timer >= 600000)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston time-out, task->finished\n");
                    vTaskDelete ( xPiston );
                    vTaskDelay(period);
                    PIS_Reset();
                    vTaskDelay(period);
                    piston_move = false;
                    piston_timer = 0;
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: porfile, Piston task->suspended, who did this?\n");
                vTaskDelete( xPiston );
                piston_move = false;
                piston_timer = 0;
            }
            else if (eStatus==eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task->finished\n");
                PIS_Get_Length(&Length);
                Volume = CTRL_calculate_volume_from_length(Length);
                Density = CTRL_calculate_lcp_density(Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Density=%.3f kg/m³, Volume=%.3fin³, Length=%.4f\n", Density, Volume, Length);
                piston_move = false;
                piston_timer = 0;

                if (crush_depth)
                {
                    /* stop here, in case of emergency blow */
                    run = false;
                    vTaskDelete(xDepth);
                    vTaskDelete(xTemp);
                    SENS_sensor_depth_off();
                    SENS_sensor_temperature_off();
                    spsEvent = MODE_POPUP;
                    vTaskDelay(pdMS_TO_TICKS(500UL));
                }
            }
        }

        /* emergency blow , extend piston to full */
        if (Depth >= CRUSH_DEPTH && !piston_move)
        {
            /* reset the piston, this will stop the movement already */
            PIS_Reset();
            vTaskDelay(pdMS_TO_TICKS(1500UL));
            PIS_task_move_full(&xPiston);
            piston_move = true;
            crush_depth = true;
            ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: profile, <<< CRUSH DEPTH activated >>> \n\n\n");
        }

        //if (Pressure <= BALLAST_DEPTH && !piston_move)
        if (Pressure <= 0.0352 && !piston_move)
        {
            run = false;
            vTaskDelete(xDepth);
            vTaskDelete(xTemp);
            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            spsEvent = MODE_DONE;
        }

#endif

        vTaskDelay(period);
    }

    prof_number++;
    SendEvent(spsEventQueue, &spsEvent);

    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Task->finished\n");
    vTaskDelete(NULL);
}

void module_sps_move_to_surface(void)
{
    /** Extend the piston fully out */
    uint32_t piston_period = pdMS_TO_TICKS(1000UL);
    uint32_t piston_timer = 0;
    bool piston_move = true;

    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_set_length(5.0);

    vTaskDelay(piston_period);
    //PIS_task_move_full(&xPiston);
    PIS_task_move_length(&xPiston);
    vTaskDelay(piston_period);

    while (piston_move)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->active, timer = %u\n", piston_timer);

            piston_timer += piston_period;
            if (piston_timer >= 300000)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston time-out, task->finished\n");
                vTaskDelete ( xPiston );
                vTaskDelay(piston_period);
                PIS_Reset();
                vTaskDelay(piston_period);
                piston_move = false;
                piston_timer = 0;
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->suspended, who did this?\n");
            vTaskDelete ( xPiston );
            piston_move = false;
            piston_timer = 0;
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task->finished\n");
            piston_move = false;
            piston_timer = 0;
        }

        vTaskDelay(piston_period);
    }

    /** Turn on the GPS */
    uint8_t s_rate = 1;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    SENS_set_gps_rate(s_rate);

#ifdef TEST
    /* do nothing */
    Event_e spsEvent;
    spsEvent = MODE_DONE;
#else

    SENS_sensor_gps_on();
    TaskHandle_t xGps;
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
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->active : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.7f\n", gps.latitude, gps.longitude, gps.altitude);
                fix++;

                if (fix > 9)
                {
                    /* update latitude and longitude globally */
                    Lat = gps.latitude;
                    Lon = gps.longitude;

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
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->suspended, who did this ?\n");
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task->finished\n");
            run = false;
            vTaskDelay(pdMS_TO_TICKS(100UL));

            /** GPS OFF */
            SENS_sensor_gps_off();
            spsEvent = MODE_DONE;
        }

        //vTaskDelay(pdMS_TO_TICKS(1000UL));
        vTaskDelayUntil(&xLastWakeTime, period);
    }

#endif

    /* wait for 5 seconds here */
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, 5 sec wait...\n");
    vTaskDelay(pdMS_TO_TICKS(5000UL));
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
        vTaskDelay(pdMS_TO_TICKS(1000UL));
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
                vTaskDelay(pdMS_TO_TICKS(1000UL));
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
        /* reset test profile */
        datalogger_read_test_profile(true);
        spsEvent = MODE_IDLE;
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium not charged, try again\n");
        SendEvent(spsEventQueue, &spsEvent);
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Task->finished abruptly, NOT transmitting today\n");
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(1000UL));

    TaskHandle_t xSatellite;
    task_Iridium_satellite_visibility(&xSatellite);
    eTaskState eStatus;

    /* run to see the satellites visibility */
    while(1)
    {
        eStatus = eTaskGetState( xSatellite );

        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite, task->active\n");
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite, task->suspended, who did this ?\n");
        }
        else if (eStatus==eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Satellite task->finished\n");
            bool visible = GET_Iridium_satellite();
            if (visible)
            {
                ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, Satellite <Visible>\n\n");
                break;
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, Satellite <NOT Visible>\n\n");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000UL));
    }

    /* move to transmitting bytes */

    TaskHandle_t xIridium;
    bool run = true;
    bool send_park = true;
    bool send_prof = true;

    while ( run )
    {
        /* start off with sending Park measurements pages */
        while (send_park)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park, written=%u, read=%u\n", park.cbuf.written, park.cbuf.read);

            uint8_t *ptrPark = &irid_park[0];
            uint16_t nr_park = 0;
            uint16_t txpark = create_park_page(ptrPark, &nr_park);

            if ( txpark > 0 )
            {
                /* Debug */
                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park page %u, bytes=%u\n", parkPage, txpark);

                /* we do have a page to send */
                /* send the bytes to the originated buffer */
                bool ret = i9603n_send_data(irid_park, txpark);
                if (ret)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park measurements are being transmitted\n");
                }
                vTaskDelay(pdMS_TO_TICKS(1000UL));

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
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park task->suspended, who did this ?\n");
                    }
                    else if (eStatus==eDeleted)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park task->finished\n");
                        vTaskDelay(pdMS_TO_TICKS(1000UL));

                        /* check if the page are transmitted successfully, tranmission number */
                        uint8_t recv[6] = {0};
                        bool ret = GET_Iridium_status (recv);

                        if (ret)
                        {
                            if (recv[0] <= 5)
                            {
                                park_run = false;
                                ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, Park transmit <Successful>\n\n");
                            }
                            else
                            {
                                /* reset the read length */
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park read=%u, measurements=%u\n", park.cbuf.read, nr_park);
                                parkPage--;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park resetting to page %u\n", parkPage);
                                park.cbuf.read = park.cbuf.read - nr_park;
                                park_run = false;
                                send_park = false;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park after reset read=%u\n", park.cbuf.read);
                                ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, Park transmit <NOT Successful>\n");
                                vTaskDelay(pdMS_TO_TICKS(1000UL));
                            }
                        }
                        else
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park ERROR :: getting transmit status\n");
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000UL));
                }
            }
            else
            {
                send_park = false;
            }
            vTaskDelay(pdMS_TO_TICKS(2000UL));
        }

        /* start off with sending Profile measurements pages */
        while (send_prof)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile, written=%u, read=%u\n", prof.cbuf.written, prof.cbuf.read);

            uint8_t *ptrProf = &irid_prof[0];
            uint16_t nr_prof = 0;
            uint16_t txprof = create_profile_page(ptrProf, &nr_prof);

            if ( txprof > 0 )
            {
                /* Debug */
                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile page %u, bytes=%u\n", profPage, txprof);

                /* we do have a page to send */
                /* send the bytes to the originated buffer */
                bool ret = i9603n_send_data(irid_prof, txprof);
                if (ret)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile measurements are being transmitted\n");
                }
                vTaskDelay(pdMS_TO_TICKS(1000UL));

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
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile task->suspended, who did this ?\n");
                    }
                    else if (eStatus==eDeleted)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile task->finished\n");
                        vTaskDelay(pdMS_TO_TICKS(1000UL));

                        /* check if the page are transmitted successfully, tranmission number */
                        uint8_t recv[6] = {0};
                        bool ret = GET_Iridium_status (recv);

                        if (ret)
                        {
                            if (recv[0] <= 5)
                            {
                                prof_run = false;
                                ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, Profile transmit <Successful>\n\n");
                            }
                            else
                            {
                                /* reset the read length */
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile measurements=%u, read=%u\n", nr_prof, prof.cbuf.read);
                                profPage--;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile resetting to page %u\n", profPage);
                                prof.cbuf.read = prof.cbuf.read - nr_prof;
                                prof_run = false;
                                send_prof = false;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile after reset read=%u\n", prof.cbuf.read);
                                ARTEMIS_DEBUG_PRINTF("\nSPS :: tx, Profile transmit <NOT Successful>\n");
                                vTaskDelay(pdMS_TO_TICKS(1000UL));
                            }
                        }
                        else
                        {
                            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile ERROR :: getting transmit status\n");
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000UL));
                }
            }
            else
            {
                send_prof = false;
            }
            vTaskDelay(pdMS_TO_TICKS(2000UL));
        }

        if (send_park || send_prof)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, task->not_done, continue...\n");
        }
        else
        {
            /* if we are here then transmission either was successful or failed */
            run = false;

            //vTaskDelay(pdMS_TO_TICKS(1000UL));
            /* check if we have reached the maximum profiling numbers */
            if (prof_number >= SYSTEM_PROFILE_NUMBER)
            {
                spsEvent = MODE_POPUP;
                ARTEMIS_DEBUG_PRINTF("\n\n\nSPS :: tx, << %i Profiles has been reached >>\n\n\n", prof_number);
            }
            else
            {
                /* reset test profile */
                datalogger_read_test_profile(true);
                spsEvent = MODE_IDLE;
            }
        }
        /* check after every two seconds */
        vTaskDelay(pdMS_TO_TICKS(2000UL));
    }

    i9603n_off();
    vTaskDelay(pdMS_TO_TICKS(1000UL));

    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Task->finished\n");
    vTaskDelete(NULL);
}

static uint16_t create_park_page(uint8_t *pPark, uint16_t *readlength)
{
    /** Collect Park measurements
     *  maximum 340 bytes can be transmit at a time,
     *  27 bytes are reserved for the header for every message
     *  340 - 27 = 313 bytes are left for the measurements, actually 312
     *  312 / 3 = 104 measurements can fit into one iridium message
     *  one bytes for pressure sensor, two bytes are for temperature
     */
    uint8_t size = 0;
    uint8_t pressure = 0;
    int16_t temp = 0;
    uint32_t start = 0;
    uint32_t offset = 0;
    uint16_t len = 0;
    uint16_t nrBytes = 0;

    /* set pointer to the temp park buffer */
    uint8_t *ptPark = &temp_park[0];
    do
    {
        size = DATA_get_converted(&park, &start, &offset, &pressure, &temp);
        /* check, if we have no data available */
        if (size == 0)
        {
            if (len > 0 && len <= 103)
            {
                parkPage++;
                ARTEMIS_DEBUG_PRINTF("collected measurements=%u\n", len);
                ARTEMIS_DEBUG_PRINTF("\n<< no more measurements after page %u >>\n\n", parkPage);
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("all measurements are read\n");
            }
            break;
        }
        len++;

        /* check on the length, must be < 313 */
        if (len > 103)
        {
            /* we are done here, increase the page number */

            *ptPark++ = (uint8_t) (pressure&0xFF);
            *ptPark++ = (uint8_t) (temp>>8);
            *ptPark++ = (uint8_t) (temp&0xFF);
            parkPage++;

            ARTEMIS_DEBUG_PRINTF("collected measurements=%u\n", len);
            ARTEMIS_DEBUG_PRINTF("\n< more measurements are available >\n\n");

            break;
        }

        /* start collecting data into the temp park buffer */
        *ptPark++ = (uint8_t) (pressure&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Pressure    = 0x%02X\n", *(ptPark-1) );
        *ptPark++ = (uint8_t) (temp>>8);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptPark-1) );
        *ptPark++ = (uint8_t) (temp&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptPark-1) );


    } while (1);

    /* check if we have any collected bytes */
    if (len == 0)
    {
        nrBytes = len;
    }
    else
    {
        /* create a header for the park measurements */
        create_header(pPark, start, offset, Lat, Lon, LCP_PARK_MODE, parkPage);
        /* set the start measurements position 28, 27th in the array */
        pPark = &irid_park[27];
        nrBytes = 27;
        /* collect measurements from temp buffer to irid_park buffer */
        /* set first position of temp_park to its pointer */
        ptPark = &temp_park[0];

        for (uint16_t i=0; i<len*3; i++)
        {
            *pPark++ = *ptPark++;
            nrBytes++;
            //ARTEMIS_DEBUG_PRINTF("0x%02X, 0x%02X, nrBytes = %u\n", *(pPark-1), *(ptPark-1), nrBytes);
        }
    }
    *readlength = len;
    return nrBytes;
}

static uint16_t create_profile_page(uint8_t *pProf, uint16_t *readlength)
{
    /** Collect Profile measurements
     *  maximum 340 bytes can be transmit at a time,
     *  27 bytes are reserved for the header for every message
     *  340 - 27 = 313 bytes are left for the measurements, actually 312
     *  312 / 3 = 104 measurements can fit into one iridium message
     *  one bytes for pressure sensor, two bytes are for temperature
     */
    uint8_t size = 0;
    uint8_t pressure = 0;
    int16_t temp = 0;
    uint32_t start = 0;
    uint32_t offset = 0;
    uint16_t len = 0;
    uint16_t nrBytes = 0;

    /* set pointer to the temp park buffer */
    uint8_t *ptProf = &temp_prof[0];
    do
    {
        size = DATA_get_converted(&prof, &start, &offset, &pressure, &temp);
        /* check, if we have no data available */
        if (size == 0)
        {
            if (len > 0 && len <= 103)
            {
                profPage++;
                ARTEMIS_DEBUG_PRINTF("collected measurements=%u\n", len);
                ARTEMIS_DEBUG_PRINTF("\n<< no more measurements after page %u >>\n\n", profPage);
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("all measurements are read\n");
            }
            break;
        }
        len++;

        /* check on the length, must be < 313 */
        if (len > 103)
        {
            /* we are done here, increase the page number */
            *ptProf++ = (uint8_t) (pressure&0xFF);
            *ptProf++ = (uint8_t) (temp>>8);
            *ptProf++ = (uint8_t) (temp&0xFF);
            profPage++;

            ARTEMIS_DEBUG_PRINTF("collected measurements=%u\n", len);
            ARTEMIS_DEBUG_PRINTF("\n< more measurements are available >\n\n");

            break;
        }

        /* start collecting data into the temp park buffer */
        *ptProf++ = (uint8_t) (pressure&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Pressure    = 0x%02X\n", *(ptProf-1) );
        *ptProf++ = (uint8_t) (temp>>8);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptProf-1) );
        *ptProf++ = (uint8_t) (temp&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptProf-1) );
        //ARTEMIS_DEBUG_PRINTF("collected measurements=%u\n", len);

    } while (1);

    /* check if we have any collected bytes */
    if (len == 0)
    {
        nrBytes = len;
    }
    else
    {
        /* create a header for the profile measurements */
        create_header(pProf, start, offset, Lat, Lon, LCP_PROFILE_MODE, profPage);
        /* set the start measurements position 28, 27th in the array */
        pProf = &irid_prof[27];
        nrBytes = 27;
        /* collect measurements from temp buffer to irid_prof buffer */
        /* set first position of temp_prof to its pointer */
        ptProf = &temp_prof[0];

        for (uint16_t i=0; i<len*3; i++)
        {
            *pProf++ = *ptProf++;
            nrBytes++;
            //ARTEMIS_DEBUG_PRINTF("0x%02X, 0x%02X, nrBytes = %u\n", *(pProf-1), *(ptProf-1), nrBytes);
        }
    }

    *readlength = len;
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
    ARTEMIS_DEBUG_PRINTF("Event waiting...\n");
    xQueueReceive(eventQueue, event, portMAX_DELAY);
    ARTEMIS_DEBUG_PRINTF("Event waiting Done\n");
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

