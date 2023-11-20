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
    //.profiler.state = SPS_TX_mode,
    .profiler.state = SPS_Idle,
    //.profiler.state = SPS_Park_mode,
    //.profiler.state = SPS_MoveToSampleDepth_mode,
    //.profiler.state = SPS_MoveToParkDepth_mode,
    //.profiler.state = SPS_Sample_mode,
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
static uint16_t create_park_page(uint8_t *pPark);
static uint16_t create_profile_page(uint8_t *pProf);

static bool sensors_check = false;
static uint16_t prof_number = 0;
static uint16_t park_number = 0;
static uint8_t parkPage = 0;
static uint8_t profPage = 0;
static float Lat = 0;
static float Lon = 0;

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
        ARTEMIS_DEBUG_PRINTF("PUS :: Popup waiting for a global event ...\n");
        ReceiveEvent(gEventQueue, &gEvent);
        ARTEMIS_DEBUG_PRINTF("PUS :: Popup received a global event\n");
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
        ARTEMIS_DEBUG_PRINTF("PUS :: Idle, going to deep sleep\n");
        ARTEMIS_DEBUG_PRINTF("PUS :: Idle, wait forever\n");
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
    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_task_move_full(&xPiston);

    bool run = true;
    Event_e pusEvent;
    while (run)
    {
        eStatus = eTaskGetState( xPiston );
        ARTEMIS_DEBUG_PRINTF("PUS :: surface_float, Piston move status = %u\n", eStatus);

        if (eStatus == eDeleted)
        {
            run = false;
            pusEvent = MODE_DONE;
        }

        vTaskDelay(pdMS_TO_TICKS(1000UL));
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
            ARTEMIS_DEBUG_PRINTF("PDS :: Transitionng to Idle State ...\n");
            system.predeploy.state = PDS_Idle;
            vTaskDelay(pdMS_TO_TICKS(3000UL));
        }
        else if (pdsEvent == MODE_PROFILE)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Switching to Profile Mode (SPS) ...\n");
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
    ARTEMIS_DEBUG_PRINTF("PDS :: Idle, wait here for piston to fully out ...\n");

    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_task_move_full(&xPiston);

    bool run = true;
    while (run)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston full-out task is active\n");
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston full-out task is suspended, who did this?\n");
        }
        else if (eStatus == eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Piston full-out task is deleted\n");
            run = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000UL));
    }

    /** Start sampling depth @ 1Hz */
    float s_rate = 1.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth;
    SENS_task_sample_depth_continuous(&xDepth);

    /** Monitor Depth */
    run = true;
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

        vTaskDelayUntil(&xLastWakeTime, period);
        if ( Pressure > 1.0135)
        {
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Pressure reached = %f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Depth reached    = %0.4f\n", Depth);
            ARTEMIS_DEBUG_PRINTF("PDS :: Idle, Start Dive to Park @ 180m, increment piston to Park Actuator position\n");
            run = false;
            vTaskDelete(xDepth);
            SENS_sensor_depth_off();
            pdsEvent = MODE_PROFILE;
        }
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
                        ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, RTC : Setting gps time\n");
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
                    ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS : No fix\n");
                }
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task is suspended, who did this ?\n");
            }
            else if (eStatus == eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS task is deleted\n");
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
    /* wait for its global event */
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: Profile waiting for a global event ...\n");
        ReceiveEvent(gEventQueue, &gEvent);
        ARTEMIS_DEBUG_PRINTF("SPS :: Profile received a global event\n");
        if (gEvent == MODE_PROFILE)
        {
            break;
        }
    }

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
            ARTEMIS_DEBUG_PRINTF("SPS :: Transitionng to next state ...\n");
            system.profiler.state++;
            vTaskDelay(pdMS_TO_TICKS(3000UL));
        }
        else if (spsEvent == MODE_IDLE)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: Profiling done, going to Idle ...\n");
            system.profiler.state = SPS_Idle;
            vTaskDelay(pdMS_TO_TICKS(3000UL));
        }
        else if (spsEvent == MODE_POPUP)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: Switching to Popup Mode ...\n");
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

    /* wait here for one second for now */
    uint32_t wait_time = 1 * 1000;
    TickType_t xDelay = wait_time / portTICK_PERIOD_MS;

    Event_e spsEvent;
    bool run = true;
    while (run)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: Idle, waiting for one second ...\n");
        vTaskDelay(xDelay);
        spsEvent = MODE_DONE;
        run = false;
    }

    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: Idle, Task is being deleted\n");
    vTaskDelete(NULL);
    ARTEMIS_DEBUG_PRINTF("SPS :: Idle, Task is deleted, do not get here\n");
}

void module_sps_move_to_park(void)
{
    /** Calculate volume to move to */
    float v_rate = 0.1;
    float volume = module_ctrl_set_buoyancy_from_rate(v_rate, true);

    /** Set volume */
    //PIS_set_volume(volume);
    PIS_set_volume(730); // dummy volume for now 650

    eTaskState eStatus;
    TaskHandle_t xPiston;
    PIS_set_piston_rate(1);
    PIS_task_move_volume(&xPiston);

    /** Start sampling depth @ 2Hz */
    float s_rate = 2.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth;
    SENS_task_sample_depth_continuous(&xDepth);

    /**  Monitor depth until we get there */
    bool run = true;
    double Volume = 0.0;
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    uint8_t count_500ms = 0;
    bool piston_move = true;

    Event_e spsEvent;
    //vTaskDelay(period);

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (run)
    {
        /* check on depth sensor */
        SENS_get_depth(&Depth, &Pressure, &Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure  = %f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth     = %0.4f, rate = %0.4f\n", Depth, Rate);

        if (Pressure >= 19.13 && Pressure < 19.15)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Pressure Reached = %0.4f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Reach PARK Depth\n");

            /* stop here, and delete the task and turn off pressure sensor, move to next state */
            run = false;
            vTaskDelete(xDepth);
            SENS_sensor_depth_off();
            spsEvent = MODE_DONE;
        }

        /* check on piston movement */
        if (piston_move)
        {
            count_500ms++;
            if (count_500ms == 2)
            {
                eStatus = eTaskGetState( xPiston );

                if ( (eStatus==eRunning) ||
                     (eStatus==eReady)   ||
                     (eStatus==eBlocked)  )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task is active\n");
                    PIS_Get_Volume(&Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Volume = %0.4f\n", Volume);
                }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task is suspended, who did this?\n");
                }
                else if (eStatus == eDeleted)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston task is deleted\n");
                    piston_move = false;
                }
                count_500ms = 0;
            }
        }

        //vTaskDelay(period);
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Task is being deleted\n");
    vTaskDelete(NULL);
}

void module_sps_park(void)
{
    /** Start 1/60Hz sampling of sensors for XXX minutes */
    /** Save data in Park Data strucutre */

    /** Sample at 1/60th Hz */
    float s_rate = 1.0; //1.0/60.01;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);

    SENS_sensor_depth_on();
    SENS_sensor_temperature_on();
    vTaskDelay(pdMS_TO_TICKS(100UL));

    TaskHandle_t xDepth, xTemp;
    SENS_task_park_sensors(&xDepth, &xTemp);

    //SENS_task_sample_depth_continuous(&xDepth);

    /** Monitor Depth and Temperature and store these */
    bool run = true;
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    float Temperature = 0.0;
    Event_e spsEvent;
    rtc_time time;

    char *filename = datalogger_park_create_file(park_number);
    vTaskDelay(pdMS_TO_TICKS(100UL));

    /** Set volume */
    //PIS_set_volume(735); // dummy volume for now 735
    eTaskState eStatus;
    TaskHandle_t xPiston;
    PIS_set_piston_rate(1);
    double Volume = 0.0;
    bool piston_move = false;
    bool task_done = false;

    /* average 10 pressure, temperature values and store */
    float samples_p[10] = {0};
    float samples_t[10] = {0};
    uint8_t samples = 0;
    bool start_time = true;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);
        SENS_get_temperature(&Temperature);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure    = %0.4f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth       = %0.4f, rate = %0.4f\n", Depth, Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Temperature = %0.4f\n", Temperature);
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Epoch       = %ld\n", epoch);

        /* store first sample with start time */
        if (start_time)
        {
            DATA_add(&park, epoch, Depth, Temperature);
            start_time = false;
        }

        /* Average Data, and store samples */
        samples_p[samples] = Pressure;
        samples_t[samples] = Temperature;
        samples++;

        if (samples > 9)
        //if (samples > 1)
        {
            float var, avg_p, avg_t;
            float std = std_div(samples_p, samples, &var, &avg_p);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            std = std_div(samples_t, samples, &var, &avg_t);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            /* store averages data locally */
            DATA_add(&park, epoch, avg_p, avg_t);
            samples = 0;
            datalogger_park_mode(filename, avg_p, avg_t, &time);
            /* just for testing */
            //read++;
            //ARTEMIS_DEBUG_PRINTF("SPS :: park, sending measurements = %u\n", read);
        }

        ///* delete this in real test */
        //if (read == 150)
        //{
        //    run = false;
        //    //vTaskDelete(xTemp);
        //    vTaskDelete(xDepth);
        //    vTaskDelay(pdMS_TO_TICKS(100UL));

        //    SENS_sensor_depth_off();
        //    //SENS_sensor_temperature_off();
        //    spsEvent = MODE_DONE;
        //}
        ///* delete */

        ///* store in park mode file */
        //datalogger_park_mode(filename, Depth, Temperature, &time);

        /* check on piston movement */
        if (piston_move)
        {
            eStatus = eTaskGetState( xPiston );

            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task is active\n");
                PIS_Get_Volume(&Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Volume = %0.4f\n", Volume);
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task is suspended, who did this?\n");
            }
            else if (eStatus == eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: park, Piston task is deleted\n");
                piston_move = false;
            }
        }

        /* check on pressure */

        if ( (Pressure >= 19.1785 && Pressure < 19.1790) && !task_done && !piston_move)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Stay at PARK within +/-10m for 3min\n");
        }
        else if( (Pressure >= 19.1383 && Pressure < 19.1400) && task_done && !piston_move)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Reached = %f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Start to move outside of PDR limit\n");
            PIS_set_volume(735); // dummy volume for now 735
            PIS_task_move_volume(&xPiston);
            piston_move = true;
        }
        else if ( (Pressure >= 19.1785 && Pressure < 19.1790) && !task_done && piston_move)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Reached = %f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            task_done = true;
        }
        else if ((Pressure >= 19.1383 && Pressure < 19.1385) && !task_done && piston_move)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Back at PARK for 30s\n");
        }
        else if (Pressure >= 19.1634 && Pressure < 19.1640)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Pressure Reached = %f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            ARTEMIS_DEBUG_PRINTF("SPS :: park, Start to FALL to Profile Depth @ 0.25m/s, ADJUST PISTON TO FALL\n");

            run = false;
            vTaskDelete(xTemp);
            vTaskDelete(xDepth);
            vTaskDelay(pdMS_TO_TICKS(100UL));

            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            spsEvent = MODE_DONE;
        }

        /* task delay time */
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    park_number++;
    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: park, Task is being deleted\n");
    vTaskDelete(NULL);
    ARTEMIS_DEBUG_PRINTF("SPS :: park, Task is deleted, do not get here\n");
}

void module_sps_move_to_profile(void)
{
    /** Calculate volume to move to */
    float v_rate = 0.1;
    float volume = module_ctrl_set_buoyancy_from_rate(v_rate, true);

    /** Set volume */
    //PIS_set_volume(volume);
    PIS_set_volume(740); // dummy volume for now 670

    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_task_move_volume(&xPiston);

    /** Sample at 2 Hz */
    float s_rate = 2.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    SENS_set_depth_rate(s_rate);

    SENS_sensor_depth_on();
    TaskHandle_t xDepth;
    SENS_task_sample_depth_continuous(&xDepth);

    /** Monitor Depth, when depth reached, mode is complete */
    bool run = true;
    double Volume = 0;
    float Depth = 0, Rate = 0;
    float Pressure = 0;
    Event_e spsEvent;
    uint8_t count_500ms = 0;
    bool piston_move = true;

    vTaskDelay(period);

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (run)
    {

        SENS_get_depth(&Depth, &Pressure, &Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure = %f\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth    = %0.4f, rate = %0.4f\n", Depth, Rate);

        /* check on piston movement */
        if (piston_move)
        {
            count_500ms++;
            if (count_500ms == 2)
            {
                eStatus = eTaskGetState( xPiston );

                if ( (eStatus==eRunning) ||
                     (eStatus==eReady)   ||
                     (eStatus==eBlocked)  )
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task is active\n");
                    PIS_Get_Volume(&Volume);
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Volume = %0.4f \n", Volume);
                }
                else if (eStatus==eSuspended)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task is suspended, who did this?\n");
                }
                else if (eStatus == eDeleted)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston task is deleted\n");
                    piston_move = false;
                }
                count_500ms = 0;
            }
        }

        if (Pressure >= 21.1521 && Pressure < 21.1523)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Pressure Reached = %f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, AT PROFILE DEPTH, ADJUST piston for PROFILE, start collecting data\n");

            run = false;
            vTaskDelete(xDepth);
            SENS_sensor_depth_off();
            spsEvent = MODE_DONE;
        }

        //run = false;
        //vTaskDelete(xDepth);
        //SENS_sensor_depth_off();
        //spsEvent = MODE_DONE;

        vTaskDelayUntil(&xLastWakeTime, period);
    }

    SendEvent(spsEventQueue, &spsEvent);
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Task is being deleted\n");
    vTaskDelete(NULL);
    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Task is deleted, do not get here\n");
}

void module_sps_profile(void)
{
    /** Calculate volume to move to (surface) */
    float v_rate = 0.1;
    float volume = module_ctrl_set_buoyancy_from_rate(v_rate, false);

    /** Start Depth and Temperature Sensor @ 1Hz */
    float s_rate = 1.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);

    SENS_sensor_gps_off();
    SENS_sensor_depth_on();
    SENS_sensor_temperature_on();

    vTaskDelay(pdMS_TO_TICKS(100UL));

    /** Set volume */
    PIS_set_volume(745); // dummy volume for now
    eTaskState eStatus;
    TaskHandle_t xPiston;
    PIS_set_piston_rate(1);
    PIS_task_move_volume(&xPiston);
    bool piston_move = true;

    TaskHandle_t xDepth, xTemp;
    SENS_task_profile_sensors(&xDepth, &xTemp);
    //SENS_task_sample_depth_continuous(&xDepth);

    /** Start recording samples */
    float Temperature = 0.0;
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    double Volume = 0.0;
    Event_e spsEvent;
    rtc_time time;

    /** Monitor depth & rate.  If rate fails, fix it.  If depth reaches surface, done */
    char *filename = datalogger_profile_create_file(prof_number);
    vTaskDelay(pdMS_TO_TICKS(500UL));

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();


    /* average 10 pressure, temperature values and store */
    float samples_p[10] = {0};
    float samples_t[10] = {0};
    uint8_t samples = 0;
    bool start_time = true;
    //uint16_t read = 0;

    bool run = true;
    while (run)
    {
        SENS_get_depth(&Depth, &Pressure, &Rate);
        SENS_get_temperature(&Temperature);

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
        }

        /* Average Data, and store samples */
        samples_p[samples] = Pressure;
        samples_t[samples] = Temperature;
        samples++;

        if (samples > 9)
        //if (samples > 1)
        {
            float var, avg_p, avg_t;
            float std = std_div(samples_p, samples, &var, &avg_p);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Variance = %0.4f, Std_Div = %0.4f\n", var, std);
            std = std_div(samples_t, samples, &var, &avg_t);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature Variance = %0.4f, Std_Div = %0.4f\n", var, std);

            /* store average data locally */
            DATA_add(&prof, epoch, avg_p, avg_t);
            samples = 0;
            datalogger_profile_mode(filename, avg_p, avg_t, &time);
            //read++;
            //ARTEMIS_DEBUG_PRINTF("SPS :: profile, sending measurements = %u\n", read);
        }

        ///* delete this */
        //if (read == 150)
        //{
        //    run = false;
        //    vTaskDelete(xDepth);
        //    //vTaskDelete(xTemp);
        //    SENS_sensor_depth_off();
        //    //SENS_sensor_temperature_off();
        //    spsEvent = MODE_DONE;
        //}
        ///* delete this end */

        /* check on piston movement */
        if (piston_move)
        {
            eStatus = eTaskGetState( xPiston );
            if ( (eStatus==eRunning) ||
                 (eStatus==eReady)   ||
                 (eStatus==eBlocked)  )
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task is active\n");
                PIS_Get_Volume(&Volume);
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Volume  = %0.4lf\n", Volume);
            }
            else if (eStatus==eSuspended)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task is suspended, who did this?\n");
            }
            else if (eStatus == eDeleted)
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston task is deleted\n");
                piston_move = false;
            }
        }

        if (Pressure > 21.2931 && Pressure < 21.3000)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Pressure Reached = %f\n", Pressure);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Depth Reached    = %0.4f, rate = %0.4f\n", Depth, Rate);
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Profile starts rising after some overshoot\n");
        }
        else if (Pressure > 1.01 && Pressure < 1.03)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Back at the SURFACE for 2 min, START SURFACE routine\n");
            run = false;
            vTaskDelete(xDepth);
            vTaskDelete(xTemp);
            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            spsEvent = MODE_DONE;
        }

        //vTaskDelay(period);
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    prof_number++;
    SendEvent(spsEventQueue, &spsEvent);

    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Task is being deleted\n");
    vTaskDelete(NULL);
    ARTEMIS_DEBUG_PRINTF("SPS :: profile, Task is deleted, do not get here\n");
}

void module_sps_move_to_surface(void)
{
    /** Extend the piston fully out */
    TaskHandle_t xPiston;
    eTaskState eStatus;
    PIS_set_piston_rate(1);
    PIS_task_move_full(&xPiston);

    bool run = true;
    while (run)
    {
        eStatus = eTaskGetState( xPiston );
        if ( (eStatus==eRunning) ||
             (eStatus==eReady)   ||
             (eStatus==eBlocked)  )
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task is active\n");
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task is suspended, who did this?\n");
        }
        else if (eStatus == eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston task is deleted\n");
            run = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1000UL));
    }

    /** Turn on the GPS */
    uint8_t s_rate = 1;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;
    SENS_set_gps_rate(s_rate);

    SENS_sensor_gps_on();
    TaskHandle_t xGps;
    SENS_task_gps(&xGps);

    /** Monitor piston position */

    SensorGps_t gps;
    run = true;
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
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.7f\n", gps.latitude, gps.longitude, gps.altitude);
                fix++;

                if (fix > 9)
                {
                    /* update latitude and longitude globally */
                    Lat = gps.latitude;
                    Lon = gps.longitude;

                    /* Calibrate the GPS UTC time into RTC */
                    ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, RTC : Setting gps time\n");
                    artemis_rtc_gps_calibration(&gps);
                    fix = 0;
                }
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS : No fix\n");
            }
        }
        else if (eStatus==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task is suspended, who did this ?\n");
        }
        else if (eStatus == eDeleted)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS task is deleted\n");
            run = false;
            vTaskDelay(pdMS_TO_TICKS(100UL));

            /** GPS OFF */
            SENS_sensor_gps_off();
            spsEvent = MODE_DONE;
        }

        //vTaskDelay(pdMS_TO_TICKS(1000UL));
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    /* wait for 5 seconds here */
    vTaskDelay(pdMS_TO_TICKS(5000UL));

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_tx(void)
{

    /** Initialize the Iridium Modem at least */
    i9603n_initialize();
    vTaskDelay(pdMS_TO_TICKS(100UL));
    /* turn on the iridium module */
    i9603n_on();
    vTaskDelay(pdMS_TO_TICKS(1000UL));

    TaskHandle_t xIridium;
    eTaskState eStatus;
    Event_e spsEvent;
    bool run = true;
    bool send_park = true;
    bool send_prof = true;

    while ( run )
    {
        /* start off with sending Park measurements pages */
        while (send_park)
        {
            uint8_t *ptrPark = &irid_park[0];
            uint16_t txpark = create_park_page(ptrPark);

            /* Debug */
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park page %u, length = %u\n", parkPage, txpark);

            if ( txpark > 0 )
            {
                //for (uint16_t i=0; i<txpark; i++)
                //{
                //    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park measurements = 0x%02X\n", irid_park[i]);
                //}

                /* we do have a page to send */
                /* send the bytes to the originated buffer */
                bool ret = i9603n_send_data(irid_park, txpark);
                if (ret)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park Bytes sending\n");
                }
                vTaskDelay(pdMS_TO_TICKS(100UL));

                /* start iridium transfer task */
                task_Iridium_transfer(&xIridium);

                /* park run */
                bool park_run = true;
                while (park_run)
                {
                    /* Iridum task state checking */
                    eStatus = eTaskGetState( xIridium );

                    if ( (eStatus==eRunning) ||
                         (eStatus==eReady)   ||
                         (eStatus==eBlocked)  )
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park Iridium task is active\n");
                    }
                    else if (eStatus==eSuspended)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park Iridium task is suspended, who did this ?\n");
                    }
                    else if (eStatus == eDeleted)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park Iridium task is deleted\n");

                        /* check if the page are transmitted successfully, tranmission number */
                        uint8_t recv[6] = {0};
                        bool ret = GET_Iridium_status (recv);

                        if (ret)
                        {
                            if ( recv[0] <= 5)
                            {
                                park_run = false;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park transmission successful\n");
                                vTaskDelay(pdMS_TO_TICKS(100UL));
                            }
                            else
                            {
                                /* reset the read length */
                                parkPage--;
                                park.cbuf.read = park.cbuf.read - txpark;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park transmission not successful\n");
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Park reseting page\n");
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
                //parkPage = 0;
            }
        }

        /* start off with sending Profile measurements pages */
        while (send_prof)
        {
            uint8_t *ptrProf = &irid_prof[0];
            uint16_t txprof = create_profile_page(ptrProf);

            /* Debug */
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile page %u, length = %u\n", profPage, txprof);

            if ( txprof > 0 )
            {

                //for (uint16_t i=0; i<txprof; i++)
                //{
                //    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile measurements = 0x%02X\n", irid_prof[i]);
                //}


                /* we do have a page to send */
                /* send the bytes to the originated buffer */
                bool ret = i9603n_send_data(irid_prof, txprof);
                if (ret)
                {
                    ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile Bytes sending\n");
                }
                vTaskDelay(pdMS_TO_TICKS(100UL));

                /* start iridium transfer task */
                task_Iridium_transfer(&xIridium);

                /* inner run */
                bool profile_run = true;
                while (profile_run)
                {
                    /* Iridum task state checking */
                    eStatus = eTaskGetState( xIridium );

                    if ( (eStatus==eRunning) ||
                         (eStatus==eReady)   ||
                         (eStatus==eBlocked)  )
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile Iridium task is active\n");
                    }
                    else if (eStatus==eSuspended)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile Iridium task is suspended, who did this ?\n");
                    }
                    else if (eStatus == eDeleted)
                    {
                        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile Iridium task is deleted\n");

                        /* check if the page are transmitted successfully, tranmission number */
                        uint8_t recv[6] = {0};
                        bool ret = GET_Iridium_status (recv);

                        if (ret)
                        {
                            if ( recv[0] <= 5)
                            {
                                profile_run = false;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile transmission successful\n");
                                vTaskDelay(pdMS_TO_TICKS(100UL));
                            }
                            else
                            {
                                /* reset the read length */
                                profPage--;
                                prof.cbuf.read = prof.cbuf.read - txprof;
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile transmission not successful\n");
                                ARTEMIS_DEBUG_PRINTF("SPS :: tx, Profile reseting page\n");
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
                //profPage = 0;
            }
        }

        if (send_park | send_prof)
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Transmission is not done yet, continue ...\n");
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: tx, Transmission task is done\n");
            /* if we are here then transmission either was successful or failed */
            run = false;

            /* uninitialize the iridium module */
            i9603n_off();
            vTaskDelay(pdMS_TO_TICKS(1000UL));

            /* check if we have reached the maximum profiling numbers */
            if (prof_number > SYSTEM_PROFILE_NUMBER)
            {
                spsEvent = MODE_POPUP;
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

    /* uinitialize the iridium module */
    i9603n_uninitialize();
    i9603n_off();

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

static uint16_t create_park_page(uint8_t *pPark)
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
    ARTEMIS_DEBUG_PRINTF("Collecting Park measurements\n\n");
    do
    {
        size = DATA_get_converted(&park, &start, &offset, &pressure, &temp);
        /* check, if we have no data available */
        if (size == 0)
        {
            if (len > 0 && len < 103)
            {
                parkPage++;
            }
            ARTEMIS_DEBUG_PRINTF("No more measurements are available\n");
            break;
        }

        /* check on the length, must be < 313 */
        if (len > 103)
        {
            /* we are done here, increase the page number */
            parkPage++;
            ARTEMIS_DEBUG_PRINTF("collect more measurements in a different page\n");
            break;
        }

        /* start collecting data into the temp park buffer */
        *ptPark++ = (uint8_t) (pressure&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Pressure    = 0x%02X\n", *(ptPark-1) );
        *ptPark++ = (uint8_t) (temp>>8);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptPark-1) );
        *ptPark++ = (uint8_t) (temp&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptPark-1) );

        len++;

        ARTEMIS_DEBUG_PRINTF("collecting Park page bytes length = %u\n", len*3);

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
    return nrBytes;
}

static uint16_t create_profile_page(uint8_t *pProf)
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
    ARTEMIS_DEBUG_PRINTF("Collecting Profile measurements\n\n");
    do
    {
        size = DATA_get_converted(&prof, &start, &offset, &pressure, &temp);
        /* check, if we have no data available */
        if (size == 0)
        {
            if (len > 0 && len < 103)
            {
                profPage++;
            }
            ARTEMIS_DEBUG_PRINTF("No more measurements are available\n");
            break;
        }
        /* check on the length, must be < 313 */
        if (len > 103)
        {
            /* we are done here, increase the page number */
            profPage++;
            ARTEMIS_DEBUG_PRINTF("collect more measurements in a different page\n");
            break;
        }
        /* start collecting data into the temp park buffer */
        *ptProf++ = (uint8_t) (pressure&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Pressure    = 0x%02X\n", *(ptProf-1) );
        *ptProf++ = (uint8_t) (temp>>8);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptProf-1) );
        *ptProf++ = (uint8_t) (temp&0xFF);
        //ARTEMIS_DEBUG_PRINTF("Temperature = 0x%02X\n", *(ptProf-1) );
        len++;
        ARTEMIS_DEBUG_PRINTF("collecting Profile page bytes length = %u\n", len*3);

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
    ARTEMIS_DEBUG_PRINTF("Event waiting ...\n");
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

