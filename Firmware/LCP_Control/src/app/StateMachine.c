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
#include "artemis_debug.h"
#include "artemis_rtc.h"
#include "piston.h"
#include "control.h"
#include "i9603n.h"

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
    .profiler.state = SPS_Idle,
    .ballast.state = ABS_DiveToExpected,
    .moored.state = MOOR_Idle,
    .popup.state = PUS_Surface_float,
};

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


//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************

//static bool module_sps_idle(void);
//static bool module_sps_move_to_park(void);
//static bool module_sps_park(void);
//static bool module_sps_move_to_profile(void);
//static bool module_sps_profile(void);
//static bool module_sps_move_to_surface(void);
//static bool module_sps_tx(void);
static bool module_sps_rx(void);

void module_sps_idle(void);
void module_sps_move_to_park(void);
void module_sps_park(void);
void module_sps_move_to_profile(void);
void module_sps_profile(void);
void module_sps_move_to_surface(void);
void module_sps_tx(void);

void module_pds_idle(void);
void module_pds_systemcheck(void);

void module_pus_idle(void);
void module_pus_surface_float(void);

#define SPS_NUMBER  10
static bool sensors_check = false;
static uint16_t sps_number = 0;
static uint16_t park_number = 0;

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
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        ARTEMIS_DEBUG_PRINTF("PUS :: Idle, wait forever\n");
        vTaskDelay(portMAX_DELAY);
        run = false;
    }
    vTaskDelete(NULL);
}

void module_pus_surface_float(void)
{
    TaskHandle_t xPiston;
    eTaskState eStatus;
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_full,
                                "Piston_Task_to_full", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                &xPiston) == pdPASS );
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
    ARTEMIS_DEBUG_PRINTF("PDS :: Idle, wait here for one min ...\n");
    Event_e pdsEvent;

    /* wait for one min */
    uint32_t wait_time = 60 * 1000;
    TickType_t xDelay = wait_time / portTICK_PERIOD_MS;

    bool run = true;
    while (run)
    {
        //vTaskDelay(portMAX_DELAY);
        run = false;
        vTaskDelay(xDelay);
        pdsEvent = MODE_PROFILE;
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
        bool run = true;
        uint8_t fix = 0;


        /* turn on datalogger, wait for 100ms */
        datalogger_power_on();
        vTaskDelay(pdMS_TO_TICKS(100UL));

        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();

        while (run)
        {

            SENS_get_gps(&gps);

            if (gps.fix == true)
            {
                //ARTEMIS_DEBUG_PRINTF("GPS : TimeStampe, %u.%u.%u, %u:%u:%u\n", gps.month, gps.day, gps.year, gps.hour, gps.min, gps.sec);
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.7f\n", gps.latitude, gps.longitude, gps.altitude);
                fix++;

                if (fix == 10)
                {
                    /* Calibrate the GPS UTC time into RTC */
                    ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, RTC : Setting gps time\n");
                    artemis_rtc_gps_calibration(&gps);
                    am_hal_gpio_output_clear(AM_BSP_GPIO_LED_GREEN);

                    //rtc_time time;
                    //artemis_rtc_get_time(&time);
                    //ARTEMIS_DEBUG_PRINTF("RTC : TimeStampe, %u.%u.%u, %u:%u:%u\n", time.month, time.day, time.year, time.hour, time.min, time.sec);

                    run = false;
                    vTaskDelete(xGps);
                    vTaskDelay(pdMS_TO_TICKS(100UL));
                    SENS_sensor_gps_off();

                    /* store data in the SDcard */
                    datalogger_predeploy_mode(&gps, true);
                    datalogger_power_off();

                    //vTaskDelay(portMAX_DELAY);
                    pdsEvent = MODE_PRE_DEPLOY;
                }
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("PDS :: systemcheck, GPS : No fix\n");
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
                                        "sps_idle", 256, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_MoveToParkDepth_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_move_to_park,
                                        "sps_move_to_park", 256, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_Park_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_park,
                                        "sps_park", 256, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_MoveToSampleDepth_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_move_to_profile,
                                        "move_to_profile", 256, NULL,
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
                                        "move_to_surface", 256, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(spsEventQueue, &spsEvent);
                break;
            case SPS_TX_mode:
                configASSERT(xTaskCreate((TaskFunction_t) module_sps_tx,
                                        "task_iridium", 512, NULL,
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

    /* wait here for one min for now */
    uint32_t wait_time = 60 * 1000;
    TickType_t xDelay = wait_time / portTICK_PERIOD_MS;

    Event_e spsEvent;
    bool run = true;
    while (run)
    {
        ARTEMIS_DEBUG_PRINTF("SPS :: Idle, waiting for one min ...\n");
        vTaskDelay(xDelay);
        spsEvent = MODE_DONE;
        run = false;
    }

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_move_to_park(void)
{
    /** Calculate volume to move to */
    float v_rate = 0.1;
    float volume = module_ctrl_set_buoyancy_from_rate(v_rate, true);

    /** Set volume */
    //PIS_set_volume(volume);
    PIS_set_volume(660); // dummy volume for now 650
    TaskHandle_t xPiston;
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_volume,
                                "Piston_Task", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                &xPiston) == pdPASS );

    /** Start sampling depth @ 2Hz */
    float s_rate = 2.0;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth;
    SENS_task_sample_depth_continuous(&xDepth);

    /**  Monitor depth until we get there */
    bool run = true;
    double Volume = 0;
    float Depth = 0, Rate = 0;
    Event_e spsEvent;
    uint8_t count_500ms = 0;
    
    vTaskDelay(period);

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (run)
    {
        SENS_get_depth(&Depth, &Rate);
        count_500ms++;

        if (count_500ms == 2)
        {
            PIS_Get_Volume(&Volume);

            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Volume = %0.5f \n", Volume);
            if (Volume >=(660-0.1) && Volume <=(660+0.1))
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Piston Volume approximately reached\n");
                run = false;
                vTaskDelete(xDepth);
                SENS_sensor_depth_off();
                spsEvent = MODE_DONE;
            }
            count_500ms = 0;
        }

        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_park, Depth  = %0.5f, rate = %0.5f \n", Depth, Rate);
        //vTaskDelay(period);
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_park(void)
{
    /** Start 1/60Hz sampling of sensors for XXX minutes */
    /** Save data in Park Data strucutre */

    /* turn datalogger ON*/
    datalogger_power_on();

    /** Sample at 1/60th Hz */
    float s_rate = 1.0/60.01;
    uint32_t period = pdMS_TO_TICKS(1000UL)/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);

    SENS_sensor_depth_on();
    SENS_sensor_temperature_on();

    TaskHandle_t xDepth, xTemp;
    SENS_task_park_sensors(&xDepth, &xTemp);

    /** Monitor Depth and Temperature and store these */
    bool run = true;
    float Depth = 0, Rate = 0;
    float Temperature = 0;
    uint8_t wait_min = 0;
    Event_e spsEvent;
    rtc_time time;

    char *filename = datalogger_park_create_file(park_number);
    //vTaskDelay(pdMS_TO_TICKS(100UL));

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (run)
    {
        SENS_get_depth(&Depth, &Rate);
        SENS_get_temperature(&Temperature);

        ARTEMIS_DEBUG_PRINTF("SPS :: park, Depth  = %0.5f, rate = %0.5f \n", Depth, Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Temperature  = %0.5f\n", Temperature);

        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SPS :: park, Epoch = %ld \n", epoch);

        size_t data_size = DATA_add(&park, epoch, Depth, Temperature);

        /* store in park mode file */
        datalogger_park_mode(filename, Depth, Temperature, &time);

        wait_min++;
        vTaskDelayUntil(&xLastWakeTime, period);

        if (wait_min == 5)
        {
            run = false;

            vTaskDelete(xTemp);
            vTaskDelete(xDepth);

            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            datalogger_power_off();
            spsEvent = MODE_DONE;
        }
    }

    park_number++;
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_move_to_profile(void)
{
    /** Calculate volume to move to */
    float v_rate = 0.1;
    float volume = module_ctrl_set_buoyancy_from_rate(v_rate, true);

    /** Set volume */
    //PIS_set_volume(volume);
    PIS_set_volume(670); // dummy volume for now 670
    TaskHandle_t xPiston;
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_volume,
                                "Piston_Task", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                &xPiston) == pdPASS );


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
    Event_e spsEvent;
    uint8_t count_500ms = 0;

    vTaskDelay(period);

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (run)
    {
        count_500ms++;

        SENS_get_depth(&Depth, &Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Depth  = %0.5f, rate = %0.5f \n", Depth, Rate);

        if (count_500ms == 2)
        {
            PIS_Get_Volume(&Volume);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Volume = %0.5f \n", Volume);

            if (Volume >=(670-0.1) && Volume <=(670+0.1))
            {
                ARTEMIS_DEBUG_PRINTF("SPS :: move_to_profile, Piston Volume approximately reached\n");
                run = false;
                vTaskDelete(xDepth);
                SENS_sensor_depth_off();
                spsEvent = MODE_DONE;
            }
            count_500ms = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, period);
    }

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
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

    /* turn datalogger ON*/
    datalogger_power_on();
    vTaskDelay(pdMS_TO_TICKS(100UL));

    /** Set volume */
    PIS_set_volume(680); // dummy volume for now
    TaskHandle_t xPiston;
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_volume,
                                "Piston_Task", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                &xPiston) == pdPASS );


    TaskHandle_t xDepth, xTemp;
    SENS_task_profile_sensors(&xDepth, &xTemp);

    /** Start recording samples */
    float Temperature = 0;
    float Depth = 0;
    float Rate = 0;
    double Volume = 0;
    Event_e spsEvent;
    rtc_time time;

    /** Monitor depth & rate.  If rate fails, fix it.  If depth reaches surface, done */
    char *filename = datalogger_profile_create_file(sps_number);
    vTaskDelay(pdMS_TO_TICKS(500UL));

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    bool run = true;
    while (run)
    {
        PIS_Get_Volume(&Volume);
        SENS_get_depth(&Depth, &Rate);
        SENS_get_temperature(&Temperature);

        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Depth = %0.5f, rate = %0.5f \n", Depth, Rate);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Temperature = %0.5f\n", Temperature);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Volume = %0.5lf \n", Volume);

        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SPS :: profile, Epoch = %ld \n", epoch);

        size_t data_size = DATA_add(&prof, epoch, Depth, Temperature);

        /* datalogger */
        datalogger_profile_mode(filename, Depth, Temperature, Volume, &time);

        if (Volume >=(680-0.1) && Volume <=(680+0.1))
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: profile, Piston Volume approximately reached\n");
            run = false;
            vTaskDelete(xDepth);
            vTaskDelete(xTemp);
            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            datalogger_power_off();
            spsEvent = MODE_DONE;
        }

        //vTaskDelay(period);
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    sps_number++;
    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_move_to_surface(void)
{
    /** Extend the piston fully out */

    TaskHandle_t xPiston;
    eTaskState eStatus;
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_full,
                                "Piston_Task_to_full", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                &xPiston) == pdPASS );
    bool run = true;
    while (run)
    {
        eStatus = eTaskGetState( xPiston );
        ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, Piston move status = %u\n", eStatus);
        if (eStatus == eDeleted)
        {
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
    uint16_t count = 0;
    Event_e spsEvent;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (run)
    {
        SENS_get_gps(&gps);

        if (gps.fix == true)
        {
            //ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS : Time, %d:%02d:%02d\n", gps.hour, gps.min, gps.sec);
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.2f\n", gps.latitude, gps.longitude, gps.altitude);
            count++;

            if (count == 10)
            {
                artemis_rtc_gps_calibration(&gps);
                run = false;
                vTaskDelete(xGps);
                vTaskDelay(pdMS_TO_TICKS(100UL));
                /** GPS OFF */
                SENS_sensor_gps_off();
                spsEvent = MODE_DONE;
            }
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("SPS :: move_to_surface, GPS : No fix\n");
        }

        //vTaskDelay(pdMS_TO_TICKS(1000UL));
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);
}

void module_sps_tx(void)
{
    /** Turn on the Iridium Modem */
    i9603n_initialize();
    vTaskDelay(pdMS_TO_TICKS(1000UL));
    i9603n_on();
    vTaskDelay(pdMS_TO_TICKS(1000UL));

    /** Monitor for lock or timeout */

    /** Transmit all data */
    TaskHandle_t xIridium;
    eTaskState eStatus;
    configASSERT(xTaskCreate((TaskFunction_t) task_Iridium,
                                "task Iridium", 256, NULL,
                                tskIDLE_PRIORITY + 2UL,
                                &xIridium) == pdPASS );

    bool run = true;
    uint8_t count = 0;
    Event_e spsEvent;

    while (run)
    {
        eStatus = eTaskGetState( xIridium );
        ARTEMIS_DEBUG_PRINTF("SPS :: tx, Iridium task status = %u\n", eStatus);
        count++;

        if (count == 10)
        {
            run = false;
            vTaskDelete(xIridium);
            vTaskDelay(pdMS_TO_TICKS(100UL));
            i9603n_off();

            if (sps_number > 0)
            {
                spsEvent = MODE_POPUP;
            }
            else
            {
                spsEvent = MODE_IDLE;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000UL));
    }

    i9603n_uninitialize();
    i9603n_off();

    SendEvent(spsEventQueue, &spsEvent);
    vTaskDelete(NULL);

}

static bool module_sps_rx(void)
{
    /** Check for messages */

    /** If message, receive it */

    /** Iridium OFF */

    return false;
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

