#include "sensors.h"
#include "depth.h"
#include "temperature.h"
#include "GPS.h"
#include "artemis_max14830.h"
#include "artemis_debug.h"
#include "MAX14830.h"
#include "sysinfo.h"
#include "piston.h"
#include "artemis_rtc.h"
#include "config.h"
#include "am_util_string.h"

#define SENSOR_MAX_DEPTH_RATE           ( 10 )
#define SENSOR_MAX_TEMPERATURE_RATE     ( 2 )
#define SENSOR_MAX_GPS_RATE             ( 2 )

static SemaphoreHandle_t xTDSemaphore = NULL;

typedef enum {
    RECEIVE,
    DELETE,
    IDLE
} Event_e;

static QueueHandle_t xTempEventQueue;
static QueueHandle_t xDepthEventQueue;
static QueueHandle_t xTDEventQueue;
static void SendEvent(QueueHandle_t eventQueue, Event_e *event);
static void ReceiveEvent(QueueHandle_t eventQueue, Event_e *event);
static void EmptyEventQueue(QueueHandle_t eventQueue);

static volatile bool xGPS_run   = true;
static volatile bool xTemp_run  = false;
static volatile bool xDepth_run = false;

static volatile bool TaskLive_GPS = false;

/* timer callback function */
void xGPSTimer(TimerHandle_t xTimer)
{
    ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Timer has expired\n");
    if (xGPS_run)
    {
        xGPS_run = false;
    }
}

void killGPS(void)
{
    ARTEMIS_DEBUG_PRINTF("SENSORS :: Setting GPS Run Flag to False...\n");
    xGPS_run = false;
}

static SensorData_t sensor_data;

bool SENS_initialize(void)
{
    bool success = false;

    /* LCP system information */
    success = SYS_lcp_info();

    /* initialize RTC module */
    artemis_rtc_initialize();

    /* initialize MAX14830 chip for the sensors */
    success = MAX14830_initialize();

    /* initialize pressure sensor */
    success = DEPTH_initialize(DEPTH_Keller_PR9LX);

    /* initialize temperature sensor */
    success = TEMP_initialize(TEMP_SoundNine_OEM);

    /* disable max14830*/
    MAX14830_disable_direct();

    /* initialize GPS */
    success = GPS_initialize();

    /* Piston Board information */
    success = PIS_initialize();

    /* Create a single semaphore for Temperature and Pressure sensors */
    xTDSemaphore = xSemaphoreCreateMutex();
    if (xTDSemaphore == NULL)
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: ERROR, xTDSemaphore is NULL\n");
        success = false;
    }
    else
    {
        success = true;
    }

    /** Create semaphore for the gps */
    sensor_data.gps.semaphore = xSemaphoreCreateMutex();
    if (sensor_data.gps.semaphore == NULL)
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: ERROR, sensor_data.gps.semaphore is NULL\n");
        success = false;
    }
    else
    {
        success = true;
    }

    xTempEventQueue = xQueueCreate(1, sizeof(Event_e));
    xDepthEventQueue = xQueueCreate(1, sizeof(Event_e));
    xTDEventQueue = xQueueCreate(1, sizeof(Event_e));
    if (xTempEventQueue == NULL || xDepthEventQueue == NULL || xTDEventQueue == NULL)
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: ERROR, Temp-Depth Events is NULL\n");
        success = false;
    }
    else
    {
        success = true;
    }
    ARTEMIS_DEBUG_PRINTF("SENSORS :: Sensors are initialized\n\n");
    return success;
}

void SENS_uninitialize(void)
{
    /** uninitialize MAX14830 */
    MAX14830_uninitialize();
    /** uninitialize GPS */
    GPS_uninitialize();
    /** Piston Board information */
    PIS_uninitialize();

    // Delete Semaphores
    if (xTDSemaphore != NULL) {
        vSemaphoreDelete(xTDSemaphore);
        xTDSemaphore = NULL;
    }
    if (sensor_data.gps.semaphore != NULL) {
        vSemaphoreDelete(sensor_data.gps.semaphore);
        sensor_data.gps.semaphore = NULL;
    }

    // --- Delete Event Queues ---
    if (xTempEventQueue != NULL) {
        vQueueDelete(xTempEventQueue);
        xTempEventQueue = NULL;
    }
    if (xDepthEventQueue != NULL) {
        vQueueDelete(xDepthEventQueue);
        xDepthEventQueue = NULL;
    }
    if (xTDEventQueue != NULL) {
        vQueueDelete(xTDEventQueue);
        xTDEventQueue = NULL;
    }

    vTaskDelay(xDelay10ms);
    ARTEMIS_DEBUG_PRINTF("SENSORS :: Sensors are uninitialized\n\n");
}

void SENS_sensor_depth_off(void)        { DEPTH_uninitialize_RTOS();    }
void SENS_sensor_depth_on(void)         { DEPTH_initialize_RTOS();      }
void SENS_sensor_gps_off(void)          { GPS_off();                    }
void SENS_sensor_gps_on(void)           { GPS_on();                     }
void SENS_sensor_temperature_off(void)  { TEMP_uninitialize_RTOS();     }
void SENS_sensor_temperature_on(void)   { TEMP_initialize_RTOS();       }

bool SENS_get_depth(float *depth, float *pressure, float *rate)
{
    bool retVal = false;
    Event_e dEvent = IDLE;
    ReceiveEvent(xDepthEventQueue, &dEvent);
    if (dEvent == RECEIVE)
    {
        taskENTER_CRITICAL();
        *depth = sensor_data.depth.current;
        *rate = sensor_data.depth.ascent_rate;
        *pressure = sensor_data.pressure.current;
        retVal = true;
        taskEXIT_CRITICAL();
    }
    return retVal;
}

bool SENS_get_temperature(float *temperature)
{
    bool retVal = false;
    Event_e tEvent = IDLE;
    ReceiveEvent(xTempEventQueue, &tEvent);
    if (tEvent == RECEIVE)
    {
        taskENTER_CRITICAL();
        *temperature = sensor_data.temperature.current;
        retVal = true;
        taskEXIT_CRITICAL();
    }
    return retVal;
}

bool SENS_get_gps(SensorGps_t *gps)
{
    bool retVal = false;
    taskENTER_CRITICAL();
    *gps = sensor_data.gps;
    retVal = true;
    taskEXIT_CRITICAL();
    return retVal;
}

bool GPS_running(void)
{
    return TaskLive_GPS;
}

static void EmptyEventQueue(QueueHandle_t eventQueue)
{
    int32_t discard;
    while (xQueueReceive(eventQueue, &discard, 0) == pdPASS)
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: EmptyEventQueue : emptying\n");
    }
    ARTEMIS_DEBUG_PRINTF("SENSORS :: EmptyEventQueue : emptied\n");
}

void SENS_task_delete(TaskHandle_t xHandle)
{
    TaskStatus_t xTaskDetails;
    Event_e xtdEvent = IDLE;

    while (1)
    {
        vTaskGetInfo(xHandle, &xTaskDetails, pdTRUE, eInvalid);
        eTaskState eState = xTaskDetails.eCurrentState;
        const char *task_name = xTaskDetails.pcTaskName;

        if ( (eState == eBlocked) || (eState == eSuspended) )
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: %s in eBlocked state\n", task_name);
            if ( am_util_string_strcmp(task_name, "Depth_Task") == 0 )
            {
                taskENTER_CRITICAL();
                xDepth_run = false;
                taskEXIT_CRITICAL();
                ReceiveEvent(xTDEventQueue, &xtdEvent);
                EmptyEventQueue(xDepthEventQueue);
            }
            else if ( am_util_string_strcmp(task_name, "Temperature_Task") == 0 )
            {
                taskENTER_CRITICAL();
                xTemp_run = false;
                taskEXIT_CRITICAL();
                ReceiveEvent(xTDEventQueue, &xtdEvent);
                EmptyEventQueue(xTempEventQueue);
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("SENSORS :: %s is unknonwn\n", task_name);
                taskENTER_CRITICAL();
                xTemp_run = false;
                xDepth_run = false;
                taskEXIT_CRITICAL();
                EmptyEventQueue(xTempEventQueue);
                EmptyEventQueue(xDepthEventQueue);
                break;
            }
        }
        else if (eState == eReady)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: %s in eReady state\n", task_name);
        }
        else if (eState == eRunning)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: %s in eRunning state, wait\n", task_name);
        }
        else if ( (eState == eDeleted) || (eState == eInvalid) )
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: %s->Deleted\n", task_name);
            if(xSemaphoreTake(xTDSemaphore, xDelay10ms) == pdTRUE)
            {
                xSemaphoreGive(xTDSemaphore);
                ARTEMIS_DEBUG_PRINTF("SENSORS :: %s, xTDsemaphore->available\n", task_name);
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("SENSORS :: %s, xTDsemaphore->not available\n", task_name);
            }
            break;
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: %s is in unknown state\n", task_name);
            if(xSemaphoreTake(xTDSemaphore, xDelay10ms) == pdTRUE)
            {
                xSemaphoreGive(xTDSemaphore);
                ARTEMIS_DEBUG_PRINTF("SENSORS :: %s, xTDsemaphore->available\n", task_name);
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("SENSORS :: %s, xTDsemaphore->not available\n", task_name);
            }
            break;
        }
        vTaskDelay(xDelay50ms);
    }
}

void SENS_task_profile_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 1024, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );

    configASSERT(xTaskCreate((TaskFunction_t)task_temperature,
                                "Temperature_Task", 1024, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xTemp) == pdPASS );
}

void SENS_task_park_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 1024, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );

    configASSERT(xTaskCreate((TaskFunction_t)task_temperature,
                                "Temperature_Task", 1024, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xTemp) == pdPASS );
}

void SENS_task_sample_depth_continuous(TaskHandle_t *xDepth)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 1024, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );
}

void SENS_task_gps(TaskHandle_t *xGPS)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_gps,
                                "GPS_Task", 1024, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xGPS) == pdPASS );
}

void SENS_set_depth_rate(float rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_DEPTH_RATE) )
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: Depth, Setting rate = %.6fHz\n", rate);
        sensor_data.depth.rate = rate;
    }
}

void SENS_set_temperature_rate(float rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_TEMPERATURE_RATE))
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: Temperature, Setting rate = %.6fHz\n", rate);
        sensor_data.temperature.rate = rate;
    }
}

void SENS_set_gps_rate(uint16_t rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_GPS_RATE))
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Setting rate = %uHz\n", rate);
        sensor_data.gps.rate = rate;
    }
}

void task_depth(void)
{
    sDepth_Measurement_t depth = {0};
    assert(sensor_data.depth.rate != 0);

    uint32_t period = xDelay1000ms / sensor_data.depth.rate;
    ARTEMIS_DEBUG_PRINTF("SENSORS :: Depth, Setting period = %ums\n", period);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    Event_e dEvent = IDLE;
    xDepth_run = true;

    while (xDepth_run)
    {
        /* 1. Turn ON sensor hardware rail */
        DEPTH_Power_ON();
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms warm-up margin for Keller transducer

        if (xSemaphoreTake(xTDSemaphore, period) == pdTRUE)
        {
            DEPTH_Read(&depth);

            /* 2. Turn OFF sensor hardware rail immediately to save battery */
            DEPTH_Power_OFF();

            sensor_data.depth.previous = sensor_data.depth.current;
            sensor_data.depth.current = depth.Depth;
            sensor_data.depth.ascent_rate = (sensor_data.depth.current - sensor_data.depth.previous);

            sensor_data.pressure.previous = sensor_data.pressure.current;
            sensor_data.pressure.current = depth.Pressure;

            xSemaphoreGive(xTDSemaphore);

            dEvent = RECEIVE;
            SendEvent(xDepthEventQueue, &dEvent);
        }
        else
        {
            /* If semaphore acquisition failed, ensure power is turned off */
            DEPTH_Power_OFF();
            ARTEMIS_DEBUG_PRINTF("SENSORS :: Depth, semaphore not available\n");
        }

        /* Sleep for the full period with hardware OFF and 0 heap churn */
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    dEvent = DELETE;
    SendEvent(xTDEventQueue, &dEvent);
    vTaskDelete(NULL);
}

void task_temperature(void)
{
    assert(sensor_data.temperature.rate != 0);
    Temperature_Measurement_t temperature = {0};

    uint32_t period = xDelay1000ms / sensor_data.temperature.rate;
    ARTEMIS_DEBUG_PRINTF("SENSORS :: Temperature, Setting period = %ums\n", period);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    Event_e tEvent = IDLE;
    xTemp_run = true;

    while (xTemp_run)
    {
        /* 1. Energize COM0 power rail */
        TEMP_Power_ON();

        /* 2. Hardware boot delay: SoundNine internal MCU boot */
        vTaskDelay(pdMS_TO_TICKS(100));

        /* 3. Silence auto-stream immediately after boot */
        TEMP_stop_sampling();

        if (xSemaphoreTake(xTDSemaphore, period) == pdTRUE)
        {
            /* 4. Issue polled reading ("sample\r") */
            TEMP_Read(&temperature);

            /* 5. Shut power rail off immediately to save energy */
            TEMP_Power_OFF();

            sensor_data.temperature.current = temperature.temperature;
            xSemaphoreGive(xTDSemaphore);

            tEvent = RECEIVE;
            SendEvent(xTempEventQueue, &tEvent);
        }
        else
        {
            TEMP_Power_OFF();
            ARTEMIS_DEBUG_PRINTF("SENSORS :: Temperature, semaphore not available\n");
        }

        /* Sleep in blocked state with rail at 0V until next period */
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    tEvent = DELETE;
    SendEvent(xTDEventQueue, &tEvent);
    vTaskDelete(NULL);
}

void task_gps(void)
{
    TaskLive_GPS = true;
    ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Task is starting...\n");
    assert(sensor_data.gps.rate != 0);

    GPS_Data_t gps = {0};
    uint32_t period = xDelay1000ms / sensor_data.gps.rate;
    ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Setting period = %ums\n", period);

    /* 1. Energize u-blox hardware rail */
    GPS_on();
    vTaskDelay(pdMS_TO_TICKS(150)); // Allow u-blox internal processor to boot

    /* 2. Initialize communication interface */
    GPS_initialize();

    /* 3. Constrain navigation engine to Sea dynamics */
    GPS_set_dynamic_model_sea();
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 4. Inject RTC Time with 5-minute (300 sec) uncertainty tolerance */
    GPS_inject_rtc_time_with_acc(300);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 5. Inject Last Known Position with 200 km (108 nm) uncertainty budget */
    if (sensor_data.gps.latitude != 0.0f || sensor_data.gps.longitude != 0.0f)
    {
        GPS_inject_position(sensor_data.gps.latitude,
                            sensor_data.gps.longitude,
                            0.0f,     // Ocean surface: 0 m MSL
                            200.0f);  // 200 km uncertainty radius
        ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Injected coarse position aiding (200km bound).\n");
    }

    /* Start timeout timer */
    TickType_t xTimeMinutes = GPS_TIMER * 60 * xDelay1000ms;
    TimerHandle_t xTimer = xTimerCreate("GPS_Timer", xTimeMinutes, pdFALSE, (void*)1, xGPSTimer);
    if (xTimer != NULL)
    {
        configASSERT(xTimerStart(xTimer, 0) == pdPASS);
        ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Timer has started for %f Minutes\n", GPS_TIMER);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Timer did not start\n");
        vTaskDelete(NULL);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    xGPS_run = true;
    uint8_t xFix = 0;

    while(xGPS_run)
    {
        if(GPS_Read(&gps))
        {
            configASSERT(xSemaphoreTake(sensor_data.gps.semaphore, period) == pdTRUE);
            sensor_data.gps.fix = true;
            sensor_data.gps.latitude = gps.position.lat;
            sensor_data.gps.longitude = gps.position.lon;
            sensor_data.gps.altitude = gps.position.alt;
            sensor_data.gps.year = gps.time.year;
            sensor_data.gps.month = gps.time.month;
            sensor_data.gps.day = gps.time.day;
            sensor_data.gps.hour = gps.time.hour;
            sensor_data.gps.min = gps.time.min;
            sensor_data.gps.sec = gps.time.sec;
            xSemaphoreGive(sensor_data.gps.semaphore);

            xFix++;
            // Stop once 3 valid fixes are confirmed to save active battery power
            if(xFix >= 3)
            {
                xGPS_run = false;
                xFix = 0;
                xTimerStop(xTimer, 0);
            }
        }
        else
        {
            configASSERT(xSemaphoreTake(sensor_data.gps.semaphore, xDelay100ms) == pdTRUE);
            sensor_data.gps.fix = false;
            xSemaphoreGive(sensor_data.gps.semaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, period);
    }

    /* Stop the timer and delete it */
    if (xTimer != NULL) {
        xTimerStop(xTimer, portMAX_DELAY); 
        if (xTimerDelete(xTimer, portMAX_DELAY) == pdPASS) {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS Timer deleted successfully.\n");
        } else {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: ERROR: Failed to delete GPS Timer.\n");
        }
        xTimer = NULL;
    }

    ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Task is deleting...\n");
    vTaskDelay(xDelay1000ms);
    TaskLive_GPS = false;
    vTaskDelete(NULL);
}

static void SendEvent(QueueHandle_t eventQueue, Event_e *event)
{
    xQueueSend(eventQueue, event, xDelay1000ms);
}

static void ReceiveEvent(QueueHandle_t eventQueue, Event_e *event)
{
    xQueueReceive(eventQueue, event, xDelay2000ms);
}