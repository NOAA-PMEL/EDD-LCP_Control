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

//static SemaphoreHandle_t xDepthSemaphore = NULL;
//static SemaphoreHandle_t xTempSemaphore = NULL;
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

/* timer callback function */
void xGPSTimer(TimerHandle_t xTimer)
{
    ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Timer has expired\n");
    if (xGPS_run)
    {
        xGPS_run = false;
    }
}

static SensorData_t sensor_data;
//static SensorData_t sensor_data = {
//  .depth.semaphore = &xDepthSemaphore,
//  .temperature.semaphore = &xTempSemaphore
//};

bool SENS_initialize(void)
{
    bool success = false;

    /* LCP system information */
    success = SYS_lcp_info();

    /* initialize RTC module */
    artemis_rtc_initialize();

    /* initialize MAX14830 chip for the sensors */
    //artemis_max14830_init();
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

    /* Create a single semaphore for Temperature and
       Pressure sensors for now */
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

    /** delete semaphores */
    vSemaphoreDelete(xTDSemaphore);
    vSemaphoreDelete(sensor_data.gps.semaphore);
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
        /* get pressure as well */
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
    /* turn off LEDs */
    //am_hal_gpio_output_set(AM_BSP_GPIO_LED_BLUE);
    //am_hal_gpio_output_set(AM_BSP_GPIO_LED_GREEN);
    /* check the task state */
    while (1)
    {
        vTaskGetInfo(xHandle, &xTaskDetails, pdTRUE, eInvalid);
        eTaskState eState = xTaskDetails.eCurrentState;
        const char *task_name = xTaskDetails.pcTaskName;

        if ( (eState==eBlocked) || (eState==eSuspended) )
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
        else if (eState==eReady)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: %s in eReady state\n", task_name);
        }
        else if (eState==eRunning)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: %s in eRunning state, wait\n", task_name);
        }
        else if (eState==eDeleted)
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

void SENS_task_temperature_on(TaskHandle_t *xTemp)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_temperature_on,
                                "Temperature_Task_On", 256, NULL,
                                tskIDLE_PRIORITY + 5UL,
                                xTemp) == pdPASS );
}

void SENS_task_profile_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 512, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );

    configASSERT(xTaskCreate((TaskFunction_t)task_temperature,
                                "Temperature_Task", 512, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xTemp) == pdPASS );
}

void SENS_task_park_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 512, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );

    configASSERT(xTaskCreate((TaskFunction_t)task_temperature,
                                "Temperature_Task", 512, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xTemp) == pdPASS );
}

void SENS_task_sample_depth_continuous(TaskHandle_t *xDepth)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 512, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );
}

void SENS_task_gps(TaskHandle_t *xGPS)
{
    configASSERT(xTaskCreate((TaskFunction_t)task_gps,
                                "GPS_Task", 512, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xGPS) == pdPASS );
}

void SENS_set_depth_rate(float rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_DEPTH_RATE) )
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: Depth, Setting rate = %.3fHz\n", rate);
        sensor_data.depth.rate = rate;
    }
}

void SENS_set_temperature_rate(float rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_TEMPERATURE_RATE))
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: Temperature, Setting rate = %.3fHz\n", rate);
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

    uint32_t period = xDelay1000ms/sensor_data.depth.rate;
    ARTEMIS_DEBUG_PRINTF("SENSORS :: Depth, Setting period = %ums\n", period);

    /** Create the semaphore for the depth sensor read */
    //sensor_data.depth.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    //DEPTH_initialize(DEPTH_Keller_PA9LD);

    /* Initialise the xLastWakeTime variable with the current time */
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    Event_e dEvent = IDLE;
    xDepth_run = true;

    while(xDepth_run)
    {
        DEPTH_Power_ON();
        /** Delay to warm up */
        /** @todo replace with const number */
        /** Read the Data */
        //DEPTH_Read(&depth);

        /** Power Off */
        //DEPTH_Power_OFF();
        
        if(xSemaphoreTake(xTDSemaphore, period) == pdTRUE)
        {
            //am_hal_gpio_output_toggle(AM_BSP_GPIO_LED_GREEN);
            DEPTH_Read(&depth);
            //ARTEMIS_DEBUG_PRINTF("Depth = %.4f\n", depth.Depth);
            DEPTH_Power_OFF();

            sensor_data.depth.previous = sensor_data.depth.current;
            sensor_data.depth.current = depth.Depth;
            //sensor_data.depth.ascent_rate = (sensor_data.depth.current - sensor_data.depth.previous) / sensor_data.depth.rate;
            sensor_data.depth.ascent_rate = (sensor_data.depth.current - sensor_data.depth.previous);

            /* get pressure as well */
            sensor_data.pressure.previous = sensor_data.pressure.current;
            sensor_data.pressure.current = depth.Pressure;

            /* give away the semaphore */
            xSemaphoreGive(xTDSemaphore);

            dEvent = RECEIVE;
            SendEvent(xDepthEventQueue, &dEvent);

            if (period >= xDelay10000ms)
            {
                /* delete task */
                vTaskDelete(NULL);
            }
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: Depth, semaphore not available\n");
        }

        if (period >= xDelay10000ms)
        {
            vTaskDelayUntil(&xLastWakeTime, xDelay1000ms);
        }
        else
        {
            vTaskDelayUntil(&xLastWakeTime, period);
        }
    }

    dEvent = DELETE;
    SendEvent(xTDEventQueue, &dEvent);
    vTaskDelete(NULL);
}

void task_temperature_on(void)
{
    uint32_t period = xDelay500ms;
    /* Initialise the xLastWakeTime variable with the current time */
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        if(xSemaphoreTake(xTDSemaphore, period) == pdTRUE)
        {
            SENS_sensor_temperature_on();
            xSemaphoreGive(xTDSemaphore);
            /* delete the task */
            ARTEMIS_DEBUG_PRINTF("Temperature On, Task->deleted\n");
            vTaskDelete(NULL);
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: Temperature On, semaphore->not available\n");
        }
        vTaskDelayUntil(&xLastWakeTime, period);
    }
}

void task_temperature(void)
{
    assert(sensor_data.temperature.rate != 0);
    Temperature_Measurement_t temperature = {0};

    uint32_t period = xDelay1000ms/sensor_data.temperature.rate;
    ARTEMIS_DEBUG_PRINTF("SENSORS :: Temperature, Setting period = %ums\n", period);

    /** Create the semaphore for the depth sensor read */
    //sensor_data.temperature.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    //TEMP_initialize();

    /* Initialise the xLastWakeTime variable with the current time */
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    Event_e tEvent = IDLE;
    xTemp_run = true;

    while(xTemp_run)
    {
        /** Turn power on */
        //TEMP_Power_ON();
        /** Warmup for x */
        /** @todo replace this with a macro */
        /** Read the temperature */
        //TEMP_Read(&temperature);
        /** Power off */
        //TEMP_Power_OFF();

        if(xSemaphoreTake(xTDSemaphore, period) == pdTRUE)
        {
            //am_hal_gpio_output_toggle(AM_BSP_GPIO_LED_BLUE);
            TEMP_Read(&temperature);

            //ARTEMIS_DEBUG_PRINTF("Temperature = %.3f °C\n", temperature.temperature);
            sensor_data.temperature.current = temperature.temperature;
            xSemaphoreGive(xTDSemaphore);

            /* send event */
            tEvent = RECEIVE;
            SendEvent(xTempEventQueue, &tEvent);

            if (period >= xDelay10000ms)
            {
                /* delete task */
                vTaskDelete(NULL);
            }
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS :: Temperature, semaphore not available\n");
        }

        if (period >= xDelay10000ms)
        {
            vTaskDelayUntil(&xLastWakeTime, xDelay1000ms);
        }
        else
        {
            vTaskDelayUntil(&xLastWakeTime, period);
        }
    }
    /* send event */
    tEvent = DELETE;
    SendEvent(xTDEventQueue, &tEvent);
    vTaskDelete(NULL);
}

void task_gps(void)
{
    assert(sensor_data.gps.rate != 0);

    GPS_Data_t gps = {0};
    uint32_t period = xDelay1000ms/sensor_data.gps.rate;
    ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Setting period = %ums\n", period);

    ///** Create the semaphore for the gps */
    //sensor_data.gps.semaphore = xSemaphoreCreateMutex();
    /** Initialize the GPS */
    //GPS_initialize();
    //GPS_on();
    //vTaskDelay(xDelay250ms);

    /* start a timer with GPS_TIMER in minutes */
    TickType_t xTimeMinutes = GPS_TIMER * 60 * xDelay1000ms;
    TimerHandle_t xTimer = xTimerCreate("GPS_Timer", xTimeMinutes, pdFALSE, (void*)1, xGPSTimer);
    if (xTimer != NULL)
    {
        /* Start the timer */
        configASSERT(xTimerStart(xTimer, 0) == pdPASS);
        ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Timer has started for %f Minutes\n", GPS_TIMER);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("SENSORS :: GPS, Timer did not start\n");
        vTaskDelete(NULL);
    }

    /* Initialise the xLastWakeTime variable with the current time */
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

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
            if(xFix > 9)
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
    vTaskDelete(NULL);
}

static void SendEvent(QueueHandle_t eventQueue, Event_e *event)
{
    /* wait for maximum 1 second, so it does not get stuck */
    xQueueSend(eventQueue, event, xDelay1000ms);
}

static void ReceiveEvent(QueueHandle_t eventQueue, Event_e *event)
{
    /* wait for maximum 2 seconds, so it does not get stuck */
    xQueueReceive(eventQueue, event, xDelay2000ms);
}
