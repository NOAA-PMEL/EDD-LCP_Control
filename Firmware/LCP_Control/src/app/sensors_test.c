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

typedef enum
{
    Sensors_fast,
    Sensors_normal,
    Sensors_slow

} Sensors_test;

typedef enum
{
    MODE_FAST,
    MODE_NORMAL,
    MODE_SLOW,
    MODE_IDLE,
    MODE_DONE
} Event_e_test;

static QueueHandle_t EventQueue_test;
static void SendEvent(QueueHandle_t eventQueue, Event_e_test *event);
static void ReceiveEvent(QueueHandle_t eventQueue, Event_e_test *event);

/* test functions */
void module_sensors_slow(void);
void module_sensors_fast(void);
void module_sensors_normal(void);

static Sensors_test sensors = Sensors_fast;

#define FAST_TIME       ( 1.0f * 60.0f )
#define NORMAL_TIME     ( 2.0f * 60.0f )
#define SLOW_TIME       ( 3.0f * 60.0f )

static volatile uint16_t profileNr = 0;

void SENS_Test(void)
{
    /* create a local task event queue */
    EventQueue_test = xQueueCreate(3, sizeof(Event_e_test));
    Event_e_test Event;

    ARTEMIS_DEBUG_PRINTF("\nSENSORS_TEST :: << Starting Profile No. %u >>\n\n", profileNr+1);

    for (;;)
    {
        switch(sensors)
        {
            case Sensors_fast:
                configASSERT(xTaskCreate((TaskFunction_t) module_sensors_fast,
                                        "sensors_fast", 512, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(EventQueue_test, &Event);
                break;
            case Sensors_normal:
                configASSERT(xTaskCreate((TaskFunction_t) module_sensors_normal,
                                        "sensors_normal", 512, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(EventQueue_test, &Event);
                break;
            case Sensors_slow:
                configASSERT(xTaskCreate((TaskFunction_t) module_sensors_slow,
                                        "sensors_slow", 512, NULL,
                                        tskIDLE_PRIORITY + 2UL,
                                        NULL) == pdPASS );
                ReceiveEvent(EventQueue_test, &Event);
                break;
            default:
                break;
        }

        /* check the received event */
        if(Event == MODE_DONE)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: Transitionng to next state\n");
            sensors++;
            vTaskDelay(xDelay1000ms);
        }
        else if (Event == MODE_IDLE)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: Testing done, going to Idle\n");
            vTaskDelay(xDelay1000ms);
            sensors = Sensors_fast;
            ARTEMIS_DEBUG_PRINTF("\nSENSORS_TEST :: << Starting Profile No. %u >>\n\n", profileNr+1);
            //SENS_uninitialize();
            //vTaskDelay(xDelay1000ms);
            //vTaskDelete(NULL);
        }
        else if (Event == MODE_NORMAL)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: Transitioning to normal\n");
            vTaskDelay(xDelay1000ms);
            sensors = Sensors_normal;
            //SENS_uninitialize();
            //vTaskDelay(xDelay1000ms);
            //vTaskDelete(NULL);
        }
    }
}

void module_sensors_fast(void)
{
    Event_e_test Event;
    float s_rate = MOVE_TO_PARK_SAMPLE_RATE;
    uint32_t period = xDelay1000ms/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_sensor_depth_on();
    TaskHandle_t xDepth = NULL;
    SENS_task_sample_depth_continuous(&xDepth);
    bool run = true;

    /**  Monitor depth until we get there */
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;

    uint32_t fast_time = 0;
    uint32_t wait_time = 0;
    fast_time = (xDelay1000ms * FAST_TIME);
    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: fast, < FAST_TIME = %.2f mins >\n", (float)(FAST_TIME/60));

    while (run)
    {
        /* check on depth sensor */
        SENS_get_depth(&Depth, &Pressure, &Rate);
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: fast, Pressure  = %.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: fast, Depth     = %.4f m, rate = %.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));

        /* timer */
        wait_time += period;
        if (wait_time >= fast_time)
        {
            ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: fast, << Timer out %f mins >>\n", (float) (wait_time/(60.0*xDelay1000ms)));

            run = false;
            SENS_task_delete(xDepth);
            SENS_sensor_depth_off();
            Event = MODE_DONE;
        }
        vTaskDelay(period);
    }

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: fast, FreeRTOS HEAP SIZE = %u Bytes\n", size);

    /* done here */
    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: fast, Task->finished\n\n");
    SendEvent(EventQueue_test, &Event);
    vTaskDelete(NULL);
}

void module_sensors_normal(void)
{
    Event_e_test Event;
    rtc_time time;

    float s_rate = 1.0f;
    uint32_t period = xDelay1000ms/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);

    SENS_sensor_temperature_on();
    SENS_sensor_depth_on();

    TaskHandle_t xDepth = NULL;
    TaskHandle_t xTemp  = NULL;
    vTaskDelay(xDelay100ms);
    SENS_task_park_sensors(&xDepth, &xTemp);
    bool run = true;

    /**  Monitor depth until we get there */
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    float Temperature = 0.0;

    uint32_t normal_time = 0;
    uint32_t wait_time = 0;
    normal_time = (xDelay1000ms * NORMAL_TIME);
    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: normal, < NROMAL_TIME = %.2f mins >\n", (float)(NORMAL_TIME/60.0));

    while (run)
    {
        /* check on depth sensor */
        SENS_get_depth(&Depth, &Pressure, &Rate);
        SENS_get_temperature(&Temperature);
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: normal, Pressure    = %.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: normal, Depth       = %.4f m, rate = %.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: normal, Temperature = %0.4f °C\n", Temperature);
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: normal, Epoch       = %ld\n", epoch);

        /* timer */
        wait_time += period;
        if (wait_time >= normal_time)
        {
            ARTEMIS_DEBUG_PRINTF("\nSENSORS_TEST :: normal, << Timer out %f mins >>\n", (float) (wait_time/(60.0*xDelay1000ms)));

            run = false;
            SENS_task_delete(xTemp);
            SENS_task_delete(xDepth);
            SENS_sensor_depth_off();
            SENS_sensor_temperature_off();
            Event = MODE_DONE;
        }
        vTaskDelay(period);
    }

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: normal, FreeRTOS HEAP SIZE = %u Bytes\n", size);

    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: normal, Task->finished\n\n");
    Event = MODE_DONE;
    //Event = MODE_NORMAL;
    SendEvent(EventQueue_test, &Event);
    vTaskDelete(NULL);
}

void module_sensors_slow(void)
{
    Event_e_test Event;
    rtc_time time;

    float s_rate = 1.0 / 10.0f;
    uint32_t period = xDelay1000ms/s_rate;

    SENS_set_depth_rate(s_rate);
    SENS_set_temperature_rate(s_rate);

    TaskHandle_t xDepth = NULL;
    TaskHandle_t xTemp  = NULL;
    vTaskDelay(xDelay100ms);

    bool run = true;
    float Depth = 0.0, Rate = 0.0;
    float Pressure = 0.0;
    float Temperature = 0.0;

    uint32_t slow_time = 0;
    uint32_t wait_time = 0;
    slow_time = (xDelay1000ms * SLOW_TIME);
    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: slow, < SLOW_TIME = %.2f mins >\n", (float)(SLOW_TIME/60.0));

    while (run)
    {
        /* turn on the sensors */
        SENS_sensor_temperature_on();
        SENS_sensor_depth_on();
        vTaskDelay(xDelay10ms);

        /* start the sensors task */
        SENS_task_park_sensors(&xDepth, &xTemp);
        SENS_get_depth(&Depth, &Pressure, &Rate);
        SENS_get_temperature(&Temperature);

        /* turn off the sensors */
        vTaskDelay(xDelay100ms);
        SENS_sensor_temperature_off();
        SENS_sensor_depth_off();

        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: slow, Pressure    = %.4f bar\n", Pressure);
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: slow, Depth       = %.4f m, rate = %.4fm/%.1fs\n", Depth, Rate, (float)(1/s_rate));
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: slow, Temperature = %0.4f °C\n", Temperature);
        artemis_rtc_get_time(&time);
        uint32_t epoch = get_epoch_time(time.year, time.month, time.day, time.hour, time.min, time.sec);
        ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: slow, Epoch       = %ld\n", epoch);

        /* timer */
        wait_time += period;
        if (wait_time >= slow_time)
        {
            ARTEMIS_DEBUG_PRINTF("\nSENSORS_TEST :: slow, << Timer out %f mins >>\n", (float) (wait_time/(60.0*xDelay1000ms)));
            run = false;
            Event = MODE_IDLE;
            //Event = MODE_NORMAL;
            vTaskDelay(xDelay1000ms);
            break;
        }
        vTaskDelay(period);
    }

    profileNr++;

#if defined(__TEST_PROFILE_1__) || defined(__TEST_PROFILE_2__)
        /* reset test profile */
        datalogger_read_test_profile(true);
#endif

    /* check Heap size */
    uint32_t size = xPortGetFreeHeapSize();
    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: slow, FreeRTOS HEAP SIZE = %u Bytes\n", size);

    ARTEMIS_DEBUG_PRINTF("SENSORS_TEST :: slow, Task->finished\n");
    SendEvent(EventQueue_test, &Event);
    vTaskDelete(NULL);
}


static void SendEvent(QueueHandle_t eventQueue, Event_e_test *event)
{
    xQueueSend(eventQueue, event, portMAX_DELAY);
}

static void ReceiveEvent(QueueHandle_t eventQueue, Event_e_test *event)
{
    xQueueReceive(eventQueue, event, portMAX_DELAY);
}
