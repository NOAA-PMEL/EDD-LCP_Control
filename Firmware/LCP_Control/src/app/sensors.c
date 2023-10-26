#include "sensors.h"

#include "depth.h"
#include "temperature.h"
#include "GPS.h"
#include "artemis_max14830.h"
#include "artemis_debug.h"
#include "MAX14830.h"
#include "sysinfo.h"

#define SENSOR_MAX_DEPTH_RATE           ( 10 )
#define SENSOR_MAX_TEMPERATURE_RATE     ( 2 )
#define SENSOR_MAX_GPS_RATE             ( 2 )

//static SemaphoreHandle_t xDepthSemaphore = NULL;
//static SemaphoreHandle_t xTempSemaphore = NULL;
static SemaphoreHandle_t xTDSemaphore = NULL;

static SensorData_t sensor_data;
//static SensorData_t sensor_data = {
//  .depth.semaphore = &xDepthSemaphore,
//  .temperature.semaphore = &xTempSemaphore
//};

void SENS_initialize(void)
{
    // LCP system information
    SYS_lcp_info();

    // initialize MAX14830 chip for the sensors
    //artemis_max14830_init();
    MAX14830_init();

    // initialize pressure sensor
    DEPTH_initialize(DEPTH_Keller_PR9LX);

    // initialize temperature sensor
    TEMP_initialize(TEMP_SoundNine_OEM);

    // initialize GPS
    GPS_initialize();

    ARTEMIS_DEBUG_PRINTF("Sensors are initialized\n\n");

    // create a single semaphore for Temperature and
    // Pressure sensors for now
    xTDSemaphore = xSemaphoreCreateMutex();

    if (xTDSemaphore == NULL)
    {
        ARTEMIS_DEBUG_PRINTF("ERROR :: xTDSemaphore is NULL \n\n");
    }
}

void SENS_sensor_depth_off(void)        { DEPTH_Power_OFF();    }
void SENS_sensor_depth_on(void)         { DEPTH_Power_ON();     }
void SENS_sensor_gps_off(void)          { GPS_off();            }
void SENS_sensor_gps_on(void)           { GPS_on();             }
void SENS_sensor_temperature_off(void)  { TEMP_Power_OFF();     }
void SENS_sensor_temperature_on(void)   { TEMP_Power_ON();      }

bool SENS_get_depth(float *depth, float *rate)
{
    bool retVal = false;

    //if( xSemaphoreTake(sensor_data.depth.semaphore, 10/portTICK_PERIOD_MS) == pdTRUE)
    //{
    //    *depth = sensor_data.depth.current;
    //    *rate = sensor_data.depth.ascent_rate;
    //    retVal = true;
    //    xSemaphoreGive(sensor_data.depth.semaphore);
    //}

    configASSERT( xSemaphoreTake(xTDSemaphore, pdMS_TO_TICKS(10UL)) == pdTRUE );
    *depth = sensor_data.depth.current;
    *rate = sensor_data.depth.ascent_rate;
    retVal = true;
    xSemaphoreGive(xTDSemaphore);

    return retVal;
}

bool SENS_get_temperature(float *temperature)
{
    bool retVal = false;

    //if( xSemaphoreTake(sensor_data.temperature.semaphore, 10/portTICK_PERIOD_MS) == pdTRUE)
    //{
    //    *temperature = sensor_data.temperature.current;
    //    retVal = true;
    //    xSemaphoreGive(sensor_data.temperature.semaphore);
    //}

    configASSERT( xSemaphoreTake(xTDSemaphore, pdMS_TO_TICKS(10UL)) == pdTRUE );
    *temperature = sensor_data.temperature.current;
    retVal = true;
    xSemaphoreGive(xTDSemaphore);
    return retVal;
}

bool SENS_get_gps(SensorGps_t *gps)
{
    bool retVal = false;

    //if(  xSemaphoreTake(sensor_data.gps.semaphore, 10/portTICK_PERIOD_MS) == pdTRUE)
    //{
    //    *gps = sensor_data.gps;
    //    retVal = true;
    //    xSemaphoreGive(sensor_data.gps.semaphore);
    //}

    configASSERT( xSemaphoreTake(sensor_data.gps.semaphore, pdMS_TO_TICKS(10UL)) == pdTRUE );
    *gps = sensor_data.gps;
    retVal = true;
    xSemaphoreGive(sensor_data.gps.semaphore);

    return retVal;
}

void SENS_task_profile_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp)
{
    /** Start Depth Sensor @ 1 Hz */
    SENS_set_depth_rate(1.0);
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );

    /** Start Temperature Sensor @ 1Hz */
    SENS_set_temperature_rate(1);
    configASSERT(xTaskCreate((TaskFunction_t)task_temperature,
                                "Temperature_Task", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xTemp) == pdPASS );
}

void SENS_task_park_sensors(TaskHandle_t *xDepth)
{
    /** Sample at 1/60th Hz */
    float rate = 1.0/60.0;
    SENS_set_depth_rate(rate);
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );
}

void SENS_task_sample_depth_continuous(TaskHandle_t *xDepth)
{
    /** Sample at 2 Hz */
    SENS_set_depth_rate(2.0);
    configASSERT(xTaskCreate((TaskFunction_t)task_depth,
                                "Depth_Task", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xDepth) == pdPASS );
}

void SENS_task_gps(TaskHandle_t *xGPS)
{
    SENS_set_gps_rate(1);
    configASSERT(xTaskCreate((TaskFunction_t)task_gps,
                                "GPS_Task", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xGPS) == pdPASS );
}

void SENS_set_depth_rate(float rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_DEPTH_RATE) )
    {
        ARTEMIS_DEBUG_PRINTF("Setting Depth rate = %.5f\n", rate);
        sensor_data.depth.rate = rate;
    }
}

void SENS_set_temperature_rate(uint16_t rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_TEMPERATURE_RATE))
    {
        ARTEMIS_DEBUG_PRINTF("Setting Temperature rate = %u\n", rate);
        sensor_data.temperature.rate = rate;
    }
}

void SENS_set_gps_rate(uint16_t rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_GPS_RATE))
    {
        ARTEMIS_DEBUG_PRINTF("Setting GPS rate = %u\n", rate);
        sensor_data.gps.rate = rate;
    }
}

void task_depth(void)
{
    TickType_t xLastWakeTime;
    sDepth_Measurement_t depth = {0};
    //assert(sensor_data.depth.rate != 0);
    //uint16_t period = 1000/sensor_data.depth.rate;
    //period /= portTICK_PERIOD_MS;

    uint16_t period = pdMS_TO_TICKS(1000UL)/sensor_data.depth.rate;
    ARTEMIS_DEBUG_PRINTF("Setting Depth period = %u\n", period);

    /** Create the semaphore for the depth sensor read */
    //*sensor_data.depth.semaphore = xSemaphoreCreateMutex();
    //sensor_data.depth.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    //DEPTH_initialize(DEPTH_Keller_PA9LD);

    //Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        /** Power On */
        //DEPTH_Power_ON();

        /** Delay to warm up */
        /** @todo replace with const number */
        //vTaskDelay(50/ portTICK_PERIOD_MS);

        /** Read the Data */
        //DEPTH_Read(&depth);

        /** Power Off */
        //DEPTH_Power_OFF();
        
        //if(xSemaphoreTake(sensor_data.depth.semaphore, period/portTICK_PERIOD_MS) == pdTRUE)
        configASSERT(xSemaphoreTake(xTDSemaphore, pdMS_TO_TICKS(1000UL)) == pdTRUE);

        am_hal_gpio_output_toggle(AM_BSP_GPIO_LED_GREEN);
        DEPTH_Read(&depth);
        ARTEMIS_DEBUG_PRINTF("Depth = %.3f\n", depth.Depth);
        sensor_data.depth.previous = sensor_data.depth.current;
        sensor_data.depth.current = depth.Depth;
        sensor_data.depth.ascent_rate = (sensor_data.depth.current - sensor_data.depth.previous) / sensor_data.depth.rate;
        //xSemaphoreGive(sensor_data.depth.semaphore);
        xSemaphoreGive(xTDSemaphore);

        //vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS );
        vTaskDelayUntil(&xLastWakeTime, period);
        //vTaskDelete(NULL);
    }
}

void task_temperature(void)
{
    TickType_t xLastWakeTime;
    Temperature_Measurement_t temperature = {0};
    //uint16_t period = 1000/sensor_data.temperature.rate;
    //period /= portTICK_PERIOD_MS;
    uint16_t period = pdMS_TO_TICKS(1000UL)/sensor_data.temperature.rate;

    /** Create the semaphore for the depth sensor read */
    //sensor_data.temperature.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    //TEMP_initialize();

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        /** Turn power on */
        //TEMP_Power_ON();

        /** Warmup for x */
        /** @todo replace this with a macro */

        //vTaskDelay(50/ portTICK_PERIOD_MS);

        /** Read the temperature */
        //TEMP_Read(&temperature);

        /** Power off */
        //TEMP_Power_OFF();
        
        configASSERT(xSemaphoreTake(xTDSemaphore, pdMS_TO_TICKS(1000UL)) == pdTRUE);
        //if(xSemaphoreTake(sensor_data.temperature.semaphore, period) == pdTRUE)
        //uint32_t size = xPortGetFreeHeapSize();
        //ARTEMIS_DEBUG_PRINTF("\nTask Temperature :: HEAP size %d\n", size);
        am_hal_gpio_output_toggle(AM_BSP_GPIO_LED_BLUE);
        TEMP_Read(&temperature);
        ARTEMIS_DEBUG_PRINTF("Temperature = %.3f °C\n", temperature.temperature);
        sensor_data.temperature.current = temperature.temperature;
        xSemaphoreGive(xTDSemaphore);
        //xSemaphoreGive(sensor_data.temperature.semaphore);
        //vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS );
        vTaskDelayUntil(&xLastWakeTime, period);
        //vTaskDelete(NULL);
    }
}

void task_gps(void)
{
    TickType_t xLastWakeTime;
    uint16_t period = pdMS_TO_TICKS(1000UL)/sensor_data.gps.rate;
    GPS_Data_t gps = {0};

    /** Create the semaphore for the gps */
    sensor_data.gps.semaphore = xSemaphoreCreateMutex();

    /** Initialize the GPS */
    //GPS_initialize();
    //GPS_on();
    vTaskDelay(pdMS_TO_TICKS(300UL));

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {

        if(GPS_Read(&gps))
        {
            configASSERT(xSemaphoreTake(sensor_data.gps.semaphore, pdMS_TO_TICKS(1000UL)) == pdTRUE);
            //configASSERT(xSemaphoreTake(sensors, pdMS_TO_TICKS(100UL)) == pdTRUE);

            am_hal_gpio_output_toggle(AM_BSP_GPIO_LED_RED);
            ARTEMIS_DEBUG_PRINTF("GPS : Time, %d:%02d:%02d\n", gps.time.hour, gps.time.min, gps.time.sec);
            ARTEMIS_DEBUG_PRINTF("GPS : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.2f\n", gps.position.lat, gps.position.lon, gps.position.alt);
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
        }
        else
        {
            configASSERT(xSemaphoreTake(sensor_data.gps.semaphore, pdMS_TO_TICKS(1000UL)) == pdTRUE);
            am_hal_gpio_output_set(AM_BSP_GPIO_LED_RED);
            ARTEMIS_DEBUG_PRINTF("GPS : No fix \n");
            xSemaphoreGive(sensor_data.gps.semaphore);
        }

        //uint32_t size = xPortGetFreeHeapSize();
        //ARTEMIS_DEBUG_PRINTF("\nTask GPS :: HEAP size %d\n", size);
        vTaskDelayUntil(&xLastWakeTime, period);
    }
}

