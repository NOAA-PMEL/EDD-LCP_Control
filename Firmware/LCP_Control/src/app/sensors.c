#include "sensors.h"


#include "depth.h"
#include "temperature.h"

#define SENSOR_MAX_DEPTH_RATE           ( 10 ) 
#define SENSOR_MAX_TEMPERATURE_RATE     ( 2 ) 
#define SENSOR_MAX_GPS_RATE             ( 2 )
SensorData_t sensor_data;


bool SENS_get_depth(int16_t *depth)
{
    bool retVal = false;

    if( xSemaphoreTake(sensor_data.depth.semaphore) == pdTRUE)
    {
        sensor_data.depth.previous = sensor_data.depth.current;

        *depth = sensor_data.depth.current;
        retVal = true;
        xSemaphoreGive(sensor_data.depth.semaphore);
    }
    return retVal;
}


bool SENS_get_temperature(int16_t *temperature)
{
    bool retVal = false;

    if( sensor_data.temperature.current || xSemaphoreTake(sensor_data.temperature.semaphore) == pdTRUE)
    {
        *temperature = sensor_data.temperature.current;
        retVal = true;
        xSemaphoreGive(sensor_data.temperature.semaphore);
    }
    return retVal;
}

void SENS_task_profile_sensors(void)
{
    /** Start Depth Sensor @ 1 Hz */
    task_depth(1);

    /** Start Temperature Sensor @ 1Hz */
    task_temperature(1);

}

void SENS_task_park_sensors(void)
{
    /** Sample at 1/60th Hz */
    task_depth(1/60);
}

void SENS_task_sample_depth_continuous(void)
{
    /** Sample at 2 Hz */
    task_depth(2);


}


void SENS_set_depth_rate(uint16_t rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_DEPTH_RATE) )
    {
        sensor_data.depth.rate = rate;
    }
}

void SENS_set_temperature_rate(uint16_t rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_TEMPERATURE_RATE))
    {
        sensor_data.temperature.rate = rate;
    }
}

void SENS_set_gps_rate(uint16_t rate)
{
    if( (rate > 0) && (rate < SENSOR_MAX_GPS_RATE))
    {
        sensor_data.temperature.rate = rate;
    }
}

void task_depth(void)
{
    TickType_t xLastWakeTime;
    sDepth_Measurement_t depth = {0};
    assert(sensor_data.depth.rate != 0);
    uint16_t period = 1000/sensor_data.depth.rate;

    /** Create the semaphore for the depth sensor read */
    sensor_data.depth.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    DEPTH_initialize();

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        /** Power On */
        DEPTH_Power_ON();

        /** Delay to warm up */
        /** @todo replace with const number */
        vTaskDelay(50/ portTICK_RATE_MS);

        /** Read the Data */
        DEPTH_Read(&depth);

        /** Power Off */
        DEPTH_Power_OFF();
        
        
        if(xSemaphoreTake(sensor_data.depth.semaphore, period/portTICK_RATE_MS) == pdTRUE)
        {
            sensor_data.depth.previous = sensor_data.depth.current;
            sensor_data.depth.current = depth.Depth;
            sensor_data.depth.ascent_rate = (sensor_data.depth.current - sensor_data.depth.previous) / sensor_data.depth.rate;
            xSemaphoreGive(sensor_data.depth.semaphore);
        }

        vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_RATE_MS );
    }

}

void task_temperature(void)
{
    TickType_t xLastWakeTime;
    Temperature_Measurement_t temperature = {0};
    uint16_t period = 1000/rate;
    period /= portTICK_RATE_MS;

    /** Create the semaphore for the depth sensor read */
    sensor_data.temperature.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    TEMP_initialize();

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        /** Turn power on */
        TEMP_Power_ON();

        /** Warmup for x */
        /** @todo replace this with a macro */

        vTaskDelay(50/ portTICK_RATE_MS);

        /** Read the temperature */
        TEMP_Read(&temperature);

        /** Power off */
        TEMP_Power_OFF();
        
        if(xSemaphoreTake(sensor_data.temperature.semaphore, 10/portTICK_RATE_MS) == pdTRUE)
        {
            sensor_data.temperature.current = depth.Depth;
            xSemaphoreGive(sensor_data.temperature.semaphore);
        }

        vTaskDelayUntil( &xLastWakeTime, period );
    }

}




void task_GPS(uint8_t rate)
{

    TickType_t xLastWakeTime;
    Temperature_Measurement_t temperature = {0};
    uint16_t period = 1000/rate;
    GPS_Data_t gps = {0};

    /** Create the semaphore for the depth sensor read */
    sensor_data.GPS.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    GPS_initialize();



    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();


    while(1)
    {

        if(GPS_Read(&gps))
        {
            if(xSemaphoreTake(sensor_data.GPS.semaphore, 10/portTICK_RATE_MS)==pdTRUE)
            {
                sensor_data.GPS.fix = true;
                sensor_data.GPS.latitude = gps.lat;
                sensor_data.GPS.longitude = gps.lon;
                sensor_data.GPS.altitude = gps.alt;
            }
        }
    
        vTaskDelayUntil( &xLastWakeTime, period );
    }

}