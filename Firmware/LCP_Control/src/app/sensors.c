#include "sensors.h"


#include "depth.h"

SensorData_t sensor_data;


bool SENS_get_depth(int16_t *depth)
{
    bool retVal = false;

    if( sensor_data.depth.value || xSemaphoreTake(sensor_data.depth.semaphore) == pdTRUE)
    {
        *depth = sensor_data.depth.value;
        retVal = true;
        xSemaphoreGive(sensor_data.depth.semaphore);
    }
    return retVal;
}


bool SENS_get_temperature(int16_t *temperature)
{
    bool retVal = false;

    if( sensor_data.temperature.value || xSemaphoreTake(sensor_data.temperature.semaphore) == pdTRUE)
    {
        *temperature = sensor_data.temperature.value;
        retVal = true;
        xSemaphoreGive(sensor_data.temperature.semaphore);
    }
    return retVal;
}

void SENS_task_profile_sensors(void)
{
    /** Start Depth Sensor @ 4 Hz */
    task_depth(4);

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


void task_depth(uint8_t rate)
{
    sDepth_Measurement_t depth = {0};
    uint16_t period = 1000/rate;

    /** Create the semaphore for the depth sensor read */
    sensor_data.depth.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    DEPTH_initialize();

    while(1)
    {
        DEPTH_Read(&depth);
        
        if(xSemaphoreTake(sensor_data.depth.semaphore, period/portTICK_RATE_MS) == pdTRUE)
        {
            sensor_data.depth.value = depth.Depth;
            xSemaphoreGive(sensor_data.depth.semaphore);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

}

void task_temperature(uint8_t rate)
{
    Temperature_Measurement_t temperature = {0};
    uint16_t period = 1000/rate;

    /** Create the semaphore for the depth sensor read */
    sensor_data.temperature.semaphore = xSemaphoreCreateMutex();

    /** Initialize the Depth Sensor */
    TEMP_initialize();

    while(1)
    {
        TEMP_Read(&temperature);
        
        if(xSemaphoreTake(sensor_data.temperature.semaphore, period/portTICK_RATE_MS) == pdTRUE)
        {
            sensor_data.temperature.value = depth.Depth;
            xSemaphoreGive(sensor_data.temperature.semaphore);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

}


