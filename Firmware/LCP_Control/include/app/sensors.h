#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"
#include "rtos.h"
#include "timers.h"

typedef struct sSensorType_t{
    int16_t value;
    bool data_valid;
    SemaphoreHandle_t semaphore;
}SensorType_t;

typedef struct sSensorGps_t {        
        uint16_t rate;          /**< Sample rate (Hz) */
        bool fix;
        float latitude;
        float longitude;
        float altitude;
        uint16_t year;  /**< Calendar Year UTC */
        uint8_t month;  /**< Calendar Month UTC */
        uint8_t day;    /**< Calendar Day UTC */
        uint8_t hour;   /**< Hour UTC */    
        uint8_t min;    /**< Minute UTC */
        uint8_t sec;    /**< Second UTC */
        bool data_valid;
        SemaphoreHandle_t semaphore;
}SensorGps_t;

typedef struct sSensorData_t
{
    struct{
        float rate;             /**< Sample rate (Hz) */
        float current;          /**< Current depth (m) */
        float previous;         /**< Previous depth (m) */
        float ascent_rate;      /**< Calculated ascnet rate (m/s) */
        bool data_valid;
        SemaphoreHandle_t semaphore;
    }depth;
    struct{
        float rate;             /**< Sample rate (Hz) */
        float current;          /**< Current pressure (bar) */
        float previous;         /**< Previous pressure (bar) */
        float ascent_rate;      /**< Calculated ascnet rate (m/s) */
        bool data_valid;
    }pressure;
    struct {
        float rate;             /**< Sample rate (Hz) */
        float current;          /**< (int16_t) T_actual = temperature / 1000 */
        bool data_valid;
        SemaphoreHandle_t semaphore;
    }temperature;
    SensorGps_t gps;
}SensorData_t;

/**********************************************************************************
* Function Prototypes
*********************************************************************************/

void task_depth(void);
void task_temperature(void);
void task_gps(void);

void SENS_sensor_depth_off(void);
void SENS_sensor_depth_on(void);

void SENS_sensor_gps_off(void);
void SENS_sensor_gps_on(void);

void SENS_sensor_temperature_off(void);
void SENS_sensor_temperature_on(void);

bool SENS_get_depth(float *depth, float *pressure, float *rate);
//bool SENS_get_depth(float *depth, float *rate);
bool SENS_get_temperature(float *temperature);
bool SENS_get_gps(SensorGps_t *gps);

//void SENS_get_depth(void *Depth_s);
//void SENS_get_temperature(void *temperature);
//void SENS_get_gps(void *gps);

//void SENS_task_profile_sensors(void);
//void SENS_task_park_sensors(void);
//void SENS_task_sample_depth_continuous(void);

void SENS_task_profile_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp);
void SENS_task_park_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp);
void SENS_task_sample_depth_continuous(TaskHandle_t *xDepth);
void SENS_task_gps(TaskHandle_t *xGPS);
void SENS_task_delete(TaskHandle_t xHandle);

void SENS_set_depth_rate(float rate);
void SENS_set_temperature_rate(float rate);
void SENS_set_gps_rate(uint16_t rate);

bool SENS_initialize(void);
void SENS_uninitialize(void);

#endif // SENSORS_H
