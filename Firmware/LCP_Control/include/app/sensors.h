#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "GPS.h"

typedef struct
{
    float current;
    float previous;
    float ascent_rate;
    float rate;
} SensorDepth_t;

typedef struct
{
    float current;
    float previous;
} SensorPressure_t;

typedef struct
{
    float current;
    float rate;
} SensorTemperature_t;

typedef struct
{
    SemaphoreHandle_t semaphore;
    bool fix;
    float latitude;
    float longitude;
    float altitude;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint16_t rate;
} SensorGps_t;

typedef struct
{
    SensorDepth_t depth;
    SensorPressure_t pressure;
    SensorTemperature_t temperature;
    SensorGps_t gps;
} SensorData_t;

/************************************************************************
 * Function Prototypes
 ************************************************************************/

bool SENS_initialize(void);
void SENS_uninitialize(void);

void SENS_sensor_depth_on(void);
void SENS_sensor_depth_off(void);
void SENS_sensor_temperature_on(void);
void SENS_sensor_temperature_off(void);
void SENS_sensor_gps_on(void);
void SENS_sensor_gps_off(void);

bool SENS_get_depth(float *depth, float *pressure, float *rate);
bool SENS_get_temperature(float *temperature);
bool SENS_get_gps(SensorGps_t *gps);

void SENS_set_depth_rate(float rate);
void SENS_set_temperature_rate(float rate);
void SENS_set_gps_rate(uint16_t rate);

void SENS_task_profile_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp);
void SENS_task_park_sensors(TaskHandle_t *xDepth, TaskHandle_t *xTemp);
void SENS_task_sample_depth_continuous(TaskHandle_t *xDepth);
void SENS_task_gps(TaskHandle_t *xGPS);
void SENS_task_delete(TaskHandle_t xHandle);

bool GPS_running(void);
void killGPS(void);

void task_depth(void);
void task_temperature(void);
void task_gps(void);

#endif // SENSORS_H