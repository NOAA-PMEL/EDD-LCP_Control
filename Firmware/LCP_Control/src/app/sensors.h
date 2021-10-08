#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"


typedef struct sSensorType_t{
    int16_t value;
    bool data_valid;
    SemaphoreHandle_t semaphore;
}SensorType_t;

typedef struct sSensorData_t
{
    SensorType_t depth;         /** (int16_t) D_actual = depth / 1000 */
    SensorType_t temperature;   /** (int16_t) T_actual = temperature / 1000 */
}SensorData_t;




/**********************************************************************************
* Function Prototypes
*********************************************************************************/




#endif // SENSORS_H
