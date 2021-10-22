#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>
//#include "artemis_pa9ld.h"


typedef enum eTemperatureSensor_t{
    TEMP_Undefined,
    TEMP_SoundNine_OEM,
}TemperatureSensor_t;


typedef struct sTemperature_t
{   
    TemperatureSensor_t sensor;
     struct {
        uint8_t uart;          /**< UART Port */
    }uart;
    struct {
        uint32_t i2c;           /**< I2C Channel (IOM Module)*/
    }i2c;
//    struct {
//        module_manufacturer_t manufacturer; /**< Keller Manufacturer Info */
//        module_scaling_t scaling;           /**< Keller Scaling Coefficients */
//    }device;
}Temperature_t;

typedef struct sTemperature_Measurement_t
{
    float temperature;
}Temperature_Measurement_t;


/**********************************************************************************
* Function Prototypes
*********************************************************************************/

/** @brief Temperature Sensor Init
 *
 *	Initializes the Temperature Sensors
 *
 *  @return None
 */
void TEMP_initialize(void);

/** @brief Temperature Power On
 *
 *	Provide Power to the Temperature Sensor
 *
 *  @return None
 */
void TEMP_Power_ON(void);

/**
 * @brief Turn Temperature Sensor OFF
 * 
 * Disable power to Temperature Sensor
 */
void TEMP_Power_OFF(void);

/**
 * @brief Read the depth sensor.
 * 
 * Read the depth sensor and return relevant data
 * 
 * @param data Pointer to data struct
 */
void TEMP_Read(Temperature_Measurement_t *data);

#endif // TEMPERATURE_H
