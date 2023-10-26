/**********************************************************************************
* @Title       :   LCP Control Depth Application
* @Filename    :   depth.h
* @Author      :   Matt Casari
* @Origin Date :   1/16/2021
* @Version     :   1.0.0
* @Compiler    :   IAR, GCC
* @Target      :   
* @Notes       :   None
* @Bugs        :   No known bugs
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights 
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*********************************************************************************/
/**********************************************************************************
* Build Log
* Data    | Initials |  Description
*--------------------------------
* 6/30/21 |    MJC   | 
*********************************************************************************/

#ifndef DEPTH_H_
#define DEPTH_H_

/**********************************************************************************
* Includes
*********************************************************************************/
#include "bsp_uart.h"
//#include "artemis_pa9ld.h"
#include "artemis_max14830.h"
#include "MAX14830.h"

/**********************************************************************************
* Configuration Constants
*********************************************************************************/

/**********************************************************************************
* MACROS
*********************************************************************************/
#define TYPICAL_DENSITY_OF_SALTWATER    (1036.0f)

/**********************************************************************************
* Typdefs
*********************************************************************************/
/** @enum Depth Sensor Type
 * These are the types of Depth Sensors supported by
 * this driver.
 */
typedef enum {
    DEPTH_Keller_Not_Configured = 0,
    DEPTH_Keller_PA9LD = 1,
    DEPTH_Keller_PR9LX = 2
}eDEPTH_Sensor_t;

/** @struct Depth Struct
 * This is the primary depth structure which contains
 * the different settings and scaling factors.
 */
typedef struct {
    eDEPTH_Sensor_t sensor;     /**< Sensor Type */
    struct {
        //e_uart_t uart;          /**< UART Port */
        eMAX18430_ComPort_t uart; /**< UART Port */
    }uart;
    //struct {
    //    uint32_t i2c;           /**< I2C Channel (IOM Module)*/
    //}i2c;
    //struct {
    //    module_manufacturer_t manufacturer; /**< Keller Manufacturer Info */
    //    module_scaling_t scaling;           /**< Keller Scaling Coefficients */
    //}device;
    struct {
        float density;          /**< Density of Water */
    }conversion;
}sDepth_t;


/** @struct Depth Measurement Struct
 * Struct to use for reading data out of sensor into 
 * other apps.
 */
typedef struct {
    float Depth;                /**< Measured Depth (m)*/
    float Pressure;             /**< Measured Pressure (bar) */
    float Temperature;          /**< Measured Temperature (C) */
}sDepth_Measurement_t;

/**********************************************************************************
* Function Prototypes
*********************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

/** @brief Depth Sensor Init
 *
 *	Initializes the Depth Sensors
 *
 *  @return None
 */
void DEPTH_initialize(eDEPTH_Sensor_t sensor);

/** @brief Depth Power On
 *
 *	Provide Power to the Depth Sensor
 *
 *  @return None
 */
void DEPTH_Power_ON(void);

/**
 * @brief Turn Depth Sensor OFF
 * 
 * Disable power to Depth Sensor
 */
void DEPTH_Power_OFF(void);

/**
 * @brief Read the depth sensor.
 * 
 * Read the depth sensor and return relevant data
 * 
 * @param data Pointer to data struct
 */
void DEPTH_Read(sDepth_Measurement_t *data);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // DEPTH_H_
