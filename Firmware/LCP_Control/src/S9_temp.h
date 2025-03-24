
/**! @file S9_temp.h
 * @brief Sound Nine OEM Temperature Sensor
 *
 * @author Matt Casari, matthew.casari@noaa.gov
 * @date September 30, 2020
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the Sound Nine Temperature Sensors
 *
 *
 * @bug  No known bugs
 */
#ifndef _S9_TEMP_H
#define _S9_TEMP_H

/** Remove STATIC and PERSISTENT values if running TEST */
/** Add the actual values if running release */
#ifdef TEST
#ifndef STATIC
#define STATIC  
#endif
#ifndef PERSISTENT
#define PERSISTENT
#endif
#define __delay_cycles(x) 
#else
#ifndef STATIC
#define STATIC  static
#endif
#ifndef PERSISTENT
#define PERSISTENT __persistent 
#endif
#endif

/************************************************************************
*						STANDARD LIBRARIES
************************************************************************/
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/************************************************************************
*							HEADER FILES
************************************************************************/
#include "am_hal_status.h"
#include "am_hal_gpio.h"
#include "bsp_uart.h"
#include "am_hal_gpio.h"
/************************************************************************
*							MACROS
************************************************************************/

/************************************************************************
*							Structs & Enums
************************************************************************/
//typedef struct {
//  
//  
//}sS9Uart_t;
typedef struct {
    float temperature;
    float resistance;
    struct {
        char MID[8];
        float C0;
        float C1;
        float C2;
        float C3;
        float R0;
        uint8_t UID[16];
        char sensor[8];
        struct {
            uint8_t major;
            uint8_t minor;
        }firmware;
        char status[16];
        
    }info;
    struct {
        struct {
            am_hal_gpio_pincfg_t *pin;
            uint32_t pin_number;
        }power;
        struct {
            e_uart_t port;
            uint32_t baudrate;
        }uart;
    }device;

}sS9_t;


/************************************************************************
*					               Extern Variables
************************************************************************/

/************************************************************************
*					   Functions Prototypes
************************************************************************/
/** @brief S9 Temp Sensor Initialize
 * 
 * Initialize the S9 Temperature Sensor
 * 
 * @return None
 * 
 */
void S9T_init( const e_uart_t port, const am_hal_gpio_pincfg_t *power, const uint32_t power_pin);

/** @brief S9 Temp ON 
 * 
 * Turn the S9 Temperature Sensor ON
 * 
 * @return None
 * 
 */

void S9T_enable(void);
void S9T_disable(void);

void S9T_ON(void);

/** @brief S9 Temp OFF
 * 
 * Turn the S9 Temperature Sensor OFF
 * 
 * @return None
 * 
 */
void S9T_OFF(void);

/** @brief Read S9 Temperature
 * 
 * Read the S9 Temperature Sensor
 * 
 * @return Temperature (degC)
 */
float S9T_Read_T(void);

/** @brief Read S9 Resistance
 * 
 * Read the S9 Temperature Sensor Resistance
 * 
 * @return Resistance (ohms);
 */
float S9T_Read_R(void);

/** @brief Read S9 Temperature Sensor 
 * 
 * Read all parameters of the S9 Temp sensor
 * Temperature in degC
 * Resistance in Ohms
 * 
 * @param *t Pointer to temperature
 * @param *r Pointer to resistance
 * 
 * @return None
 */
float S9T_Read(float *t, float *r);

#ifdef TEST
STATIC void _parse_msg(char *data, uint8_t len, sS9_t *p);
STATIC void _parse_version(char *data, sS9_t *p );
#endif

#endif // _S9_TEMP_H
