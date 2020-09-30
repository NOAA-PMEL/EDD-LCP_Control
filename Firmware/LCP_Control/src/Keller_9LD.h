/** @file Keller_9LD.h
* @brief Keller Series 9LD OEM Pressure Sensor
*
* @author Matt Casari, matthew.casari@noaa.gov
* @date September 29, 2020
* @version 1.0.0
*
* @copyright National Oceanic and Atmospheric Administration
* @copyright Pacific Marine Environmental Lab
* @copyright Environmental Development Division
*
* @note Controls the Keller 9LD Pressure Sensors
*
*
* @bug  No known bugs
*/
#ifndef _Keller_9LD_H
#define _Keller_9LD_H

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

/************************************************************************
*							HEADER FILES
************************************************************************/

/************************************************************************
*							MACROS
************************************************************************/

/************************************************************************
*							Structs & Enums
************************************************************************/
typedef struct Keller_9LD
{
    /* data */
    uint32_t id;
    struct {
        uint16_t year;
        uint8_t month;
        uint8_t day;
    }date;
    float p_min;
    float p_max;
}sKeller_9LD_t;

/************************************************************************
*					               Extern Variables
************************************************************************/

/************************************************************************
*					   Functions Prototypes
************************************************************************/

#ifdef TEST
STATIC void _convert_user_information_bytes_to_struct( uint8_t *data, sKeller_9LD_t *p);
STATIC float _convert_bytes_to_fp(uint8_t *data);
STATIC void _convert_bytes_to_cal_date(
                                        uint8_t *data,
                                        uint16_t *year,
                                        uint8_t *month,
                                        uint8_t *day
                                        );
STATIC uint32_t _convert_product_codes( uint8_t *data );
#endif
#endif // _Keller_9LD_H
