/**! @file S9_temp.h
 * @brief Sound Nine OEM Temperature Sensor
 *
 * @author Matt Casari, matthew.casari@noaa.gov
 * @date September 30, 2020
 * @version 1.1.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the Sound Nine Temperature Sensors
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
-                        STANDARD LIBRARIES
************************************************************************/
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/************************************************************************
-                        AMBIG & DRIVER HEADERS
************************************************************************/
#include "am_mcu_apollo.h"
#include "MAX14830.h"

/************************************************************************
-                              MACROS
************************************************************************/
#define SHORT_RESPONSE_BUFFER_SIZE  16

/************************************************************************
-                          Structs & Enums
************************************************************************/
typedef struct
{
    float temperature;
    float resistance;
    struct
    {
        char MID[12];           /**< Increased to 12 bytes for null safety */
        float C0;
        float C1;
        float C2;
        float C3;
        float R0;
        uint8_t average;
        char UID[34];           /**< 32-char UID + null terminator */
        char sensor[16];        /**< 10-12 char firmware model + null */
        struct
        {
            uint8_t major;
            uint8_t minor;
        } firmware;
        char status[4];         /**< "OK" / "ER" + null terminator */
    } info;

    struct
    {
        struct
        {
            const am_hal_gpio_pincfg_t *pin;
            uint32_t pin_number;
        } power;

        struct
        {
            eMAX18430_ComPort_t port;        /**< Matches MAX14830 port enum */
            eMAX14830_Baudrate_t baudrate;   /**< Matches MAX14830 baud enum */
        } uart;
    } device;

} sS9_t;

typedef enum eS9_result_t
{
    S9_RESULT_FAIL  = 0,
    S9_RESULT_OK    = 1,
    S9_RESULT_ERROR = 2
} S9_result_t;

typedef struct
{
    eMAX18430_ComPort_t port;
    eMAX14830_Baudrate_t baudrate;
    const am_hal_gpio_pincfg_t *pin_config;
    uint32_t pin_number;
} S9_init_param;

/************************************************************************
-                         Function Prototypes
************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Initialize the S9 Temperature Sensor hardware and configuration.
 * @param p Pointer to initialization parameter structure.
 */
void S9T_init(S9_init_param *p);

/** 
 * @brief Query and parse device information, calibration coefficients, and UID.
 */
void S9T_dev_info(void);

/** 
 * @brief Enable the sensor power gate.
 */
void S9T_enable(void);

/** 
 * @brief Disable the sensor power gate.
 */
void S9T_disable(void);

/** 
 * @brief Direct drive pin to energize S9 sensor rail.
 */
void S9T_ON(void);

/** 
 * @brief Direct drive pin to de-energize S9 sensor rail.
 */
void S9T_OFF(void);

/** 
 * @brief Read temperature in degrees Celsius.
 * @return Temperature (degC), or NAN if read fails.
 */
float S9T_Read_T(void);

/** 
 * @brief Read thermistor resistance in Ohms.
 * @return Resistance (Ohms), or NAN if read fails.
 */
float S9T_Read_R(void);

/** 
 * @brief Polled read of both temperature and resistance.
 * @param *t Output pointer for temperature (degC). Can be NULL.
 * @param *r Output pointer for resistance (Ohms). Can be NULL.
 * @return Temperature (degC), or NAN if read fails.
 */
float S9T_Read(float *t, float *r);

/** 
 * @brief Stops autonomous continuous streaming in FreeRTOS environment.
 */
void _module_s9_stop_sampling_RTOS(void);

#ifdef __cplusplus
}
#endif

#endif // _S9_TEMP_H