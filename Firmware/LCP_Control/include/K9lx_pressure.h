/**! @file artemis_k9lx.h
 * @brief Maxim SPI-to-UART Converter
 *
 * @author Basharat Martin basharat.martin@noaa.gov
 * @date Feb 23, 2023
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the Keller 9LXe, circuit board (9L140) Pressure sensor over RS-485
 * 
 *
 *
 * @bug  No known bugs
 */

#ifndef K9LX_H_PRESSURE
#define K9LX_H_PRESSURE

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	K9LX_MODE_PR   = 0,        /**< Mode: Vented Gauge. Zero at atmospheric pressure */
	K9LX_MODE_PA   = 1,        /**< Mode: Sealed Gauge. Zero at 1.0 bar abs */
	K9LX_MODE_PAA  = 2,        /**< Mode: Absolute.  Zero at vacuum */ 
	K9LX_MODE_NONE = 3         /**< Mode: not defined */
}eK9LX_PMode_t;

typedef struct s_module_scaling_t
{
	float low;
	float high;
	float diff;
}module_scaling_t;

typedef struct s_module_manufacturer_t{
	uint32_t custom_id;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	eK9LX_PMode_t mode;
}module_manufacturer_t;

void K9lx_init(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate);
void K9lx_power_on(void);
void K9lx_power_off(void);
void K9lx_read(float *pressure, float *temperature);

#ifdef __cplusplus
}
#endif

#endif // K9LX_H_PRESSURE

