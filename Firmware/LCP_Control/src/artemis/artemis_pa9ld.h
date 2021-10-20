/**! @file artemis_pa9ld.h
 * @brief Maxim SPI-to-UART Converter
 *
 * @author Matt Casari, matthew.casari@noaa.gov
 * @date August 11, 2021
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the Keller PA-9LD Pressure sensor over I2C
 *
 *
 * @bug  No known bugs
 */
#ifndef ARTEMIS_PA9LD_H
#define ARTEMIS_PA9LD_H



#include <stdint.h>
   
#include "am_hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PA9LD_MODE_PR   = 0,        /**< Mode: Vented Gauge. Zero at atmospheric pressure */
    PA9LD_MODE_PA   = 1,        /**< Mode: Sealed Gauge. Zero at 1.0 bar abs */
    PA9LD_MODE_PAA  = 2,        /**< Mode: Absolute.  Zero at vacuum */ 
    PA9LD_MODE_NONE = 3         /**< Mode: not defined */
}ePA9LD_PMode_t;

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
  ePA9LD_PMode_t mode;
}module_manufacturer_t;

void artemis_pa9ld_initialize(const am_hal_gpio_pincfg_t *power, 
                              const uint32_t power_pin);
void artemis_pa9ld_power_on(void);
void artemis_pa9ld_power_off(void);
void artemis_pa9ld_read(float *pressure, float *temperature);
void artemis_pa9ld_get_calibration(module_manufacturer_t *manufacturer, 
                                  module_scaling_t *scaling );

#ifdef __cplusplus
}
#endif






#endif