/**! @file artemis_accel.h
 * @brief STM Accelerometer
 *
 * @author Basharat Martin basharat.martin@noaa.gov
 * @date April 24, 2023
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the STM LIS2DW12 Acceleration sensor over SPI
 * 
 *
 * @bug  No known bugs
 *
 **/

#ifndef ARTEMIS_ACCEL_H
#define ARTEMIS_ACCEL_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void artemis_accel_init(void);

#ifdef __cplusplus
}
#endif

#endif // ARTEMIS_ACCEL_H


