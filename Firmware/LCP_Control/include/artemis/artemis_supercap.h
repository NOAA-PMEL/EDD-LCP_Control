/**
 * @file artemis_supercap.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief
 * @version 0.1
 * @date 2021-09-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef ARTEMIS_SUPERCAP_H
#define ARTEMIS_SUPERCAP_H

#ifdef __cplusplus
extern "C" {
#endif
/************************************************************************
*							HEADER FILES
************************************************************************/
#include <stdbool.h>

/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/
void artemis_sc_initialize(void);
bool artemis_sc_power_startup(void);
void artemis_sc_power_on(void);
void artemis_sc_power_off(void);
bool artemis_sc_power_good(void);

#ifdef __cplusplus
}
#endif

#endif // ARTEMIS_SUPERCAP_H
