/**
 * @file artemis_i9603n.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-30
 * 
 */
#ifndef ARTEMIS_I9603N_H
#define ARTEMIS_I9603N_H


/************************************************************************
*							HEADER FILES
************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/************************************************************************
*						STANDARD LIBRARIES
// ************************************************************************/
// #include "artemis_i2c.h"

/************************************************************************
*							MACROS
************************************************************************/
#define ARTEMIS_UBLOX_I2C_DATA_LEN_REG  ( 0xFD )
#define ARTEMIS_UBLOX_I2C_DATA_REG      ( 0xFF )
/************************************************************************
*							CONSTS
************************************************************************/

/************************************************************************
*							ENUM & STRUCTS
************************************************************************/

/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/

void artemis_i9603n_initialize(void);
void artemis_i9603n_power_on(void);
void artemis_i9603n_power_off(void);
bool artemis_i9603n_is_network_available(void);
bool artemis_i9603n_is_ringing(void);
void artemis_i9603n_send(char *msg, uint16_t len);
uint16_t artemis_i9603n_receive(char *msg, uint16_t bufLen);


#endif // ARTEMIS_I9603N_H
