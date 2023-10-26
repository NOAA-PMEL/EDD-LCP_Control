/**
 * @file artemis_piston.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-14
 * 
 * 
 */
#ifndef ARTEMIS_PISTON_H
#define ARTEMIS_PISTON_H


/** Remove STATIC and PERSISTENT values if running TEST */
/** Add the actual values if running release */
#ifdef TEST
#ifndef STATIC
#define STATIC  
#endif
#ifndef PERSISTENT
#define PERSISTENT
#endif
#else
#ifndef STATIC
#define STATIC  static
#endif
#ifndef PERSISTENT
#define PERSISTENT __persistent 
#endif
#endif

/************************************************************************
*							HEADER FILES
************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/************************************************************************
*						STANDARD LIBRARIES
************************************************************************/
#include "artemis_i2c.h"

/************************************************************************
*							MACROS
************************************************************************/
#define ARTEMIS_PISTON_I2C_DATA_LEN_REG  ( 0xFD )
#define ARTEMIS_PISTON_I2C_DATA_REG      ( 0xFF )
/************************************************************************
*							CONSTS
************************************************************************/

/************************************************************************
*							ENUM & STRUCTS
************************************************************************/

/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/

void artemis_piston_i2c_initialize(uint8_t i2c_addr);
void artemis_piston_i2c_power_on(void);
void artemis_piston_i2c_power_off(void);
void artemis_piston_i2c_send_msg(uint8_t *msg, uint16_t len, bool stop);
uint16_t artemis_piston_i2c_read_data(uint8_t *pBuf);
void artemis_piston_set_write_mode(bool state);

void artemis_piston_i2c_read(uint8_t addr, uint8_t *data, uint16_t len);

#endif // ARTEMIS_PISTON_H
