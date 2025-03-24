/**
 * @file i9603n.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-30
 * 
 */
#ifndef I9603N_H
#define I9603N_H

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
*						STANDARD LIBRARIES
************************************************************************/
#include <stdint.h>
#include <stdbool.h>
/************************************************************************
*							HEADER FILES
************************************************************************/


/************************************************************************
*							MACROS
************************************************************************/


/************************************************************************
*							ENUM & STRUCTS
************************************************************************/

typedef enum ei9603n_conn_state_t
{
    I9603N_CONN_NOT_AVAILABLE,
    I9603N_CONN_LOW_SIGNAL,
    I9603N_CONN_READY
}i9603n_conn_state_t;


typedef enum ei9603n_result_t
{
    I9603N_RESULT_FAIL,
    I9603N_RESULT_OK,
    I9603N_RESULT_ERROR,
    I9603N_RESULT_READY,
    I9603N_RESULT_SBD_RING,
    I9603N_RESULT_HARDWARE_FAILURE

} i9603n_result_t;





/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/

void i9603n_initialize(void);
void i9603n_on(void);
void i9603_off(void);
bool i9603_send_msg(uint8_t *msg, uint16_t len);
uint16_t i9603_read_msg(uint8_t *msg, uint8_t len);


#ifdef TEST
//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
static bool module_i9603_power_on(void);
static void module_i9603_power_off(void);
static bool module_i9603_check_net_available(void);
static bool module_i9603n_send(uint8_t *msg, uint16_t len);
STATIC i9603n_result_t module_i9603n_read_at(void);
#endif 

#endif // UBLOX_H