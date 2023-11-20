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

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"
#include "rtos.h"

/************************************************************************
*							HEADER FILES
************************************************************************/


/************************************************************************
*							MACROS
************************************************************************/

#define IRIDIUM_TMER    30   /* 5 mins for iridium task */

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
void i9603n_uninitialize(void);
void i9603n_on(void);
void i9603n_off(void);

uint8_t i9603n_status(uint8_t *rxData);
uint8_t i9603n_signal_quality(uint8_t *rxData);
uint16_t i9603n_send_AT_cmd(uint8_t *cmd, uint8_t *rxData);

uint16_t i9603n_read_imei(uint8_t *rxData);
uint16_t i9603n_read_model(uint8_t *rxData);
uint16_t i9603n_read_revision(uint8_t *rxData);

bool i9603n_send_text(char *txText);
bool i9603n_send_data(uint8_t *txData, uint16_t txlen);
uint16_t i9603n_read_text(char *rxText);
uint16_t i9603n_read_data(uint8_t *rxData);
uint16_t i9603n_test_transfer(uint8_t *rxData);
uint8_t i9603n_initiate_transfer(uint8_t *rxData);

/* FreeRTOS task*/
void task_Iridium (void);
void task_Iridium_transfer(TaskHandle_t *xIridium);
bool GET_Iridium_status (uint8_t *rData);

//#ifndef TEST
////*****************************************************************************
////
//// Static Function Prototypes
////
////*****************************************************************************
//static bool module_i9603n_power_on(void);
//static void module_i9603n_power_off(void);
//static bool module_i9603n_check_net_available(void);
//static bool module_i9603n_send(uint8_t *msg, uint16_t len);
//STATIC i9603n_result_t module_i9603n_read_at(void);
//#endif

#endif // I9603N_H
