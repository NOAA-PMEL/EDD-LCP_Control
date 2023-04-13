#include "i9603n.h"
/**
 * @file i9603n.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-30
 * 
 */

#include "artemis_i9603n.h"
#include "artemis_supercap.h"

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#ifndef TEST
//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Artemis specific files
//
//*****************************************************************************
#include "artemis_supercap.h"
#include "artemis_i9603n.h"
#include "artemis_debug.h"
#include "artemis_stream.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"



//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "am_bsp_pins.h"
#include "buffer_c.h"
#include "isbd_at_cmd.h"

#endif // TEST
//*****************************************************************************
//
//  Macros & Constants
//
//*****************************************************************************
#define I9603N_BUFFER_SIZE  ( 300 )
#define I9603N_CONN_ATTEMPT_COUNT   ( 120 )
//*****************************************************************************
//
// Structs
//
//*****************************************************************************
typedef uint8_t module_buffer_t[I9603N_BUFFER_SIZE];
//typedef struct s_module_t
//{
//     artemis_uart_t uart;
//} module_t;

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************
/**
 * @brief Module Parameters
 * 
 */
//static module_t module;
static uint8_t irid_buf[I9603N_BUFFER_SIZE];
//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
#ifndef TEST
static bool module_i9603_power_on(void);
static void module_i9603_power_off(void);
static bool module_i9603_check_net_available(void);
static bool module_i9603n_send(uint8_t *msg, uint16_t len);
STATIC i9603n_result_t module_i9603n_read_at(void);
#endif
//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void i9603n_initialize(void)
{
    /** Initalize the 9603n UART & IO */
    artemis_i9603n_initialize();

    /** Initialize the power circuitry */
    artemis_sc_initialize();

}


void i9603n_on(void)
{
    module_i9603_power_on();
}


void i9603n_off(void)
{
    module_i9603_power_off();
}



bool i9603n_send_data(uint8_t *msg, uint16_t len)
{
    bool retVal = false;
	uint8_t rxData[128] = {0};
	uint8_t rxLen = 9;
	uint8_t msg_len = 0;

    if(len <= I9603N_BUFFER_SIZE)
    {
        retVal = true;
    }

    

    /** Write the message */
    if(retVal)
    {
        
        //strcpy((char*)irid_buf, "AT+SBWD=%u");
        strcpy((char*)irid_buf, (char*)msg);
        module_i9603n_send(irid_buf, strlen(irid_buf));
		am_util_stdio_printf("\n");
        msg_len = artemis_i9603n_receive(rxData, rxLen);
		for (uint8_t i=0; i<msg_len; i++){
			am_util_stdio_printf("%c", rxData[i]);
		}
		am_util_stdio_printf("\n");
    }

    /** Wait for the ACK */
    if(retVal)
    {

    }

    return retVal;
}




uint16_t I9603N_read_incoming_msg(uint8_t *msg, uint8_t len)
{
    
}


//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************

static bool module_i9603_power_on(void)
{
	bool retVal = false;

	retVal = artemis_sc_power_startup();

	if (retVal){
		artemis_i9603n_power_on();
	}

}


static void module_i9603_power_off(void)
{
    artemis_sc_power_off();
}


static bool module_i9603_attempt_network_connection(uint8_t attempts, uint16_t attempt_delay_us)
{
    bool retVal = false;

    /** Make sure the modem is on */
    i9603n_on();

    /** Ensure there is network connection via pin*/
    uint8_t connection_attempts = attempts;

    while( (attempts-- > 0) &&  
            !module_i9603_check_net_available())
    {
        #ifndef TEST
        am_hal_systick_delay_us(attempt_delay_us);
        #endif
    }
    
    

    return retVal;
}

static bool module_i9603_check_net_available(void)
{
    bool retVal = false;
    if(artemis_i9603n_is_network_available())
    {
        retVal = true;
    }

    if(retVal)
    {
        isbd_at_packet_t packet = {0};
        /** Create the packet */
        retVal = ISBD_AT_create_packet( ISBD_AT_CMD_CSQ,
                                        0,
                                        NULL,
                                        &packet );

        /** Send the packet */

        // /** Query for connection strength */
        // sprintf(irid_buf, "AT+CSQ\r");
        // module_i9603n_send(irid_buf, 7);

        // /** Read & Parse result */

            
    }
}

static uint16_t module_i9603n_send_packet(
                            isbd_at_packet_t *packet,
                            uint8_t *result
                            )
{
    /** Send the command */
    artemis_i9603n_send(packet->cmd.msg, strlen(packet->cmd.msg));

    /** */

    switch(packet->cmd.ecmd)
    {
         case ISBD_AT_CMD_AT:
            break;
        case ISBD_AT_CMD_ATK0:
            break;
        case ISBD_AT_CMD_SBDWT:
            break;
        case ISBD_AT_CMD_SBDIX:
            break;
        case ISBD_AT_CMD_SBDRT:
            break;
        case ISBD_AT_CMD_SBDRB:
            break;
        case ISBD_AT_CMD_NONE:
        default:
            break;  
    }
    
}   

static bool module_i9603n_send(uint8_t *msg, uint16_t len)
{
    artemis_i9603n_send(msg, len);
}

STATIC i9603n_result_t module_i9603n_read_at(void)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;

    uint16_t buf_len_remaining = I9603N_BUFFER_SIZE;
    uint16_t msg_len = 0;
    uint16_t buf_idx = 0;
    uint16_t total_len =0;
    uint8_t *pStart = &irid_buf[0];

    // msg_len = artemis_i9603n_receive(&irid_buf[buf_idx], buf_len_remaining);

    bool contFlag = true;
    while(contFlag)
    {   
        am_util_stdio_printf("buf_idx=%u\n", buf_idx);
        msg_len = artemis_i9603n_receive(&irid_buf[buf_idx], buf_len_remaining);
        total_len += msg_len;

        am_util_stdio_printf("%s\n", irid_buf);
        if(msg_len > 0)
        {   
            do{
                if(strncmp(pStart, "OK\r\n", 4) == 0)
                {
                    result = I9603N_RESULT_OK;
                } else if(strncmp(pStart, "ERROR\r\n", 7)==0)
                {
                    result = I9603N_RESULT_ERROR;
                } else if(strncmp(pStart, "READY\r\n", 7)==0)
                {
                    result = I9603N_RESULT_READY;
                } else if(strncmp(pStart, "HARDWARE FAILURE\r\n", 18)==0)
                {
                    result = I9603N_RESULT_HARDWARE_FAILURE;
                } else if(strncmp(pStart, "SBDRING\r\n", 9) == 0)
                {
                    result = I9603N_RESULT_SBD_RING;
                }

                if(result == I9603N_RESULT_FAIL)
                {
                    am_util_stdio_printf("total_len=%u\n", total_len);
                    am_util_stdio_printf("before loop loc= %p\n", pStart);
                    for(uint16_t i=0; i<total_len; i++)
                    {
                        printf("%u ", i);
                        if(pStart[i] == '\r' && pStart[i+1] == '\n')
                        {
                            pStart += (i + 2);
                            total_len -= (i+2);
                        } else if(pStart[i] == '\n')
                        {
                            pStart += (i+1);
                            total_len -= (i+1);
                        }
                    }
                    am_util_stdio_printf("after loop loc= %p\n", pStart);
                } else {
                    contFlag = false;
                }
            }while(result == I9603N_RESULT_FAIL);
        }

        buf_idx += msg_len;
    }
    

   
    
    return result;
}





