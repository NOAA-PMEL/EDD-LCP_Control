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
#define I9603N_TX_BUFFER_SIZE  ( 400 )
#define I9603N_RX_BUFFER_SIZE  ( 300 )
#define I9603N_CONN_ATTEMPT_COUNT   ( 120 )
//*****************************************************************************
//
// Structs
//
//*****************************************************************************
//typedef uint8_t module_buffer_t[I9603N_BUFFER_SIZE];
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
static volatile uint8_t irid_buf_tx[I9603N_TX_BUFFER_SIZE];
static volatile uint8_t irid_buf_rx[I9603N_RX_BUFFER_SIZE];

//*****************************************************************************
//
// FreeRTOS Functions and global variables, parameters
//
//*****************************************************************************

static bool xVisible = false;
static uint8_t xTStatus[6];

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
#ifndef TEST

static bool module_i9603_power_on(void);
static void module_i9603_power_off(void);

static bool module_i9603_check_net_available(void);
static void module_i9603n_send(uint8_t *txData, uint16_t txlen);
static uint16_t module_i603n_receive(uint8_t *rxData);

static bool module_i9603n_echo_off(void);
static bool module_i9603n_echo_on(void);

static bool module_i9603n_clear_oBuff(void);
static bool module_i9603n_clear_iBuff(void);
static bool module_i9603n_clear_MOMSN(void);
static void module_i9603n_flush(uint8_t *buff);
static bool module_i9603n_AT_check(void);

static int16_t parse_AT(uint8_t *rxData, uint8_t *pattern, uint16_t len);
static uint16_t parse_data(uint8_t *inData, uint8_t *outData);
static i9603n_result_t module_i9603n_read_AT(uint16_t *len);

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
    //am_util_delay_ms(500);
}

void i9603n_uninitialize(void)
{
    artemis_i9603n_uninitialize();
    artemis_sc_power_off();
}

bool i9603n_on(void)
{
    return module_i9603_power_on();
    //am_util_delay_ms(1000);
    //module_i9603n_echo_off();
}

void i9603n_off(void)
{
    module_i9603_power_off();
}

bool GET_Iridium_satellite (void)
{
    bool retVal = false;

    taskENTER_CRITICAL();
    retVal = xVisible;
    taskEXIT_CRITICAL();

    return retVal;
}

void task_Iridium_satellite_visibility (TaskHandle_t *xSatellite)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_Iridium_satellite,
                                "task_Iridium_satellite", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xSatellite) == pdPASS );
}

void task_Iridium_satellite (void)
{
    /* 1 seocnds period delay */
    uint32_t period = pdMS_TO_TICKS(1000UL / 1);

    ARTEMIS_DEBUG_PRINTF("Satellite delay PERIOD=%dms, %isec\n", period, SATELLITE_TIMER);
    SemaphoreHandle_t Ssemaphore = xSemaphoreCreateMutex();

    /* timer */
    uint16_t timer = 0;

    bool run = true;
    while(run)
    {
        /* check satellite visibility */
        bool check = artemis_i9603n_is_network_available();
        ARTEMIS_DEBUG_PRINTF("Satellite visibility check=%i\n", (uint8_t)check);

        if( xSemaphoreTake(Ssemaphore, pdMS_TO_TICKS(1000UL)) == pdTRUE )
        {
            if (check)
            {
                taskENTER_CRITICAL();
                xVisible = true;
                taskEXIT_CRITICAL();
                run = false;
            }
            else
            {
                taskENTER_CRITICAL();
                xVisible = false;
                taskEXIT_CRITICAL();
            }
            xSemaphoreGive(Ssemaphore);
            timer++;
            if (timer >= SATELLITE_TIMER)
            {
                run = false;
                timer = 0;
            }
        }
        vTaskDelay(period);
    }
    /* delete the task */
    vTaskDelete(NULL);
}

bool GET_Iridium_status (uint8_t *rData)
{
    bool retVal = false;

    taskENTER_CRITICAL();
    /* do we need data back ? */
    for (uint8_t i=0; i<6; i++)
    {
        rData[i] = xTStatus[i];
    }
    retVal = true;
    taskEXIT_CRITICAL();

    return retVal;
}

void task_Iridium_transfer(TaskHandle_t *xIridium)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_Iridium,
                                "task_Iridium_transfer", 1024, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xIridium) == pdPASS );
}

void task_Iridium (void)
{
    /* 5 seocnds period delay */
    uint32_t period = pdMS_TO_TICKS(1000UL / 0.2);

    ARTEMIS_DEBUG_PRINTF("Iridium delay PERIOD=%dms, max tries=%i\n", period, IRIDIUM_TRIES);
    SemaphoreHandle_t Isemaphore = xSemaphoreCreateMutex();

    uint8_t len = 0;
    uint8_t buf[32] = {0};

    /* try tranmitting 20 times ? */
    uint8_t timer = 0;

    bool run = true;
    while(run)
    {
        //len = i9603n_initiate_transfer(buf);
        if( xSemaphoreTake(Isemaphore, pdMS_TO_TICKS(10000UL)) == pdTRUE )
        {
            len = i9603n_initiate_transfer(buf);

            if (len > 0)
            {
                ARTEMIS_DEBUG_PRINTF("Transfer status : ");
                for(uint16_t i=0; i<len; i++)
                {
                    ARTEMIS_DEBUG_PRINTF("%i ", (int8_t)buf[i]);
                    //ARTEMIS_DEBUG_PRINTF("%c", buf[i]);
                }
                ARTEMIS_DEBUG_PRINTF("\n");

                /* check the transfer status */
                if(buf[0] <= 5)
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: Transfer Successful, clearing buffer\n");

                    /* fill up the status to XTStatus buffer */
                    taskENTER_CRITICAL();
                    for (uint8_t i=0; i<6; i++)
                    {
                        xTStatus[i] = buf[i];
                    }
                    taskEXIT_CRITICAL();

                    // clear originated buffer
                    vTaskDelay(pdMS_TO_TICKS(2000UL));
                    ARTEMIS_DEBUG_PRINTF("Clearing the Originated buffer:\n");
                    len = i9603n_send_AT_cmd("AT+SBDD0\r", buf);

                    if(len>0)
                    {
                        for (uint8_t i=0; i<len; i++)
                        {
                            ARTEMIS_DEBUG_PRINTF("%c", (char)buf[i]);
                        }
                        ARTEMIS_DEBUG_PRINTF("\n");
                        //vTaskDelay(pdMS_TO_TICKS(2000UL));
                        run = false;
                    }
                }
                else
                {
                    /* fill up the status to XTStatus buffer */
                    taskENTER_CRITICAL();
                    for (uint8_t i=0; i<6; i++)
                    {
                        xTStatus[i] = buf[i];
                    }
                    taskEXIT_CRITICAL();
                }

            }

            xSemaphoreGive(Isemaphore);

            timer++;
            if (timer >= IRIDIUM_TRIES)
            {
                ARTEMIS_DEBUG_PRINTF("Reached max tries\n");
                run = false;
            }
        }
        vTaskDelay(period);
        //vTaskDelayUntil(&xLastWakeTime, period);
    }
    /* delete the task */
    vTaskDelete(NULL);
}

uint16_t i9603n_send_AT_cmd(uint8_t *cmd, uint8_t *rxData)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t txLen = strlen(cmd);
    uint8_t lData[300] = {0};
    uint16_t len=0;

    if (txLen > 120)
    {
        // Debug
        ARTEMIS_DEBUG_PRINTF("limit of cmd size is 120 bytes !, %u\n", txLen);
        return len;
    }
    else
    {
        module_i9603n_send(cmd, txLen);
        result = module_i9603n_read_AT(&len);
        if (result == I9603N_RESULT_OK)
        {
            len += 1;
            memcpy (lData, irid_buf_rx, len);
            for (uint16_t i=0; i<len; i++)
            {
                rxData[i] = lData[i];
            }
        }
        else
        {
            len = 0;
        }
    }
    module_i9603n_flush(irid_buf_rx);
    return len;
}

uint16_t i9603n_test_transfer(uint8_t *rxData)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t *cmd = "AT+SBDTC\r";
    uint16_t rxLen = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    uint16_t len = 0;
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        uint8_t *lData = NULL;
        uint16_t len = 0;
        uint16_t number = 0;
        uint16_t i=0;

        if (irid_buf_rx[0] == 'A' && irid_buf_rx[1] == 'T')
        {
            while (irid_buf_rx[i]!='\n' && i<300)
            {
                i++;
            }
            lData = &irid_buf_rx[i+1];
        }
        else
        {
            lData = &irid_buf_rx[0];
        }

        char *tok = strtok(lData, "\r\n");
        while (tok !=NULL)
        {
            if (strcmp(tok,"OK")==0)
            {
                break;
            }

            char *i_tok = strtok(tok, ":");
            while (i_tok != NULL)
            {
                uint16_t len = strlen(i_tok)+1;
                for (i=0; i<len; i++)
                {
                    if (i_tok[i] == '=')
                    {
                        i++;
                        while(i<len)
                        {
                            if (i_tok[i] == '\0')
                            {
                                if (number > 255)
                                {
                                    rxData[rxLen] = (number>>8)&0xFF;
                                    rxLen++;
                                    rxData[rxLen] = (number&0xFF);
                                    rxLen++;
                                    number = 0;
                                }
                                else
                                {
                                    rxData[rxLen] = number;
                                    rxLen++;
                                    number = 0;
                                }
                            }
                            else if(i_tok[i] == ' ')
                            {
                                // do nothing
                            }
                            else
                            {
                                number = number * 10 + (i_tok[i] - 48);
                                //ARTEMIS_DEBUG_PRINTF("number = %u\n", number);
                            }
                            i++;
                        }
                    }
                }
                i_tok = strtok(NULL, ":");
            }
            tok = strtok(NULL, "\r\n");
        }
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("Transfer Result NOT OK\n");
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint16_t i9603n_read_text(char *rxText)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    char lData[300] = {0};
    uint16_t len = 0;

    uint8_t *cmd = "AT+SBDRT\r";
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);
    memcpy (lData, irid_buf_rx, len);
    module_i9603n_flush(irid_buf_rx);
    if (result == I9603N_RESULT_OK)
    {
        char *local = NULL;
        char *tok = strtok(lData, "\r\n");
        while(tok != NULL)
        {
            if (strcmp(tok, "OK") == 0)
            {
                break;
            }

            local = tok;
            tok = strtok(NULL, "\r\n");
        }

        len = strlen(local);
        memcpy(rxText, local, len);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("Result NOT OK :: length = %u\n", len);
        // handle this
        len = 0;
    }

    module_i9603n_flush(irid_buf_rx);
    return len;
}

bool i9603n_send_text(char *txText)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t txLen = strlen(txText);
    bool ret = false;
    uint16_t len=0;

    if (txLen > 120)
    {
        // Debug
        ARTEMIS_DEBUG_PRINTF("limit of text size is 120 bytes, (%u !!!)\n", txLen);
        return ret;
    }
    else
    {
        am_util_stdio_sprintf((char*)irid_buf_tx, "AT+SBDWT=%s\r", txText);
        uint16_t txlen = strlen(irid_buf_tx);
        module_i9603n_send(irid_buf_tx, txlen);
        result = module_i9603n_read_AT(&len);
        if (result == I9603N_RESULT_OK)
        {
            ret = true;
        }
        else
        {
            ret = false;
        }
    }
    module_i9603n_flush(irid_buf_rx);
    module_i9603n_flush(irid_buf_tx);
    return ret;
}

uint16_t i9603n_read_data(uint8_t *rxData)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t lData[300] = {0};
    uint16_t len = 0;
    uint8_t *cmd = "AT+SBDRB\r";
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);

    result = module_i9603n_read_AT(&len);
    if (result == I9603N_RESULT_OK)
    {

        int16_t pos = parse_AT(irid_buf_rx, "\r\nOK\r\n", len);
        if (pos!=-1)
        {
            len = pos;
            memcpy (lData, irid_buf_rx, len);
            module_i9603n_flush(irid_buf_rx);
        }
        else
        {
            module_i9603n_flush(irid_buf_rx);
            return 0;
        }

        if (lData[0]=='A' && lData[1]=='T')
        {
            uint16_t i=0;
            while(lData[i]!='\r')
            {
                i++;
            }
            i++;
            uint16_t msgLen = lData[i] << 8 | lData[i+1];
            uint16_t recvSum = (lData[len-2] << 8) | lData[len-1];
            uint16_t checkSum = 0;
            uint16_t j = 0;
            for (i=i+2; i<len-2; i++)
            {
                checkSum += lData[i];
                rxData[j] = lData[i];
                j++;
            }

            // checksum check
            if (checkSum == recvSum) {return msgLen;}
            else {return 0;}
        }
        else
        {
            uint16_t msgLen = lData[0] << 8 | lData[1];
            uint16_t recvSum = (lData[len-2] << 8) | lData[len-1];
            uint16_t checkSum = 0;
            for (uint16_t i=2; i<len-2; i++)
            {
                checkSum += lData[i];
                rxData[i-2] = lData[i];
            }

            // checksum check
            if (checkSum == recvSum) {return msgLen;}
            else {return 0;}
        }
    }
    else
    {
        // Debug
        module_i9603n_flush(irid_buf_rx);
        ARTEMIS_DEBUG_PRINTF("Try again ...\n");
        return 0;
    }
}

bool i9603n_send_data(uint8_t *txData, uint16_t txlen)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    //uint16_t txLen = strlen(txData);
    uint16_t txLen = txlen;
    char cmd[20] = {0};
    bool ret = false;
    uint16_t len=0;

    if (txLen > 340)
    {
        // Debug
        ARTEMIS_DEBUG_PRINTF("length exceeds 340 bytes !, (%u !!!)\n", txLen);
        return ret;
    }

    // Prepare for a message to send
    else
    {
        am_util_stdio_sprintf(cmd, "AT+SBDWB=%u\r", txLen);
        uint8_t cmd_len = strlen(cmd);
        module_i9603n_send(cmd, cmd_len);
        result = module_i9603n_read_AT(&len);
        if (result == I9603N_RESULT_READY)
        {
            ARTEMIS_DEBUG_PRINTF("Result is READY \n");
            ret = true;
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("Result is NOT READY \n");
            ret = false;
        }

        if (ret == true)
        {
            uint16_t checksum = 0;
            memcpy(irid_buf_tx, txData, txLen);
            uint16_t i = 0;

            for (i=0; i<txLen; i++)
            {
                checksum += irid_buf_tx[i];
            }

            irid_buf_tx[i] = (checksum >> 8) & 0xFF ;
            i++;
            irid_buf_tx[i] = (checksum & 0xFF) ;
            i++;

            /* for Debugging */
            //ARTEMIS_DEBUG_PRINTF("Sending String = ");
            //for (uint16_t i=0; i<strlen(irid_buf_tx); i++)
            //{
            //    ARTEMIS_DEBUG_PRINTF("0x%02X ", irid_buf_tx[i]);
            //}
            //ARTEMIS_DEBUG_PRINTF("\n");

            // send bytes
            module_i9603n_send(irid_buf_tx, i);
            result = module_i9603n_read_AT(&len);
            if (result == I9603N_RESULT_OK)
            {
                if (parse_AT(irid_buf_rx, "0\r\n", len) != -1)
                {
                    ret = true;
                }
                else
                {
                    ret = false;
                }
            }
        }
        else
        {
            ret = false;
        }
    }
    // flush rx and tx buffer in any case
    module_i9603n_flush(irid_buf_rx);
    module_i9603n_flush(irid_buf_tx);
    return ret;
}

uint8_t i9603n_signal_quality(uint8_t *rxData)
{
    uint8_t *cmd = "AT+CSQ\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;

    uint8_t cmd_len = strlen(cmd);
    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        rxLen = parse_data(irid_buf_rx, rxData);
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint8_t i9603n_initiate_transfer(uint8_t *rxData)
{
    uint8_t *cmd = "AT+SBDIX\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen =0;
    uint16_t len = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        rxLen = parse_data(irid_buf_rx, rxData);
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint8_t i9603n_status(uint8_t *rxData)
{
    uint8_t *cmd = "AT+SBDSX\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        rxLen = parse_data(irid_buf_rx, rxData);
    }

    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint16_t i9603n_read_imei(uint8_t *rxData)
{
    uint8_t *cmd = "AT+CGSN\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        uint8_t *lData = NULL;
        if (irid_buf_rx[0] == 'A' && irid_buf_rx[1] == 'T')
        {
            uint16_t i=0;
            while (irid_buf_rx[i]!='\n' && i<300)
            {
                i++;
            }
            lData = &irid_buf_rx[i+1];
        }
        else
        {
            lData = &irid_buf_rx[0];
        }

        char *tok = strtok(lData, "\r\n");
        while (tok !=NULL)
        {
            if (strcmp(tok,"OK")==0)
            {
                break;
            }

            len = strlen(tok);
            strncpy (rxData, tok, len);
            rxData += len;
            rxLen += len;
            tok = strtok(NULL, "\r\n");
        }
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint16_t i9603n_read_model(uint8_t *rxData)
{
    uint8_t *cmd = "AT+GMM\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        uint8_t *lData = NULL;
        if (irid_buf_rx[0] == 'A' && irid_buf_rx[1] == 'T')
        {
            uint16_t i=0;
            while (irid_buf_rx[i]!='\n' && i<300)
            {
                i++;
            }
            lData = &irid_buf_rx[i+1];
        }
        else
        {
            lData = &irid_buf_rx[0];
        }

        char *tok = strtok(lData, "\r\n");
        while (tok !=NULL)
        {
            if (strcmp(tok,"OK")==0)
            {
                break;
            }
            len = strlen(tok);
            strncpy (rxData, tok, len);
            rxData += len;
            rxLen += len;
            tok = strtok(NULL, "\r\n");
        }
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint16_t i9603n_read_revision(uint8_t *rxData)
{
    uint8_t *cmd = "AT+CGMR\r";
    uint16_t rxLen = 0;
    uint16_t len = 0;
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        uint8_t *lData = NULL;
        if (irid_buf_rx[0] == 'A' && irid_buf_rx[1] == 'T')
        {
            uint16_t i=0;
            while (irid_buf_rx[i]!='\n' && i<300)
            {
                i++;
            }
            lData = &irid_buf_rx[i+1];
        }
        else
        {
            lData = &irid_buf_rx[0];
        }

        char *tok = strtok(lData, "\r");
        while (tok !=NULL)
        {
            if (strcmp(tok,"\nOK")==0)
            {
                break;
            }
            len = strlen(tok);
            strncpy (rxData, tok, len);
            rxData += len;
            rxLen += len;
            tok = strtok(NULL, "\r");
        }
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
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
	if (retVal)
    {
		artemis_i9603n_power_on();
        am_util_delay_ms(500);
	}
    return retVal;
}

static void module_i9603_power_off(void)
{
    artemis_i9603n_power_off();
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
        // am_util_stdio_sprintf(irid_buf, "AT+CSQ\r");
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

static uint16_t module_i603n_receive(uint8_t *rxData)
{
    uint16_t len = artemis_i9603n_receive(rxData);
    return len;
}

static void module_i9603n_send(uint8_t *txData, uint16_t txlen)
{
    artemis_i9603n_send(txData, txlen);
}

static bool module_i9603n_AT_check(void)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    bool ret = false;
    module_i9603n_send("AT\r", 3);
    uint16_t len = 0;
    result = module_i9603n_read_AT(&len);
    if (result == I9603N_RESULT_OK) { ret = true; }
    else { ret = false; }
    module_i9603n_flush(irid_buf_rx);
    return ret;
}

static bool module_i9603n_echo_off(void)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    bool ret = false;
    module_i9603n_send("ATE0\r", 5);
    uint16_t len = 0;
    result = module_i9603n_read_AT(&len);
    if (result == I9603N_RESULT_OK) { ret = true; }
    else { ret = false; }
    module_i9603n_flush(irid_buf_rx);
    return ret;
}

static bool module_i9603n_echo_on(void)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    bool ret = false;
    module_i9603n_send("ATE1\r", 5);
    uint16_t len = 0;
    result = module_i9603n_read_AT(&len);
    if (result == I9603N_RESULT_OK) { ret = true; }
    else { ret = false; }
    module_i9603n_flush(irid_buf_rx);
    return ret;
}

static void module_i9603n_flush(uint8_t *buff)
{
    uint16_t len = strlen(buff)+1;
    for (uint16_t i=0; i<len; i++)
    {
        buff[i] = 0;
    }
}

static bool module_i9603n_clear_MOMSN(void)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t rxData[8] = {0};
    uint16_t len = 0;

    len = i9603n_status(rxData);
    if (len > 0)
    {
        if (rxData[1] == 0)
        {
            return true;
        }
        else if (rxData[1] > 0)
        {
            uint8_t *cmd = "AT+SBDC\r";
            uint8_t cmd_len = strlen(cmd);
            module_i9603n_send(cmd, cmd_len);
            result = module_i9603n_read_AT(&len);
            if (result == I9603N_RESULT_OK)
            {
                return true;
            }
        }
        else
        {
            // handle this
            return false;
        }
    }
    else
    {
        // handle this
        return false;
    }
}

static bool module_i9603n_clear_oBuff(void)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t rxData[8] = {0};
    uint16_t len = 0;

    len = i9603n_status(rxData);
    if (len > 0)
    {
        if (rxData[0] == 0)
        {
            return true;
        }
        else if (rxData[0] == 1)
        {
            uint8_t *cmd = "AT+SBDD0\r";

            module_i9603n_send(cmd, 9);
            result = module_i9603n_read_AT(&len);
            if (result == I9603N_RESULT_OK)
            {
                return true;
            }
        }
        else
        {
            // handle this
            return false;
        }
    }
    else
    {
        // handle this
        return false;
    }
}

static bool module_i9603n_clear_iBuff(void)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t rxData[8] = {0};
    uint16_t len = 0;

    len = i9603n_status(rxData);
    if (len > 0)
    {
        if (rxData[2] == 0)
        {
            return true;
        }
        else if (rxData[2] == 1)
        {
            uint8_t *cmd = "AT+SBDD1\r";
            uint8_t cmd_len = strlen(cmd);

            module_i9603n_send(cmd, cmd_len);
            result = module_i9603n_read_AT(&len);
            if (result == I9603N_RESULT_OK)
            {
                return true;
            }
        }
        else
        {
            // handle this
            return false;
        }
    }
    else
    {
        // handle this
        return false;
    }
}

static uint16_t parse_data(uint8_t *inData, uint8_t *outData)
{
    uint16_t rxLen=0;
    uint16_t len = 0;
    int16_t number = 0;
    bool negative = false;
    uint8_t *lData = NULL;
    uint8_t buff[300] = {0};

    memcpy(buff, inData, strlen(inData)+1);

    if (buff[0] == 'A' && buff[1] == 'T')
    {
        uint16_t i=0;
        while (buff[i]!='\n' && i<312)
        {
            i++;
        }
        lData = &buff[i+1];
    }
    else
    {
        lData = &buff[0];
    }

    // Debug
    //ARTEMIS_DEBUG_PRINTF("Received String = ");
    //for (uint8_t i=0; lData[i]!='\0'; i++)
    //{
    //    ARTEMIS_DEBUG_PRINTF("%c", lData[i]);
    //}
    //ARTEMIS_DEBUG_PRINTF("\n");

    char *o_tok = strtok(lData, "\r\n");
    while (o_tok !=NULL)
    {
        if (strcmp(o_tok,"OK")==0)
        {
            break;
        }
        char *i_tok = strtok(o_tok, ":");
        while (i_tok != NULL)
        {
            if (i_tok[0] != '+')
            {
                len = strlen(i_tok);
                for (uint16_t i=0; i<len+1; i++)
                {
                    if ( (i_tok[i] == ',')  || (i_tok[i] == '\0'))
                    {
                        outData[rxLen] = number;
                        rxLen++;
                        number = 0;
                    }
                    else if ( (i_tok[i] == ' ') )
                    {
                        // do nothing
                    }
                    else
                    {
                        if ( i_tok[i] == '-')
                        {
                            negative = true;
                        }
                        else
                        {
                            number = number * 10 + (i_tok[i] - 48) ;
                            if (negative)
                            {
                                number *= -1 ;
                                negative = false;
                            }
                        }
                    }
                }
            }
            i_tok = strtok(NULL, ":");
        }
        o_tok = strtok(NULL, "\r\n");
    }
    return rxLen;
}

static int16_t parse_AT(uint8_t *rxData, uint8_t *pattern, uint16_t len)
{
    int16_t pos = -1;
    uint16_t i, j, k = 0;
    uint16_t rxLen = len;
    uint8_t pattLen = strlen((char*)pattern);

    if (pattLen > rxLen)
    {
        return -1;
    }

    for (i=0; i<=(rxLen-pattLen); i++)
    {
        pos = k = i;
        for (j=0; j<pattLen; j++)
        {
            if (pattern[j] == rxData[k])
            {
                k++;
            }
            else
            {
                break;
            }
        }
        if (j == pattLen)
        {
            return pos;
        }
    }
    return -1;
}

static i9603n_result_t module_i9603n_read_AT(uint16_t *len)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t msg_len = 0;
    uint16_t rem_len = 0;
    char *pStart = &irid_buf_rx[0];
    uint8_t remBuff[64] = {0};
    bool contFlag = true;
    uint32_t wait = 0;

    while(contFlag && wait < 400000)
    {
        msg_len = artemis_i9603n_receive(irid_buf_rx);

        if(msg_len > 0)
        {
            ///* Debug */
            //for (uint16_t i=0; i<msg_len; i++)
            //{
            //    ARTEMIS_DEBUG_PRINTF("0x%02X ", pStart[i]);
            //}
            //ARTEMIS_DEBUG_PRINTF("\n");
            //ARTEMIS_DEBUG_PRINTF("msg_len = %i, rem_len = %i\n", msg_len, rem_len);

            do{
                if(parse_AT(pStart, "OK\r\n", msg_len+rem_len) != -1)
                {
                    result = I9603N_RESULT_OK;
                }
                else if(parse_AT(pStart, "ERROR\r\n", msg_len+rem_len) != -1)
                {
                    result = I9603N_RESULT_ERROR;
                }
                else if(parse_AT(pStart, "READY\r\n", msg_len+rem_len) != -1)
                {
                    result = I9603N_RESULT_READY;
                }
                else if(parse_AT(pStart, "HARDWARE FAILURE\r\n", msg_len+rem_len) != -1)
                {
                    result = I9603N_RESULT_HARDWARE_FAILURE;
                }
                else if(parse_AT(pStart, "SBDRING\r\n", msg_len+rem_len) != -1)
                {
                    result = I9603N_RESULT_SBD_RING;
                }

                if(result == I9603N_RESULT_FAIL)
                {
                    rem_len = artemis_i9603n_receive(remBuff);
                    if (rem_len > 0)
                    {
                        for (uint16_t i=0; i<rem_len; i++)
                        {
                            irid_buf_rx[msg_len+i] = remBuff[i];
                            //ARTEMIS_DEBUG_PRINTF("0x%02X ", pStart[msg_len+i]);
                        }

                        ///* Debug */
                        //ARTEMIS_DEBUG_PRINTF("\n");
                        //for (uint16_t i=0; i<rem_len+msg_len; i++)
                        //{
                        //    ARTEMIS_DEBUG_PRINTF("%c", pStart[i]);
                        //    //ARTEMIS_DEBUG_PRINTF("%c", irid_buf_rx[i]);
                        //    //ARTEMIS_DEBUG_PRINTF("%c", remBuff[i]);
                        //}
                        //ARTEMIS_DEBUG_PRINTF("\n");
                    }
                    wait++;
                }
                else
                {
                    contFlag = false;
                    wait = 0;
                }
                wait++;
            } while(result == I9603N_RESULT_FAIL && wait < 5000);
        }
        am_hal_systick_delay_us(50);
    }
    *len = msg_len+rem_len;
    return result;
}
