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
#include "sensors.h"

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
#include <ctype.h>

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
#include "config.h"

#endif // TEST
//*****************************************************************************
//
//  Macros & Constants
//
//*****************************************************************************
#define I9603N_TX_BUFFER_SIZE       ( 400 )
#define I9603N_RX_BUFFER_SIZE       ( 300 )
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
static char irid_buf_tx[I9603N_TX_BUFFER_SIZE];
static char irid_buf_rx[I9603N_RX_BUFFER_SIZE];

//*****************************************************************************
//
// FreeRTOS Functions and global variables, parameters
//
//*****************************************************************************

#define IRIDIUM_MAXIMUM_RATE    (0.5f)
#define FREERTOS
static bool xVisible = false;
static uint8_t xTStatus[6];
static float iridium_delay_rate = 0.333f;

//#define TEST_SBD_MSG

#ifdef TEST_SBD_MSG
static uint8_t imei_filename[15];
static uint8_t imei_length;
static uint8_t sbd_transfer = 0;
static uint8_t Sbd_transfer = 0;
#endif


//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
#ifndef TEST

static bool module_i9603_power_on(void);
static void module_i9603_power_off(void);

static bool module_i9603_check_net_available(void);
static void module_i9603n_send(char *txData, uint16_t txlen);
static uint16_t module_i603n_receive(uint8_t *rxData);

static bool module_i9603n_echo_off(void);
static bool module_i9603n_echo_on(void);

static bool module_i9603n_clear_oBuff(void);
static bool module_i9603n_clear_iBuff(void);
static bool module_i9603n_clear_MOMSN(void);
static void module_i9603n_flush(char *buff);
static bool module_i9603n_AT_check(void);

static int16_t parse_AT(char *rxData, char *pattern, uint16_t len);
static uint16_t parse_data(char *inData, uint16_t *outData);
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

#ifdef TEST_SBD_MSG

    i9603n_on();
    vTaskDelay(xDelay500ms);
    imei_length = i9603n_read_imei(imei_filename);
    if (imei_length > 0)
    {
        ARTEMIS_DEBUG_PRINTF("Iridium :: IMEI Number = ");
        for (uint8_t i=0; i<imei_length; i++)
        {
            ARTEMIS_DEBUG_PRINTF("%c", imei_filename[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n\n");
    }
#endif

}

void i9603n_uninitialize(void)
{
    artemis_i9603n_uninitialize();
    artemis_sc_power_off();
}

bool i9603n_on(void)
{
    return module_i9603_power_on();
    //module_i9603n_echo_off();
}

void i9603n_off(void)
{
    module_i9603_power_off();
}

void i9603n_sleep(void)
{
    artemis_i9603n_power_off();
}

void i9603n_wakeup(void)
{
    artemis_i9603n_power_on();
}

void i9603n_indicator_event_reporting(uint8_t *rssi, uint8_t *network_service)
{
    /* Enable indicator event report for RSSI and Network Service Availability */
    char *cmd = "AT+CIER=1,1,1\r";
    uint16_t len = 0;
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        if (irid_buf_rx[0] == 'A' && irid_buf_rx[1] == 'T')
        {
            uint16_t i=0;
            while (irid_buf_rx[i]!=':' && i<len)
            {
                i++;
            }
            i++;
            while(irid_buf_rx[i]!='\r' && i<len)
            {
                i++;
            }

            /* Collect RSSI value */
            *rssi = (uint8_t)(irid_buf_rx[i-1] - 48);

            i++;
            while (irid_buf_rx[i]!=':' && i<len)
            {
                i++;
            }
            i++;
            while(irid_buf_rx[i]!='\r' && i<len)
            {
                i++;
            }

            /* Collect Network Service value */
            *network_service = (uint8_t) (irid_buf_rx[i-1] - 48);
            *rssi = 0;
        }
    }
    module_i9603n_flush(irid_buf_rx);
}

uint8_t i9603n_system_network_time(uint8_t *rxData)
{
    char *cmd = "AT-MSSTM\r";
    uint16_t rxLen = 0;
    uint16_t len = 0;
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        if (irid_buf_rx[0] == 'A' && irid_buf_rx[1] == 'T')
        {
            uint16_t i=0;
            while (irid_buf_rx[i]!=':' && i<len)
            {
                i++;
            }
            i+=2;

            while(irid_buf_rx[i]!='\r' && i<len)
            {
                rxData[rxLen++] = irid_buf_rx[i];
                i++;
            }
        }
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

void i9603n_stop_flowcontrol(void)
{
    char *cmd = "AT&K0\r";
    uint8_t len = 0;
    char res[16] = {0};

    len = i9603n_send_AT_cmd(cmd, res);
    if (len > 0)
    {
        for(uint8_t i=0; i<len; i++)
        {
            ARTEMIS_DEBUG_PRINTF("%c", res[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");
    }
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
    uint32_t period = xDelay1000ms;
    ARTEMIS_DEBUG_PRINTF("Iridium :: Satellite delay PERIOD=%dms, %isec\n", period, SATELLITE_TIMER);

    /* timer */
    uint16_t timer = 0;
    bool run = true;
    bool check = false;
    uint8_t visibility_count = 0;

    while(run)
    {
        /* check satellite visibility */
        check = artemis_i9603n_is_network_available();
        ARTEMIS_DEBUG_PRINTF("Iridium :: Satellite visibility check=%i\n", (uint8_t)check);

        if (check)
        {
            taskENTER_CRITICAL();
            xVisible = true;
            taskEXIT_CRITICAL();

            visibility_count++;
            if (visibility_count == 3)
            {
                run = false;
                visibility_count = 0;
            }
        }
        else
        {
            taskENTER_CRITICAL();
            xVisible = false;
            taskEXIT_CRITICAL();
        }

        timer++;
        if (timer >= SATELLITE_TIMER)
        {
            run = false;
            timer = 0;
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
                                "task_Iridium_transfer", 512, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xIridium) == pdPASS );
}

void SET_Iridium_delay_rate(float rate)
{
    if( (rate > 0.0) && (rate <= IRIDIUM_MAXIMUM_RATE))
    {
        ARTEMIS_DEBUG_PRINTF("Iridium :: Setting Iridium delay rate = %.3fHz\n", rate);
        iridium_delay_rate = rate;
    }
}

void task_Iridium (void)
{
    /* default 5 seocnds period delay */
    uint32_t period = xDelay1000ms/iridium_delay_rate;
    ARTEMIS_DEBUG_PRINTF("Iridium :: Delay PERIOD=%ums, Max Tries=%u\n", period, IRIDIUM_TRIES);

    uint8_t len = 0;
    uint16_t buf[10] = {0};
    char buf_AT[16] = {0};

    /* try tranmitting 20 times ? */
    uint8_t timer = 0;

    bool run = true;
    while(run)
    {
        len = i9603n_initiate_transfer(buf);
        if (len > 0)
        {
            /* fill up the status to XTStatus buffer */
            taskENTER_CRITICAL();
            for (uint8_t i=0; i<len; i++)
            {
                xTStatus[i] = buf[i];
            }
            taskEXIT_CRITICAL();


            ARTEMIS_DEBUG_PRINTF("Iridium :: Transfer status : ");
            for(uint16_t i=0; i<len; i++)
            {
                ARTEMIS_DEBUG_PRINTF("%u ", buf[i]);
            }
            ARTEMIS_DEBUG_PRINTF("\n");

            /* check the transfer status */
            if(buf[0] <= 4)
            {
                ARTEMIS_DEBUG_PRINTF("Iridium :: Transfer Successful, clearing buffer\n");
                // clear originated buffer, wait for 2 seconds
                vTaskDelay(xDelay2000ms);
                ARTEMIS_DEBUG_PRINTF("Iridium :: Clearing the Originated buffer:\n");
                len = i9603n_send_AT_cmd("AT+SBDD0\r", buf_AT);
                if(len>0)
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: Originated buffer is cleared\n");
                    ///* for Debugging */
                    //for (uint8_t i=0; i<len; i++)
                    //{
                    //    ARTEMIS_DEBUG_PRINTF("%c", buf_AT[i]);
                    //}
                    run = false;
                }
            }
            else if (buf[0] == 32)
            {
                ARTEMIS_DEBUG_PRINTF("Iridium :: No Network Service\n");
            }
            else
            {
                if (buf[0] == 18)
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: Connection Lost (RF drop).\n");
                }
                else if (buf[0] == 37)
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: SBD service is temporarily disabled.\n");
                }
                else if (buf[0] == 38)
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: Try later, Traffic management period.\n");
                }

                // clear originated buffer, wait for 2 seconds
                vTaskDelay(xDelay2000ms);
                ARTEMIS_DEBUG_PRINTF("Iridium :: Clearing the Originated buffer:\n");
                len = i9603n_send_AT_cmd("AT+SBDD0\r", buf_AT);
                if(len>0)
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: Originated buffer is cleared\n");
                    ///* for Debugging */
                    //for (uint8_t i=0; i<len; i++)
                    //{
                    //    ARTEMIS_DEBUG_PRINTF("%c", buf_AT[i]);
                    //}
                    run = false;
                }
            }
        }

        /* continue checking the iridium max tries */
        timer++;
        if (timer >= IRIDIUM_TRIES)
        {
            if (run)
            {
                ARTEMIS_DEBUG_PRINTF("Iridium :: Reached Max Tries\n");
                // clear originated buffer, wait for 2 seconds
                vTaskDelay(xDelay2000ms);
                ARTEMIS_DEBUG_PRINTF("Iridium :: Clearing the Originated buffer:\n");
                len = i9603n_send_AT_cmd("AT+SBDD0\r", buf_AT);
                if(len>0)
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: Originated buffer is cleared\n");
                    ///* for Debugging */
                    //for (uint8_t i=0; i<len; i++)
                    //{
                    //    ARTEMIS_DEBUG_PRINTF("%c", buf_AT[i]);
                    //}
                    run = false;
                }
                else
                {
                    ARTEMIS_DEBUG_PRINTF("Iridium :: Originated buffer was not cleared\n");
                    run = false;
                }
            }
        }

        if (run)
        {
            vTaskDelay(period);
        }
        //vTaskDelayUntil(&xLastWakeTime, period);
    }

    /* delete the task */
    ARTEMIS_DEBUG_PRINTF("Iridium :: Task Deleted\n");
    vTaskDelete(NULL);
}

uint16_t i9603n_send_AT_cmd(char *cmd, char *rxData)
{
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t txLen = strlen(cmd);
    char lData[300] = {0};
    uint16_t len=0;

    if (txLen > 120)
    {
        // Debug
        ARTEMIS_DEBUG_PRINTF("Iridium :: limit of cmd size is 120 bytes !, %u\n", txLen);
        return len;
    }
    else
    {
        module_i9603n_send(cmd, txLen);
        result = module_i9603n_read_AT(&len);
        if (result == I9603N_RESULT_OK)
        {
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
    char *cmd = "AT+SBDTC\r";
    uint16_t rxLen = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    uint16_t len = 0;
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        char *lData = NULL;
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

    char *cmd = "AT+SBDRT\r";
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
    char *cmd = "AT+SBDRB\r";
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

#ifdef TEST_SBD_MSG
    //datalogger_test_sbd_messages((char*)imei_filename, txData, txlen);
    ARTEMIS_DEBUG_PRINTF("Iridium :: \n\n");
    for (uint16_t i=0; i<txlen; i++)
    {
        ARTEMIS_DEBUG_PRINTF("0x%02x ", txData[i]);
        if ( (i%322) == 0)
        {
            ARTEMIS_DEBUG_PRINTF("\n");
        }
    }
    ARTEMIS_DEBUG_PRINTF("\n\n");
    ARTEMIS_DEBUG_PRINTF("Iridium :: put into the output buffer (%u) bytes\n", txlen);
    return true;

#else

    i9603n_result_t result = I9603N_RESULT_FAIL;
    //uint16_t txLen = strlen(txData);
    uint16_t txLen = txlen;
    char cmd[20] = {0};
    bool ret = false;
    uint16_t len=0;

    if (txLen > 340)
    {
        // Debug
        ARTEMIS_DEBUG_PRINTF("Iridium :: length exceeds 340 bytes !, (%u !!!)\n", txLen);
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
            ARTEMIS_DEBUG_PRINTF("Iridium :: Result is READY \n");
            ret = true;
        }
        else
        {
            ARTEMIS_DEBUG_PRINTF("Iridium :: Result is NOT READY \n");
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

#endif

}

uint8_t i9603n_traffic_mgmt_time(uint16_t *rxData)
{
    char *cmd = "AT+SBDLOE\r";
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

uint8_t i9603n_signal_quality(uint8_t *rxData)
{
    char *cmd = "AT+CSQ\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;

    uint8_t cmd_len = strlen(cmd);
    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        rxLen = parse_data(irid_buf_rx, (uint16_t *)rxData);
    }
    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint8_t i9603n_initiate_transfer(uint16_t *rxData)
{

#ifdef TEST_SBD_MSG

    if (sbd_transfer == 2)
    {
        uint8_t i=0;
        for (i=0; i<6; i++)
        {
            rxData[i] = 0;
        }
        Sbd_transfer++;

        if (Sbd_transfer == 5)
        {
            sbd_transfer = 0;
            Sbd_transfer = 0;
        }
        return i;
    }
    else
    {
        uint8_t i=0;
        for (i=0; i<6; i++)
        {
            rxData[i] = 32;
        }
        sbd_transfer++;
        return i;
    }

#else

    char *cmd = "AT+SBDIX\r";
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

#endif

}

uint8_t i9603n_status(uint8_t *rxData)
{
    char *cmd = "AT+SBDSX\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        rxLen = parse_data(irid_buf_rx, (uint16_t *)rxData);
    }

    module_i9603n_flush(irid_buf_rx);
    return rxLen;
}

uint16_t i9603n_read_imei(uint8_t *rxData)
{
    char *cmd = "AT+CGSN\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        char *lData = NULL;
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

            len = strlen(tok)+1;
            strncpy ((char *)rxData, tok, len);
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
    char *cmd = "AT+GMM\r";
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint16_t rxLen = 0;
    uint16_t len = 0;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        char *lData = NULL;
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
            len = strlen(tok)+1;
            strncpy ((char *)rxData, tok, len);
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
    char *cmd = "AT+CGMR\r";
    uint16_t rxLen = 0;
    uint16_t len = 0;
    i9603n_result_t result = I9603N_RESULT_FAIL;
    uint8_t cmd_len = strlen(cmd);

    module_i9603n_send(cmd, cmd_len);
    result = module_i9603n_read_AT(&len);

    if (result == I9603N_RESULT_OK)
    {
        char *lData = NULL;
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
            len = strlen(tok)+1;
            strncpy ((char *)rxData, tok, len);
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
        //am_hal_systick_delay_us(500000);
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
    //uint8_t connection_attempts = attempts;

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

    //if(retVal)
    //{
    //    isbd_at_packet_t packet = {0};
    //    /** Create the packet */
    //    retVal = ISBD_AT_create_packet( ISBD_AT_CMD_CSQ,
    //                                    0,
    //                                    NULL,
    //                                    &packet );

        /** Send the packet */

        // /** Query for connection strength */
        // am_util_stdio_sprintf(irid_buf, "AT+CSQ\r");
        // module_i9603n_send(irid_buf, 7);

        // /** Read & Parse result */
    //}

    return retVal;
}

static uint16_t module_i9603n_send_packet(
                            isbd_at_packet_t *packet,
                            uint8_t *result
                            )
{
    /** Send the command */
    artemis_i9603n_send((char *)packet->cmd.msg, strlen((char *)packet->cmd.msg));

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
    return 0;
}   

static uint16_t module_i603n_receive(uint8_t *rxData)
{
    uint16_t len = artemis_i9603n_receive((char *)rxData);
    return len;
}

static void module_i9603n_send(char *txData, uint16_t txlen)
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

static void module_i9603n_flush(char *buff)
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
            char *cmd = "AT+SBDC\r";
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
            char *cmd = "AT+SBDD0\r";

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
            char *cmd = "AT+SBDD1\r";
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

static uint16_t parse_data(char *inData, uint16_t *outData)
{
    uint16_t rxLen=0;
    uint16_t len = 0;
    int16_t number = 0;
    bool negative = false;
    char *lData = NULL;
    char buff[300] = {0};

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

static int16_t parse_AT(char *rxData, char *pattern, uint16_t len)
{
    int16_t pos = -1;
    uint16_t i, j, k = 0;
    uint16_t rxLen = len;
    uint8_t pattLen = strlen(pattern);

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
    char remBuff[64] = {0};
    bool contFlag = true;
    uint8_t wait = 30 * 2; //30 seconds maximum for iridium response

    while(contFlag && wait-- > 0)
    {
        msg_len = artemis_i9603n_receive(irid_buf_rx);

        if(msg_len > 0)
        {
            /* Debug */
            //for (uint16_t i=0; i<msg_len; i++)
            //{
            //    ARTEMIS_DEBUG_PRINTF("0x%02X ", pStart[i]);
            //}
            //ARTEMIS_DEBUG_PRINTF("\n");

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
                    uint16_t length = artemis_i9603n_receive(remBuff);
                    if (length > 0)
                    {
                        for (uint16_t i=0; i<length; i++)
                        {
                            irid_buf_rx[msg_len+i] = remBuff[i];
                            //ARTEMIS_DEBUG_PRINTF("0x%02X ", pStart[msg_len+i+1]);
                        }
                        rem_len += length;
                        ///* Debug */
                        //ARTEMIS_DEBUG_PRINTF("\n");
                        //for (uint16_t i=0; i<(msg_len+rem_len); i++)
                        //{
                        //    //ARTEMIS_DEBUG_PRINTF("%c", pStart[i]);
                        //    //ARTEMIS_DEBUG_PRINTF("%c", irid_buf_rx[i]);
                        //    ARTEMIS_DEBUG_PRINTF("0x%02X ", irid_buf_rx[i]);
                        //    //ARTEMIS_DEBUG_PRINTF("%c", remBuff[i]);
                        //}
                        //ARTEMIS_DEBUG_PRINTF("\n");
                    }
                }
                else
                {
                    contFlag = false;
                    //wait = 0;
                }

#ifdef FREERTOS
                /* 500ms delay */
                vTaskDelay(xDelay500ms);
#else
                /* 500ms delay */
                am_hal_systick_delay_us(500000);
#endif
            } while(result == I9603N_RESULT_FAIL && wait-- > 0);
        }

#ifdef FREERTOS
        /* 500ms delay */
        vTaskDelay(xDelay500ms);
#else
        /* 500ms delay */
        am_hal_systick_delay_us(500000);
#endif
    }

    *len = msg_len+rem_len;
    return result;
}
