/** @file S9_temp.c
 *  @brief SoundNine OEM Temperature Sensor
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date September 30, 2020
 *  @version 0.0.1
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

//*****************************************************************************
//
// Artemis specific files
//
//*****************************************************************************
#include "artemis_max14830.h"
#include "artemis_debug.h"
#include "artemis_stream.h"
#include "MAX14830.h"

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
#include "buffer_c.h"
#include "S9_temperature.h"

//*****************************************************************************
//
// MACROS
//
//*****************************************************************************

#define FREE_RTOS

static sS9_t s9;
static sS9_t *pS9 = &s9;

STATIC void module_s9_parse_msg(char *data, uint8_t len, sS9_t *p);
STATIC void _parse_version(uint8_t *data, sS9_t *p, uint8_t rxLen);
STATIC int16_t _parse_response(uint8_t *rData, char *pattern, uint16_t len);
STATIC S9_result_t receive_response(uint8_t *rData, uint8_t *len);
STATIC S9_result_t receive_response_RTOS(uint8_t *rData, uint16_t *len);
STATIC void _module_s9_stop_sampling(void);

void S9T_init(S9_init_param *p)
{
    /** Default Buadrate */
    pS9->device.uart.port = p->port;
    pS9->device.uart.baudrate = p->baudrate;

    /** Attach values to struct */
    pS9->device.power.pin = p->pin_config;
    pS9->device.power.pin_number = p->pin_number;

    /** Initialize the COM Port Power Pin */
    am_hal_gpio_pinconfig(pS9->device.power.pin_number, *pS9->device.power.pin);

    /* set the baudrate */
    //artemis_max14830_Set_baudrate(pS9->device.uart.port, pS9->device.uart.baudrate);
    MAX14830_Set_baudrate(pS9->device.uart.port, pS9->device.uart.baudrate);

    /* turn on the module*/
    S9T_ON();

    /* stop sampling*/
    _module_s9_stop_sampling();

    // fetch device info, if required
    S9T_dev_info();
    ARTEMIS_DEBUG_PRINTF("S9 Temperature Sensor is initialized\n\n");

    /* turn off the module*/
    S9T_OFF();
}

STATIC void _module_s9_stop_sampling(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t sampleStr[12];
    uint8_t rxLen = 0;

    //artemis_max14830_UART_Write (pS9->device.uart.port, (uint8_t*)"stop\r", 5);
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"stop\r", 5);
    result = receive_response(sampleStr, &rxLen);
    if (result == S9_RESULT_OK)
    {
        /* Debug */
        ARTEMIS_DEBUG_PRINTF("S9 : Sampling Stopped\n");
    }
    else
    {
        /* Debug */
        ARTEMIS_DEBUG_PRINTF("S9 : Sampling did not stop\n");
    }
}

void _module_s9_stop_sampling_RTOS(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t sampleStr[12];
    uint16_t rxLen = 0;

    MAX14830_UART_Write(pS9->device.uart.port, (uint8_t*)"stop\r", 5);
    result = receive_response_RTOS(sampleStr, &rxLen);
    if (result == S9_RESULT_OK)
    {
        /* Debug */
        ARTEMIS_DEBUG_PRINTF("S9 : Sampling Stopped, RTOS\n");
    }
    else
    {
        /* Debug */
        ARTEMIS_DEBUG_PRINTF("S9 : Sampling did not stop, RTOS\n");
    }
}

void S9T_dev_info(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t verStr[256] = {0};
    uint8_t rxLen = 0;
    uint8_t i=0;

    /* using artemis module */
    //artemis_max14830_UART_Write (pS9->device.uart.port, "ver\r", 4);

    /* using  MAX14830 c file, non-RTOS*/
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"ver\r", 4);
    result = receive_response(verStr, &rxLen);
    if (result == S9_RESULT_OK)
    {
        /* Debug */
        //ARTEMIS_DEBUG_PRINTF("S9 : Sampling Stopped\n");

        // parse the return data
        _parse_version(verStr, pS9, rxLen);

        // print pS9 structure
        ARTEMIS_DEBUG_PRINTF("S9 Temperature Sensor\n");
        ARTEMIS_DEBUG_PRINTF("**************************************\n");

        // MID
        ARTEMIS_DEBUG_PRINTF("\tMID\t: ");
        for (i=0; i<8; i++){
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.MID[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");

        // C0-C3, R0
        ARTEMIS_DEBUG_PRINTF("\tC0\t: %.7f\n", pS9->info.C0);
        ARTEMIS_DEBUG_PRINTF("\tC1\t: %.7f\n", pS9->info.C1);
        ARTEMIS_DEBUG_PRINTF("\tC2\t: %.7f\n", pS9->info.C2);
        ARTEMIS_DEBUG_PRINTF("\tC3\t: %.7f\n", pS9->info.C3);
        ARTEMIS_DEBUG_PRINTF("\tR0\t: %.7f\n", pS9->info.R0);
        ARTEMIS_DEBUG_PRINTF("\tAverage\t: %u\n", pS9->info.average);

        // UID
        ARTEMIS_DEBUG_PRINTF("\tUID\t: ");
        for (i=0; i<32; i++){
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.UID[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");

        // Sensor Name
        ARTEMIS_DEBUG_PRINTF("\tFW Ver\t: ");
        for (i=0; i<10; i++){
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.sensor[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");
        //// Firmware Version
        ////ARTEMIS_DEBUG_PRINTF("%c", pS9->info.firmware.major);
        ////ARTEMIS_DEBUG_PRINTF("%c", pS9->info.firmware.minor);
        ////ARTEMIS_DEBUG_PRINTF("\n");

        // Sensor Status
        ARTEMIS_DEBUG_PRINTF("\tStatus\t: ");
        for (i=0; i<2; i++){
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.status[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");
    }
    else
    {
        /* Debug */
        ARTEMIS_DEBUG_PRINTF("S9 : Something went wrong\n");
    }
}

void S9T_enable(void)
{
    /** Enable the Power Pin */
    S9T_ON();
}

void S9T_disable(void)
{
    /** Disable the Power Pin */
    S9T_OFF();
}

void S9T_ON(void)
{
    am_hal_gpio_output_clear(pS9->device.power.pin_number);
    //am_hal_systick_delay_us(5000);
}

void S9T_OFF(void)
{
    am_hal_gpio_output_set(pS9->device.power.pin_number);
}

float S9T_Read_T(void)
{
    float t;
    S9T_Read(&t, NULL);
    return t;
}

float S9T_Read_R(void)
{
    float r;
    S9T_Read(NULL, &r);
    return r;
}

float S9T_Read(float *t, float *r)
{
    //bsp_uart_puts(pS9->device.uart.port, "sample\r", 7);
    uint8_t sampleStr[64];
    uint16_t rxLen = 0;
    S9_result_t result = S9_RESULT_FAIL;

#ifdef FREE_RTOS
    MAX14830_UART_Write(pS9->device.uart.port, (uint8_t *)"sample\r", 7);
    result = receive_response_RTOS(sampleStr, &rxLen);
#else
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t *)"sample\r", 7);
    result = receive_response(sampleStr, &rxLen);
#endif

    if (result == S9_RESULT_OK)
    {
        /* debug */
        //ARTEMIS_DEBUG_PRINTF("result OK\n");

        char *ptok = (char *)sampleStr;
        char *tok = strtok_r (ptok, "\r\n", &ptok);
        char *last_token = NULL;
        char sample[64] = {0};

        while (tok != NULL)
        {
            if (strcmp(tok,"OK")==0)
            {
                uint8_t len = strlen(last_token)+1;
                sample[len] = '\r';
                for (uint8_t i=0; i<len; i++)
                {
                    sample[i]=  last_token[i];
                }

                module_s9_parse_msg(sample, len+1, pS9);
                *t = pS9->temperature; // Tempearture in °C
                //*t = pS9->temperature * (9/5) + 32;  // convert to °F
                *r = pS9->resistance;
                break;
            }
            last_token = tok;
            tok = strtok_r(NULL, "\r\n", &ptok);
        }
    }
    else
    {
        /* Debug */
        //ARTEMIS_DEBUG_PRINTF("result NOT OK\n");
    }
	return 0.0;
}

/** @brief Parse S9 Temperature response
 *
 * The S9 Temperature sensor returns a data string
 * which is a reponse with either OK or ERROR.
 *
 * @param *rData Pointer to data string
 * @param *pattern to compare lies in the data string
 * @param len length of string
 */

STATIC int16_t _parse_response(uint8_t *rData, char *pattern, uint16_t len)
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
            if (pattern[j] == rData[k])
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

/** @brief Parse S9 Temperature response
 * 
 * The S9 Temperature sensor returns a data string 
 * in the format:
 * RRRR.RRRR, TT.TTTT\r
 * 
 * Where RRRR.RRRR is the thermistor resistance in Ohms
 * and TT.TTTT is the calculated temperature in degC
 * 
 * @param *data Pointer to data string
 * @param len length of string
 * @param *p Pointer to S9 Temperature structure
 */

STATIC void module_s9_parse_msg(char *data, uint8_t len, sS9_t *p)
{
    uint8_t comma=0;
    uint8_t end=0;
    uint8_t i;

    if(len <= 4)
    {
        p->temperature = NAN;
        p->resistance = NAN;
        return;
    }

    for(i=0;i<len;i++)
    {
        if(data[i] == ',')
        {
            comma = i;
        }
        else if(data[i] == '\r')
        {
            end = i;
        }
    }

    if(end <= comma)
    {
        p->temperature = NAN;
        p->resistance = NAN;
        return;
    }

    /* Copy Resistance */
    char temp[32];
    strncpy(temp, &data[0],comma);
    p->resistance = atof(temp);

    /* Copy Temperature */
    strncpy(temp, &data[comma+1], end-comma);
    p->temperature = atof(temp);
}

/** @brief Parse version info
 * 
 * When the S9 Temp is sent the "ver" command, 
 * the following response is sent:
 * S9>ver
 * MID=T003
 * C0=0.000855
 * C1=0.000293
 * C2=0.000000
 * C3=0.000000
 * R0=10000.000
 * UID=000000000F0F1A08535722E74FBC90B1
 * S9T0 V0.45
 * OK
 * 
 * This function parses for each individual structure
 * variable and returns.
 * 
 * @param *data Pointer to response string
 * @param *p Pointer to S9 temperature structure
 * 
 */

STATIC void _parse_version(uint8_t *data, sS9_t *p, uint8_t rxLen)
{
    //uint8_t i=0;
    char temp[256];
    float temp_f;

    strcpy(temp, (char*)data);
    // printf("%s\n", temp);
    char *tok;

    /** @todo strtok needs RTOS case!!! */
    /* Find MID */
    tok = strtok(temp,"=");
    tok = strtok(NULL, "\r");
    strcpy(p->info.MID, tok);

    /* Find C0 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    temp_f = atof(tok);
    p->info.C0 = temp_f;

    /* Find C1 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.C1 = atof(tok);

    /* Find C2 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.C2 = atof(tok);

    /* Find C3 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.C3 = atof(tok);

    /* Find R0 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.R0 = atof(tok);

    /* Find Average number/value */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.average = atof(tok);

    /* Find UID */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    strcpy(p->info.UID, tok);

    //uint8_t len = strlen(tok);
    //ARTEMIS_DEBUG_PRINTF("UID length = %u \n", len);
    ////uint8_t sub_val = 0;
    //uint8_t temp_hex[32];
    //memset(temp,0,32);

    //for(i=0;i<len;i++)
    //{
    //    if( (tok[i] >= '0') && (tok[i] <= '9'))
    //    {
    //        temp_hex[i] = tok[i] - '0';
    //    } else if ( (tok[i] >= 'A') && (tok[i] <= 'F'))
    //    {
    //        temp_hex[i] = tok[i] -'A' + 10;
    //    } else {
    //        /** @todo - Error condition */
    //    }
    //}
    //i=0;
    //uint8_t cnt =0;
    //while(i < 32)
    //{
    //    p->info.UID[cnt] = temp_hex[i++] << 4;
    //    p->info.UID[cnt] |= temp_hex[i++];
    //    cnt++;
    //}

    /* Find Sensor */
    tok = strtok(NULL, "\r\n");
    strcpy(p->info.sensor, tok);

    ///** Find Firmware Major Version */
    //tok = strtok(NULL, " ");
    //tok = strtok(NULL, ".");
    //p->info.firmware.major = (uint8_t) atoi(tok);
    //ARTEMIS_DEBUG_PRINTF("Firmware major = %u\n", p->info.firmware.major);

    ///** Find Firmware Minor Version */
    //tok = strtok(NULL, "\r");
    //p->info.firmware.minor = (uint8_t) atoi(tok);
    //ARTEMIS_DEBUG_PRINTF("Firmware minor = %u\n", p->info.firmware.minor);

    /** Find Status */
    tok = strtok(NULL, "\r\n");
    strcpy(p->info.status, tok);
}

STATIC S9_result_t receive_response(uint8_t *rData, uint8_t *len)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t msg_len = 0;
    uint8_t rem_len = 0;
    uint8_t lBuf[256] = {0};
    bool contFlag = true;
    uint32_t wait = 0;

    // maximum wait cycles
    while(contFlag && wait < 100000)
    {
        //artemis_max14830_UART_Read (pS9->device.uart.port, lBuf, &msg_len);
        msg_len = MAX14830_UART_Read_direct (pS9->device.uart.port, lBuf);
        //ARTEMIS_DEBUG_PRINTF("\nS9: rxlen = %u\n", msg_len);

        if(msg_len > 0)
        {
            for (uint16_t i=0; i<msg_len; i++)
            {
                rData[i] = lBuf[i];
                /* Debug */
                //ARTEMIS_DEBUG_PRINTF("%c", lBuf[i]);
            }
            *len = msg_len;

            do{
                if(_parse_response(rData, "OK\r\n", *len) != -1)
                {
                    //ARTEMIS_DEBUG_PRINTF("\nTemp OK\n");
                    result = S9_RESULT_OK;
                }
                else if(_parse_response(rData, "ERROR\r\n", *len) != -1)
                {
                    result = S9_RESULT_ERROR;
                }
                if(result == S9_RESULT_FAIL)
                {
                    //artemis_max14830_UART_Read (pS9->device.uart.port, lBuf, &rem_len);
                    rem_len = MAX14830_UART_Read_direct (pS9->device.uart.port, lBuf);
                    //rem_len = MAX14830_UART_Read (pS9->device.uart.port, lBuf);
                    if (rem_len > 0)
                    {
                        //ARTEMIS_DEBUG_PRINTF("\nremaining Lenth happened = %u \n", rem_len);
                        for (uint16_t i=0; i<rem_len; i++)
                        {
                            rData[*len+i] = lBuf[i];
                            /* Debug */
                            //ARTEMIS_DEBUG_PRINTF("%c", lBuf[i]);
                        }
                        *len += rem_len;
                    }
                    wait++;
                }
                else
                {
                    contFlag = false;
                    wait = 0;
                }
                wait++;
                //ARTEMIS_DEBUG_PRINTF("\nPrinting in S9 inner loop\n");
                //am_hal_systick_delay_us(50);
            } while(result == S9_RESULT_FAIL && wait < 5000);
        }
        //am_hal_systick_delay_us(50);
        //ARTEMIS_DEBUG_PRINTF("\nPrinting in S9 outer loop\n");
    }
    return result;
}

STATIC S9_result_t receive_response_RTOS(uint8_t *rData, uint16_t *len)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t msg_len = 0;
    uint8_t rem_len = 0;
    uint8_t lBuf[256] = {0};
    bool contFlag = true;
    uint32_t wait = 0;

    while (contFlag)
    {
        msg_len = MAX14830_UART_Read (pS9->device.uart.port, lBuf);
        //ARTEMIS_DEBUG_PRINTF("\nS9: rxlen = %u\n", msg_len);

        if(msg_len > 0)
        {
            for (uint16_t i=0; i<msg_len; i++)
            {
                rData[i] = lBuf[i];
                /* Debug */
                //ARTEMIS_DEBUG_PRINTF("%c", lBuf[i]);
            }
            *len = msg_len;

            do{
                if(_parse_response(rData, "OK\r\n", *len) != -1)
                {
                    //ARTEMIS_DEBUG_PRINTF("\nTemp OK\n");
                    result = S9_RESULT_OK;
                    break;
                }
                else if(_parse_response(rData, "ERROR\r\n", *len) != -1)
                {
                    //ARTEMIS_DEBUG_PRINTF("\nTemp ERROR\n");
                    result = S9_RESULT_ERROR;
                    break;
                }
                if(result == S9_RESULT_FAIL)
                {
                    rem_len = MAX14830_UART_Read (pS9->device.uart.port, lBuf);
                    if (rem_len > 0)
                    {
                        //ARTEMIS_DEBUG_PRINTF("\nremaining Lenth happened = %u \n", rem_len);
                        for (uint16_t i=0; i<rem_len; i++)
                        {
                            rData[*len+i] = lBuf[i];
                            /* Debug */
                            //ARTEMIS_DEBUG_PRINTF("%c", lBuf[i]);
                        }
                        *len += rem_len;
                    }
                    wait++;
                }
                else
                {
                    contFlag = false;
                    wait = 0;
                }
                wait++;
                //ARTEMIS_DEBUG_PRINTF("\nPrinting in S9 inner loop\n");
                //am_hal_systick_delay_us(50);
            } while(result == S9_RESULT_FAIL && wait < 5000);
        }

        if ( result == S9_RESULT_OK || result == S9_RESULT_ERROR )
        {
            break;
        }
        //ARTEMIS_DEBUG_PRINTF("\nS9: stuck here ? \n");
    }
    return result;
}
