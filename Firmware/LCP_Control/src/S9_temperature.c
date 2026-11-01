/** @file S9_temp.c
 *  @brief SoundNine OEM Temperature Sensor
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date September 30, 2020
 *  @version 0.1.0
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
STATIC S9_result_t receive_response(uint8_t *rData, uint8_t *len, uint16_t rDataMaxSize);
STATIC S9_result_t receive_response_RTOS(uint8_t *rData, uint16_t *len, uint16_t rDataMaxSize);
STATIC void _module_s9_stop_sampling(void);

void S9T_init(S9_init_param *p)
{
    /** Default Baudrate */
    pS9->device.uart.port = p->port;
    pS9->device.uart.baudrate = p->baudrate;

    /** Attach values to struct */
    pS9->device.power.pin = p->pin_config;
    pS9->device.power.pin_number = p->pin_number;

    /** Initialize the COM Port Power Pin */
    am_hal_gpio_pinconfig(pS9->device.power.pin_number, *pS9->device.power.pin);

    /* Set the baudrate */
    MAX14830_Set_baudrate(pS9->device.uart.port, pS9->device.uart.baudrate);

    /* Turn on the module */
    S9T_ON();
    am_hal_systick_delay_us(350000); // 350ms S9 microcontroller cold-boot delay

    /* Stop sampling */
    _module_s9_stop_sampling();

    // Fetch device info
    S9T_dev_info();
    ARTEMIS_DEBUG_PRINTF("S9 Temperature Sensor is initialized\n\n");

    /* Turn off the module */
    S9T_OFF();
}

STATIC void _module_s9_stop_sampling(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t sampleStr[SHORT_RESPONSE_BUFFER_SIZE];
    uint8_t rxLen = 0;

    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"stop\r", 5);
    result = receive_response(sampleStr, &rxLen, sizeof(sampleStr));
    if (result == S9_RESULT_OK)
    {
        ARTEMIS_DEBUG_PRINTF("S9 : Sampling Stopped\n");
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("S9 : Sampling did not stop\n");
    }
}

void _module_s9_stop_sampling_RTOS(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t response_buffer[SHORT_RESPONSE_BUFFER_SIZE];
    uint16_t response_len = 0;
    const uint8_t MAX_STOP_COMMAND_ATTEMPTS = 3;

    for (uint8_t attempt = 0; attempt < MAX_STOP_COMMAND_ATTEMPTS; ++attempt)
    {
        if (attempt > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            MAX14830_reset_rx_fifo((eMAX18430_ComPort_t)pS9->device.uart.port);
        }

        MAX14830_UART_Write((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t*)"stop\r", 5);
        result = receive_response_RTOS(response_buffer, &response_len, sizeof(response_buffer));

        if (result == S9_RESULT_OK)
        {
            ARTEMIS_DEBUG_PRINTF("S9: Sampling stopped successfully (attempt %u).\n", attempt + 1);
            break;
        }
    }

    if (result != S9_RESULT_OK)
    {
        ARTEMIS_DEBUG_PRINTF("S9: ERROR - Failed to stop sampling after retries.\n");
    }
}

void S9T_dev_info(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t verStr[256] = {0};
    uint8_t rxLen = 0;
    uint8_t i = 0;

    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"ver\r", 4);
    result = receive_response(verStr, &rxLen, sizeof(verStr));
    if (result == S9_RESULT_OK)
    {
        _parse_version(verStr, pS9, rxLen);

        ARTEMIS_DEBUG_PRINTF("S9 Temperature Sensor\n");
        ARTEMIS_DEBUG_PRINTF("**************************************\n");

        ARTEMIS_DEBUG_PRINTF("\tMID\t: ");
        for (i = 0; i < 8; i++) {
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.MID[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");

        ARTEMIS_DEBUG_PRINTF("\tC0\t: %.7f\n", pS9->info.C0);
        ARTEMIS_DEBUG_PRINTF("\tC1\t: %.7f\n", pS9->info.C1);
        ARTEMIS_DEBUG_PRINTF("\tC2\t: %.7f\n", pS9->info.C2);
        ARTEMIS_DEBUG_PRINTF("\tC3\t: %.7f\n", pS9->info.C3);
        ARTEMIS_DEBUG_PRINTF("\tR0\t: %.7f\n", pS9->info.R0);
        ARTEMIS_DEBUG_PRINTF("\tAverage\t: %u\n", pS9->info.average);

        ARTEMIS_DEBUG_PRINTF("\tUID\t: ");
        for (i = 0; i < 32; i++) {
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.UID[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");

        ARTEMIS_DEBUG_PRINTF("\tFW Ver\t: ");
        for (i = 0; i < 10; i++) {
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.sensor[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");

        ARTEMIS_DEBUG_PRINTF("\tStatus\t: ");
        for (i = 0; i < 2; i++) {
            ARTEMIS_DEBUG_PRINTF("%c", pS9->info.status[i]);
        }
        ARTEMIS_DEBUG_PRINTF("\n");
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("S9 : Something went wrong\n");
    }
}

void S9T_enable(void)
{
    S9T_ON();
}

void S9T_disable(void)
{
    S9T_OFF();
}

void S9T_ON(void)
{
    am_hal_gpio_output_clear(pS9->device.power.pin_number);
}

void S9T_OFF(void)
{
    am_hal_gpio_output_set(pS9->device.power.pin_number);
}

float S9T_Read_T(void)
{
    float t = NAN;
    S9T_Read(&t, NULL);
    return t;
}

float S9T_Read_R(void)
{
    float r = NAN;
    S9T_Read(NULL, &r);
    return r;
}

float S9T_Read(float *t, float *r)
{
    uint8_t sampleStr[64] = {0};
    uint16_t rxLen = 0;
    S9_result_t result = S9_RESULT_FAIL;

#ifdef FREE_RTOS
    MAX14830_UART_Write((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t *)"sample\r", 7);
    result = receive_response_RTOS(sampleStr, &rxLen, sizeof(sampleStr));
#else
    MAX14830_UART_Write_direct((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t *)"sample\r", 7);
    result = receive_response(sampleStr, &rxLen, sizeof(sampleStr));
#endif

    if (result == S9_RESULT_OK)
    {
        char *ptok = (char *)sampleStr;
        char *tok = strtok_r(ptok, "\r\n", &ptok);
        char *last_token = NULL;

        while (tok != NULL)
        {
            if (strcmp(tok, "OK") == 0)
            {
                if (last_token != NULL)
                {
                    module_s9_parse_msg(last_token, (uint8_t)strlen(last_token), pS9);

                    if (t != NULL)
                    {
                        *t = pS9->temperature;
                    }
                    if (r != NULL)
                    {
                        *r = pS9->resistance;
                    }
                    return pS9->temperature;
                }
                break;
            }
            last_token = tok;
            tok = strtok_r(NULL, "\r\n", &ptok);
        }
    }

    if (t != NULL) *t = NAN;
    if (r != NULL) *r = NAN;
    return NAN;
}

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

    for (i = 0; i <= (rxLen - pattLen); i++)
    {
        pos = k = i;
        for (j = 0; j < pattLen; j++)
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

STATIC void module_s9_parse_msg(char *data, uint8_t len, sS9_t *p)
{
    char *comma_ptr = strchr(data, ',');
    if (comma_ptr == NULL)
    {
        p->temperature = NAN;
        p->resistance = NAN;
        return;
    }

    /* Safely isolate resistance substring and guarantee null termination */
    char res_buf[32] = {0};
    size_t res_len = comma_ptr - data;
    if (res_len >= sizeof(res_buf)) res_len = sizeof(res_buf) - 1;
    memcpy(res_buf, data, res_len);
    res_buf[res_len] = '\0';
    p->resistance = (float)atof(res_buf);

    /* Advance past comma and any whitespace to parse temperature */
    char *temp_ptr = comma_ptr + 1;
    while (*temp_ptr == ' ') temp_ptr++;

    p->temperature = (float)atof(temp_ptr);
}

STATIC void _parse_version(uint8_t *data, sS9_t *p, uint8_t rxLen)
{
    char temp[256];
    float temp_f;

    strncpy(temp, (char*)data, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    char *tok;

    tok = strtok(temp, "=");
    tok = strtok(NULL, "\r");
    if (tok) strncpy(p->info.MID, tok, sizeof(p->info.MID) - 1);

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) {
        temp_f = atof(tok);
        p->info.C0 = temp_f;
    }

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) p->info.C1 = atof(tok);

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) p->info.C2 = atof(tok);

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) p->info.C3 = atof(tok);

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) p->info.R0 = atof(tok);

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) p->info.average = (float)atof(tok);

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) strncpy(p->info.UID, tok, sizeof(p->info.UID) - 1);

    tok = strtok(NULL, "\r\n");
    if (tok) strncpy(p->info.sensor, tok, sizeof(p->info.sensor) - 1);

    tok = strtok(NULL, "\r\n");
    if (tok) strncpy(p->info.status, tok, sizeof(p->info.status) - 1);
}

STATIC S9_result_t receive_response(uint8_t *rData, uint8_t *len, uint16_t rDataMaxSize)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t msg_len_read;
    uint16_t total_len_received = 0;
    uint8_t lBuf[256] = {0};
    uint32_t wait_cycles = 0;
    const uint32_t MAX_OUTER_WAIT_CYCLES = 100000;

    if (rDataMaxSize == 0) return S9_RESULT_FAIL;

    while (wait_cycles < MAX_OUTER_WAIT_CYCLES)
    {
        msg_len_read = MAX14830_UART_Read_direct((eMAX18430_ComPort_t)pS9->device.uart.port, lBuf);

        if (msg_len_read > 0)
        {
            uint16_t space_left = rDataMaxSize - total_len_received - 1;
            uint16_t bytes_to_copy = (msg_len_read > space_left) ? space_left : msg_len_read;

            memcpy(rData + total_len_received, lBuf, bytes_to_copy);
            total_len_received += bytes_to_copy;
            rData[total_len_received] = '\0';

            if (_parse_response(rData, "OK\r\n", total_len_received) != -1)
            {
                result = S9_RESULT_OK;
                break;
            }
            else if (_parse_response(rData, "ERROR\r\n", total_len_received) != -1)
            {
                result = S9_RESULT_ERROR;
                break;
            }
        }
        wait_cycles++;
    }

    *len = (uint8_t)total_len_received;
    return result;
}

STATIC S9_result_t receive_response_RTOS(uint8_t *rData, uint16_t *len, uint16_t rDataMaxSize)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint16_t current_read_len;
    uint16_t total_len_received = 0;
    uint8_t lBuf[256];

    // Cap outer loop attempts so task is not blocked indefinitely
    const uint32_t MAX_TOTAL_ATTEMPTS = 15;

    if (rDataMaxSize == 0) {
        if (len) *len = 0;
        return S9_RESULT_FAIL;
    }

    rData[0] = '\0';
    if (len) *len = 0;

    for (uint32_t attempt_count = 0; attempt_count < MAX_TOTAL_ATTEMPTS; ++attempt_count)
    {
        current_read_len = MAX14830_UART_Read((eMAX18430_ComPort_t)pS9->device.uart.port, lBuf);

        if (current_read_len > 0)
        {
            uint16_t space_left = (rDataMaxSize - 1) - total_len_received;
            uint16_t bytes_to_copy = (current_read_len > space_left) ? space_left : current_read_len;

            if (bytes_to_copy > 0)
            {
                memcpy(rData + total_len_received, lBuf, bytes_to_copy);
                total_len_received += bytes_to_copy;
                rData[total_len_received] = '\0';
            }

            if (_parse_response(rData, "OK\r\n", total_len_received) != -1)
            {
                result = S9_RESULT_OK;
                break;
            }
            else if (_parse_response(rData, "ERROR\r\n", total_len_received) != -1)
            {
                result = S9_RESULT_ERROR;
                break;
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    if (len) *len = total_len_received;
    return result;
}