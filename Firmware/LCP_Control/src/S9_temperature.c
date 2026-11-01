/** @file S9_temp.c
 *  @brief SoundNine OEM Temperature Sensor
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date September 30, 2020
 *  @version 0.0.1
 */

//*****************************************************************************
// Standard AmbiqSuite includes.
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//*****************************************************************************
// Artemis specific files
//*****************************************************************************
#include "artemis_max14830.h"
#include "artemis_debug.h"
#include "artemis_stream.h"
#include "MAX14830.h"

//*****************************************************************************
// FreeRTOS include files.
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

//*****************************************************************************
// Project Files
//*****************************************************************************
#include "buffer_c.h"
#include "S9_temperature.h"

#define FREE_RTOS

static sS9_t s9;
static sS9_t *pS9 = &s9;

STATIC void module_s9_parse_msg(char *data, uint8_t len, sS9_t *p);
STATIC void _parse_version(uint8_t *data, sS9_t *p, uint8_t rxLen);
STATIC int16_t _parse_response(uint8_t *rData, char *pattern, uint16_t len);
STATIC S9_result_t receive_response(uint8_t *rData, uint8_t *len, uint16_t rDataMaxSize);
STATIC S9_result_t receive_response_RTOS(uint8_t *rData, uint16_t *len, uint16_t rDataMaxSize);
STATIC void _module_s9_stop_sampling(void);
void _module_s9_stop_sampling_RTOS(void);

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

    /* Allow S9 internal MCU settling time */
    am_util_delay_ms(150);

    /* Stop autonomous streaming */
    _module_s9_stop_sampling();

    /* Fetch device info */
    S9T_dev_info();
    ARTEMIS_DEBUG_PRINTF("S9 Temperature Sensor is initialized\n\n");

    /* Turn off the module until RTOS task begins */
    S9T_OFF();
}

STATIC void _module_s9_stop_sampling(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t sampleStr[64];
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
    uint8_t response_buffer[64];
    uint16_t response_len = 0;
    const uint8_t MAX_STOP_COMMAND_ATTEMPTS = 2;

    for (uint8_t attempt = 0; attempt < MAX_STOP_COMMAND_ATTEMPTS; ++attempt)
    {
        if (attempt > 0)
        {
            ARTEMIS_DEBUG_PRINTF("S9: Retrying 'stop\\r' command (attempt %u of %u).\n", attempt + 1, MAX_STOP_COMMAND_ATTEMPTS);
            vTaskDelay(pdMS_TO_TICKS(50));

            uint8_t dummy_flush_buffer[32];
            uint16_t flushed_bytes_count;
            uint8_t flush_attempts = 0;
            const uint8_t MAX_FLUSH_READ_ATTEMPTS = 5;

            do
            {
                flushed_bytes_count = MAX14830_UART_Read((eMAX18430_ComPort_t)pS9->device.uart.port, dummy_flush_buffer);
                flush_attempts++;
                if (flushed_bytes_count > 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            } while (flushed_bytes_count > 0 && flush_attempts < MAX_FLUSH_READ_ATTEMPTS);
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
        ARTEMIS_DEBUG_PRINTF("S9: ERROR - Failed to stop sampling after %u attempts.\n", MAX_STOP_COMMAND_ATTEMPTS);
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
    float t = 0.0f;
    S9T_Read(&t, NULL);
    return t;
}

float S9T_Read_R(void)
{
    float r = 0.0f;
    S9T_Read(NULL, &r);
    return r;
}

float S9T_Read(float *t, float *r)
{
    uint8_t sampleStr[64];
    uint16_t rxLen = 0;
    S9_result_t result = S9_RESULT_FAIL;

#ifdef FREE_RTOS
    MAX14830_UART_Write(pS9->device.uart.port, (uint8_t *)"sample\r", 7);
    result = receive_response_RTOS(sampleStr, &rxLen, sizeof(sampleStr));
#else
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t *)"sample\r", 7);
    result = receive_response(sampleStr, &rxLen, sizeof(sampleStr));
#endif

    if (result == S9_RESULT_OK)
    {
        char *ptok = (char *)sampleStr;
        char *tok = strtok_r(ptok, "\r\n", &ptok);
        char *last_token = NULL;
        char sample[64] = {0};

        while (tok != NULL)
        {
            if (strcmp(tok, "OK") == 0)
            {
                if (last_token != NULL)
                {
                    uint8_t len = strlen(last_token) + 1;
                    sample[len] = '\r';
                    for (uint8_t i = 0; i < len; i++)
                    {
                        sample[i] = last_token[i];
                    }

                    module_s9_parse_msg(sample, len + 1, pS9);
                    if (t) *t = pS9->temperature;
                    if (r) *r = pS9->resistance;
                }
                break;
            }
            last_token = tok;
            tok = strtok_r(NULL, "\r\n", &ptok);
        }
    }
    return (t ? *t : 0.0f);
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
    uint8_t comma = 0;
    uint8_t end = 0;
    uint8_t i;

    if (len <= 4)
    {
        p->temperature = NAN;
        p->resistance = NAN;
        return;
    }

    for (i = 0; i < len; i++)
    {
        if (data[i] == ',')
        {
            comma = i;
        }
        else if (data[i] == '\r')
        {
            end = i;
        }
    }

    if (end <= comma)
    {
        p->temperature = NAN;
        p->resistance = NAN;
        return;
    }

    char temp[32];
    strncpy(temp, &data[0], comma);
    temp[comma] = '\0';
    p->resistance = atof(temp);

    strncpy(temp, &data[comma + 1], end - comma);
    temp[end - comma] = '\0';
    p->temperature = atof(temp);
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
    if (tok) strcpy(p->info.MID, tok);

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
    if (tok) p->info.average = atof(tok);

    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    if (tok) strcpy(p->info.UID, tok);

    tok = strtok(NULL, "\r\n");
    if (tok) strcpy(p->info.sensor, tok);

    tok = strtok(NULL, "\r\n");
    if (tok) strcpy(p->info.status, tok);
}

STATIC S9_result_t receive_response(uint8_t *rData, uint8_t *len, uint16_t rDataMaxSize)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t msg_len_read;
    uint16_t total_len_received = 0;
    uint8_t lBuf[256] = {0};
    uint32_t wait_cycles = 0;
    const uint32_t MAX_OUTER_WAIT_CYCLES = 600; // 600ms total timeout

    if (rDataMaxSize == 0) return S9_RESULT_FAIL;

    rData[0] = '\0';
    if (len) *len = 0;

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

            if (total_len_received >= rDataMaxSize - 1)
            {
                result = S9_RESULT_FAIL;
                break;
            }
        }
        else
        {
            am_util_delay_ms(1);
            wait_cycles++;
        }
    }

    if (len) *len = (uint8_t)total_len_received;
    return result;
}

STATIC S9_result_t receive_response_RTOS(uint8_t *rData, uint16_t *len, uint16_t rDataMaxSize)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint16_t current_read_len;
    uint16_t total_len_received = 0;
    uint8_t lBuf[256];

    const uint32_t MAX_TOTAL_ATTEMPTS = 60;
    const uint8_t MAX_CONSECUTIVE_EMPTY_READS = 100;
    uint8_t consecutive_empty_reads = 0;

    if (rDataMaxSize == 0)
    {
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
            consecutive_empty_reads = 0;

            uint16_t space_left_for_data = (rDataMaxSize - 1) - total_len_received;
            uint16_t bytes_to_copy = (current_read_len > space_left_for_data) ? space_left_for_data : current_read_len;

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

            if (total_len_received >= rDataMaxSize - 1)
            {
                result = S9_RESULT_FAIL;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            consecutive_empty_reads++;
            if (consecutive_empty_reads >= MAX_CONSECUTIVE_EMPTY_READS)
            {
                result = S9_RESULT_FAIL;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    if (len) *len = total_len_received;
    return result;
}