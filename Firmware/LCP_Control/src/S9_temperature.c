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

    /* Stop sampling immediately after power-up */
    _module_s9_stop_sampling();

    /* Fetch device info */
    S9T_dev_info();
    ARTEMIS_DEBUG_PRINTF("S9 Temperature Sensor is initialized\n\n");

    /* Turn off the module */
    S9T_OFF();
}

STATIC void _module_s9_stop_sampling(void)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint8_t sampleStr[64] = {0};  /* Expanded from 16 to 64 to hold in-flight measurement + OK */
    uint8_t rxLen = 0;

    /* Issue stop command directly */
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
    uint8_t response_buffer[64] = {0};  /* Expanded from 16 to 64 bytes */
    uint16_t response_len = 0;
    const uint8_t MAX_STOP_COMMAND_ATTEMPTS = 2;

    for (uint8_t attempt = 0; attempt < MAX_STOP_COMMAND_ATTEMPTS; ++attempt)
    {
        if (attempt > 0)
        {
            ARTEMIS_DEBUG_PRINTF("S9: Retrying 'stop\\r' command (attempt %u of %u).\n", attempt + 1, MAX_STOP_COMMAND_ATTEMPTS);

            vTaskDelay(pdMS_TO_TICKS(50));

            /* Drain software ring buffer before retry */
            uint8_t dummy_flush_buffer[64];
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

        /* Send the stop command */
        MAX14830_UART_Write((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t*)"stop\r", 5);

        /* Wait for response */
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
    uint8_t rx_buffer[128];
    uint8_t rx_len = 0;
    uint8_t flush_buf[32];
    S9_result_t result;

    ARTEMIS_DEBUG_PRINTF("S9 Temperature Sensor\n");
    ARTEMIS_DEBUG_PRINTF("**************************************\n");

    /* 1. Model ID */
    while (MAX14830_UART_Read_direct((eMAX18430_ComPort_t)pS9->device.uart.port, flush_buf) > 0);
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"mid\r", 4);
    result = receive_response(rx_buffer, &rx_len, sizeof(rx_buffer));
    if (result == S9_RESULT_OK)
    {
        rx_buffer[rx_len] = '\0';
        while (rx_len > 0 && (rx_buffer[rx_len - 1] == '\r' || rx_buffer[rx_len - 1] == '\n'))
        {
            rx_buffer[--rx_len] = '\0';
        }
        ARTEMIS_DEBUG_PRINTF("\tMID     : %s\n", (char*)rx_buffer);
    }

    /* 2. UID */
    while (MAX14830_UART_Read_direct((eMAX18430_ComPort_t)pS9->device.uart.port, flush_buf) > 0);
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"uid\r", 4);
    result = receive_response(rx_buffer, &rx_len, sizeof(rx_buffer));
    if (result == S9_RESULT_OK)
    {
        rx_buffer[rx_len] = '\0';
        while (rx_len > 0 && (rx_buffer[rx_len - 1] == '\r' || rx_buffer[rx_len - 1] == '\n'))
        {
            rx_buffer[--rx_len] = '\0';
        }
        ARTEMIS_DEBUG_PRINTF("\tUID     : %s\n", (char*)rx_buffer);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("\tUID     : TIMEOUT\n");
    }

    /* 3. Firmware Version */
    while (MAX14830_UART_Read_direct((eMAX18430_ComPort_t)pS9->device.uart.port, flush_buf) > 0);
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"ver\r", 4);
    result = receive_response(rx_buffer, &rx_len, sizeof(rx_buffer));
    if (result == S9_RESULT_OK)
    {
        rx_buffer[rx_len] = '\0';
        while (rx_len > 0 && (rx_buffer[rx_len - 1] == '\r' || rx_buffer[rx_len - 1] == '\n'))
        {
            rx_buffer[--rx_len] = '\0';
        }
        ARTEMIS_DEBUG_PRINTF("\tFW Ver  : %s\n", (char*)rx_buffer);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("\tFW Ver  : TIMEOUT\n");
    }

    /* 4. Status */
    while (MAX14830_UART_Read_direct((eMAX18430_ComPort_t)pS9->device.uart.port, flush_buf) > 0);
    MAX14830_UART_Write_direct(pS9->device.uart.port, (uint8_t*)"status\r", 7);
    result = receive_response(rx_buffer, &rx_len, sizeof(rx_buffer));
    if (result == S9_RESULT_OK)
    {
        rx_buffer[rx_len] = '\0';
        while (rx_len > 0 && (rx_buffer[rx_len - 1] == '\r' || rx_buffer[rx_len - 1] == '\n'))
        {
            rx_buffer[--rx_len] = '\0';
        }
        ARTEMIS_DEBUG_PRINTF("\tStatus  : %s\n", (char*)rx_buffer);
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
    uint8_t flush_buf[32];

#ifdef FREE_RTOS
    /* 1. Send stop and rate 0 to halt incoming autonomous transmissions */
    MAX14830_UART_Write((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t *)"stop\r", 5);
    vTaskDelay(pdMS_TO_TICKS(15));
    MAX14830_UART_Write((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t *)"rate 0\r", 7);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* 2. Flush autonomous frames spewed during the power-on warmup */
    while (MAX14830_UART_Read((eMAX18430_ComPort_t)pS9->device.uart.port, flush_buf) > 0);

    /* 3. Issue clean on-demand sample */
    MAX14830_UART_Write((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t *)"sample\r", 7);
    result = receive_response_RTOS(sampleStr, &rxLen, sizeof(sampleStr));
#else
    /* Non-RTOS / Boot path */
    MAX14830_UART_Write_direct((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t *)"stop\r", 5);
    am_util_delay_ms(15);
    MAX14830_UART_Write_direct((eMAX18430_ComPort_t)pS9->device.uart.port, (uint8_t *)"rate 0\r", 7);
    am_util_delay_ms(15);

    while (MAX14830_UART_Read_direct((eMAX18430_ComPort_t)pS9->device.uart.port, flush_buf) > 0);

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
    //uint8_t msg_len = 0;
    //uint8_t rem_len = 0;
    uint8_t msg_len_read;
    uint16_t total_len_received = 0; // PREVENT OVERFLOWS
    uint8_t lBuf[256] = {0};
    bool contFlag = true;
    uint32_t wait_cycles = 0;
    const uint32_t MAX_OUTER_WAIT_CYCLES = 100000;
    const uint32_t MAX_INNER_WAIT_CYCLES = 5000;

    if (rDataMaxSize == 0) return S9_RESULT_FAIL; // Cannot write to a zero-sized buffer

    while(contFlag && wait_cycles < MAX_OUTER_WAIT_CYCLES)
    {
        msg_len_read = MAX14830_UART_Read_direct (pS9->device.uart.port, lBuf);

        if(msg_len_read > 0)
        {
            uint16_t space_left = rDataMaxSize - total_len_received - 1; // -1 for null terminator
            uint16_t bytes_to_copy = (msg_len_read > space_left) ? space_left : msg_len_read;

            if (msg_len_read > space_left) {
                 ARTEMIS_DEBUG_PRINTF("S9: receive_response buffer nearly full. Truncating read.\n");
            }

            for (uint16_t i=0; i<bytes_to_copy; i++)
            {
                rData[total_len_received + i] = lBuf[i];
            }
            total_len_received += bytes_to_copy;
            rData[total_len_received] = '\0';

            uint32_t inner_wait_cycles_local = 0;
            do{
                if(_parse_response(rData, "OK\r\n", total_len_received) != -1)
                {
                    result = S9_RESULT_OK;
                }
                else if(_parse_response(rData, "ERROR\r\n", total_len_received) != -1)
                {
                    result = S9_RESULT_ERROR;
                }

                if(result == S9_RESULT_FAIL && inner_wait_cycles_local < MAX_INNER_WAIT_CYCLES)
                {
                    if (total_len_received >= rDataMaxSize -1) { // Buffer is full
                        result = S9_RESULT_FAIL; // Or a new error code like S9_RESULT_BUFFER_FULL
                        break; 
                    }
                    msg_len_read = MAX14830_UART_Read_direct (pS9->device.uart.port, lBuf);
                    if (msg_len_read > 0)
                    {
                        space_left = rDataMaxSize - total_len_received - 1;
                        bytes_to_copy = (msg_len_read > space_left) ? space_left : msg_len_read;

                        if (msg_len_read > space_left) {
                             ARTEMIS_DEBUG_PRINTF("S9: receive_response (inner) buffer nearly full. Truncating.\n");
                        }

                        for (uint16_t i=0; i<bytes_to_copy; i++)
                        {
                            rData[total_len_received+i] = lBuf[i];
                        }
                        total_len_received += bytes_to_copy;
                        rData[total_len_received] = '\0';
                    }
                    inner_wait_cycles_local++;
                }
                else 
                {
                    contFlag = false; 
                    break; 
                }
            } while(true); 
        }
        wait_cycles++;
        if(!contFlag) break; // Exit outer loop if inner loop determined result or timed out
    }
    *len = (uint8_t)total_len_received; // Cast back if original API needs uint8_t for len
    return result;
}

STATIC S9_result_t receive_response_RTOS(uint8_t *rData, uint16_t *len, uint16_t rDataMaxSize)
{
    S9_result_t result = S9_RESULT_FAIL;
    uint16_t current_read_len;
    uint16_t total_len_received = 0;
    uint8_t lBuf[256]; // Temporary buffer for MAX14830_UART_Read

    const uint32_t MAX_TOTAL_ATTEMPTS = 60; // Max outer loop iterations
    const uint8_t MAX_CONSECUTIVE_EMPTY_READS = 100; 
    uint8_t consecutive_empty_reads = 0;

    if (rDataMaxSize == 0) {
        if (len) *len = 0;
        return S9_RESULT_FAIL;
    }

    // Ensure rData is null-terminated from the start if buffer has space
    if (rDataMaxSize > 0) {
        rData[0] = '\0';
    }
    // Initialize output length
    if (len) *len = 0;

    for (uint32_t attempt_count = 0; attempt_count < MAX_TOTAL_ATTEMPTS; ++attempt_count) {
        // Try to read data from the UART via MAX14830
        current_read_len = MAX14830_UART_Read(pS9->device.uart.port, lBuf);

        if (current_read_len > 0) {
            consecutive_empty_reads = 0; // Reset counter as we received data

            // Calculate space left, ensuring space for null terminator
            uint16_t space_left_for_data = (rDataMaxSize - 1) - total_len_received;
            if (total_len_received >= rDataMaxSize - 1) {
                space_left_for_data = 0;
            }

            uint16_t bytes_to_copy = (current_read_len > space_left_for_data) ? space_left_for_data : current_read_len;

            if (bytes_to_copy > 0) {
                memcpy(rData + total_len_received, lBuf, bytes_to_copy);
                total_len_received += bytes_to_copy;
                rData[total_len_received] = '\0'; // Always null-terminate
            }

            // Check for terminators in the accumulated data
            if (_parse_response(rData, "OK\r\n", total_len_received) != -1) {
                result = S9_RESULT_OK;
                break; // Found "OK", successfully exit the attempts loop
            } else if (_parse_response(rData, "ERROR\r\n", total_len_received) != -1) {
                result = S9_RESULT_ERROR;
                break; // Found "ERROR", successfully exit the attempts loop
            }

            // If buffer is now full and no terminator was found, fail and exit
            if (total_len_received >= rDataMaxSize - 1) {
                result = S9_RESULT_FAIL;
                break; 
            }

            // Pacing delay: allow remaining UART bytes to accumulate in FIFO before next iteration
            vTaskDelay(pdMS_TO_TICKS(10));
        } else { // current_read_len == 0
            consecutive_empty_reads++;
            if (consecutive_empty_reads >= MAX_CONSECUTIVE_EMPTY_READS) {
                result = S9_RESULT_FAIL; // Timeout due to no new data
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms delay before next attempt
        }
    } // End of for loop (attempts_count)

    if (len) *len = total_len_received;

    return result;
}