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
#include "S9_temp.h"
#include "bsp_uart.h"
//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

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
#include "bsp_uart.h"
#include "MAX14830.h"

static sS9_t s9;
static sS9_t *pS9 = &s9;


void S9T_init( const e_uart_t port, const am_hal_gpio_pincfg_t *power, const uint32_t power_pin)
{

  assert(
          (port == BSP_UART_COM0) ||
          (port == BSP_UART_COM1) ||
          (port == BSP_UART_COM2) ||
          (port == BSP_UART_COM3)
            
         );

  /** Default Buadrate */
  pS9->device.uart.baudrate = 9600;
  
  /** Attach values to struct */
  pS9->device.power.pin = (am_hal_gpio_pincfg_t*)power;
  pS9->device.power.pin_number = (uint32_t)power_pin;

  pS9->device.uart.port = port;

  /** Initialize the COM Port Power Pin */
  am_hal_gpio_pinconfig(pS9->device.power.pin_number, *pS9->device.power.pin);// g_LCP_BSP_COM0_POWER_ON);
  S9T_OFF();
  
  /** Initialize the COM Port UART */
  bsp_uart_init();
  bsp_uart_set_baudrate(pS9->device.uart.port, pS9->device.uart.baudrate);
    
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
  bsp_uart_puts(pS9->device.uart.port, "SAMPLE\r", 7);
  char sampleStr[256];
//  bsp_uart_gets(pS9->device.uart.port, sampleStr, 256);
  return 0;
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
STATIC void _parse_msg(char *data, uint8_t len, sS9_t *p)
{
    uint8_t comma, end;
    uint8_t i;

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
STATIC void _parse_version(char *data, sS9_t *p )
{
    uint8_t i;
    char temp[255];
    float temp_f; 

    strcpy(temp,data);
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

    /* Find UID */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    
    uint8_t len = strlen(tok);
//    uint8_t sub_val = 0;
    uint8_t temp_hex[32];
    memset(temp,0,32);

    for(i=0;i<len;i++)
    {
        if( (tok[i] >= '0') && (tok[i] <= '9'))
        {
            temp_hex[i] = tok[i] - '0';
        } else if ( (tok[i] >= 'A') && (tok[i] <= 'F'))
        {
            temp_hex[i] = tok[i] -'A' + 10;
        } else {
            /** @todo - Error condition */
        }
    }
    i=0;
    uint8_t cnt =0;
    while(i < 32)
    {
        p->info.UID[cnt] = temp_hex[i++] << 4;
        p->info.UID[cnt] |= temp_hex[i++];
        cnt++;
    }
    
    /* Find Sensor */
    tok = strtok(NULL, " ");
    strcpy(p->info.sensor, tok);

    /** Find Firmware Major Version */
    tok = strtok(NULL, ".");
    p->info.firmware.major = (uint8_t) atoi(tok);

    /** Find Firmware Minor Version */
    tok = strtok(NULL, "\r");
    p->info.firmware.minor = (uint8_t) atoi(tok);

    /** Find Status */
    tok = strtok(NULL, "\r");
    strcpy(p->info.status, tok);
}