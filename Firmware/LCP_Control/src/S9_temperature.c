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

//*****************************************************************************
//
// Artemis specific files
//
//*****************************************************************************
#include "artemis_max14830.h"
#include "artemis_debug.h"
#include "artemis_stream.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
//#include "FreeRTOS.h"
//#include "task.h"
//#include "event_groups.h"
//#include "semphr.h"

//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "buffer_c.h"
#include "S9_temperature.h"

static sS9_t s9;
static sS9_t *pS9 = &s9;

STATIC void module_s9_parse_msg(char *data, uint8_t len, sS9_t *p);
STATIC void _parse_version(char *data, sS9_t *p, uint8_t rxLen);

//void S9T_init( const e_uart_t port, const am_hal_gpio_pincfg_t *power, const uint32_t power_pin)
void S9T_init(const e_uart_t port, uint32_t baudrate)
{

  //assert(
  //        (port == BSP_UART_COM0) ||
  //        (port == BSP_UART_COM1) ||
  //        (port == BSP_UART_COM2) ||
  //        (port == BSP_UART_COM3)
  //
  //       );

  /** Default Buadrate */
  pS9->device.uart.baudrate = baudrate;
  
  /** Attach values to struct */
  pS9->device.power.pin = (am_hal_gpio_pincfg_t*)&g_AM_BSP_GPIO_COM0_POWER_PIN;
  pS9->device.power.pin_number = AM_BSP_GPIO_COM0_POWER_PIN;

  pS9->device.uart.port = port;

  /** Initialize the COM Port Power Pin */
  am_hal_gpio_pinconfig(pS9->device.power.pin_number, *pS9->device.power.pin);// g_LCP_BSP_COM0_POWER_ON);
  S9T_OFF();
  S9T_ON();
  am_hal_systick_delay_us(500000);
  
  /** Initialize the COM Port UART */
  //bsp_uart_init();
  //bsp_uart_set_baudrate(pS9->device.uart.port, pS9->device.uart.baudrate);
  //bsp_uart_puts(pS9->device.uart.port, "\r", 1);
  //bsp_uart_puts(pS9->device.uart.port, "\r", 1);
  //bsp_uart_puts(pS9->device.uart.port, "stop\r", 5);
  //bsp_uart_puts(pS9->device.uart.port, "STOP\r", 5);
 
  artemis_max14830_Set_baudrate(pS9->device.uart.port, pS9->device.uart.baudrate);
  artemis_max14830_UART_Write (pS9->device.uart.port, "stop\r", 5);
  S9T_dev_info();  

}

void S9T_dev_info(void){

	char verStr[256] = {0};
	uint8_t rxLen = 0;
	uint8_t i=0;

	artemis_max14830_UART_Write (pS9->device.uart.port, "ver\r", 4);
	artemis_max14830_UART_Read (pS9->device.uart.port, verStr, &rxLen);

	// parse the return data
	_parse_version(verStr, pS9, rxLen);

	// print pS9 structure 
	ARTEMIS_DEBUG_PRINTF("\nS9 Temperature Sensor\n");
	ARTEMIS_DEBUG_PRINTF("*****************************\n");

	// MID
	ARTEMIS_DEBUG_PRINTF("\tMID\t= ");
	for (i=0; i<8; i++){
		ARTEMIS_DEBUG_PRINTF("%c", pS9->info.MID[i]);
	}
	ARTEMIS_DEBUG_PRINTF("\n");

	// C0-C3, R0
	ARTEMIS_DEBUG_PRINTF("\tC0\t= %.7f\n", pS9->info.C0);
	ARTEMIS_DEBUG_PRINTF("\tC1\t= %.7f\n", pS9->info.C1);
	ARTEMIS_DEBUG_PRINTF("\tC2\t= %.7f\n", pS9->info.C2);
	ARTEMIS_DEBUG_PRINTF("\tC3\t= %.7f\n", pS9->info.C3);
	ARTEMIS_DEBUG_PRINTF("\tR0\t= %.7f\n", pS9->info.R0);
	ARTEMIS_DEBUG_PRINTF("\tAverage\t= %u\n", pS9->info.average);

	// UID
	ARTEMIS_DEBUG_PRINTF("\tUID\t= ");
	for (i=0; i<32; i++){
		ARTEMIS_DEBUG_PRINTF("%c", pS9->info.UID[i]);
	}
	ARTEMIS_DEBUG_PRINTF("\n");

	// Sensor Name
	ARTEMIS_DEBUG_PRINTF("\tFirmware Ver = ");
	for (i=0; i<10; i++){
		ARTEMIS_DEBUG_PRINTF("%c", pS9->info.sensor[i]);
	}
	ARTEMIS_DEBUG_PRINTF("\n");

	//// Firmware Version
	////ARTEMIS_DEBUG_PRINTF("%c", pS9->info.firmware.major);
	////ARTEMIS_DEBUG_PRINTF("%c", pS9->info.firmware.minor);
	////ARTEMIS_DEBUG_PRINTF("\n");

	// Sensor Status
	ARTEMIS_DEBUG_PRINTF("\tStatus\t=  ");
	for (i=0; i<2; i++){
		ARTEMIS_DEBUG_PRINTF("%c", pS9->info.status[i]);
	}
	ARTEMIS_DEBUG_PRINTF("\n");
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
	//bsp_uart_puts(pS9->device.uart.port, "sample\r", 7);

	char sampleStr[256];
	uint8_t rxLen = 0;

	artemis_max14830_UART_Write (pS9->device.uart.port, "sample\r", 7);
	am_hal_systick_delay_us(750000);
	artemis_max14830_UART_Read (pS9->device.uart.port, sampleStr, &rxLen);
	//bsp_uart_gets(pS9->device.uart.port, sampleStr, 256);

	/** Find values */
	uint8_t *pStr = strstr(sampleStr, "sample\r\n");
	if(pStr != NULL)
	{
		pStr += 8;
		uint8_t len = strlen(pStr);
		module_s9_parse_msg(pStr, len, pS9);
		*t = pS9->temperature;
		*r = pS9->resistance;
	}
	else {
		*t = NAN;
		*r = NAN;
	}

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

STATIC void module_s9_parse_msg(char *data, uint8_t len, sS9_t *p)
{
	uint8_t comma, end;
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

STATIC void _parse_version(char *data, sS9_t *p, uint8_t rxLen)
{
	uint8_t i=0;
	char temp[256];
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
