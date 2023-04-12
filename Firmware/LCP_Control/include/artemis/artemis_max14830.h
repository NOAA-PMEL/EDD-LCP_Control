/**! @file artemis_max14830.h
 * @brief Maxim SPI-to-UART Converter
 *
 * @author Matt Casari, matthew.casari@noaa.gov
 * @date August 11, 2021
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the MAX14830 SPI to UART Converter for 4 channels
 *
 *
 * @bug  No known bugs
 */
#ifndef ARTEMIS_MAX14830_H
#define ARTEMIS_MAX14830_H

#include <stdint.h>

#define MAX14830_GPIO_PIN_TO_BIT(x)     ( 1u << (x % 4))

typedef enum 
{
	MAX14830_GPIO_PushPull = 0,
	MAX14830_GPIO_OpenDrain = 1
}eMAX14830_GPIO_Output_t;

typedef enum
{
	MAX14830_COM_PORT0    = 0,
	MAX14830_COM_PORT1    = 1,
	MAX14830_COM_PORT2    = 2,
	MAX14830_COM_PORT3    = 3  
}eMAX18430_ComPort_t;

typedef enum
{
	MAX14830_COM_BAUDRATE_1200    = 1200,
	MAX14830_COM_BAUDRATE_2400    = 2400,
	MAX14830_COM_BAUDRATE_4800    = 4800,
	MAX14830_COM_BAUDRATE_9600    = 9600,
	MAX14830_COM_BAUDRATE_19200   = 19200,
	MAX14830_COM_BAUDRATE_28800   = 28800,
	MAX14830_COM_BAUDRATE_38400   = 38400,
	MAX14830_COM_BAUDRATE_57600   = 57600,
	MAX14830_COM_BAUDRATE_115200  = 115200,
	MAX14830_COM_BAUDRATE_230400  = 230400
}eMAX14830_Baudrate_t;

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void artemis_max14830_init(void);
void artemis_max14830_enable(eMAX18430_ComPort_t port);
void artemis_max14830_disable(eMAX18430_ComPort_t port);
uint32_t artemis_max14830_Set_baudrate(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate);
void artemis_max14830_UART_Write(eMAX18430_ComPort_t port, uint8_t *txData, uint8_t txLen);
void artemis_max14830_UART_Read(eMAX18430_ComPort_t port, uint8_t *rxData, uint8_t *rxLen);
uint32_t artemis_max14830_UART_Read_bytes_waiting(eMAX18430_ComPort_t port);
void artemis_max14830_gpio_configure_output(uint8_t pin, eMAX14830_GPIO_Output_t type);
void artemis_max14830_gpio_set(uint8_t pin);
void artemis_max14830_gpio_clear(uint8_t pin);
#endif // _MAX14830_H
