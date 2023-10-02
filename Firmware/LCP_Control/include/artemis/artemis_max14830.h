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
 * @note Controls the max14830 SPI to UART Converter for 4 channels
 *
 *
 * @bug  No known bugs
 */
#ifndef ARTEMIS_max14830_H
#define ARTEMIS_max14830_H

#include <stdint.h>

#define max14830_GPIO_PIN_TO_BIT(x)     ( 1u << (x % 4))

typedef enum 
{
	max14830_GPIO_PushPull = 0,
	max14830_GPIO_OpenDrain = 1
}emax14830_GPIO_Output_t;

typedef enum
{
	max14830_COM_PORT0    = 0,
	max14830_COM_PORT1    = 1,
	max14830_COM_PORT2    = 2,
	max14830_COM_PORT3    = 3
}emax18430_ComPort_t;

typedef enum
{
	max14830_COM_BAUDRATE_1200    = 1200,
	max14830_COM_BAUDRATE_2400    = 2400,
	max14830_COM_BAUDRATE_4800    = 4800,
	max14830_COM_BAUDRATE_9600    = 9600,
	max14830_COM_BAUDRATE_19200   = 19200,
	max14830_COM_BAUDRATE_28800   = 28800,
	max14830_COM_BAUDRATE_38400   = 38400,
	max14830_COM_BAUDRATE_57600   = 57600,
	max14830_COM_BAUDRATE_115200  = 115200,
	max14830_COM_BAUDRATE_230400  = 230400
}emax14830_Baudrate_t;

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void artemis_max14830_init(void);
void artemis_max14830_enable(emax18430_ComPort_t port);
void artemis_max14830_disable(emax18430_ComPort_t port);
uint32_t artemis_max14830_Set_baudrate(emax18430_ComPort_t port, emax14830_Baudrate_t baudrate);
void artemis_max14830_UART_Write(emax18430_ComPort_t port, uint8_t *txData, uint8_t txLen);
void artemis_max14830_UART_Read(emax18430_ComPort_t port, uint8_t *rxData, uint8_t *rxLen);
uint32_t artemis_max14830_UART_Read_bytes_waiting(emax18430_ComPort_t port);
void artemis_max14830_gpio_configure_output(uint8_t pin, emax14830_GPIO_Output_t type);
void artemis_max14830_gpio_set(uint8_t pin);
void artemis_max14830_gpio_clear(uint8_t pin);
#endif // _max14830_H
