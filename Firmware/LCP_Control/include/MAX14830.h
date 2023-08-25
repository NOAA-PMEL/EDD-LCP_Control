/**! @file MAX14830.h
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
#ifndef _MAX14830_H
#define _MAX14830_H


/**********************************************************************************
* Includes
*********************************************************************************/

#include <stdint.h>


/**********************************************************************************
* Configuration Constants
*********************************************************************************/

/**********************************************************************************
* MACROS
*********************************************************************************/



/**********************************************************************************
* Typdefs
*********************************************************************************/
/**
 * @brief MAX14830 Port Selectin
 *
 */
typedef enum
{
  MAX14830_COM_PORT0    = 0,
  MAX14830_COM_PORT1    = 1,
  MAX14830_COM_PORT2    = 2,
  MAX14830_COM_PORT3    = 3  
}eMAX18430_ComPort_t;

/**
 * @brief Baudarate selections availalbe for MAX14830
 * 
 */
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

#define MAX14830_GPIO_PIN_TO_BIT(x)     ( 1u << (x % 4))


/**********************************************************************************
* Function Prototypes
*********************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void MAX14830_init(void);
void MAX14830_port_enable(eMAX18430_ComPort_t port);
void MAX14830_port_enable_direct(eMAX18430_ComPort_t port);
void MAX14830_port_disable(eMAX18430_ComPort_t port);
uint32_t MAX14830_Set_baudrate(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate);
void MAX14830_UART_Write(eMAX18430_ComPort_t port, uint8_t *data, uint16_t txlen);
void MAX14830_UART_Write_direct(eMAX18430_ComPort_t port, uint8_t *data, uint16_t len);
uint32_t MAX14830_UART_Read(eMAX18430_ComPort_t port, uint8_t *pData);
uint32_t MAX14830_UART_Read_bytes_waiting(eMAX18430_ComPort_t port);
uint16_t MAX14830_UART_Read_direct(eMAX18430_ComPort_t port, uint8_t *pData);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _MAX14830_H
