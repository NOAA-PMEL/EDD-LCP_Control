#ifndef BSP_UART_H
#define BSP_UART_H

#include "am_bsp_pins.h"

typedef enum{
    BSP_UART_COM0,
    BSP_UART_COM1,
    BSP_UART_COM2,
    BSP_UART_COM3,
    BSP_UART_COM4,
    BSP_UART_LOG,
    BSP_UART_DEBUG
}e_uart_t;

void bsp_uart_init(void);
void bsp_uart_enable(e_uart_t port);
void bsp_uart_diable(e_uart_t port);
void bsp_uart_low_power(e_uart_t port);
void bsp_uart_set_baudrate(e_uart_t port, uint32_t baudrate );
void bsp_uart_putc(e_uart_t port, char c);
void bsp_uart_puts(e_uart_t port, char *pStr, uint32_t len);
char bsp_uart_getc(e_uart_t port);
uint32_t bsp_uart_gets(e_uart_t port, char *str, uint32_t len);
void BSP_UART_Test(void);

#endif // BSP_UART_H