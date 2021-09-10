#include "am_bsp.h"
#include "am_util.h"
//#include "MAX14830.h"

#include "bsp_uart.h"
#include "artemis_max14830.h"


#define MAX_UART_PACKET_SIZE        ( 256 )
uint8_t g_pui8UARTTXBuffer[MAX_UART_PACKET_SIZE];
void *g_pvLOG;
static am_hal_uart_config_t sUartConfig =
    {
        //
        // Standard UART settings: 115200-8-N-1
        //
        .ui32BaudRate    = 115200,
        .ui32DataBits    = AM_HAL_UART_DATA_BITS_8,
        .ui32Parity      = AM_HAL_UART_PARITY_NONE,
        .ui32StopBits    = AM_HAL_UART_ONE_STOP_BIT,
        .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

        //
        // Set TX and RX FIFOs to interrupt at three-quarters full.
        //
        .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_3_4 |
                           AM_HAL_UART_RX_FIFO_3_4),

        //
        // This code will use the standard interrupt handling for UART TX, but
        // we will have a custom routine for UART RX.
        //
        .pui8TxBuffer = g_pui8UARTTXBuffer,
        .ui32TxBufferSize = sizeof(g_pui8UARTTXBuffer),
        .pui8RxBuffer = 0,
        .ui32RxBufferSize = 0,
    };

void bsp_uart_init(void)
{
    /** Init All MAX1430 COM ports */
    artemis_max14830_init();
    

    /** Init all HAL COM ports */
    am_hal_uart_initialize(0, &g_pvLOG);
    am_hal_uart_power_control(g_pvLOG, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(g_pvLOG, &sUartConfig);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_CONSOLE_UART_TX, g_AM_BSP_GPIO_CONSOLE_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_CONSOLE_UART_RX, g_AM_BSP_GPIO_CONSOLE_UART_RX);
    
    
    /** Init the MAX14830 GPIO Outputs */
    artemis_max14830_gpio_configure_output(1, 0);
    artemis_max14830_gpio_configure_output(2, 0);
//    artemis_max14830_gpio_configure_output(3, 0);
    artemis_max14830_gpio_set(1);
//    artemis_max14830_gpio_clear(1);
    artemis_max14830_gpio_set(2);
//    artemis_max14830_gpio_clear(2);
//    artemis_max14830_gpio_set(3);
//    artemis_max14830_gpio_clear(3);

    /** Start the RTOS tasks? */
}

void bsp_uart_enable(e_uart_t port)
{
    switch(port)
    {
        case BSP_UART_COM0:
            artemis_max14830_enable(MAX14830_COM_PORT0);
            break;
        case BSP_UART_COM1:
            artemis_max14830_enable(MAX14830_COM_PORT1);
            break;
        case BSP_UART_COM2:
            artemis_max14830_enable(MAX14830_COM_PORT2);
            break;
        case BSP_UART_COM3:
            artemis_max14830_enable(MAX14830_COM_PORT3);
            break;
        case BSP_UART_LOG:
//            sUartConfig.ui32BaudRate = baudrate;
//            am_hal_uart_configure(g_pvLOG, &sUartConfig);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }
}

void bsp_uart_diable(e_uart_t port)
{
    switch(port)
    {
        case BSP_UART_COM0:
            artemis_max14830_disable(MAX14830_COM_PORT0);
            break;
        case BSP_UART_COM1:
            artemis_max14830_disable(MAX14830_COM_PORT1);
            break;
        case BSP_UART_COM2:
            artemis_max14830_disable(MAX14830_COM_PORT2);
            break;
        case BSP_UART_COM3:
            artemis_max14830_disable(MAX14830_COM_PORT3);
            break;
        case BSP_UART_LOG:
//            sUartConfig.ui32BaudRate = baudrate;
//            am_hal_uart_configure(g_pvLOG, &sUartConfig);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }
}

void bsp_uart_low_power(e_uart_t port)
{
    switch(port)
    {
        case BSP_UART_COM0:
            break;
        case BSP_UART_COM1:
            break;
        case BSP_UART_COM2:
            break;
        case BSP_UART_COM3:
            break;
        case BSP_UART_LOG:
//            sUartConfig.ui32BaudRate = baudrate;
//            am_hal_uart_power_control(g_pvLOG, AM_HAL_SYSCTRL_DEEPSLEEP, true);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }
}

void bsp_uart_set_baudrate(e_uart_t port, uint32_t baudrate )
{
    switch(port)
    {
        case BSP_UART_COM0:
            artemis_max14830_Set_baudrate(MAX14830_COM_PORT0, (eMAX14830_Baudrate_t)baudrate);
            break;
        case BSP_UART_COM1:
            artemis_max14830_Set_baudrate(MAX14830_COM_PORT1, (eMAX14830_Baudrate_t)baudrate);
            break;
        case BSP_UART_COM2:
            artemis_max14830_Set_baudrate(MAX14830_COM_PORT2, (eMAX14830_Baudrate_t)baudrate);
            break;
        case BSP_UART_COM3:
            artemis_max14830_Set_baudrate(MAX14830_COM_PORT3, (eMAX14830_Baudrate_t)baudrate);
            break;
        case BSP_UART_LOG:
            sUartConfig.ui32BaudRate = baudrate;
            am_hal_uart_configure(g_pvLOG, &sUartConfig);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }
}

void bsp_uart_putc(e_uart_t port, char c)
{
    uint32_t numTx = 0;
    am_hal_uart_transfer_t tx = {
                .ui32Direction = AM_HAL_UART_WRITE,
                .pui8Data = (uint8_t*)&c,
                .ui32NumBytes = 1,
                .ui32TimeoutMs = 0,
                .pui32BytesTransferred = &numTx,
            };
    switch(port)
    {
        case BSP_UART_COM0:
            artemis_max14830_UART_Write(MAX14830_COM_PORT0, &c, 1);
            break;
        case BSP_UART_COM1:
            artemis_max14830_UART_Write(MAX14830_COM_PORT1, &c, 1);
            break;
        case BSP_UART_COM2:
            artemis_max14830_UART_Write(MAX14830_COM_PORT2, &c, 1);
            break;
        case BSP_UART_COM3:
            artemis_max14830_UART_Write(MAX14830_COM_PORT3, &c, 1);
            break;
        case BSP_UART_LOG:
            
            
            am_hal_uart_transfer(g_pvLOG, &tx);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }
}

void bsp_uart_puts(e_uart_t port, char *pStr, uint32_t len)
{
    uint32_t numTx = 0;
    am_hal_uart_transfer_t tx = {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t*) pStr,
        .ui32NumBytes = len,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &numTx
    };
    switch(port)
    {
        case BSP_UART_COM0:
            artemis_max14830_UART_Write(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_COM1:
            artemis_max14830_UART_Write(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_COM2:
            artemis_max14830_UART_Write(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_COM3:
            artemis_max14830_UART_Write(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_LOG:
            
            am_hal_uart_transfer(g_pvLOG, &tx);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }
}

char bsp_uart_getc(e_uart_t port)
{
    char c = NULL;
    uint32_t numTx = 0;
    am_hal_uart_transfer_t tx = {
        .ui32Direction = AM_HAL_UART_READ,
        .pui8Data = (uint8_t*)&c,
        .ui32NumBytes = 1,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &numTx
    };
    switch(port)
    {
        case BSP_UART_COM0:
            artemis_max14830_UART_Read(MAX14830_COM_PORT0, &c, 1);
            break;
        case BSP_UART_COM1:
            artemis_max14830_UART_Read(MAX14830_COM_PORT1, &c, 1);
            break;
        case BSP_UART_COM2:
            artemis_max14830_UART_Read(MAX14830_COM_PORT2, &c, 1);
            break;
        case BSP_UART_COM3:
            artemis_max14830_UART_Read(MAX14830_COM_PORT3, &c, 1);
            break;
        case BSP_UART_LOG:
            
            am_hal_uart_transfer(g_pvLOG, &tx);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }

    return (char) c;
}

uint32_t bsp_uart_gets(e_uart_t port, char *pStr, uint32_t len)
{
    uint32_t rxLen = 0;
    am_hal_uart_transfer_t tx = {
        .ui32Direction = AM_HAL_UART_READ,
        .pui8Data = (uint8_t*)pStr,
        .ui32NumBytes = 1,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &rxLen
    };
    char c = NULL;
    switch(port)
    {
        case BSP_UART_COM0:
            rxLen = artemis_max14830_UART_Read(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_COM1:
            rxLen = artemis_max14830_UART_Read(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_COM2:
            rxLen = artemis_max14830_UART_Read(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_COM3:
            rxLen = artemis_max14830_UART_Read(MAX14830_COM_PORT0, pStr, len);
            break;
        case BSP_UART_LOG:
            
            am_hal_uart_transfer(g_pvLOG, &tx);
            break;
        case BSP_UART_DEBUG:
            break;
        default:
            break;
    }

    return c;
}


void BSP_UART_Test(void)
{
  char data[256];
  
  artemis_max14830_UART_Read(MAX14830_COM_PORT0, data, 256);
  
}