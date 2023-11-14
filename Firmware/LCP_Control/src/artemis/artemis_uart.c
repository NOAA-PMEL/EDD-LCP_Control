/// @file artemis_uart.c

#include "artemis_debug.h"
#include "artemis_uart.h"
#include "string.h"

#define ARTEMIS_UART_BUFFER_LENGTH (512) // transmit and receive buffer length

typedef uint8_t module_buffer_t[ARTEMIS_UART_BUFFER_LENGTH];

typedef struct s_module_t
{
    module_buffer_t txbuffer;
    module_buffer_t rxbuffer;
} module_t;

static module_t module;

void *uHandle;
static uint8_t rxbuffer[512] = {0};
static volatile uint16_t rxindex = 0;
static volatile bool Flag = false;

void artemis_uart_initialize(artemis_uart_t *uart, uint32_t baudrate)
{
    uart->config.ui32BaudRate = baudrate;
    uart->config.ui32DataBits = AM_HAL_UART_DATA_BITS_8;
    uart->config.ui32Parity = AM_HAL_UART_PARITY_NONE;
    uart->config.ui32StopBits = AM_HAL_UART_ONE_STOP_BIT;
    uart->config.ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE;
    uart->config.pui8TxBuffer = module.txbuffer;
    uart->config.ui32TxBufferSize = ARTEMIS_UART_BUFFER_LENGTH;
    uart->config.pui8RxBuffer = module.rxbuffer;
    uart->config.ui32RxBufferSize = ARTEMIS_UART_BUFFER_LENGTH;
    uart->config.ui32FifoLevels = AM_HAL_UART_TX_FIFO_7_8 | AM_HAL_UART_RX_FIFO_7_8 ;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_initialize(uart->module, &uart->handle));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_power_control(uart->handle, AM_HAL_SYSCTRL_WAKE, false));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_configure(uart->handle, &uart->config));

    NVIC_EnableIRQ((IRQn_Type)(UART1_IRQn));
    am_hal_uart_interrupt_enable(uart->handle, (AM_HAL_UART_INT_RX | AM_HAL_UART_INT_RX_TMOUT));
    am_hal_interrupt_master_enable();

    uHandle = uart->handle;
}

void artemis_uart_uninitialize(artemis_uart_t *uart)
{
    ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_power_control(uart->handle, AM_HAL_SYSCTRL_DEEPSLEEP, false));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_deinitialize(uart->handle));
    uart->handle = 0;
    uHandle = 0;
}

void artemis_uart_flush(artemis_uart_t *uart)
{
    ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_tx_flush(uart->handle));
}

void artemis_uart_send(artemis_uart_t *uart, artemis_stream_t *txstream)
{
    uint32_t read = 0;
    am_hal_uart_transfer_t transfer = {0};

    transfer.ui32Direction = AM_HAL_UART_WRITE;
    transfer.pui8Data = txstream->buffer;
    transfer.ui32NumBytes = txstream->written;
    transfer.ui32TimeoutMs = 0;
    transfer.pui32BytesTransferred = &read;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_transfer(uart->handle, &transfer));

    // update the number of bytes read from the txstream
    txstream->read = read;
    artemis_uart_flush(uart);
}

/* UART RX Interrupt, */
void am_uart1_isr(void)
{
    uint32_t status;

    am_hal_uart_interrupt_status_get(uHandle, &status, true);
    am_hal_uart_interrupt_clear(uHandle, status);
    am_hal_uart_interrupt_service(uHandle, status, 0);

    if (status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX))
    {
        uint32_t bytesread;

        am_hal_uart_transfer_t sRead =
        {
            .ui32Direction = AM_HAL_UART_READ,
            .pui8Data = (uint8_t *) &(rxbuffer[rxindex]),
            .ui32NumBytes = ARTEMIS_UART_BUFFER_LENGTH,
            .ui32TimeoutMs = 0,
            .pui32BytesTransferred = &bytesread,
        };

        ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_transfer(uHandle, &sRead));
        rxindex += bytesread;

        if (status & (AM_HAL_UART_INT_RX_TMOUT))
        {
            Flag = true;
            NVIC_DisableIRQ((IRQn_Type)(UART1_IRQn));
        }
    }
}

void artemis_uart_receive(artemis_stream_t *rxstream)
{
    //uint32_t wait_cycles = 0;;
    //while(Flag == false && wait_cycles < 5000)
    //{
    //    wait_cycles++;
    //}
    if (Flag == true)
    {
        memcpy(rxstream->buffer, rxbuffer, rxindex);
        rxstream->written = rxindex;
        memset(rxbuffer, 0, rxindex);
        Flag = false;
        rxindex = 0;
        NVIC_EnableIRQ((IRQn_Type)(UART1_IRQn));
    }
    else
    {
        memcpy(rxstream->buffer, NULL, 0);
        rxstream->written = 0;
        // do nothing
    }
}

//void artemis_uart_receive(artemis_uart_t *uart, artemis_stream_t *rxstream, uint32_t rxnumber)
//{
//	uint32_t written = 0;
//	am_hal_uart_transfer_t transfer = {0};
//	uint32_t status = AM_HAL_STATUS_SUCCESS;
//
//	transfer.ui32Direction = AM_HAL_UART_READ;
//	transfer.pui8Data = rxstream->buffer;
//	transfer.ui32NumBytes = rxnumber;
//	transfer.ui32TimeoutMs = 0;
//	transfer.pui32BytesTransferred = &written;
//
//	//ARTEMIS_DEBUG_HALSTATUS(am_hal_uart_transfer(uart->handle, &transfer));
//
//	status = am_hal_uart_transfer(uart->handle, &transfer);
//	if (status != AM_HAL_STATUS_SUCCESS){
//		ARTEMIS_DEBUG_PRINTF("UART:: RECEIVE ERROR\n");
//	}
//
//	// update the number of bytes written to the rxstream
//	rxstream->written = written;
//}
