/**
 * @file artemis_i9603n.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-30
 * 
 */


#include "artemis_i9603n.h"
//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

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
#include "artemis_uart.h"
#include "artemis_debug.h"
#include "artemis_stream.h"

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
#include "buffer_c.h"

//*****************************************************************************
//
//  Macros & Constants
//
//*****************************************************************************
#define I2C_MSG_LEN_MAX     ( 128 )
#define ARTEMIS_UBLOX_BUFFER_LENGTH (I2C_MSG_LEN_MAX)

//*****************************************************************************
//
// Structs
//
//*****************************************************************************
typedef uint8_t module_buffer_t[ARTEMIS_UBLOX_BUFFER_LENGTH];
typedef struct s_module_t
{
    artemis_uart_t uart;
    module_buffer_t txbuffer;
    module_buffer_t rxbuffer;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }power;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }tx;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }rx;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }ring_ind;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }net_avail;
} module_t;

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************
/**
 * @brief Module Parameters
 * 
 */
static module_t module;

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************


//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************

/**
 * @brief Initialize the UBLOX Module
 * 
 * Initializes Power Pin, EXTINT Pin & IOM Module for I2C.
 * 
 * @param i2c_addr I2C Address of Module
 */
void artemis_i9603n_initialize(void)
{
    artemis_uart_t *uart = &module.uart;

    /**  Select UART module */
    uart->module = ARTEMIS_UART_MODULE_1;

    /** Prep Pins */
    module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_IRIDIUM_ON;
    module.power.pin = AM_BSP_GPIO_IRIDIUM_ON;
    module.rx.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_IRIDIUM_UART_RX;
    module.rx.pin = AM_BSP_GPIO_IRIDIUM_UART_RX;
    module.tx.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_IRIDIUM_UART_RX;
    module.tx.pin = AM_BSP_GPIO_IRIDIUM_UART_RX;
    module.ring_ind.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_IRIDIUM_RING_IND;
    module.ring_ind.pin = AM_BSP_GPIO_IRIDIUM_RING_IND;
    module.net_avail.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_IRIDIUM_NET_AVAIL;
    module.net_avail.pin = AM_BSP_GPIO_IRIDIUM_NET_AVAIL;

    
    /** Configure Pins */
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IRIDIUM_ON, g_AM_BSP_GPIO_IRIDIUM_ON));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IRIDIUM_UART_RX, g_AM_BSP_GPIO_IRIDIUM_UART_RX));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IRIDIUM_UART_TX, g_AM_BSP_GPIO_IRIDIUM_UART_TX));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IRIDIUM_RING_IND, g_AM_BSP_GPIO_IRIDIUM_RING_IND));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IRIDIUM_NET_AVAIL, g_AM_BSP_GPIO_IRIDIUM_NET_AVAIL));

    /** Initialize UART Port */
    artemis_uart_initialize(uart);

    /** Set the baud rate */
    /** @todo SOON!!!! */

}


void artemis_i9603n_power_on(void)
{
    am_hal_gpio_output_set(module.power.pin);
}


void artemis_i9603n_power_off(void)
{
    am_hal_gpio_output_clear(module.power.pin);
}


bool artemis_i9603n_is_network_available(void)
{
    return am_hal_gpio_input_read(module.net_avail.pin);
}

bool artemis_i9603n_is_ringing(void)
{
    return am_hal_gpio_input_read(module.ring_ind.pin);
}

void artemis_i9603n_send(char *msg, uint16_t len)
{
    artemis_uart_flush(&module.uart);

    artemis_stream_t txstream = {0};
    artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_UBLOX_BUFFER_LENGTH);
    artemis_stream_reset(&txstream);
    artemis_stream_write(&txstream, (uint8_t*)msg, len);
    artemis_uart_send(&module.uart, &txstream);
    
}


uint16_t artemis_i9603n_receive(char *msg, uint16_t bufLen)
{
    artemis_stream_t rxstream = {0};
    artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_UBLOX_BUFFER_LENGTH);
    artemis_stream_reset(&rxstream);
    artemis_uart_receive(&module.uart, &rxstream, bufLen);
    artemis_stream_read(&rxstream, (uint8_t*)msg, rxstream.written);

    return rxstream.written;
}