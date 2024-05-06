/**
 * @file artemis_ublox_i2c.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-28
 * 
 * 
 */

#include "artemis_ublox_i2c.h"
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
#include "artemis_i2c.h"
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
//#define TEST_ON_IOM4 ( true )

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
	artemis_i2c_t i2c;
	module_buffer_t txbuffer;
	module_buffer_t rxbuffer;
	struct {
		uint32_t pin;
		am_hal_gpio_pincfg_t *pinConfig;
	}power;
	struct {
		uint32_t pin;
		am_hal_gpio_pincfg_t *pinConfig;
	}extint;
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
void artemis_ublox_i2c_initialize(uint8_t i2c_addr)
{
    artemis_i2c_t *i2c = &module.i2c;

    #ifdef TEST_ON_IOM4
    module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_PRES_ON;
    module.power.pin = AM_BSP_GPIO_PRES_ON;
    module.extint.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_GPS_EXTINT;
    module.extint.pin = AM_BSP_GPIO_GPS_EXTINT;
    #else
    module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_GPS_ON;
    module.power.pin = AM_BSP_GPIO_GPS_ON;
    module.extint.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_GPS_EXTINT;
    module.extint.pin = AM_BSP_GPIO_GPS_EXTINT;
    #endif

    i2c->address = i2c_addr;
    
    #ifdef TEST_ON_IOM4
    i2c->iom.module = 4;
    #else
    i2c->iom.module = 1;
    #endif
    i2c->iom.config.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
    i2c->iom.config.ui32ClockFreq = AM_HAL_IOM_100KHZ;
    artemis_iom_initialize(&i2c->iom);
    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.power.pin, *module.power.pinConfig));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.extint.pin, *module.extint.pinConfig));
    artemis_ublox_i2c_power_on();
    
    #ifdef TEST_ON_IOM4
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCL, g_AM_BSP_GPIO_IOM4_SCL));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SDA, g_AM_BSP_GPIO_IOM4_SDA));
    #else
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL, g_AM_BSP_GPIO_IOM1_SCL));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA, g_AM_BSP_GPIO_IOM1_SDA));
    #endif
}

/**
 * @brief DeInitialize the UBLOX Module
 *
 * uninitializes IOM Module for I2C.
 *
 */
void artemis_ublox_i2c_uninitialize(void)
{
    artemis_i2c_t *i2c = &module.i2c;
    #ifdef TEST_ON_IOM4
    i2c->iom.module = 4;
    #else
    i2c->iom.module = 1;
    #endif
    artemis_iom_uninitialize(&i2c->iom);
}

/**
 * @brief Power Up the UBLOX Module
 *
 */
void artemis_ublox_i2c_power_on(void)
{
	am_hal_gpio_output_clear(module.power.pin);
    am_hal_systick_delay_us(300000);
}

/**
 * @brief Power Down the UBLOX Module
 *
 */
void artemis_ublox_i2c_power_off(void)
{
	am_hal_gpio_output_set(module.power.pin);
}

/**
 * @brief Send I2C message
 *
 * Send a message over I2C.
 * 
 * @param msg Pointer to message buffer
 * @param len Length of message to send
 * @param stop Send stop after transfer
 */
void artemis_ublox_i2c_send_msg(uint8_t *msg, uint16_t len, bool stop)
{
	artemis_i2c_t *i2c = &module.i2c;
	artemis_stream_t txstream = {0};
	artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_UBLOX_BUFFER_LENGTH);
	artemis_stream_reset(&txstream);

	while(len > 0)
	{
		if(len > ARTEMIS_UBLOX_BUFFER_LENGTH)
		{
			artemis_stream_write(&txstream, msg, ARTEMIS_UBLOX_BUFFER_LENGTH);
			artemis_i2c_send(i2c, false, &txstream);
			artemis_stream_reset(&txstream);
			len -= ARTEMIS_UBLOX_BUFFER_LENGTH;
		}
		else {
			artemis_stream_write(&txstream, msg, len);
			artemis_i2c_send(i2c, stop, &txstream);
			len =0;
		}
	}
}

/**
 * @brief Read the I2C data buffer @addr 0xFF
 * 
 * Check the length of the message @addr 0xFE/0xFD
 * if length > 0, read the message @addr 0xFF
 * 
 * Requires a data buffer >= 2048 bytes
 * 
 * @param pBuf pointer to transfer data buffer (size >= 2048 Bytes)
 * @return uint16_t length of message
 */
uint16_t artemis_ublox_i2c_read_data(uint8_t *pBuf)
{
	artemis_i2c_t *i2c = &module.i2c;

	artemis_stream_t rxstream = {0};
	artemis_stream_t txstream = {0};
	artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_UBLOX_BUFFER_LENGTH);
	artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_UBLOX_BUFFER_LENGTH);

	/** Send the command to retreive data length @addr 0xFD */
	artemis_stream_put(&txstream, ARTEMIS_UBLOX_I2C_DATA_LEN_REG);
	artemis_i2c_send(i2c, false, &txstream);

	artemis_i2c_receive(i2c, true, &rxstream, 2);

	uint8_t u8Len[2];
	artemis_stream_read(&rxstream, u8Len, 2);


	if(u8Len[1] == 0xFF)
	{
		/** Error, shouldn't be 0xFF */
		return 0;
	}

	int16_t len = (u8Len[0] << 8) | u8Len[1];

	uint8_t *pBufStart = pBuf;

	if(len > 0)
	{
		artemis_stream_reset(&txstream);
		artemis_stream_reset(&rxstream);

		/** Send the command to retreive data @addr 0xFF */
		artemis_stream_put(&txstream, ARTEMIS_UBLOX_I2C_DATA_REG);
		artemis_i2c_send(i2c, false, &txstream);

		while(len > 0)
		{
			if(len > ARTEMIS_UBLOX_BUFFER_LENGTH)
			{
				artemis_i2c_receive(i2c, false, &rxstream, ARTEMIS_UBLOX_BUFFER_LENGTH);
				artemis_stream_read(&rxstream, pBuf, ARTEMIS_UBLOX_BUFFER_LENGTH);
				artemis_stream_reset(&rxstream);
				pBuf += ARTEMIS_UBLOX_BUFFER_LENGTH;
				len -= ARTEMIS_UBLOX_BUFFER_LENGTH;
			}
			else {
				artemis_i2c_receive(i2c, true, &rxstream, len);
				artemis_stream_read(&rxstream, pBuf, ARTEMIS_UBLOX_BUFFER_LENGTH);
				pBuf += len;
				len = 0;
			}
		}
	}

	int16_t retsize = (pBuf - pBufStart);
	return (uint16_t) retsize;
}
