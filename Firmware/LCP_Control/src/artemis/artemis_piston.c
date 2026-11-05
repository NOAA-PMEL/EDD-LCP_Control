#include "artemis_piston.h"

//*****************************************************************************
// Required built-ins.
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

//*****************************************************************************
// Standard AmbiqSuite includes.
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
// Artemis specific files
//*****************************************************************************
#include "artemis_i2c.h"
#include "artemis_debug.h"
#include "artemis_stream.h"

//*****************************************************************************
// FreeRTOS include files.
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

//*****************************************************************************
// Project Files
//*****************************************************************************
#include "am_bsp_pins.h"
#include "buffer_c.h"

//*****************************************************************************
// Macros & Constants
//*****************************************************************************
#define ARTEMIS_PISTON_BUFFER_LENGTH ( 128 )

//*****************************************************************************
// Structs
//*****************************************************************************
typedef uint8_t module_buffer_t[ARTEMIS_PISTON_BUFFER_LENGTH];
typedef struct s_module_t
{
    artemis_i2c_t i2c;
    module_buffer_t txbuffer;
    module_buffer_t rxbuffer;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    } power;
} module_t;

//*****************************************************************************
// Static Variables
//*****************************************************************************
static module_t module;

//*****************************************************************************
// Global Functions
//*****************************************************************************
static inline void pis_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        uint32_t ticks = pdMS_TO_TICKS(ms);
        vTaskDelay(ticks > 0 ? ticks : 1);
    }
    else
    {
        am_util_delay_ms(ms);
    }
}

void artemis_piston_i2c_initialize(uint8_t i2c_addr)
{
    artemis_i2c_t *i2c = &module.i2c;
    module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_PWR_CTRL_EN;
    module.power.pin = AM_BSP_GPIO_PWR_CTRL_EN;

    i2c->address = i2c_addr;
    i2c->iom.module = 2;
    i2c->iom.config.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
    i2c->iom.config.ui32ClockFreq = AM_HAL_IOM_400KHZ;   // 100% matched to working 10/30
    artemis_iom_initialize(&i2c->iom);

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCL, g_AM_BSP_GPIO_IOM2_SCL));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SDA, g_AM_BSP_GPIO_IOM2_SDA));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.power.pin, *module.power.pinConfig));

    artemis_piston_i2c_power_off();
    am_hal_systick_delay_us(1000);
    am_hal_systick_delay_us(50000);
    artemis_piston_i2c_power_on();
    am_hal_systick_delay_us(100000); // 100ms boot settling
}

void artemis_piston_i2c_uninitialize(void)
{
    artemis_i2c_t *i2c = &module.i2c;
    i2c->iom.module = 2;
    artemis_iom_uninitialize(&i2c->iom);
}

void artemis_piston_i2c_power_on(void)
{
    am_hal_gpio_output_set(module.power.pin);
}

void artemis_piston_i2c_power_off(void)
{
    am_hal_gpio_output_clear(module.power.pin);
}

void artemis_piston_i2c_send_msg(uint8_t *msg, uint16_t len, bool stop)
{
    artemis_i2c_t *i2c = &module.i2c;
    artemis_stream_t txstream = {0};
    artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PISTON_BUFFER_LENGTH);
    artemis_stream_reset(&txstream);

    while (len > 0)
    {
        if (len > ARTEMIS_PISTON_BUFFER_LENGTH)
        {
            artemis_stream_write(&txstream, msg, ARTEMIS_PISTON_BUFFER_LENGTH);
            artemis_i2c_send(i2c, true, &txstream);
            artemis_stream_reset(&txstream);
            len -= ARTEMIS_PISTON_BUFFER_LENGTH;
            msg += ARTEMIS_PISTON_BUFFER_LENGTH;
        }
        else
        {
            artemis_stream_write(&txstream, msg, len);
            artemis_i2c_send(i2c, stop, &txstream);
            len = 0;
        }
    }
}

uint16_t artemis_piston_i2c_read_data(uint8_t *pBuf)
{
    artemis_i2c_t *i2c = &module.i2c;

    artemis_stream_t rxstream = {0};
    artemis_stream_t txstream = {0};
    artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PISTON_BUFFER_LENGTH);
    artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PISTON_BUFFER_LENGTH);

    /** Query data length */
    artemis_stream_put(&txstream, ARTEMIS_PISTON_BUFFER_LENGTH);
    artemis_i2c_send(i2c, false, &txstream);

    artemis_i2c_receive(i2c, true, &rxstream, 2);

    uint8_t u8Len[2];
    artemis_stream_read(&rxstream, u8Len, 2);

    if (u8Len[1] == 0xFF)
    {
        return 0;
    }

    uint16_t len = (u8Len[0] << 8) | u8Len[1];
    uint8_t *pBufStart = pBuf;

    if (len > 0)
    {
        artemis_stream_reset(&txstream);
        artemis_stream_reset(&rxstream);

        artemis_stream_put(&txstream, ARTEMIS_PISTON_I2C_DATA_REG);
        artemis_i2c_send(i2c, false, &txstream);

        while (len > 0)
        {
            if (len > ARTEMIS_PISTON_BUFFER_LENGTH)
            {
                artemis_i2c_receive(i2c, false, &rxstream, ARTEMIS_PISTON_BUFFER_LENGTH);
                artemis_stream_read(&rxstream, pBuf, ARTEMIS_PISTON_BUFFER_LENGTH);
                artemis_stream_reset(&rxstream);
                pBuf += ARTEMIS_PISTON_BUFFER_LENGTH;
                len -= ARTEMIS_PISTON_BUFFER_LENGTH;
            }
            else
            {
                artemis_i2c_receive(i2c, true, &rxstream, len);
                artemis_stream_read(&rxstream, pBuf, len);
                pBuf += len;
                len = 0;
            }
        }
    }

    return (uint16_t)(pBuf - pBufStart);
}

void artemis_piston_i2c_read(uint8_t addr, uint8_t *data, uint16_t len)
{
    artemis_i2c_t *i2c = &module.i2c;

    artemis_stream_t rxstream = {0};
    artemis_stream_t txstream = {0};
    artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PISTON_BUFFER_LENGTH);
    artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PISTON_BUFFER_LENGTH);

    artemis_stream_reset(&txstream);
    artemis_stream_reset(&rxstream);

    // 1. Send address with STOP = TRUE
    artemis_stream_put(&txstream, addr);
    artemis_i2c_send(i2c, true, &txstream);

    // 2. Non-blocking delay for slave MCU decode
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms yields CPU to other tasks
    }
    else
    {
        am_hal_systick_delay_us(50000);
    }

    // 3. Receive data from the slave
    artemis_i2c_receive(i2c, true, &rxstream, len);
    artemis_stream_read(&rxstream, data, len);
}

void artemis_piston_set_write_mode(bool state)
{
    uint8_t key = 0x00;
    uint8_t msg[2] = {0x07, 0x00};

    if (state)
    {
        key = 0xA5;
    }
    msg[1] = key;

    artemis_piston_i2c_send_msg(msg, 2, true);
    pis_delay_ms(100);  // Allow slave MSP430 eUSCI ISR to commit RAM_VAR_WRITE
}