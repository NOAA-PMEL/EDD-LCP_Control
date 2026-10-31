#include "artemis_piston.h"

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
#include "piston.h"

//*****************************************************************************
//
// Macros & Constants
//
//*****************************************************************************
#define ARTEMIS_PISTON_BUFFER_LENGTH      ( 128 )

// The unpatched slave code runs _delay_ms(3200) inside DRV8874_disable().
// We enforce a 3500ms master blackout margin to prevent bus collisions.
#define ARTEMIS_PISTON_SLAVE_FREEZE_MS    ( 3500 )
#define ARTEMIS_PISTON_RETRY_COUNT        ( 40 )
#define ARTEMIS_PISTON_RETRY_DELAY_MS     ( 100 )

//*****************************************************************************
//
// Structs
//
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
//
// Static Variables
//
//*****************************************************************************
static module_t module;

//*****************************************************************************
//
// Helper Functions
//
//*****************************************************************************

/// Delay helper that yields to FreeRTOS scheduler if active, or falls back to SysTick.
static void artemis_piston_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        am_hal_systick_delay_us(ms * 1000);
    }
}

/// Blackout helper: holds master I2C calls until slave exits _delay_ms(3200)
void artemis_piston_blackout_wait(void)
{
    artemis_piston_delay_ms(ARTEMIS_PISTON_SLAVE_FREEZE_MS);
}

/// Recovers I2C bus if MSP430 entered blocking delay with SDA held low
void artemis_piston_bus_clear(void)
{
    // Reconfigure SCL & SDA as open-drain GPIO outputs
    am_hal_gpio_pincfg_t gpio_out = AM_HAL_GPIO_PINCFG_OPENDRAIN;

    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCL, gpio_out);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SDA, gpio_out);

    am_hal_gpio_output_set(AM_BSP_GPIO_IOM2_SCL);
    am_hal_gpio_output_set(AM_BSP_GPIO_IOM2_SDA);
    am_hal_systick_delay_us(10);

    // Toggle SCL up to 9 cycles to prompt slave shift-register release
    for (uint8_t i = 0; i < 9; i++)
    {
        am_hal_gpio_output_clear(AM_BSP_GPIO_IOM2_SCL);
        am_hal_systick_delay_us(5);
        am_hal_gpio_output_set(AM_BSP_GPIO_IOM2_SCL);
        am_hal_systick_delay_us(5);
    }

    // Generate STOP condition
    am_hal_gpio_output_clear(AM_BSP_GPIO_IOM2_SDA);
    am_hal_systick_delay_us(5);
    am_hal_gpio_output_set(AM_BSP_GPIO_IOM2_SCL);
    am_hal_systick_delay_us(5);
    am_hal_gpio_output_set(AM_BSP_GPIO_IOM2_SDA);
    am_hal_systick_delay_us(10);

    // Re-bind pins back to IOM2 peripheral
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCL, g_AM_BSP_GPIO_IOM2_SCL);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SDA, g_AM_BSP_GPIO_IOM2_SDA);
}

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void artemis_piston_i2c_initialize(uint8_t i2c_addr)
{
    artemis_i2c_t *i2c = &module.i2c;
    module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_PWR_CTRL_EN;
    module.power.pin = AM_BSP_GPIO_PWR_CTRL_EN;

    i2c->address = i2c_addr;
    i2c->iom.module = 2;
    i2c->iom.config.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
    
    // Set to 100 kHz standard mode so slave micro software ISR
    // can reliably process incoming bytes and acknowledge frames.
    i2c->iom.config.ui32ClockFreq = AM_HAL_IOM_100KHZ;
    artemis_iom_initialize(&i2c->iom);

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCL, g_AM_BSP_GPIO_IOM2_SCL));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SDA, g_AM_BSP_GPIO_IOM2_SDA));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.power.pin, *module.power.pinConfig));

    artemis_piston_i2c_power_off();
    am_hal_systick_delay_us(50000);
    artemis_piston_i2c_power_on();
    am_hal_systick_delay_us(250000); // 250ms slave cold-boot margin
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

    artemis_stream_put(&txstream, ARTEMIS_PISTON_I2C_DATA_LEN_REG);
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

    // 1. Send register address pointer (stop = false for Repeated START)
    artemis_stream_put(&txstream, addr);
    artemis_i2c_send(i2c, false, &txstream);

    // 2. Allow slave micro sufficient time (5ms) to process address offset
    // and load the internal register into its output shift register.
    am_hal_systick_delay_us(5000);

    // 3. Receive data from the slave
    artemis_i2c_receive(i2c, true, &rxstream, len);

    // 4. Fill with sentinel 0xFF if read dropped/failed
    if (artemis_stream_read(&rxstream, data, len) < len)
    {
        memset(data, 0xFF, len);
    }
}

/// Non-blocking read with retry backoff to survive slave freezes
bool artemis_piston_i2c_read_retry(uint8_t addr, uint8_t *data, uint16_t len, uint8_t max_retries)
{
    uint8_t dummy[len];
    memset(dummy, 0xFF, len);

    for (uint8_t attempt = 0; attempt < max_retries; attempt++)
    {
        artemis_piston_i2c_read(addr, data, len);

        // A valid response will not be all 0xFF (sentinel returned on NACK/drop)
        if (memcmp(data, dummy, len) != 0)
        {
            return true;
        }

        // Bus may be locked by slave spinloop; clear pins and back off
        if (attempt % 5 == 4)
        {
            artemis_piston_bus_clear();
        }

        artemis_piston_delay_ms(ARTEMIS_PISTON_RETRY_DELAY_MS);
    }

    return false;
}

void artemis_piston_set_write_mode(bool state)
{
    uint8_t key = 0x00;
    uint8_t msg[2] = {PISTON_I2C_W_VAR_WRITE, 0x00};
    
    if (state)
    {
        key = PISTON_I2C_W_VAR_WRITE_KEY;
    }
    msg[1] = key;

    artemis_piston_i2c_send_msg(msg, 2, true);

    // Allow slave micro sufficient time to latch register 0x07 and exit ISR
    artemis_piston_delay_ms(25);
}

/// Commands actuator stop and executes blackout to avoid colliding with slave's 3.2s freeze
void artemis_piston_stop_and_wait(void)
{
    uint8_t stop_cmd[2] = {PISTON_I2C_RW_TRV_ENG, 0x00};

    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(stop_cmd, 2, true);
    artemis_piston_set_write_mode(false);

    // Yield CPU during slave's 3.2s freeze
    artemis_piston_blackout_wait();
}

/// Supervisory poll loop for homing cycles that tolerates multiple 3.2s delays
bool artemis_piston_wait_homing_complete(uint32_t timeout_ms)
{
    uint8_t zero_status = 0;
    uint32_t elapsed_ms = 0;
    const uint32_t poll_interval_ms = 250;

    while (elapsed_ms < timeout_ms)
    {
        // Read register 0x68 (TRV_zero) with tolerant retry logic
        if (artemis_piston_i2c_read_retry(PISTON_I2C_R_TRV_ZERO, &zero_status, 1, 3))
        {
            if (zero_status == 1)
            {
                return true; // Successfully homed
            }
        }

        artemis_piston_delay_ms(poll_interval_ms);
        elapsed_ms += poll_interval_ms;
    }

    return false; // Timed out
}