///
/// @file artemis_iom.c
///

#include "artemis_debug.h"
#include "artemis_iom.h"
#include "am_bsp.h"
#include "am_bsp_pins.h"
#include "am_util.h"

///
/// @brief Toggles SCL up to 9 times if SDA is stuck low by a slave device
///
static void artemis_iom_recover_i2c_bus(uint32_t scl_pin, uint32_t sda_pin)
{
    uint32_t sda_state = 1;
    am_hal_gpio_pinconfig(sda_pin, g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_state_read(sda_pin, AM_HAL_GPIO_INPUT_READ, &sda_state);

    if (sda_state == 0) // SDA is stuck low by a slave
    {
        ARTEMIS_DEBUG_PRINTF("IOM :: WARNING - SDA stuck low! Pulsing SCL to clear slave...\n");

        // Define open-drain output configuration for Apollo3 GPIO
        const am_hal_gpio_pincfg_t gpio_opendrain =
        {
            .uFuncSel       = 3, // GPIO mode
            .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
            .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
            .eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
            .eGPRdZero      = AM_HAL_GPIO_PIN_RDZERO_READPIN,
            .eIntDir        = AM_HAL_GPIO_PIN_INTDIR_NONE
        };

        // 1. Configure SCL as open-drain output, SDA as input
        am_hal_gpio_pinconfig(scl_pin, gpio_opendrain);

        // 2. Pulse SCL up to 9 times until slave releases SDA
        for (uint8_t i = 0; i < 9; i++)
        {
            am_hal_gpio_state_write(scl_pin, AM_HAL_GPIO_OUTPUT_CLEAR);
            am_util_delay_us(5);
            am_hal_gpio_state_write(scl_pin, AM_HAL_GPIO_OUTPUT_SET);
            am_util_delay_us(5);

            am_hal_gpio_state_read(sda_pin, AM_HAL_GPIO_INPUT_READ, &sda_state);
            if (sda_state != 0)
            {
                break; // Slave successfully released SDA line
            }
        }

        // 3. Issue manual STOP condition: SDA low -> high while SCL is high
        am_hal_gpio_pinconfig(sda_pin, gpio_opendrain);
        am_hal_gpio_state_write(sda_pin, AM_HAL_GPIO_OUTPUT_CLEAR);
        am_util_delay_us(5);
        am_hal_gpio_state_write(scl_pin, AM_HAL_GPIO_OUTPUT_SET);
        am_util_delay_us(5);
        am_hal_gpio_state_write(sda_pin, AM_HAL_GPIO_OUTPUT_SET);
        am_util_delay_us(5);
    }
}

///
///
///
void artemis_iom_initialize(artemis_iom_t *iom)
{
    // Auto-detect and clear stuck I2C lines if configuring I2C mode
    if (iom->config.eInterfaceMode == AM_HAL_IOM_I2C_MODE)
    {
        if (iom->module == 2)
        {
            // Piston Controller I2C bus pins (IOM2)
            artemis_iom_recover_i2c_bus(AM_BSP_GPIO_IOM2_SCL, AM_BSP_GPIO_IOM2_SDA);
        }
#ifdef AM_BSP_GPIO_IOM0_SCL
        else if (iom->module == 0)
        {
            artemis_iom_recover_i2c_bus(AM_BSP_GPIO_IOM0_SCL, AM_BSP_GPIO_IOM0_SDA);
        }
#endif
    }

    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_initialize(iom->module, &iom->handle));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_power_ctrl(iom->handle, AM_HAL_SYSCTRL_WAKE, false));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_configure(iom->handle, &iom->config));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_enable(iom->handle));
}

///
///
///
void artemis_iom_uninitialize(artemis_iom_t *iom)
{
    if (iom == NULL || iom->handle == NULL)
    {
        return; // Guard against redundant teardown calls
    }

    // Allow in-flight I2C/SPI clock pulses and STOP condition to complete
    am_hal_systick_delay_us(150);

    // Disable and clean up without fatal assertion traps
    am_hal_iom_disable(iom->handle);
    am_hal_iom_uninitialize(iom->handle);
    iom->handle = NULL; // Invalidate handle to block re-entry
}