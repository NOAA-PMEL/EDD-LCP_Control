  
///
/// @file artemis_iom.c
///

#include "artemis_debug.h"
#include "artemis_iom.h"

///
///
///
void artemis_iom_initialize(artemis_iom_t *iom)
{
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