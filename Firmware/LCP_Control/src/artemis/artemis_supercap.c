/**
 * @file artemis_supercap.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */


//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>

#include "artemis_mcu.h"

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp_pins.h"

//*****************************************************************************
//
// Artemis specific files
//
//*****************************************************************************
#include "artemis_debug.h"
#include "artemis_supercap.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#define FREERTOS

//*****************************************************************************
//
//  Macros
//
//*****************************************************************************
#define ARTEMIS_SC_POWER_TIMEOUT_SEC ( 60 )
//*****************************************************************************
//
// Structs, Enums & Typedefs
//
//*****************************************************************************
typedef struct s_module_t 
{
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }power;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }shutdown;
    struct {
        uint32_t pin;
        am_hal_gpio_pincfg_t *pinConfig;
    }good;
}module_t;

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************
static module_t module = {0};

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void artemis_sc_initialize(void)
{
    module.power.pin = AM_BSP_GPIO_SC_ON;
    module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_SC_ON;
    module.shutdown.pin = AM_BSP_GPIO_SC_NSHDN;
    module.shutdown.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_SC_NSHDN;
    module.good.pin = AM_BSP_GPIO_SC_PGOOD;
    module.good.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_SC_PGOOD;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.power.pin, *module.power.pinConfig));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.shutdown.pin, *module.shutdown.pinConfig));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.good.pin, *module.good.pinConfig));
}

bool artemis_sc_power_startup(void)
{
	artemis_sc_power_on();
	for(uint16_t i=0; i<ARTEMIS_SC_POWER_TIMEOUT_SEC; i++)
	{
		if(artemis_sc_power_good())
		{
			ARTEMIS_DEBUG_PRINTF("Capacitors charged\n");
			return true;
		}
		ARTEMIS_DEBUG_PRINTF("Capacitor charging...\n");

		/** 1 second delay */
#ifdef FREERTOS
        vTaskDelay(pdMS_TO_TICKS(1000UL));
#else
        am_hal_systick_delay_us(1000000);
#endif

	}

	return false;
}

void artemis_sc_power_on(void)
{
	am_hal_gpio_output_set(module.power.pin);
	am_hal_gpio_output_set(module.shutdown.pin);
}

void artemis_sc_power_off(void)
{
	am_hal_gpio_output_clear(module.shutdown.pin);
	am_hal_gpio_output_clear(module.power.pin);
}

bool artemis_sc_power_good(void)
{
	return am_hal_gpio_input_read(module.good.pin);
}
