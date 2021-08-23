//*****************************************************************************
//
//! @file freertos_lowpower.c
//!
//! @brief Example of the app running under FreeRTOS.
//!
//! This example implements LED task within the FreeRTOS framework. It monitors
//! three On-board buttons, and toggles respective on-board LEDs in response.
//! To save power, this application is compiled without print
//! statements by default. To enable them, add the following project-level
//! macro definitions.
//!
//! AM_DEBUG_PRINTF
//!
//! If enabled, debug messages will be sent over ITM.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "main.h"
#include "rtos.h"
#include "stdio.h"
#include "MAX14830.h"
#include "S9_temp.h"


#define IOM_MODULE  ( 0 )

//*****************************************************************************
//
// Enable printing to the console.
//
//*****************************************************************************
void
enable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_itm_printf_enable();
}

//*****************************************************************************
//
// Disable printing to the console.
//
//*****************************************************************************
void
disable_print_interface(void)
{
    //
    // Deinitialize a debug printing interface.
    //
    am_bsp_itm_printf_disable();
}
//static void 
//iom_set_up(void)
//{
//    //
//    // Enable power to IOM.
//    //
//    
//
//    //
//    // Set the required configuration settings for the IOM.
//    //
//    am_hal_iom_config(IOM_MODULE, &g_sIOMI2cConfig);
//
//    //
//    // Set pins high to prevent bus dips.
//    //
////     am_hal_gpio_out_bit_set(5);
////     am_hal_gpio_out_bit_set(6);
//
//// #ifdef INTERNAL_LOOPBACK
////     am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCLLB | AM_HAL_GPIO_PULLUP);
////     am_hal_gpio_pin_config(6, AM_HAL_PIN_6_SLSDALB | AM_HAL_GPIO_PULLUP);
////     AM_REG(GPIO, LOOPBACK) = IOM_MODULE;
//// #else
////     am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCL | AM_HAL_GPIO_PULLUP);
////     am_hal_gpio_pin_config(6, AM_HAL_PIN_6_M0SDA | AM_HAL_GPIO_PULLUP);
//// #endif
//
//    am_hal_iom_int_enable(IOM_MODULE, 0xFF);
//    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER0);
//
//    //
//    // Turn on the IOM for this operation.
//    //
//    am_bsp_iom_enable(IOM_MODULE);
//}
//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
    am_hal_wdt_halt();
    am_hal_wdt_int_disable();
    
    // Set the clock frequency.
    
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();


#ifndef NOFPU
    //
    // Enable the floating point module, and configure the core for lazy
    // stacking.
    //
    am_hal_sysctrl_fpu_enable();
    am_hal_sysctrl_fpu_stacking_enable(true);
#else
    am_hal_sysctrl_fpu_disable();
#endif

    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();

    //
    // Turn off unneeded Flash & SRAM
    //
#if defined(AM_PART_APOLLO3)
    am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_SRAM_96K);
#endif
#if defined(AM_PART_APOLLO3P)
    am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_SRAM_32K_DTCM);
#endif

    am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_MIN);

    //
    // Enable printing to the console.
    //
#ifdef AM_DEBUG_PRINTF
    enable_print_interface();
#endif

    am_bsp_itm_printf_enable();
    //
    // Initialize plotting interface.
    //
    am_util_stdio_printf("Hello World!\n\n");
    printf("Program start\n");
    
    // Initi the IOM
//    iom_set_up();
    
    // Turn the COM0 Power Pin On (SoundNine Temperature Power)
    S9T_init(BSP_UART_COM0, &g_LCP_BSP_COM0_POWER_ON, AM_HAL_PIN_11_GPIO);
    S9T_enable();
//    
//    printf("The power to COM0 has been se\r\nt");
    
//    S9T_disable();
    
    /** Test the MAX Startups */
//    MAX14830_init();
//    MAX14830_enable(MAX14830_COM_PORT0);
//    MAX14830_disable(MAX14830_COM_PORT0);
      //
    // Run the application.
    //
//    run_tasks();
//    vTaskStartScheduler();
    
    //
    // We shouldn't ever get here.
    //
    while (1)
    {
      float t, r;
      t=1;
      r=2;
//      S9T_Read(&t, &r);
      BSP_UART_Test();
      printf("t=%.3f, r=%.3f\r\n", t, r);
      
//      am_util_delay_ms(3000);
      
    }

}

