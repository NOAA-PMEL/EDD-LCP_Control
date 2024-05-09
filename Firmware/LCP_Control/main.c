/* @file main.c
 *
 *  @brief LCP Control-board main file
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date September 30, 2020
 *  @version 0.0.1
 *
 *  @co-author Basharat Martin, basharat.martin@noaa.gov
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */

#include "main.h"
#include "sensors.h"
#include "StateMachine.h"

#define LCP_FREE_RTOS

int main(void)
{
    /** initialize mcu features */
    artemis_mcu_initialize();

    /** initialize debug features */
    artemis_debug_initialize();

    /** 1 second delay */
    am_util_delay_ms(1000);

    /* clear the screen output, if wanted */
    //am_util_stdio_terminal_clear();

    /** Test the Debug output */
    ARTEMIS_DEBUG_PRINTF("\n*****************************\n");
    ARTEMIS_DEBUG_PRINTF("DEBUG :: LCP Controlboard");
    ARTEMIS_DEBUG_PRINTF("\n*****************************\n");

    /** initialize time functions */
    artemis_time_initialize();

    /** LEDS just for testing */
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_GREEN, g_AM_BSP_GPIO_LED_GREEN);
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_RED, g_AM_BSP_GPIO_LED_RED);
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_BLUE, g_AM_BSP_GPIO_LED_BLUE);
	am_hal_gpio_output_set(AM_BSP_GPIO_LED_GREEN);
	am_hal_gpio_output_set(AM_BSP_GPIO_LED_RED);
	am_hal_gpio_output_set(AM_BSP_GPIO_LED_BLUE);
	//am_hal_gpio_output_clear(AM_BSP_GPIO_LED_GREEN);
	//am_hal_gpio_output_clear(AM_BSP_GPIO_LED_RED);
	//am_hal_gpio_output_clear(AM_BSP_GPIO_LED_BLUE);

#ifdef LCP_FREE_RTOS

    /** initialize sensors */
    STATE_initialize(SYSST_SimpleProfiler_mode);

    /** create tasks for PreDeploy_mode, Profile_mode, Popup_mode */
    configASSERT(xTaskCreate( (TaskFunction_t) STATE_Predeploy, "PreDeploy_task", 512, NULL, tskIDLE_PRIORITY + 4UL, NULL) == pdPASS);
    configASSERT(xTaskCreate( (TaskFunction_t) STATE_Profiler, "Profiler_task", 512, NULL, tskIDLE_PRIORITY + 4UL, NULL) == pdPASS);
    configASSERT(xTaskCreate( (TaskFunction_t) STATE_Popup, "Popup_task", 256, NULL, tskIDLE_PRIORITY + 3UL, NULL) == pdPASS);

    ARTEMIS_DEBUG_PRINTF("\n*****************************\n");
    ARTEMIS_DEBUG_PRINTF("FreeRTOS here\n");
    ARTEMIS_DEBUG_PRINTF("schedular is going to start\n");
    ARTEMIS_DEBUG_PRINTF("*****************************\n\n");
    vTaskStartScheduler();
    ARTEMIS_DEBUG_PRINTF("Do not get here\n");

#endif

    ARTEMIS_DEBUG_PRINTF("\n*****************************\n");
    ARTEMIS_DEBUG_PRINTF("FreeRTOS here\n");
    ARTEMIS_DEBUG_PRINTF("schedular is going to start\n");
    ARTEMIS_DEBUG_PRINTF("*****************************\n\n");
    vTaskStartScheduler();
    ARTEMIS_DEBUG_PRINTF("Do not get here\n");

    am_util_delay_ms(500);
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("do not get here in case of RTOS\n");
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        am_util_delay_ms(1000);
    }
}
