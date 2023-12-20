/* @file main.c
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "bsp_uart.h"

#include "artemis_debug.h"
#include "artemis_mcu.h"
#include "artemis_time.h"

#include "FreeRTOSConfig.h"
#include "rtos.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#include "sensors.h"
#include "StateMachine.h"

#define FREE_RTOS

int main(void)
{
    // initialize mcu features
    artemis_mcu_initialize();
    // initialize debug features
    artemis_debug_initialize();

    am_util_delay_ms(1000);
    //am_util_stdio_terminal_clear();
    ARTEMIS_DEBUG_PRINTF("DEBUG  :: Hello LCP Controlboard\n");

    // initialize time functions
    artemis_time_initialize();

    /* for now, LEDS just for testing */
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_GREEN, g_AM_BSP_GPIO_LED_GREEN);
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_RED, g_AM_BSP_GPIO_LED_RED);
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_BLUE, g_AM_BSP_GPIO_LED_BLUE);
	//am_hal_gpio_output_set(AM_BSP_GPIO_LED_GREEN);
	//am_hal_gpio_output_set(AM_BSP_GPIO_LED_RED);
	//am_hal_gpio_output_set(AM_BSP_GPIO_LED_BLUE);
	am_hal_gpio_output_clear(AM_BSP_GPIO_LED_GREEN);
	am_hal_gpio_output_clear(AM_BSP_GPIO_LED_RED);
	am_hal_gpio_output_clear(AM_BSP_GPIO_LED_BLUE);

    // initialize the scheduler
    // artemis_scheduler_initialize();
    // run the application
    // artemis_scheduler_run();

#ifdef FREE_RTOS

    /* initialize sensors */
    STATE_initialize(SYSST_SimpleProfiler_mode);

    /* create tasks for PreDeploy_mode, Profile_mode, Popup_mode */

    configASSERT(xTaskCreate( (TaskFunction_t) STATE_Predeploy, "PreDeploy_task", 512, NULL, tskIDLE_PRIORITY + 4UL, NULL) == pdPASS);
    configASSERT(xTaskCreate( (TaskFunction_t) STATE_Profiler, "Profiler_task", 512, NULL, tskIDLE_PRIORITY + 4UL, NULL) == pdPASS);
    configASSERT(xTaskCreate( (TaskFunction_t) STATE_Popup, "Popup_task", 256, NULL, tskIDLE_PRIORITY + 3UL, NULL) == pdPASS);

    ARTEMIS_DEBUG_PRINTF("\n****************************\n");
    ARTEMIS_DEBUG_PRINTF("FreeRTOS here\n");
    ARTEMIS_DEBUG_PRINTF("schedular is going to start\n");
    ARTEMIS_DEBUG_PRINTF("******************************\n\n");
    vTaskStartScheduler();
    ARTEMIS_DEBUG_PRINTF("Do not get here\n");

#endif

    am_util_delay_ms(500);
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("do not get here in case of RTOS\n");
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        am_util_delay_ms(1000);
    }
}
