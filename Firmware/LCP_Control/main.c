/* @file artemis_main.c
 *
 *
 */

#include "main.h"
#include "bsp_uart.h"
#include "stdlib.h"

#include "artemis_debug.h"
#include "artemis_mcu.h"
#include "artemis_time.h"

#include "FreeRTOSConfig.h"
#include "rtos.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#include "sensors.h"
#include "piston.h"
#include "artemis_rtc.h"
#include "temperature.h"
#include "control.h"
#include "StateMachine.h"
#include "datalogger.h"

#define FUNC

int main(void)
{
    // initialize mcu features
    artemis_mcu_initialize();
    // initialize debug features
    artemis_debug_initialize();

    am_util_delay_ms(1000);
    //am_util_stdio_terminal_clear();
    am_util_stdio_printf("NDEBUG :: Hello LCP Controlboard\n");
    ARTEMIS_DEBUG_PRINTF("DEBUG  :: Hello LCP Controlboard\n");

    // initialize time functions
    artemis_time_initialize();

    /* for now, LEDS just for testing */
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_GREEN, g_AM_BSP_GPIO_LED_GREEN);
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_RED, g_AM_BSP_GPIO_LED_RED);
	am_hal_gpio_pinconfig(AM_BSP_GPIO_LED_BLUE, g_AM_BSP_GPIO_LED_BLUE);
	am_hal_gpio_output_set(AM_BSP_GPIO_LED_GREEN);
	am_hal_gpio_output_set(AM_BSP_GPIO_LED_RED);
	am_hal_gpio_output_set(AM_BSP_GPIO_LED_BLUE);

    // initialize the scheduler
    // artemis_scheduler_initialize();
    // run the application
    // artemis_scheduler_run();

    /* datalogger init */
    //datalogger_init(4);
    //datalogger_predeploy_mode(0.0, 0.0, true);


#ifdef FUNC

    /* initialize sensors */
    SENS_initialize();
    PIS_initialize();
    PIS_set_piston_rate(1);
    SENS_sensor_depth_on();
    SENS_sensor_temperature_on();

    ///* set rate, and turn on temperature sensor*/
    //SENS_set_temperature_rate(1);
    ///* set rate, and turn on pressure sensor*/
    //SENS_set_depth_rate(2);
    /* set rate, and turn on gps*/
    //SENS_set_gps_rate(1);
    //artemis_rtc_initialize();
    //rtc_time time;

    TaskHandle_t Depth, Temp, xGPS;
    SENS_task_profile_sensors(&Depth, &Temp);
    //SENS_task_park_sensors(&Depth);
    //SENS_task_sample_depth_continuous(&Depth);
    //SENS_task_gps(&xGPS);
    ///* set rate, and turn on piston*/
    //PIS_calibration(true);
    //PIS_extend();
    PIS_set_volume(655);
    //PIS_Reset();

    /* setpoint for length*/
    //PIS_set_volume(0.05);

    /* create tasks locally for now */
    //configASSERT(xTaskCreate( (TaskFunction_t) task_temperature,"temperature task", 256, NULL, tskIDLE_PRIORITY + 3UL, NULL) == pdPASS);
    //configASSERT(xTaskCreate( (TaskFunction_t) task_depth, "Pressure task", 256, NULL, tskIDLE_PRIORITY + 4UL, NULL) == pdPASS);
    //configASSERT(xTaskCreate( (TaskFunction_t) task_gps, "GPS task", 256, NULL, tskIDLE_PRIORITY + 3UL, NULL) == pdPASS);
    //configASSERT(xTaskCreate( (TaskFunction_t) task_move_piston_to_length, "Piston length task", 256, NULL, tskIDLE_PRIORITY + 4UL, NULL) == pdPASS);
    configASSERT(xTaskCreate( (TaskFunction_t) task_move_piston_to_volume, "Piston volume task", 256, NULL, tskIDLE_PRIORITY + 4UL, NULL) == pdPASS);
    //configASSERT(xTaskCreate(task_move_piston_to_full, "Piston volume task", 128, NULL, 5, NULL) == pdPASS);

    ARTEMIS_DEBUG_PRINTF("\n\n****************************\n");
    ARTEMIS_DEBUG_PRINTF("FreeRTOS here\n");
    ARTEMIS_DEBUG_PRINTF("schedular is going to start\n");
    ARTEMIS_DEBUG_PRINTF("****************************\n\n");
    vTaskStartScheduler();
    ARTEMIS_DEBUG_PRINTF("Not here \n");

#endif

    am_util_delay_ms(500);
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("do not get here in case of RTOS\n");
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
