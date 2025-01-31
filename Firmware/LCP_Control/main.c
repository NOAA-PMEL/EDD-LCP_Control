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

//Joe Flash & Datalogger Test Includes
#include "artemis_flash.h"
#include <string.h>
#include "datalogger.h"

//#define LCP_FREE_RTOS
//#define SENSORS_Test

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

#ifdef SENSORS_Test
    /** initialize sensors */
    STATE_initialize(SYSST_SimpleProfiler_mode);
    configASSERT(xTaskCreate( (TaskFunction_t) SENS_Test, "Sensor_Test_Task", 256, NULL, tskIDLE_PRIORITY + 3UL, NULL) == pdPASS);
#endif

#ifdef LCP_FREE_RTOS
    ARTEMIS_DEBUG_PRINTF("\n*****************************\n");
    ARTEMIS_DEBUG_PRINTF("FreeRTOS here\n");
    ARTEMIS_DEBUG_PRINTF("scheduler is going to start\n");
    ARTEMIS_DEBUG_PRINTF("*****************************\n\n");
    vTaskStartScheduler();
    ARTEMIS_DEBUG_PRINTF("Do not get here\n");
#endif


/*
    // Test flash operations
    ARTEMIS_DEBUG_PRINTF("\nTesting Flash Operations\n");

    uint32_t test_data[4] = {0x12345678, 0x9ABCDEF0, 0xDEADBEEF, 0xFEEDC0DE}; // Data to write
    uint32_t read_buffer[4] = {0}; // Buffer to read back data
    uint32_t test_offset = 0x100; // Offset within NVSTORAGE for testing

    // 1. Erase the flash memory at the specified offset
    ARTEMIS_DEBUG_PRINTF("Erasing Flash at offset 0x%X...\n", test_offset);
    if (flash_erase(test_offset, sizeof(test_data)) == FLASH_SUCCESS)
    {
        ARTEMIS_DEBUG_PRINTF("Flash erase successful.\n");
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("Flash erase failed.\n");
    }

    // 2. Write data to flash memory
    ARTEMIS_DEBUG_PRINTF("Writing data to Flash at offset 0x%X...\n", test_offset);
    if (flash_write(test_data, sizeof(test_data), test_offset) == FLASH_SUCCESS)
    {
        ARTEMIS_DEBUG_PRINTF("Flash write successful.\n");
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("Flash write failed.\n");
    }

    // 3. Read data back from flash memory
    ARTEMIS_DEBUG_PRINTF("Reading data from Flash at offset 0x%X...\n", test_offset);
    if (flash_read(read_buffer, sizeof(read_buffer), test_offset) == FLASH_SUCCESS)
    {
        ARTEMIS_DEBUG_PRINTF("Flash read successful.\n");
        ARTEMIS_DEBUG_PRINTF("Read Data: 0x%08X 0x%08X 0x%08X 0x%08X\n",
                             read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3]);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("Flash read failed.\n");
    }

    // Verify the data
    if (memcmp(test_data, read_buffer, sizeof(test_data)) == 0)
    {
        ARTEMIS_DEBUG_PRINTF("Data verification successful: Read data matches written data.\n");
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("Data verification failed: Read data does not match written data.\n");
    }
*/
    am_util_delay_ms(500);
    while (1)
    {
        ARTEMIS_DEBUG_PRINTF("do not get here in case of RTOS\n");
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        am_util_delay_ms(1000);
    }
}
