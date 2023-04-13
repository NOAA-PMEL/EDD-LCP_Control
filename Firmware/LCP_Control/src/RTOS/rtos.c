//*****************************************************************************
//
//! @file rtos.c
//!
//! @brief Essential functions to make the RTOS run correctly.
//!
//! These functions are required by the RTOS for ticking, sleeping, and basic
//! error checking.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "portable.h"
#include "main.h"
#include "assert.h"
#include "LED.h"

#define STACK_SIZE  ( 200 )

typedef enum {
  STATE_IDLE                    = 0,
  STATE_GREEN_FAST_BLUE_SLOW    = 1,
  STATE_BLUE_FAST_GREEN_SLOW    = 2,
  STATE_GREEN_FAST_BLUE_FAST    = 3,
  STATE_GREEN_SLOW_BLUE_SLOW    = 4
}eState_t;


//*****************************************************************************
//
// Task handle for the initial setup task.
//
//*****************************************************************************
TaskHandle_t xSetupTask;
TaskHandle_t xStateTask;
StaticTask_t xTaskBuffer;
StackType_t xStack[ STACK_SIZE ];

TaskHandle_t xBlueLedHandle = NULL;
TaskHandle_t xGreenLedHandle = NULL;
TaskHandle_t xNothingHandle = NULL;
//*****************************************************************************
//
// Interrupt handler for the CTIMER module.
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    uint32_t ui32Status;

    //
    // Check the timer interrupt status.
    //
    ui32Status = am_hal_ctimer_int_status_get(false);
    am_hal_ctimer_int_clear(ui32Status);

    //
    // Run handlers for the various possible timer events.
    //
    am_hal_ctimer_int_service(ui32Status);
}

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    return;
}


//*****************************************************************************
//
// FreeRTOS debugging functions.
//
//*****************************************************************************
void
vApplicationMallocFailedHook(void)
{
    //
    // Called if a call to pvPortMalloc() fails because there is insufficient
    // free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    // internally by FreeRTOS API functions that create tasks, queues, software
    // timers, and semaphores.  The size of the FreeRTOS heap is set by the
    // configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
    //
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    //
    // Run time stack overflow checking is performed if
    // configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    // function is called if a stack overflow is detected.
    //
    while (1)
    {
        __asm("BKPT #0\n") ; // Break into the debugger
    }
}


//*****************************************************************************
//
// Example state task that switches between state setups.
//
//*****************************************************************************
void 
test_state_led_tasks(void *pvParameters)
{
  static eState_t volatile next_state = STATE_GREEN_FAST_BLUE_SLOW;
  printf("Setting up state_tasks\r\n");
  /** Get the current time */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  while(true) 
  {
    
    /** Delay*/
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4000UL));
    
    /** Kill the current tasks */
    if((green_led_task_handle != NULL) && 
       (blue_led_task_handle != NULL))
    {
        vTaskSuspend(green_led_task_handle);
        vTaskSuspend(blue_led_task_handle); 
        vTaskDelete(green_led_task_handle);
        vTaskDelete(blue_led_task_handle);
        green_led_task_handle = NULL;
        blue_led_task_handle = NULL;
    }else{
      LED_Toggle(LED_GREEN);
    }
    /** LEDs OFF */
    LED_Off(LED_GREEN);
    LED_Off(LED_BLUE);
    
    
    switch(next_state)
    {
      case STATE_IDLE:
        next_state = STATE_GREEN_FAST_BLUE_SLOW;
        break;
      case STATE_GREEN_FAST_BLUE_SLOW:
        xTaskCreate(GreenLedTask, "Green LED", STACK_SIZE, (void*)100UL, tskIDLE_PRIORITY + 5, &green_led_task_handle);
        xTaskCreate(BlueLedTask, "Blue LED", STACK_SIZE, (void*)500UL, tskIDLE_PRIORITY + 4, &blue_led_task_handle);
        next_state = STATE_BLUE_FAST_GREEN_SLOW;
        break;
      case STATE_BLUE_FAST_GREEN_SLOW:
        xTaskCreate(GreenLedTask, "Green LED", STACK_SIZE, (void*)500UL, tskIDLE_PRIORITY + 5, &green_led_task_handle);
        xTaskCreate(BlueLedTask, "Blue LED", STACK_SIZE, (void*)100UL, tskIDLE_PRIORITY + 4, &blue_led_task_handle);
        next_state = STATE_GREEN_FAST_BLUE_FAST;
        break;
      case STATE_GREEN_FAST_BLUE_FAST:
        xTaskCreate(GreenLedTask, "Green LED", STACK_SIZE, (void*)100UL, tskIDLE_PRIORITY + 5, &green_led_task_handle);
        xTaskCreate(BlueLedTask, "Blue LED", STACK_SIZE, (void*)100UL, tskIDLE_PRIORITY + 4, &blue_led_task_handle);
        next_state = STATE_GREEN_SLOW_BLUE_SLOW;
        break;
      case STATE_GREEN_SLOW_BLUE_SLOW:
        xTaskCreate(GreenLedTask, "Green LED", STACK_SIZE, (void*)500UL, tskIDLE_PRIORITY + 5, &green_led_task_handle);
        xTaskCreate(BlueLedTask, "Blue LED", STACK_SIZE, (void*)500UL, tskIDLE_PRIORITY + 4, &blue_led_task_handle);
        next_state = STATE_IDLE;
        break;
      default:
        break;
    }
  }  
}
//*****************************************************************************
//
// Initializes all tasks
//
//*****************************************************************************
void
run_tasks(void)
{
    /** Setup the LEDs */
    LED_Init();
    
    /** Disable print interface*/
    disable_print_interface();
    
    // Create the functional tasks
    xTaskCreate(test_state_led_tasks, "System State", STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, &xStateTask);
    
    //
    // Start the scheduler.
    //
    vTaskStartScheduler();
}
