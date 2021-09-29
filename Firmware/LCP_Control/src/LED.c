//#include "freertos_lowpower.h"
#include "FreeRTOS.h"
#include "main.h"
#include <assert.h>
//#include "LED.h"
//#include "am_devices_led.h"
//#include "am_bsp_pins.h"



const am_devices_led_t LED[3] = {
  {
    .ui32GPIONumber   = AM_BSP_GPIO_LED_GREEN,
    .ui32Polarity     = AM_DEVICES_LED_ON_LOW,
  },
  {
    .ui32GPIONumber   = AM_BSP_GPIO_LED_RED,
    .ui32Polarity     = AM_DEVICES_LED_ON_LOW
  },
  {
    .ui32GPIONumber   = AM_BSP_GPIO_LED_BLUE,
    .ui32Polarity     = AM_DEVICES_LED_ON_LOW
  }   
  
};
const am_devices_led_t *psLED = &LED[0];

TaskHandle_t green_led_task_handle;
TaskHandle_t blue_led_task_handle;

SemaphoreHandle_t xLedMutex = NULL;
StaticSemaphore_t xLedSemaphoreBuffer;

const TickType_t xDelay1ms = pdMS_TO_TICKS( 1UL );
const TickType_t xDelay5ms = pdMS_TO_TICKS( 5UL );
const TickType_t xDelay10ms = pdMS_TO_TICKS( 10UL );
const TickType_t xDelay50ms = pdMS_TO_TICKS( 50UL );
const TickType_t xDelay100ms = pdMS_TO_TICKS( 100UL );
const TickType_t xDelay250ms = pdMS_TO_TICKS( 250UL );
const TickType_t xDelay500ms = pdMS_TO_TICKS( 500UL );

SemaphoreHandle_t xSemaphore = NULL;


void LED_Init(void)
{
  
    /** Create the Mutex */
    xLedMutex = xSemaphoreCreateMutex( );
    configASSERT(xLedMutex);
    
    /** Initialize the LED Array */
    am_devices_led_array_init((am_devices_led_t*)psLED, 3);
    
    /** Enable the interrupts */
    am_hal_interrupt_master_enable();
    
    
    /** Toggle the LEDs ON for the heck of it*/
    LED_Toggle(LED_GREEN);
    LED_Toggle(LED_BLUE);
    
    /** Mutex is ready */
    xSemaphoreGive(xLedMutex);
    
}   


void LED_Toggle(eLED_t led)
{

  uint8_t led_num = 0;
  switch(led)
  {
  case LED_RED:
    led_num = 1;
    break;
  case LED_GREEN:
    led_num = 0;
    break;
  case LED_BLUE:
    led_num = 2;
    break;
  default:
    break;
  }  

  am_devices_led_toggle((am_devices_led_t*)psLED, led_num);
  
}

void LED_Off(eLED_t led)
{
  uint8_t led_num = 0;
  switch(led)
  {
  case LED_RED:
    led_num = 1;
    break;
  case LED_GREEN:
    led_num = 0;
    break;
  case LED_BLUE:
    led_num = 2;
    break;
  default:
    break;
  }  

  am_devices_led_off((am_devices_led_t*)psLED, led_num);
}


void GreenLedTask(void *pvParameters)
{
    uint32_t time_ms = (uint32_t) pvParameters;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while(true)
    {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( time_ms));
      if(xSemaphoreTake(xLedMutex, xDelay5ms) == pdPASS)
      {
        LED_Toggle(LED_GREEN);
        xSemaphoreGive(xLedMutex);
      } else {
        
      }
      
    }  
}


void BlueLedTask(void *pvParameters)
{
  
    uint32_t time_ms = (uint32_t) pvParameters;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while(true)
    {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( time_ms ));
      if(xSemaphoreTake(xLedMutex, xDelay5ms) == pdPASS)
      {
        LED_Toggle(LED_BLUE);
        xSemaphoreGive(xLedMutex);
      } else {
        
      }
      
    }  
}
