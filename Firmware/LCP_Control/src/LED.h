#ifndef LED_H
#include <stdint.h>
#include <stdbool.h>
typedef enum {
  LED_RED,
  LED_GREEN,
  LED_BLUE
}eLED_t;

extern TaskHandle_t green_led_task_handle;
extern TaskHandle_t blue_led_task_handle;

void LED_Init(void);
void LED_Toggle(eLED_t led);
void LED_Off(eLED_t led);
void GreenLedTask(void *pvParameters) ;
void BlueLedTask(void *pvParameters) ;



#endif