///
/// @file artemis_main.c
///

#include "main.h"
#include "S9_temp.h"
#include "bsp_uart.h"

#include "artemis_debug.h"
#include "artemis_mcu.h"
//#include "artemis_scheduler.h"
//#include "artemis_time.h"
#include <stdlib.h>

///
///
///
int main(void)
{
    // initialize mcu features
    artemis_mcu_initialize();

    // initialize debug features
    artemis_debug_initialize();

    // initialize time functions
    artemis_time_initialize();

    // initialize the scheduler
//    artemis_scheduler_initialize();

    // run the application
//    artemis_scheduler_run();

    S9T_init(BSP_UART_COM0, &g_AM_BSP_GPIO_COM0_POWER_PIN, AM_BSP_GPIO_COM0_POWER_PIN);
    S9T_enable();
    
    float r, t;
    while(true)
    {
      S9T_Read(&t, &r);
      printf("t=%.3f, r=%.3f\n", t, r); 
//      bsp_uart_putc(BSP_UART_COM0, 'C');
//      printf("%c", bsp_uart_getc(BSP_UART_COM0));
      am_hal_systick_delay_us(500000);
    }
    return(EXIT_SUCCESS);
}