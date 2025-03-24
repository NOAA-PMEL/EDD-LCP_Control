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
//#include "artemis_pa9ld.h"
//#include "artemis_ublox_i2c.h"
#include "artemis_supercap.h"
#include "ublox.h"

#include "control.h"
#include "piston.h"
#include "sensors.h"
#include <stdlib.h>

//#define TEST_UBLOX  true
//#define TEST_SUPERCAP true
#define TEST_S9 true

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
    
    // initialize the piston 
//    PIS_initialize();
    
//#ifdef TEST_S9
//    /** Init Soundnine Temperature Sensor */
//    S9T_init(BSP_UART_COM1, &g_AM_BSP_GPIO_COM1_POWER_PIN, AM_BSP_GPIO_COM1_POWER_PIN);
//    S9T_enable();
//    float p, r, t;
//#endif
    
    
//#ifdef TEST_UBLOX
//    /** Init GPS */
//    UBLOX_initialize(UBLOX_COM_I2C, UBLOX_MSG_UBX, UBLOX_MSG_UBX, 1);
//    UBLOX_Nav_t gps = {0};
//#endif
//    
//#ifdef TEST_SUPERCAP
//    artemis_sc_initialize();
//    
//    if(artemis_sc_power_startup())
//    {
//      printf("Success in starting supercap charging!\n");
//    } else {
//      printf("Failed to start supercap charge\n");
//    }
//#endif
//    PIS_retract();
//    PIS_extend();
//    am_hal_systick_delay_us(2500000);
//    PIS_stop();
//    am_hal_systick_delay_us(2500000);
//    PIS_retract();
//    while(true)
//    {
//#ifdef TEST_UBLOX
//      UBLOX_read_nav(&gps);
//      printf("GPS Fix = %u\n", gps.fix);
//
//      if(gps.fix)
//      {
//        printf("Lat=%.7f, Lon=%.7f, Alt=%.3f\n", gps.position.lat, gps.position.lon, gps.position.alt);
//      }
//#endif
//      
//#ifdef TEST_S9
//    S9T_Read(&t, &r);
//    printf("t=%.3f, r=%.3f\n", t, r); 
//#endif
//    
//
//    }
    SENS_initialize();
    TaskHandle_t xDepthHandle = NULL;
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, &xDepthHandle);
    vTaskStartScheduler();
    float d, r;
    while(1)
    {
      
      SENS_get_depth(&d, &r);
      printf("d=%0.3f\n", d);
    }
    
    
    
    return(EXIT_SUCCESS);
}