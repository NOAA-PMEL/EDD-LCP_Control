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

#include <stdlib.h>

//#define TEST_UBLOX  true
#define TEST_SUPERCAP true


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
    
    
#ifdef TEST_S9
    /** Init Soundnine Temperature Sensor */
    S9T_init(BSP_UART_COM1, &g_AM_BSP_GPIO_COM1_POWER_PIN, AM_BSP_GPIO_COM1_POWER_PIN);
    S9T_enable();
    float p, r, t;
#endif
    
    
#ifdef TEST_UBLOX
    /** Init GPS */
    UBLOX_initialize(UBLOX_COM_I2C, UBLOX_MSG_UBX, UBLOX_MSG_UBX, 1);
    UBLOX_Nav_t gps = {0};
#endif
    
#ifdef TEST_SUPERCAP
    artemis_sc_initialize();
    
    if(artemis_sc_power_startup())
    {
      printf("Success in starting supercap charging!\n");
    } else {
      printf("Failed to start supercap charge\n");
    }
#endif
    
    

    while(true)
    {
#ifdef TEST_UBLOX
      UBLOX_read_nav(&gps);
      printf("GPS Fix = %u\n", gps.fix);

      if(gps.fix)
      {
        printf("Lat=%.7f, Lon=%.7f, Alt=%.3f\n", gps.position.lat, gps.position.lon, gps.position.alt);
      }
#endif
      
#ifdef TEST_S9
    S9T_Read(&t, &r);
    printf("t=%.3f, r=%.3f\n", t, r); 
#endif
    

    }
    return(EXIT_SUCCESS);
}