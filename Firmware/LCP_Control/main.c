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
//
//    S9T_init(BSP_UART_COM1, &g_AM_BSP_GPIO_COM1_POWER_PIN, AM_BSP_GPIO_COM1_POWER_PIN);
//    S9T_enable();
    
    /** Init GPS */

//    UBLOX_initialize(UBLOX_COM_I2C, UBLOX_MSG_UBX, UBLOX_MSG_UBX, 1);
    
    artemis_sc_initialize();
    
    if(artemis_sc_power_startup())
    {
      printf("Success in starting supercap charging!\n");
    } else {
      printf("Failed to start supercap charge\n");
    }
    
    artemis_sc_power_off();
    
//  float p, r, t;
//  UBLOX_Nav_t gps = {0};
    
//    ublox_data_t gps;
    while(true)
    {
//      UBLOX_read_nav(&gps);
//      printf("GPS Fix = %u\n", gps.fix);
      
//      if(gps.fix)
//      {
//        printf("Lat=%.7f, Lon=%.7f, Alt=%.3f\n", gps.position.lat, gps.position.lon, gps.position.alt);
//      }
//      artemis_pa9ld_read(&p, &t);
//      printf("p=%0.3f, t=%0.3f\n", p, t);
//      S9T_Read(&t, &r);
//      printf("t=%.3f, r=%.3f\n", t, r); 
//      artemis_ublox_i2c_read_position(&gps);
//      artemis_ublox_read_data(&gps);
//      artemis_ublox_test();
//    artemis_ublox_read_register(0x00, 240);

//      printf("lat=%.6f%c, lon=%.6f%c, alt=%.3f\n", gps.lat, gps.NS, gps.lon, gps.EW, gps.alt);
//      bsp_uart_putc(BSP_UART_COM0, 'C');
//      printf("%c", bsp_uart_getc(BSP_UART_COM0));
//      am_hal_systick_delay_us(500000);
    }
    return(EXIT_SUCCESS);
}