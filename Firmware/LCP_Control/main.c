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
#include "artemis_ublox_i2c.h"
#include "artemis_max14830.h"

#include "control.h"
#include "piston.h"
#include "sensors.h"

#include "ublox.h"
#include "GPS.h"
#include "S9_temperature.h"
#include "K9lx_pressure.h"
#include "i9603n.h"


/* uncomment these to test the peripherals */
//#define TEST_GPS
//#define TEST_MODEM
//#define TEST_MAX
//#define TEST_KELLER
//#define TEST_S9
//#define TEST_PISTON

int main(void)
{
    // initialize mcu features
    artemis_mcu_initialize();

    // initialize debug features
    artemis_debug_initialize();
    am_util_stdio_printf("Hello Ambiq World\n");

    // initialize time functions
    //artemis_time_initialize();

    // initialize the scheduler
    // artemis_scheduler_initialize();

    // run the application
    // artemis_scheduler_run();

    // initialize the piston
    // PIS_initialize();


#ifdef TEST_PISTON
    PIS_initialize();
#endif

#ifdef TEST_GPS
    /** Init GPS */
    UBLOX_initialize(UBLOX_COM_I2C, UBLOX_MSG_UBX, UBLOX_MSG_UBX, 1);
    //UBLOX_Nav_t gps = {0};
    GPS_Data_t gps = {0};
#endif

#ifdef TEST_MODEM
    i9603n_initialize();
    i9603n_on();
#endif

#ifdef TEST_MAX
    artemis_max14830_init();
    #ifdef TEST_KELLER
        K9lx_init(3, 9600);
        float k_temperature, k_pressure = 0;
    #endif
    #ifdef TEST_S9
        S9T_init(0, 9600);
        float s_temperature, s_resistance = 0;
    #endif
#endif

am_util_stdio_printf("\n\n");

    while(1)
    {
        #ifdef TEST_GPS
        if (GPS_Read(&gps))
        {
            am_util_stdio_printf("GPS : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.2f", \
                                            gps.position.lat, gps.position.lon, gps.position.alt);
        }
        else
        {
            am_util_stdio_printf("GPS, no fix");
        }
        #endif

        #ifdef TEST_MODEM
            i9603n_send_data("AT\r", 3);
        #endif
        #ifdef TEST_MAX
            #ifdef TEST_KELLER
                K9lx_read(&k_pressure, &k_temperature);
                am_util_stdio_printf("K9lx :    Pressure   = %0.5f bar, \tTemperature = %0.3f °F\n", k_pressure, k_temperature);
            #endif
            #ifdef TEST_S9
                S9T_Read(&s_temperature, &s_resistance);
                am_util_stdio_printf("S9   :    Resistance = %0.5f Ohm, \tTemperature = %0.3f °F\n", s_resistance, s_temperature);
            #endif

        #endif

        #ifdef TEST_PISTON
            //am_util_stdio_printf("piston moving forward for 1 second...\n");
            //PIS_extend();
            //am_util_delay_ms(1000);
            //am_util_stdio_printf("piston stopping for 1 second...\n");
            //PIS_stop();
            //am_util_delay_ms(1000);
            //am_util_stdio_printf("piston moving reverse for 1 second...\n");
            //PIS_retract();
            //am_util_delay_ms(1000);
        #endif

        // 1s delay
        #if defined(TEST_GPS) || defined(TEST_MODEM) || defined(TEST_MAX) || defined(TEST_KELLER) || defined(TEST_S9) || defined (TEST_PISTON)
            // do nothing , see the output of the peripherals if attached
        #else
            am_util_stdio_printf("1 second delay, uncomment the #define to test the peripherals \n");
        #endif

        am_util_delay_ms(1000);
        //PIS_retract();
    }
}
