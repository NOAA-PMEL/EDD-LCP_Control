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
#include "artemis_rtc.h"
#include "artemis_accel.h"

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
    //#define TEST_MODEM_SEND
    //#define TEST_MODEM_RECV
    //#define TEST_MODEM_TEST_TRANSFER
    //#define TEST_MODEM_IMEI
//#define TEST_MAX
//#define TEST_KELLER
//#define TEST_S9
//#define TEST_PISTON
//#define TEST_RTC
//#define TEST_ACCL

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

#ifdef TEST_ACCL
    artemis_accel_init();
#endif

#ifdef TEST_RTC
    artemis_rtc_initialize();
    rtc_time time;
#endif

#ifdef TEST_PISTON
    PIS_initialize();
#endif

#ifdef TEST_GPS
    /** Init GPS */
    UBLOX_initialize(UBLOX_COM_I2C, UBLOX_MSG_UBX, UBLOX_MSG_UBX, 1);
    //UBLOX_Nav_t gps = {0};
    GPS_Data_t gps = {0};
    uint8_t cal = 0;
#endif

#ifdef TEST_MODEM
    i9603n_initialize();
    i9603n_on();
    uint16_t len = 0;
    uint8_t recv[300] = {0};
    bool transfer = false;
    bool ret = false;
    uint16_t tBytes = 0;
    bool sent_received = false;
    uint8_t tries = 0;
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
        #ifdef TEST_RTC
            uint8_t something = artemis_rtc_get_time(&time);
            am_util_stdio_printf("\n");
        #endif

        #ifdef TEST_GPS
        if (GPS_Read(&gps))
        {
            am_util_stdio_printf("GPS : Time, %d:%02d:%02d\n", gps.time.hour, gps.time.min, gps.time.sec);
            am_util_stdio_printf("GPS : fixed, latitude=%0.7f , longitude=%0.7f, altitude=%0.2f", \
                                            gps.position.lat, gps.position.lon, gps.position.alt);
            am_util_stdio_printf("\n");
            if (cal == 0){
                artemis_rtc_gps_calibration(&gps);
                cal = 1;
            }
        }
        else
        {
            am_util_stdio_printf("GPS, no fix");
            am_util_stdio_printf("\n");
        }
        #endif

        #ifdef TEST_MODEM

            #ifdef TEST_MODEM_IMEI
                //uint8_t *cmd = "AT\r";
                len = i9603n_read_imei(recv);
                //len = i9603n_read_model(recv);
                //len = i9603n_read_revision(recv);
                //len = i9603n_status(recv);
                //len = i9603n_signal_quality(recv);
                //len = i9603n_send_AT_cmd(cmd, recv);
                if (len > 0)
                {
                    for (uint16_t i=0; i<len; i++)
                    {
                        am_util_stdio_printf("%c", recv[i]);
                        //am_util_stdio_printf("%i ", (int8_t)recv[i]);
                    }
                    am_util_stdio_printf("\n");
                    am_util_delay_ms(1000);
                    am_util_stdio_printf("Turning off modem ...\n");
                    i9603n_off();
                    am_util_stdio_printf("Done !\n");
                }
            #endif

            /* Test an emulated outBound-inBound message */
            #ifdef TEST_MODEM_TEST_TRANSFER
                if (transfer == false)
                {
                    am_util_stdio_printf("### Testing Iridium modem :: emulating the send_and_receive message ###\n\n");
                    //char *cmd = "Hello";
                    char *cmd = "Hello Ambiq world! Hello Ambiq world! Hello Ambiq world! Hello Ambiq world! Hello Ambiq world! Hello Ambiq world!  END";

                    //ret = i9603n_send_text(cmd);
                    ret = i9603n_send_data((uint8_t *)cmd);
                    if (ret == true)
                    {
                        transfer = true;
                    }
                    if (transfer == true)
                    {
                        len =  i9603n_test_transfer(recv);
                        if (len > 0)
                        {
                            am_util_stdio_printf("\nTransferred bytes = ");
                            if (len > 1)
                            {
                                tBytes = (recv[0] << 8) | recv[1];
                            }
                            else
                            {
                                tBytes = recv[0] ;
                            }
                            am_util_stdio_printf("%u\n", tBytes);
                        }
                        len = i9603n_status(recv);
                        if (len > 0)
                        {
                            am_util_stdio_printf("Status : ");
                            for(uint16_t i=0; i<len; i++)
                            {
                                am_util_stdio_printf("%u ", (uint8_t)recv[i]);
                                //am_util_stdio_printf("%c", recv[i]);
                            }
                            am_util_stdio_printf("\n");
                            if ((int8_t)recv[2] == 1)
                            {
                                ret = true;
                            }
                            else
                            {
                                ret = false;
                            }
                        }
                        am_util_delay_ms(1000);
                    }
                }
                if (tBytes > 270)
                {
                    am_util_stdio_printf("terminated buffer is full\n");
                }
                else
                {
                    if (transfer == true)
                    {
                        len =  i9603n_read_text((char*)recv);
                        am_util_stdio_printf("Reading in text form : length = %u\n", len);
                        if (len > 0)
                        {
                            for(uint16_t i=0; i<len; i++)
                            {
                                am_util_stdio_printf("%c", recv[i]);
                            }
                            am_util_stdio_printf("\n\n");
                        }
                        am_util_delay_ms(1000);
                        len =  i9603n_read_data(recv);
                        am_util_stdio_printf("Reading in binary form : length = %u\n", len);
                        if (len > 0)
                        {
                            for(uint16_t i=0; i<len; i++)
                            {
                                //am_util_stdio_printf("0x%02X ", recv[i]);
                                am_util_stdio_printf("%c", recv[i]);
                            }
                            am_util_stdio_printf("\n\n");
                        }
                    }
                }
            #endif

            #ifdef TEST_MODEM_SEND
                /* Test a real-time satellitel communication, send bytes, limit is 340 */
                if (transfer == false)
                {
                    am_util_stdio_printf("### Testing Iridium modem for sending a message ###\n\n");
                    uint8_t *data = (uint8_t *)"Hello Ambiq world! how are you?";
                    ret = i9603n_send_data(data);
                    if (ret == true)
                    {
                        am_util_stdio_printf("Sending: ");
                        for(uint16_t i=0; i<strlen(data); i++)
                        {
                            am_util_stdio_printf("%c", (char)data[i]);
                        }
                        am_util_stdio_printf("\n");
                        transfer = true;
                    }
                }
                if (transfer == true && !sent_received)
                {
                    // try to communicate with iridium satellite
                    len =  i9603n_initiate_transfer(recv);
                    if (len > 0)
                    {
                        am_util_stdio_printf("Status : ");
                        for(uint16_t i=0; i<len; i++)
                        {
                            am_util_stdio_printf("%u ", recv[i]);
                            //am_util_stdio_printf("%c", recv[i]);
                        }
                        am_util_stdio_printf("\n");

                        if(recv[0] <= 5)
                        {
                            am_util_stdio_printf("Transfer is Successful\n");
                            sent_received = true;
                            // clear originated buffer
                            am_util_delay_ms(2000);
                            len = i9603n_send_AT_cmd("AT+SBDD0\r", recv);
                            if(len>0)
                            {
                                am_util_stdio_printf("Clearing Originated buffer:\n");
                                for (uint8_t i=0; i<len; i++)
                                {
                                    am_util_stdio_printf("%c", (char)recv[i]);
                                }
                                am_util_stdio_printf("\n");
                                am_util_delay_ms(1000);
                                am_util_stdio_printf("Turning off modem ...\n");
                                i9603n_off();
                            }
                        }
                        else
                        {
                            // terminate after 20 tries
                            tries++;
                            if (tries >= 20)
                            {
                                am_util_delay_ms(3000);
                                am_util_stdio_printf("No visible Satellites are available, terminating ...\n");
                                sent_received = true;
                                len = i9603n_send_AT_cmd("AT+SBDD0\r", recv);
                                if(len>0)
                                {
                                    am_util_stdio_printf("Clearing Originated buffer:\n");
                                    for (uint8_t i=0; i<len; i++)
                                    {
                                        am_util_stdio_printf("%c", recv[i]);
                                    }
                                    am_util_stdio_printf("\n");
                                    am_util_delay_ms(1000);
                                    am_util_stdio_printf("Turning off modem ...\n");
                                    i9603n_off();
                                }
                            }
                        }
                    }
                }
            #endif

            #ifdef TEST_MODEM_RECV
                /* Test a real-time satellitel communication, receive bytes, limit is 270 */
                if (transfer == false && !sent_received)
                {
                    // try to communicate with iridium satellite
                    len =  i9603n_initiate_transfer(recv);
                    if (len > 0)
                    {
                        am_util_stdio_printf("Transmission Status : ");
                        for(uint16_t i=0; i<len; i++)
                        {
                            am_util_stdio_printf("%u ", recv[i]);
                            //am_util_stdio_printf("%c", (char)recv[i]);
                        }
                        am_util_stdio_printf("\n");

                        if(recv[0] <= 5)
                        {
                            am_util_stdio_printf("communication is Successful\n");
                            sent_received = true;
                            transfer = true;

                            // receive data from terminated buffer
                            am_util_delay_ms(1000);
                            // check status first
                            len = i9603n_status(recv);
                            if (len > 0)
                            {
                                am_util_stdio_printf("Module Status : ");
                                for (uint16_t i=0; i<len; i++)
                                {
                                    am_util_stdio_printf("%i ", (int8_t)recv[i]);
                                }
                                am_util_stdio_printf("\n");
                                if (recv[2] == 1)
                                {
                                    // receive data from terminated buffer
                                    am_util_delay_ms(1000);
                                    am_util_stdio_printf("Recevied Data: ");
                                    len = i9603n_read_data(recv);
                                    while (len == 0)
                                    {
                                        am_util_delay_ms(1000);
                                        len = i9603n_read_data(recv);
                                    }
                                    if (len > 0)
                                    {
                                        for (uint8_t i=0; i<len; i++)
                                        {
                                            am_util_stdio_printf("%c", recv[i]);
                                        }
                                        am_util_stdio_printf("\n");
                                        am_util_stdio_printf("Turning off modem ...\n");
                                        am_util_delay_ms(1000);
                                        i9603n_off();
                                    }
                                }
                            }
                        }
                        else
                        {
                            // terminate after 20 tries
                            tries ++;
                            if (tries >= 20)
                            {
                                am_util_delay_ms(2000);
                                am_util_stdio_printf("No visible Satellites are available, terminating ...\n");
                                sent_received = true;
                                transfer = true;
                                am_util_stdio_printf("Turning off modem ...\n");
                                am_util_delay_ms(1000);
                                i9603n_off();
                            }
                        }
                    }
                }
            #endif

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
    }
}
