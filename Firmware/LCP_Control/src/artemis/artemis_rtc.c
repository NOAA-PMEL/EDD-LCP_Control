/** @file artemis_rtc.c
 *  based on RTC print example from AmbiqSDK
 */

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_util_delay.h"
#include "artemis_rtc.h"
#include "artemis_debug.h"

static void artemis_rtc_timer(void);
static bool utc = false;

//void artemis_rtc_set_time(void);
//void artemis_rtc_get_time(void);

char *days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", 
                "Thursday", "Friday", "Saturday" };
char *Months[] = {"January","February", "March", "April", 
                  "May", "June", "July", "August", "September", 
                  "October", "November", "December"};


am_hal_ctimer_config_t g_sTimer0 = {0, 
                                    (AM_HAL_CTIMER_FN_REPEAT |
                                    AM_HAL_CTIMER_INT_ENABLE |
                                    AM_HAL_CTIMER_LFRC_32HZ),
                                    0,};
am_hal_rtc_time_t hal_time;
uint32_t g_LastSecond = 0;
uint32_t g_TestCount = 0;

uint16_t toVal(const char *pcAsciiStr)
{
    uint16_t iRetVal = 0;
    iRetVal += pcAsciiStr[1] - '0';
    iRetVal += pcAsciiStr[0] == ' ' ? 0 : (pcAsciiStr[0] - '0') * 10;
    return iRetVal;
}

uint8_t mthToIndex(const char *pcMon)
{
    uint8_t idx;
    for (idx = 0; idx < 12; idx++)
    {
        if ( am_util_string_strnicmp(Months[idx], pcMon, 3) == 0 )
        {
            return idx+1;
        }
    }
    return 12;
}

void am_ctimer_isr(void)
{
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
    //am_hal_rtc_time_get(&hal_time);
}

static void artemis_rtc_timer(void)
{
    /* Enable the LFRC */
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);

    /* Set up timer A0 */
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(0, &g_sTimer0);

    /* Set the timing for timerA0 */
    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, 31, 0);

    /* Clear the timer Interrupt */
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_OVERFLOW);
}

void artemis_rtc_initialize(void)
{
    /* Enable the XT for the RTC */
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);

    /* Select XT for RTC clock source */
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);

    /* Enable 24 hours */
    am_hal_rtc_time_12hour(false);

    /* Enable the RTC Oscillator */
    am_hal_rtc_osc_enable();

#if defined(__GNUC__)  ||  defined(__ARMCC_VERSION)  ||  defined(__IAR_SYSTEMS_ICC__)
    hal_time.ui32Hour = toVal(&__TIME__[0]);
    hal_time.ui32Minute = toVal(&__TIME__[3]);
    hal_time.ui32Second = toVal(&__TIME__[6]);
    hal_time.ui32Hundredths = 00;
    hal_time.ui32Weekday = am_util_time_computeDayofWeek(2000 + toVal(&__DATE__[9]), mthToIndex(&__DATE__[0]) + 1, toVal(&__DATE__[4]) );
    hal_time.ui32DayOfMonth = toVal(&__DATE__[4]);
    hal_time.ui32Month = mthToIndex(&__DATE__[0]);
    hal_time.ui32Year = toVal(&__DATE__[9]);
    hal_time.ui32Century = 0;
#else
    hal_time.ui32Hour = 14;
    hal_time.ui32Minute = 24;
    hal_time.ui32Second = 33;
    hal_time.ui32Hundredths = 50;
    hal_time.ui32Weekday = 3;
    hal_time.ui32DayOfMonth = 11;
    hal_time.ui32Month = 11;
    hal_time.ui32Year = 23;
    hal_time.ui32Century = 0;
#endif

    am_hal_rtc_time_set(&hal_time);
    artemis_rtc_timer();

    /* Enable the timer Interrupt */
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    /* Enable the timer interrupt in the NVIC */
    NVIC_EnableIRQ(CTIMER_IRQn);
    am_hal_interrupt_master_enable();

    /* Enable the timer */
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

    rtc_time time;
    artemis_rtc_get_time(&time);
    ARTEMIS_DEBUG_PRINTF("\nRTC Timer\n");
    ARTEMIS_DEBUG_PRINTF("***************************************\n");
    //ARTEMIS_DEBUG_PRINTF("Clock started on %s at %s.\n\n",__DATE__,__TIME__);
    ARTEMIS_DEBUG_PRINTF("Clock started on %02d.%02d.20%02d at %02d:%02d:%02d (local)\n\n",
                            time.month, time.day, time.year, time.hour, time.min, time.sec);
}

bool artemis_rtc_get_time(rtc_time *time)
{
    am_hal_rtc_time_get(&hal_time);

    if (hal_time.ui32ReadError){
        ARTEMIS_DEBUG_PRINTF("RTC :: ERROR, hal_time\n");
        return false;
    }

    time->year = (uint16_t) hal_time.ui32Year;
    time->month = (uint8_t) hal_time.ui32Month;
    time->day = (uint8_t) hal_time.ui32DayOfMonth;
    time->hour = (uint8_t) hal_time.ui32Hour;
    time->min = (uint8_t) hal_time.ui32Minute;
    time->sec = (uint8_t) hal_time.ui32Second;
    time->msec = (uint16_t) hal_time.ui32Hundredths / 10 ;

    //ARTEMIS_DEBUG_PRINTF("Time : ");
    //ARTEMIS_DEBUG_PRINTF("%d:", time->hour);
    //ARTEMIS_DEBUG_PRINTF("%02d:", time->min);
    //ARTEMIS_DEBUG_PRINTF("%02d.", time->sec);
    //ARTEMIS_DEBUG_PRINTF("%02d ", time->msec);

    //ARTEMIS_DEBUG_PRINTF("Time : ");
    //ARTEMIS_DEBUG_PRINTF("%d:", hal_time.ui32Hour);
    //ARTEMIS_DEBUG_PRINTF("%02d:", hal_time.ui32Minute);
    //ARTEMIS_DEBUG_PRINTF("%02d.", hal_time.ui32Second);
    //ARTEMIS_DEBUG_PRINTF("%02d ", hal_time.ui32Hundredths);
    //ARTEMIS_DEBUG_PRINTF(days[hal_time.ui32Weekday]);
    //ARTEMIS_DEBUG_PRINTF(" ");
    //ARTEMIS_DEBUG_PRINTF(Months[hal_time.ui32Month]);
    //ARTEMIS_DEBUG_PRINTF(" ");
    //ARTEMIS_DEBUG_PRINTF("%d, ", hal_time.ui32DayOfMonth);
    //ARTEMIS_DEBUG_PRINTF("20%02d", hal_time.ui32Year);
    //ARTEMIS_DEBUG_PRINTF("\n");

    if (utc)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void artemis_rtc_set_time(rtc_time *pTime){

    /* The RTC is initialized from gps time */
    artemis_rtc_disable();

    hal_time.ui32Hour = pTime->hour;
    hal_time.ui32Minute = pTime->min;
    hal_time.ui32Second = pTime->sec;
    hal_time.ui32Hundredths = 00;
    hal_time.ui32Weekday = pTime->day;
    hal_time.ui32DayOfMonth = pTime->day;
    hal_time.ui32Month = pTime->month;
    hal_time.ui32Year = (pTime->year - 2000);
    hal_time.ui32Century = 0;

    am_hal_rtc_time_set(&hal_time);
    am_hal_rtc_osc_enable();
}

void artemis_rtc_gps_calibration(SensorGps_t *gps_time)
{
    rtc_time time;

    time.year = (uint16_t) gps_time->year;
    time.month = (uint8_t) gps_time->month;
    time.day = (uint8_t) gps_time->day;
    time.hour = (uint8_t) gps_time->hour;
    time.min = (uint8_t) gps_time->min;
    time.sec = (uint8_t) gps_time->sec;

    //time.year = (uint16_t) gps_time->time.year;
    //time.month = (uint8_t) gps_time->time.month;
    //time.day = (uint8_t) gps_time->time.day;
    //time.hour = (uint8_t) gps_time->time.hour;
    //time.min = (uint8_t) gps_time->time.min;
    //time.sec = (uint8_t) gps_time->time.sec;

    /* Set the gps time */
    artemis_rtc_set_time(&time);
    utc = true;
}

void artemis_rtc_set12hour(void){
    am_hal_rtc_time_12hour(true);
}

void artemis_rtc_set24hour(void){
    am_hal_rtc_time_12hour(false);
}

void artemis_rtc_enable(void){
    am_hal_rtc_osc_enable();
}

void artemis_rtc_disable(void){
    am_hal_rtc_osc_disable();
}

