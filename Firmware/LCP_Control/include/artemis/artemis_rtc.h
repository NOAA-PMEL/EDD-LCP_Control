// @file artemis_rtc.h

#ifndef ARTEMIS_RTC_H_
#define ARTEMIS_RTC_H_

#include "stdbool.h"
#include "stdint.h"
#include "GPS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {

    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint16_t msec;
    bool hourType;

} rtc_time;

void artemis_rtc_initialize(void);
void artemis_rtc_set_time(rtc_time *pTime);
uint8_t artemis_rtc_get_time(rtc_time *pTime);
uint8_t artemis_rtc_gps_calibration(GPS_Data_t *gps_time);
void artemis_rtc_enable(void);
void artemis_rtc_disable(void);

#ifdef __cplusplus
}
#endif

#endif // ARTEMIS_RTC_H_
