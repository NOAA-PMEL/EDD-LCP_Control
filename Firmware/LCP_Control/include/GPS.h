#ifndef GPS_H
#define GPS_H

#include "stdint.h"

typedef struct sGPS_Data_t
{
    uint8_t fix;        /**< GPS Fix valid */
    struct {
        float lat;      /**< Latitude (deg)*/
        float lon;      /**< Longitude (deg) */
        float alt;      /**< Altitude (mm) */
    }position;
    struct {
        uint16_t year;  /**< Calendar Year UTC */
        uint8_t month;  /**< Calendar Month UTC */
        uint8_t day;    /**< Calendar Day UTC */
        uint8_t hour;   /**< Hour UTC */    
        uint8_t min;    /**< Minute UTC */
        uint8_t sec;    /**< Second UTC */
    }time;
}GPS_Data_t;

void GPS_initialize(void);
uint8_t GPS_Read(GPS_Data_t *data);

#endif // GPS_H
