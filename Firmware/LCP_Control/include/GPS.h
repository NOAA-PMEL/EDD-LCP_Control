#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct sGPS_Data_t
{
    uint8_t fix;         /**< GPS Fix valid */
    struct {
        float lat;       /**< Latitude (deg)*/
        float lon;       /**< Longitude (deg) */
        float alt;       /**< Altitude (mm) */
    } position;
    struct {
        uint16_t year;   /**< Calendar Year UTC */
        uint8_t month;   /**< Calendar Month UTC */
        uint8_t day;     /**< Calendar Day UTC */
        uint8_t hour;    /**< Hour UTC */    
        uint8_t min;     /**< Minute UTC */
        uint8_t sec;     /**< Second UTC */
    } time;
} GPS_Data_t;

/************************************************************************
 * Core Hardware & Protocol Functions
 ************************************************************************/

bool GPS_initialize(void);
void GPS_uninitialize(void);
bool GPS_Read(GPS_Data_t *data);
void GPS_on(void);
void GPS_off(void);

/************************************************************************
 * UBX Navigation & Aiding Configuration
 ************************************************************************/

/**
 * @brief Configures the u-blox CFG-NAV5 dynamic platform model to Sea.
 *
 * @return true if transmitted successfully, false otherwise.
 */
bool GPS_set_dynamic_model_sea(void);

/**
 * @brief Injects current Artemis RTC time into the u-blox receiver via UBX-MGA-INI-TIME_UTC.
 *
 * @param time_accuracy_sec Estimated clock uncertainty in seconds (e.g. 300 for 5 mins drift).
 * @return true if transmitted successfully, false otherwise.
 */
bool GPS_inject_rtc_time_with_acc(uint16_t time_accuracy_sec);

/**
 * @brief Injects approximate coordinates into the u-blox receiver via UBX-MGA-INI-POS_LLH.
 *
 * @param latitude_deg  Latitude in decimal degrees (e.g. 47.68645f)
 * @param longitude_deg Longitude in decimal degrees (e.g. -122.25446f)
 * @param altitude_m    Altitude above sea level/ellipsoid in meters
 * @param accuracy_km   Estimated 1-sigma horizontal position uncertainty in kilometers (e.g. 200.0f)
 * @return true if transmitted successfully, false otherwise.
 */
bool GPS_inject_position(float latitude_deg, float longitude_deg, float altitude_m, float accuracy_km);

#endif // GPS_H