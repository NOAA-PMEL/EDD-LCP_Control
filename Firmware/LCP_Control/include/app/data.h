/**
 * @file data.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-15
 */
#ifndef DATA_H
#define DATA_H

/** Remove STATIC and PERSISTENT values if running TEST */
/** Add the actual values if running release */
#ifdef TEST
#ifndef STATIC
#define STATIC  
#endif
#ifndef PERSISTENT
#define PERSISTENT
#endif
#else
#ifndef STATIC
#define STATIC  static
#endif
#ifndef PERSISTENT
#if defined(__IAR_SYSTEMS_ICC__)
#define PERSISTENT __persistent 
#elif defined(__GNUC__)
#define PERSISTENT __attribute__((section (".persistent")))
#endif
#endif
#endif

/**********************************************************************************
 * Includes
 *********************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
/**********************************************************************************
 * Configuration Constants
 *********************************************************************************/
#define IRID_DATA_OUT           ( 340 )
#define IRID_DATA_IN            ( 270 )
#define IRID_HEADER_LENGTH      ( 29 )
#define IRID_HEADER_LENGTH_EXT  ( 19 )

#define LCP_PARK_MODE           ( 0x00 )
#define LCP_PROFILE_MODE        ( 0x01 )

/* SENSORS DATA bits */
#define TEMPERATURE_BITS        ( 16 )
#define PRESSURE_BITS           ( 12 )
/* add more bits for future sensor inclusion*/

#define MEASUREMENT_BITS        ( TEMPERATURE_BITS + PRESSURE_BITS )
#define MEASUREMENT_MAX         ( (((IRID_DATA_OUT - IRID_HEADER_LENGTH) * 8) / MEASUREMENT_BITS ) - 1 )

/**********************************************************************************
 * MACROS
 *********************************************************************************/

/**********************************************************************************
 * Typdefs
 *********************************************************************************/

typedef struct __attribute__((packed))
{
    uint8_t modeType;
    uint8_t profNumber;
    uint16_t mLength;
    uint8_t pageNumber;
} sData;

typedef struct __attribute__((packed))
{
    uint32_t pStart;
    uint32_t pStop;
    float pLatitude;
    float pLongitude;
    uint16_t pLength;
    uint16_t pIndex;
} pData;

typedef struct __attribute__((packed))
{
    struct
    {
        uint32_t length;
        uint32_t written;
        uint32_t read;
    } cbuf;
    /* measurements struct */
    struct
    {
        float *pressure;
        float *temperature;
        /* add more measurement variables */
    } data;
    /* local length and profile numbers manipulation */
    uint8_t pNumber;
    uint16_t wLength;
    uint16_t rLength;
    /* this holds the nr. of profiles data */
    pData *p;

} Data_t;

typedef struct cData_t
{
    uint16_t pressure;
    int16_t temp;
} cData;


/**********************************************************************************
 * Function Prototypes
 *********************************************************************************/

 #ifdef __cplusplus
extern "C"{
#endif
void DATA_reset(Data_t *p);
void DATA_setbuffer(Data_t *buf, pData *P, float *pressure, float *temperature, uint32_t length);
void DATA_add(Data_t *buf, uint32_t time, float pressure, float temperature, uint8_t pNumber);
void DATA_add_gps(Data_t *buf, float latitude, float longitude);
void DATA_get_original(Data_t *buf, pData *P, float *pressure, float *temperature, uint8_t pNumber);
void DATA_get_converted(Data_t *buf, pData *P, uint16_t *pressure, uint16_t *temperature, uint8_t pNumber);
uint32_t get_epoch_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);
void create_header(uint8_t *df, uint32_t start, uint32_t stop, float lat, float lon, uint8_t mode_type, uint8_t page);
void create_header_irid(uint8_t *df, pData *P, sData *S);
void create_header_irid_ext(uint8_t *df, pData *P, sData *S);
uint16_t pack_measurements_irid(Data_t *buf, pData *P, sData *S, uint8_t *rBuf);
uint16_t pack_measurements_irid_ext(Data_t *buf, pData *P, sData *S, uint8_t *rBuf);
float std_div(float *value, uint16_t len, float *var, float *avg);
float average(float *value, uint16_t len);

/* New functions for dynamic memory allocation */
Data_t* DATA_alloc(uint32_t numProfiles, uint32_t numMeasurements, uint8_t prof_number);
void DATA_free(Data_t *buf);

/**********************************************************************************
 * Unit Test Variables & Static Prototpyes
 *********************************************************************************/
#ifdef TEST
#ifdef DOXYGEN_IGNORE_THIS
#endif // DOXYGEN_IGNORE_THIS
#endif
#ifdef __cplusplus
} // extern "C"
#endif 
#endif // DATA_H