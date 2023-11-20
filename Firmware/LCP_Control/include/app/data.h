/**
 * @file data.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-15
 * 
 * 
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

#define LCP_PARK_MODE           ( 1 )
#define LCP_PROFILE_MODE        ( 2 )

/**********************************************************************************
 * MACROS
 *********************************************************************************/

/**********************************************************************************
 * Typdefs
 *********************************************************************************/
// typedef struct sProfileData_t
// {   
    
// }ProfileData_t;
typedef struct sData_t
{
    struct{
        size_t length;
        size_t written;
        size_t read;
    }cbuf;
    struct{
        uint32_t start_time;
        uint32_t *pTimeOffset;
        float *pPressure;
        float *pTemperature;
    }data;
}Data_t;

typedef struct cData_t
{
    uint8_t pressure;
    int16_t temp;
} cData;

typedef struct gData_t
{
    uint32_t start;
    uint32_t stop;
    float lat;
    float lon;
} gData;

/**********************************************************************************
 * Function Prototypes
 *********************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void DATA_setbuffer(Data_t *p, uint32_t *pTime, 
                            float *pPressure, float*pTemp,
                            size_t length);
void DATA_reset(Data_t *p);
size_t DATA_add(Data_t *buf, uint32_t time, float pressure, float temp);
size_t DATA_get_original(Data_t *p, uint32_t *time, float *pressure, float *temp);
size_t DATA_get_converted(Data_t *p, uint32_t *start, uint32_t *offset, uint8_t *pressure, int16_t *temp);

uint32_t get_epoch_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

void create_header(uint8_t *df, uint32_t start, uint32_t stop, float lat, float lon, uint8_t mode_type, uint8_t page);

float std_div(float *value, uint16_t len, float *var, float *avg);
float average(float *value, uint16_t len);

void DATA_get_iridium_park(uint8_t *pData);
void DATA_get_iridium_profile(uint8_t *pData);

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
