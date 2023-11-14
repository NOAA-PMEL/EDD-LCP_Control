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
        float *pDepth;
        float *pTemperature;
    }data;
}Data_t;

/**********************************************************************************
 * Function Prototypes
 *********************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void DATA_setbuffer(Data_t *p, uint32_t *pTime, 
                            float *pDepth, float*pTemp,
                            size_t length);

void DATA_reset(Data_t *p);

size_t DATA_add(Data_t *buf, uint32_t time, float depth, float temp);

size_t DATA_get_original(Data_t *p, uint32_t *time, float *depth, float *temp);

size_t DATA_get_converted(Data_t *p, uint32_t *start, uint16_t *offset, uint16_t *depth, uint16_t *temp);

uint32_t get_epoch_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

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
