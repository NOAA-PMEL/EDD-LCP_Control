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
        uint16_t *pTimeOffset;
        uint16_t *pDepth;
        uint16_t *pTemperature;
    }data;
}Data_t;

/**********************************************************************************
 * Function Prototypes
 *********************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void DATA_setbuffer(Data_t *p, uint16_t *pTime, 
                            uint16_t *pDepth, uint16_t *pTemp,
                            size_t length);

void DATA_reset(Data_t *p);

size_t DATA_add(Data_t *buf, uint32_t time, float depth, float temp);

size_t DATA_get_original(Data_t *p, uint32_t *time, float *depth, float *temp);

size_t DATA_get_converted(Data_t *p, uint32_t *start, uint16_t *offset, uint16_t *depth, uint16_t *temp);


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
