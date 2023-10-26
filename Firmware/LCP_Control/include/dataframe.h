/**
 * @file dataframe.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief LCP Dataframe for Transmitting Data back via Iridium SBD
 * @version 0.1
 * @date 2021-10-12
 * 
 * 
 */
#ifndef DATAFRAME_H
#define DATAFRAME_H


/************************************************************************
*						STANDARD LIBRARIES
************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/************************************************************************
*							MACROS
************************************************************************/

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
#define PERSISTENT __persistent 
#endif
#endif

#define MAX_DATAFRAME_OUT           ( 340 )
#define MAX_DATAFRAME_IN            ( 270 )

#define LCP_MODE_PARK               ( 1 )
#define LCP_MODE_PROFILE            ( 2 )




/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/
bool DF_create_profile( bool first, uint8_t *df, uint32_t time, uint16_t dataLen,
                            float lat, float lon, float *temp, float *depth);

bool DF_create_park( bool first, uint8_t *df, uint32_t *time, uint16_t dataLen,
                            float lat, float lon, float *temp, float *depth);

/************************************************************************
*					STATIC FUNCTION PROTOTYPES (FOR TEST ONLY)
************************************************************************/
#ifdef TEST

//STATIC uint16_t DF_create_profile_page( uint8_t *df,
//                            uint32_t time, uint16_t len,
//                            float lat, float lon,
//                            float *temp, float *depth,
//                            uint8_t page, bool last);
//
//STATIC uint16_t DF_create_park_page( uint8_t *df, uint16_t len, float lat,  float lon,
//                                uint32_t *time, float *depth, float *temp,
//                                uint8_t page, bool last);
//STATIC uint16_t module_convert_depth_to_uint16_t(float depth);
//STATIC uint16_t module_convert_temperature_to_uint16_t(float temp);
//STATIC uint64_t module_convert_latitude_to_uint64_t(float latitude);
//STATIC uint64_t module_convert_longitude_to_uint64_t(float longitude);
//STATIC uint16_t module_create_crc(uint8_t *data, uint8_t len);
//STATIC uint16_t DF_create_generic_dataframe(uint8_t *df, uint8_t mode, uint32_t time, uint16_t len,
//                                    uint8_t *data, float lat, float lon, uint8_t page, bool last);
#endif

#endif // DATAFRAME_H
