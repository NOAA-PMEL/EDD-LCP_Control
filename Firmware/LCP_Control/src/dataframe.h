#ifndef DATAFRAME_H
#define DATAFRAME_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_DATAFRAME_DATA_LEN      ( 240 )

#define LCP_MODE_PARK               ( 1 )
#define LCP_MODE_PROFILE            ( 2 )





bool DF_create_profile( bool first, uint8_t *df, uint32_t time, uint16_t dataLen,
                            float lat, float lon, float *temp, float *depth);



uint16_t DF_create_profile_page( uint8_t *df,
                            uint32_t time, uint16_t len, 
                            float lat, float lon, 
                            float *temp, float *depth,
                            uint8_t page, bool last);

uint16_t DF_create_park_page( uint8_t *df, uint16_t len, float lat,  float lon,
                                uint32_t *time, float *depth, float *temp, 
                                uint8_t page, bool last);

#ifdef TEST
uint64_t module_convert_latitude_to_uint64_t(float latitude);
uint64_t module_convert_longitude_to_uint64_t(float longitude);
uint16_t module_create_crc(uint8_t *data, uint8_t len);
uint16_t DF_create_generic_dataframe(uint8_t *df, uint8_t mode, uint32_t time, uint16_t len, 
                                    uint8_t *data, float lat, float lon, uint8_t page, bool last);
#endif

#endif // DATAFRAME_H
