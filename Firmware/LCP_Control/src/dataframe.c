#include "dataframe.h"
#include <string.h>
#include "sysinfo.h"


#include <stdio.h>


typedef struct sDF_profile_t
{
    uint16_t idx;
    uint8_t *pDf;
    uint32_t time;
    float *pTemp;
    float *pDepth;
    float lat;
    float lon;
    uint16_t remLen;
    uint8_t page;
}DF_profile_t;

static DF_profile_t profile;


bool DF_create_profile( bool first, uint8_t *df, uint32_t time, uint16_t dataLen,
                            float lat, float lon, float *temp, float *depth)
{
    uint16_t msgLen = 0;
    bool last = false;

    /** If this is the first call, initialize the variables */
    if(first)
    {
        profile.idx = 0;
        // profile.pDf = df;
        profile.time = (uint32_t) time;
        profile.remLen = (uint16_t) dataLen;
        profile.pTemp = (float*) temp;
        profile.pDepth = (float*) depth;
        profile.lat = (float) lat;
        profile.lon = (float) lon;
        profile.page = 0;
    }

    /** Determine how long the data in this message will be */
    // printf("remLen = %u\n", profile.remLen);
    if(profile.remLen > 60)
    {
        msgLen = 60;
    } else {
        msgLen = profile.remLen;
    }
    profile.remLen -= msgLen;

    /** Does the message end after this frame? */
    // printf("remLen = %u\n", profile.remLen);
    if(profile.remLen > 0)
    {
        last = false;
    } else {
        last = true;
    }

    // printf("msgLen=%u\n", msgLen);

    /** Create the message */
    DF_create_profile_page(df, profile.time, msgLen, profile.lat, profile.lon,
                            profile.pTemp, profile.pDepth, profile.page++, last);
    profile.pTemp += 60;
    profile.pDepth += 60;




    return last;
}

/**
 * @brief  Create a Profile Mode Dataframe 
 * 
 * Profile mode datafram has the following structure:
 * 
 * Start Char, ID, Serial Num, Firmware Major, Firmware Minor, Data Length, Data, CRC, Latitude, Longitude, Start Time, Page #, End Char
 * 
 * No commas are used.  All values are hex unless noted
 * Start Char (1 byte ascii): * for new message, @ continued message
 * ID (3 bytes ascii): LCP
 * Firmware Major (1 byte)
 * Firmware Minor (1 byte)
 * Data Length (1 byte)
 * Data (240 bytes)
 * CRC (2 bytes)
 * Latitude (5 bytes): Offset by 180degrees
 * Longitude (5 bytes): Adjusted for positive only, Lon = 0 - 360
 * Start time (4 byte): Epoch time
 * Page # (1 byte)
 * End Char (1 byte ascii): Either ! for end of message, or & for more message to follow
 * 
 * @param df Pointer to dataframe 
 * @param len Number of data samples
 * @param time Start time of profile
 * @param len Length of data arrays
 * @param lat Current Latitude
 * @param lon Current Longitude
 * @param temp Pointer to temperature data
 * @param depth Pointer to Depth temperature
 * @return uint16_t Data starting position of next message (0 if it all fits in one message) 
 */
uint16_t DF_create_profile_page( uint8_t *df,
                            uint32_t time, uint16_t len, 
                            float lat, float lon, 
                            float *temp, float *depth,
                            uint8_t page, bool last)
{
    uint16_t retVal = 0u;

    memset(df, 0, 270);

    /** Calculate # of bytes needed for */
    /** Current Setup */
    /** depth > uint16_t > 2 Bytes */
    /** temp > uint16_t > 2 Bytes */
    uint8_t data[240] = {0};
    uint8_t idx = 0;

    for(uint8_t i=0; i<len; i++)
    {
        uint16_t d, t;
        d = (uint16_t) (*depth * 10);
        t = (uint16_t)(*temp + 5.0) * 1000;

        data[idx++] = d >> 8;
        data[idx++] = d & 0x00FF;
        data[idx++] = t >> 8;
        data[idx++] = t & 0x00FF;
    }


    DF_create_generic_dataframe(df, LCP_MODE_PROFILE, time, idx, data, lat, lon, page, last);
    

}


uint16_t DF_create_park_page( uint8_t *df, uint16_t len, float lat,  float lon,
                                uint32_t *time, float *depth, float *temp, 
                                uint8_t page, bool last)
{
    uint16_t retVal = 0u;
    uint32_t startTime = *time;

    memset(df, 0, 270);

    /** Calculate # of bytes needed for */
    /** Current Setup */
    /** depth > uint16_t > 2 Bytes */
    /** temp > uint16_t > 2 Bytes */
    uint8_t data[240] = {0};
    uint8_t idx = 0;

    for(uint16_t i=0; i<len; i++)
    {       
        uint16_t d, t;
        d = (uint16_t) (*depth * 10);
        t = (uint16_t)(*temp + 5.0) * 1000;
        data[idx++] = time[i] >> 24;
        data[idx++] = time[i] >> 16;
        data[idx++] = time[i] >> 8;
        data[idx++] = time[i] & 0x000000FF;
        data[idx++] = d >> 8;
        data[idx++] = d & 0x00FF;
        data[idx++] = t >> 8;
        data[idx++] = t & 0x00FF;
    }

    printf("Here idx=%u\n", idx);
    DF_create_generic_dataframe(df, LCP_MODE_PARK, startTime, idx, data, lat, lon, page, last);
}

uint16_t DF_create_generic_dataframe(uint8_t *df, uint8_t mode, uint32_t time, uint16_t len, 
                                    uint8_t *data, float lat, float lon, uint8_t page, bool last)
{
    uint16_t retVal = 0u;

    memset(df, 0, 270);

    /** Calculate # of bytes needed for */
    /** Current Setup */
    /** depth > uint16_t > 2 Bytes */
    /** temp > uint16_t > 2 Bytes */
    
    if(len > MAX_DATAFRAME_DATA_LEN)
    {
        /** ERROR */
        printf("ERROR");
    }
    
    // printf("page dataLen=%u\n", dataLen);

    uint8_t *dStart = df;
    // printf("%p\n", df);
    if(page == 0)
    {
        *df++ = '*';
        // strncpy(df, "*LCP",4);
    } else {
        *df++ = '@';
    }

    strcat(df, "LCP");
    df += 3;
    
    /** Serial Number */
    // printf("%p\n", df);
    uint16_t sn = SYS_get_serial_num();
    *df++ = sn >> 8;
    *df++ = sn;

    /** Firmware Version */
    // printf("%p\n", df);
    uint8_t major, minor, build[6];
    SYS_get_firmware(&major, &minor, build);
    *df++ = major;
    *df++ = minor;

    /** LCP Mode */
    // printf("%p\n", df);
    *df++ = LCP_MODE_PROFILE;

    /** Data Length */
    // printf("%p\n", df);
    // printf("dataLen=%u\n", dataLen);
    *df++ = (uint8_t) len >> 8;
    *df++ = (uint8_t) (len & 0x00FF);
    // printf("dl>>8=0x%02x\n", dataLen>>8);
    // printf("dl&0xFF=0x%02x\n", dataLen & 0x00FF);
    /** Data */
    
    uint8_t *crcStart = df;
    for(uint8_t i=0; i<len; i++)
    {
        *df++ = *data++;
    }
    
    /** Add the CRC */
    uint16_t crc = module_create_crc(crcStart, len);
    // printf("crc=%u\n", crc);
    *df++ = (uint8_t) (crc >> 8) & 0x00FF;
    *df++ = (uint8_t) (crc & 0x00FF);

    *df++ = 0xAB;

    /** Add the lat */
    uint64_t lat64 = module_convert_latitude_to_uint64_t(lat);
    // printf("%ul\n", lat64);
    *df++ = (uint8_t) ( lat64 >> 32) & 0x00FF;
    *df++ = (uint8_t) ( lat64 >> 24 ) & 0x00FF;
    *df++ = (uint8_t) ( lat64 >> 16 ) & 0x00FF;
    *df++ = (uint8_t) ( lat64 >> 8) & 0x00FF;
    *df++ = (uint8_t) ( lat64 & 0x00FF);

    /** Add the lon */
    uint64_t lon64 = module_convert_longitude_to_uint64_t(lon);
    // printf("%ul\n", lon64);
    *df++ = (uint8_t) ( lon64 >> 32) & 0x00FF;
    *df++ = (uint8_t) ( lon64 >> 24 ) & 0x00FF;
    *df++ = (uint8_t) ( lon64 >> 16 ) & 0x00FF;
    *df++ = (uint8_t) ( lon64 >> 8) & 0x00FF;
    *df++ = (uint8_t) ( lon64 & 0x00FF);

    /** Add the time */
    // printf("time=%ul\n", time);
    *df++ = (uint8_t) (time >> 24);
    *df++ = (uint8_t) (time >> 16) & 0x00FF;
    *df++ = (uint8_t) (time >> 8) & 0x00FF;
    *df++ = (uint8_t) (time & 0x00FF);

    /** Page # */
    *df++ = page;
    
    /** Last page? */
    if(last)
    {
        *df = '!';
    } else {
        *df = '&';
    }
}



uint16_t module_create_crc(uint8_t *data, uint8_t len)
{
    uint8_t csA = 0;
    uint8_t csB = 0;

    while(len--)
    {
        csA += *data++;
        csB += csA;
    }
    // printf("csA = %u, csB = %u\n", )
    uint16_t crc = (csA << 8) | csB;

    return crc;
}




uint64_t module_convert_latitude_to_uint64_t(float latitude)
{
    latitude += 180.0f;
    latitude *= 1000.0f;
    return (uint32_t) latitude;

}

uint64_t module_convert_longitude_to_uint64_t(float longitude)
{
    if(longitude < 0.0f)
    {
        longitude += 360.0f;
    }
    longitude *= 1000.0;

    return (uint32_t) longitude;
}