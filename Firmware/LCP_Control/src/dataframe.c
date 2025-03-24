/**
 * @file dataframe.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief LCP Dataframe for transmitting data via Iridium SBD messages
 * @version 0.1
 * @date 2021-10-12
 *
 * 
 */
#include "dataframe.h"
#include <string.h>
#include "sysinfo.h"

/************************************************************************
*					STRUCTS & ENUMS
************************************************************************/
/**
 * @brief profile struct
 * Contains pointers and values for multi-page dataframes
 * 
 */
typedef struct sDF_profile_t
{
    uint16_t idx;       /**< Current index */
    uint8_t *pDf;       /**< Pointer to dataframe */
    uint32_t time;      /**< Start time of mode */
    float *pTemp;       /**< Pointer to temperature data */
    float *pDepth;      /**< Pointer to depth data */
    float lat;          /**< Latitude @ start or finish (depends on mode) */
    float lon;          /**< Longitude @ start or finish (depends on mode) */
    uint16_t remLen;    /**< Remaining length of data */
    uint8_t page;       /**< Current page number */
}DF_profile_t;


/************************************************************************
*					MODULE VARIABLES
************************************************************************/
static DF_profile_t profile;
static DF_profile_t park;

/************************************************************************
*					STATIC FUNCTION PROTOTYPES 
************************************************************************/

STATIC uint16_t module_create_crc(uint8_t *data, uint8_t len);
STATIC uint16_t module_convert_depth_to_uint16_t(float depth);
STATIC uint16_t module_convert_temperature_to_uint16_t(float temp);
STATIC int32_t module_convert_latitude_to_int32_t(float latitude);
STATIC int32_t module_convert_longitude_to_int32_t(float longitude);

STATIC void DF_create_generic_header(uint8_t *df, float lat, float lon);
STATIC uint16_t DF_create_profile_page( uint8_t *df,
                            uint32_t time, uint16_t len, 
                            float lat, float lon, 
                            float *temp, float *depth,
                            uint8_t page, bool last);

STATIC uint16_t DF_create_park_page( uint8_t *df, uint16_t len, float lat,  float lon,
                                uint32_t *time, float *depth, float *temp, 
                                uint8_t page, bool last);

STATIC uint16_t DF_create_generic_dataframe(uint8_t *df, uint8_t mode, uint32_t time, uint16_t len, 
                                    uint8_t *data, float lat, float lon, uint8_t page, bool last);


/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/
/**
 * @brief Create all profile messages to send 
 * 
 * 
 * 
 * @param first Is this the first message (t or f)
 * @param df Pointer to dataframe
 * @param time Start time of profile
 * @param dataLen Length of data array
 * @param lat Latitude @ start of profile (last known)
 * @param lon Longitude @ start of profile (last known)
 * @param temp Pointer to temperature array 
 * @param depth Pointer to depth array
 * 
 * @retval End of messages
 * @return true Last message
 * @return false More messages follow
 */
bool DF_create_profile( bool first, uint8_t *df, uint32_t time, uint16_t dataLen,
                            float lat, float lon, float *temp, float *depth)
{
    uint16_t msgLen = 0;
    bool last = false;

    /** If this is the first call, initialize the variables */
    if(first)
    {
        profile.idx = 0;
        profile.time = (uint32_t) time;
        profile.remLen = (uint16_t) dataLen;
        profile.pTemp = (float*) temp;
        profile.pDepth = (float*) depth;
        profile.lat = (float) lat;
        profile.lon = (float) lon;
        profile.page = 0;
    }

    /** Determine how long the data in this message will be */
    if(profile.remLen > 60)
    {
        msgLen = 60;
    } else {
        msgLen = profile.remLen;
    }
    profile.remLen -= msgLen;

    /** Does the message end after this frame? */
    if(profile.remLen > 0)
    {
        last = false;
    } else {
        last = true;
    }

    /** Create the message */
    DF_create_profile_page(df, profile.time, msgLen, profile.lat, profile.lon,
                            profile.pTemp, profile.pDepth, profile.page++, last);
    profile.pTemp += 60;
    profile.pDepth += 60;




    return last;
}


/**
 * @brief Create all park messages to send 
 * 
 * 
 * 
 * @param first Is this the first message (t or f)
 * @param df Pointer to dataframe
 * @param time Pointer to time array
 * @param dataLen Length of data array
 * @param lat Latitude @ start of park (last known)
 * @param lon Longitude @ start of park (last known)
 * @param temp Pointer to temperature array 
 * @param depth Pointer to depth array
 * 
 * @retval End of messages
 * @return true Last message
 * @return false More messages follow
 */
bool DF_create_park( bool first, uint8_t *df, uint32_t *time, uint16_t dataLen,
                            float lat, float lon, float *temp, float *depth)
{
    uint16_t msgLen = 0;
    bool last = false;

    /** If this is the first call, initialize the variables */
    if(first)
    {
        park.idx = 0;
        // park.pDf = df;
        park.time = (uint32_t) *time;
        park.remLen = (uint16_t) dataLen;
        park.pTemp = (float*) temp;
        park.pDepth = (float*) depth;
        park.lat = (float) lat;
        park.lon = (float) lon;
        park.page = 0;
    }

    /** Determine how long the data in this message will be */
    if(park.remLen > 60)
    {
        msgLen = 60;
    } else {
        msgLen = park.remLen;
    }
    park.remLen -= msgLen;

    /** Does the message end after this frame? */
    if(park.remLen > 0)
    {
        last = false;
    } else {
        last = true;
    }


    /** Create the message */
    DF_create_park_page(df, msgLen, park.lat, park.lon, time, depth, temp, park.page, last);
    park.pTemp += 60;
    park.pDepth += 60;




    return last;
}



/************************************************************************
*					STATIC FUNCTIONS
************************************************************************/

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
STATIC uint16_t DF_create_profile_page( uint8_t *df,
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


/**
 * @brief Create a single park page dataframe
 * 
 * Creates a dataframe for a single page with the following structure:
 * 
 * Start Char, ID, Serial Num, Firmware Major, Firmware Minor, Data Length, Data, CRC, Latitude, Longitude, Start Time, Page #, End Char
 * 
 * No commas are used.  All values are hex unless noted
 * Start Char (1 byte ascii): * for new message, @ continued message
 * ID (3 bytes ascii): LCP
 * Firmware Major (1 byte)
 * Firmware Minor (1 byte)
 * Data Length (1 byte)
 * Data (240 bytes) broken down into 8-byte data blocks (see below)
 * CRC (2 bytes)
 * Latitude (5 bytes): Offset by 180degrees
 * Longitude (5 bytes): Adjusted for positive only, Lon = 0 - 360
 * Start time (4 byte): Epoch time
 * Page # (1 byte)
 * End Char (1 byte ascii): Either ! for end of message, or & for more message to follow
 * 
 * Data block (8 bytes):
 *  data[7:0] = time[3:0], depth[1:0], temperature[1:]
 *  where
 *      depth (uint16_t) = depth (float) * 10
 *      temp (uint16_t) = (temp(float) + 5) * 1000
 * 
 * @param df pointer to dataframe
 * @param len number of data samples
 * @param lat latitude @ dive
 * @param lon longitude @ dive
 * @param time pointer to array of epoch times
 * @param depth pointer to array of depths (m)
 * @param temp pointer to array of temperatures (m)
 * @param page dataframe page #
 * @param last last frame in transmission?
 * @return uint16_t 
 */
STATIC uint16_t DF_create_park_page( uint8_t *df, uint16_t len, float lat,  float lon,
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

/**
 * @brief Create a generic dataframe header before the payload
 *
 * @param df Pointer to datafram
 * @param lat Last know latitude
 * @param lon Last know longitude
 * @
 */

STATIC void DF_create_generic_header(uint8_t *df, float lat, float lon)
{
    uint8_t buf[17] = {0};

    /** collect 17 bytes for a 14 bytes header and 3 bytes serial number
        which will be attached ahead of payload data and other parameters
    **/

    /*  1. System ID */
    buf[0] = SYS_get_system_id();

    /*  2. Firmware Version */
    buf[1] = SYS_get_firmware() >> 8 ;
    buf[2] = SYS_get_firmware() & 0xFF;

    /*  3. Build Year_Date */
    buf[3] = SYS_get_build_year_date() >> 8 ;
    buf[4] = SYS_get_build_year_date() & 0xFF;

    /*  4. Latitude  */
    int32_t latitude = module_convert_latitude_to_int32_t(lat);
    buf[5] = latitude >> 24;
    buf[6] = latitude >> 16;
    buf[7] = latitude >> 8;
    buf[8] = latitude & 0xFF;

    /*  5. Longitude  */
    int32_t longitude = module_convert_longitude_to_int32_t(lon);
    buf[9]  = longitude  >> 24;
    buf[10] = longitude  >> 16;
    buf[11] = longitude  >> 8;
    buf[12] = longitude  & 0xFF;

    /*  6. LCP Variant */
    buf[13] = SYS_get_lcp_variant();

    /*  7. Serial Number */
    uint32_t ser = SYS_get_serial_num();
    buf[14] = ser >> 16 & 0xFF;
    buf[15] = ser >> 8 & 0xFF;
    buf[16] = ser & 0xFF;

    /* copy buff to *df pointer */
    for (uint8_t i=0; i<17; i++)
    {
        df[i] = buf[i];
    }
}


/**
 * @brief Create a generic dataframe for LCP transmission
 * 
 * @param df Pointer to dataframe
 * @param mode Dataframe mode (i.e., PROFILE, PARK, etc.)
 * @param time Start time of dataframe
 * @param len Length of data
 * @param data Data block 
 * @param lat Last know latitude
 * @param lon Last know longitude
 * @param page Dataframe page number
 * @param last Last frame (true or false)
 * @return uint16_t TBD
 */
STATIC uint16_t DF_create_generic_dataframe(uint8_t *df, uint8_t mode, uint32_t time, uint16_t len, 
                                    uint8_t *data, float lat, float lon, uint8_t page, bool last)
{
    uint16_t retVal = 0u;

    memset(df, 0, 270);

    /** Calculate # of bytes needed for */
    /** Current Setup */
    /** depth > uint16_t > 2 Bytes */
    /** temp > uint16_t > 2 Bytes */
    
    if(len > MAX_DATAFRAME_OUT)
    {
        /** ERROR */
        printf("ERROR");
    }
    
    uint8_t *dStart = df;

    if(page == 0)
    {
        *df++ = '*';
    } else {
        *df++ = '@';
    }

    strcat(df, "LCP");
    df += 3;
    
    /** Serial Number */
    uint16_t sn = SYS_get_serial_num();
    *df++ = sn >> 8;
    *df++ = sn;

    /** Firmware Version */
    uint8_t major, minor, build[6];
    //SYS_get_firmware(&major, &minor, build);
    *df++ = major;
    *df++ = minor;

    /** LCP Mode */
    *df++ = LCP_MODE_PROFILE;

    /** Data Length */
    *df++ = (uint8_t) len >> 8;
    *df++ = (uint8_t) (len & 0x00FF);

    /** Data */
    
    uint8_t *crcStart = df;
    for(uint8_t i=0; i<len; i++)
    {
        *df++ = *data++;
    }
    
    /** Add the CRC */
    uint16_t crc = module_create_crc(crcStart, len);
    *df++ = (uint8_t) (crc >> 8) & 0x00FF;
    *df++ = (uint8_t) (crc & 0x00FF);

    *df++ = 0xAB;

    /** Add the lat */
    uint64_t lat64 = module_convert_latitude_to_uint64_t(lat);
    *df++ = (uint8_t) ( lat64 >> 32) & 0x00FF;
    *df++ = (uint8_t) ( lat64 >> 24 ) & 0x00FF;
    *df++ = (uint8_t) ( lat64 >> 16 ) & 0x00FF;
    *df++ = (uint8_t) ( lat64 >> 8) & 0x00FF;
    *df++ = (uint8_t) ( lat64 & 0x00FF);

    /** Add the lon */
    uint64_t lon64 = module_convert_longitude_to_uint64_t(lon);
    *df++ = (uint8_t) ( lon64 >> 32) & 0x00FF;
    *df++ = (uint8_t) ( lon64 >> 24 ) & 0x00FF;
    *df++ = (uint8_t) ( lon64 >> 16 ) & 0x00FF;
    *df++ = (uint8_t) ( lon64 >> 8) & 0x00FF;
    *df++ = (uint8_t) ( lon64 & 0x00FF);

    /** Add the time */
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


/**
 * @brief Create a CRC from the data provided
 * 
 * @param data Pointer to data array
 * @param len Length of data array
 * @return uint16_t calculated CRC
 */
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

/**
 * @brief Convert depth to uint16_t
 * 
 * Converts the depth value to fit in an unsigned 16-bit int.
 * 
 * depth(uint16_t) = depth(float) * 10;
 * 
 * @param depth Depth
 * @return uint16_t Converted value
 */

STATIC uint16_t module_convert_depth_to_uint16_t(float depth)
{
    return (uint16_t) depth * 10.0f;
}

/**
 * @brief Convert temperature to int16_t
 * 
 * Converts the S9 OEM Temperature value to fit in a signed 16-bit int.
 * 
 * temp (uint16_t) = (temp (float) + 5.0) * 1000
 * 
 * @param temp Temperature
 * @return uint16_t Converted value;
 */

STATIC int16_t module_convert_temperature_to_int16_t(float temp)
{
    int16_t temperature = temp * 100;
    return temperature;
}

/**
 * @brief Convert latitude into signed 32-bit integer
 * 
 * @param latitude Latitude to convert
 * @return int32_t Converted value
 */

STATIC int32_t module_convert_latitude_to_int32_t(float latitude)
{
    int32_t lat = latitude * 1000000;
    return lat;
}

/**
 * @brief Convert longitude into signed 32-bit integer
 * 
 * @param longitude Longitude to convert
 * @return int32_t Converted value
 */

STATIC int32_t module_convert_longitude_to_int32_t(float longitude)
{
    int32_t lon = longitude * 1000000;
    return lon;
}
