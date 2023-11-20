/**
 * @file data.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-15
 * 
 * 
 */

#include <string.h>
#include <math.h>
#include "data.h"
#include "artemis_debug.h"
#include "sysinfo.h"

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
STATIC float module_convert_uint16_t_to_depth(uint16_t depth);
STATIC float module_convert_uint16_t_to_temperature(uint16_t temp);

static uint8_t module_convert_pressure_to_uint8_t(float pressure);
static int16_t module_convert_temperature_to_int16_t(float temp);

STATIC int32_t module_convert_latitude_to_int32_t(float latitude);
STATIC int32_t module_convert_longitude_to_int32_t(float longitude);

static uint8_t header[27];

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void DATA_setbuffer(Data_t *p, uint32_t *pTime, 
                            float *pPressure, float*pTemp,
                            size_t length)
{
    p->cbuf.length = length;
    p->cbuf.read = 0;
    p->cbuf.written = 0;

    p->data.pTimeOffset = pTime;
    p->data.pPressure = pPressure;
    p->data.pTemperature = pTemp;
}

void DATA_reset(Data_t *p)
{
    p->cbuf.read = 0;
    p->cbuf.written = 0;
    p->data.start_time = 0;
    
}

size_t DATA_add(Data_t *buf, uint32_t time, float pressure, float temp)
{

    if(buf->cbuf.written == 0)
    {
        buf->data.start_time = time;        
    }

    if(buf->cbuf.written < buf->cbuf.length)
    {
        buf->data.pPressure[buf->cbuf.written] = pressure;
        buf->data.pTemperature[buf->cbuf.written] = temp;
        //buf->data.pTimeOffset[buf->cbuf.written] = time - buf->data.start_time;
        buf->data.pTimeOffset[buf->cbuf.written] = time;
        buf->cbuf.written++;
        return(1);
    }
    return(0);
}

size_t DATA_get_original(Data_t *p, uint32_t *time, float *pressure, float *temp)
{
    if( p->cbuf.read < p->cbuf.written)
    {
        *time = p->data.pTimeOffset[p->cbuf.read] + p->data.start_time;
        *pressure = p->data.pPressure[p->cbuf.read];
        *temp = p->data.pTemperature[p->cbuf.read];
        p->cbuf.read++;
        return (1);
    }
    return (0);
}

size_t DATA_get_converted(Data_t *p, uint32_t *start, uint32_t *offset, uint8_t *pressure, int16_t *temp)
{
    //ARTEMIS_DEBUG_PRINTF("read = %u, written = %u\n", p->cbuf.read, p->cbuf.written);

    if( p->cbuf.read < p->cbuf.written)
    {
        *start = p->data.start_time;
        *offset = p->data.pTimeOffset[p->cbuf.read];
        *pressure = module_convert_pressure_to_uint8_t( p->data.pPressure[p->cbuf.read] );
        *temp = module_convert_temperature_to_int16_t( p->data.pTemperature[p->cbuf.read] );
        p->cbuf.read++;
        return (1);
    }
    return (0);
}

uint32_t get_epoch_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    year = year + 2000;
    const uint16_t daysInMonth[] = {0, 31, 59, 90, 120, 151, 181, 211, 242, 272, 303, 333 };
    uint32_t epoch =    (year - 1970) * 86400 * 365 + ((year - 1968) / 4) * 86400 +
                        //daysInMonth[month - 1] * 86400 + (day - 1) * 86400 +
                        daysInMonth[month - 1] * 86400 + (day * 86400) +
                        hour * 3600 + min * 60 + sec;

    if (!(year % 4) && month <= 2)
    {
        epoch -= 86400;
    }

    return epoch;
}


//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************
/**
 * @brief Convert the uint16_t version of depth to a float version
 * 
 * @param depth uint16_t version of depth
 * @return float float version of depth
 */
static float module_convert_uint16_t_to_depth(uint16_t depth)
{
    return (float) depth / 10.0f;
}

/**
 * @brief Convert the uint16_t version of temperature to float version
 * 
 * @param temp uitn16_t representation of temperature
 * @return float Actual float representation of temperature
 */
static float module_convert_uint16_t_to_temperature(uint16_t temp)
{
    float fTemp = (float) temp;
    fTemp /= 1000;
    fTemp -= 5.0f;

    return fTemp;
}

/**
 * @brief Convert pressure to uint8_t
 * 
 * Converts the pressure value to fit in an unsigned 8-bit int.
 * 
 *  (uint8_t) pressure = (float)(pressure * 10);
 * 
 * @param pressure Pressure
 * @return uint8_t Converted value
 */
STATIC uint8_t module_convert_pressure_to_uint8_t(float pressure)
{
    return (uint8_t) (pressure * 10.0f);
}

/**
 * @brief Convert temperature to uint16_t
 * 
 * Converts the S9 OEM Temperature value to fit in a unsigned 16-bit int.
 * 
 * temp (int16_t) = (temp (float) + 5.0) * 1000
 * 
 * @param temp Temperature
 * @return int16_t Converted value;
 */
STATIC int16_t module_convert_temperature_to_int16_t(float temp)
{
    //temp += 5.0f;
    //temp *= 1000;

    return (int16_t) (temp * 100.0f);
}

void DATA_get_iridium_park(uint8_t *pData);
void DATA_get_iridium_profile(uint8_t *pData);

void create_header(uint8_t *df, uint32_t start, uint32_t stop, float lat, float lon, uint8_t mode_type, uint8_t page)
{
    uint8_t buf[27] = {0};

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

    /* Type of measurement, Mode , Profile mode = 0x02, Park mode = 0x01 , page number*/
    buf[17] = mode_type;
    buf[18] = page;

    /* start time */
    buf[19] = start >> 24;
    buf[20] = start >> 16;
    buf[21] = start >> 8;
    buf[22] = start & 0xFF;

    /* stop time */
    buf[23] = stop >> 24;
    buf[24] = stop >> 16;
    buf[25] = stop >> 8;
    buf[26] = stop & 0xFF;

    /* copy buff to *df pointer */
    for (uint8_t i=0; i<27; i++)
    {
        df[i] = buf[i];
        //ARTEMIS_DEBUG_PRINTF("0x%02X, 0x%02X \n", df[i], buf[i]);
    }
}

STATIC int32_t module_convert_latitude_to_int32_t(float latitude)
{
    int32_t lat = latitude * 1000000;
    return lat;
}

STATIC int32_t module_convert_longitude_to_int32_t(float longitude)
{
    int32_t lon = longitude * 1000000;
    return lon;
}

float std_div(float *value, uint16_t len, float *var, float *avg)
{
    *avg = average(value, len);

    float diff = 0.0f;
    float diff_sum = 0.0f;

    for (uint16_t i=0; i<len; i++)
    {
        diff = *value++ - *avg;
        diff_sum += (diff * diff);
    }

    *var = diff_sum / len ;
    float std = sqrt (*var);

    return std;
}

float average(float *value, uint16_t len)
{
    float avg = 0;
    for (uint16_t i=0; i<len; i++)
    {
        avg += *value++;
    }
    return (float) avg/len;
}
