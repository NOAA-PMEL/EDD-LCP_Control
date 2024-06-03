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
#include "config.h"

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
STATIC float module_convert_uint16_t_to_depth(uint16_t depth);
STATIC float module_convert_uint16_t_to_temperature(uint16_t temp);
STATIC float module_convert_uint8_t_to_pressure(uint8_t pressure);

STATIC uint8_t module_convert_pressure_to_uint8_t(float pressure);
STATIC uint16_t module_convert_temperature_to_uint16_t(float temp);

STATIC int32_t module_convert_latitude_to_int32_t(float latitude);
STATIC int32_t module_convert_longitude_to_int32_t(float longitude);

//static uint8_t header[27];

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************

//void DATA_setbuffer(Data_t *p, uint32_t *pStart, uint32_t *pStop,
//                            float *pPressure, float *pTemp, size_t length)
//{
//    p->cbuf.length = length;
//    p->cbuf.read = 0;
//    p->cbuf.written = 0;
//    p->data.pNumber = 0;
//
//    //p->data.pTimeOffset = pTime;
//    p->data.pStateTime = pStart;
//    p->data.pStopTime = pStop
//    p->data.pPressure = pPressure;
//    p->data.pTemperature = pTemp;
//}

void DATA_setbuffer(Data_t *buf, pData *P, float *pressure, float *temperature, uint32_t length)
{
    buf->cbuf.read = 0;
    buf->cbuf.written = 0;
    buf->cbuf.length = length;

    /* measurements arrays */
    buf->data.pressure = pressure;
    buf->data.temperature = temperature;

    /* profiles related */
    buf->pNumber = 0;
    buf->wLength = 0;
    buf->rLength = 0;

    /* profile arrays */
    buf->p = P;

    ARTEMIS_DEBUG_PRINTF("DATA :: SET, Maixmum Measurements (%u)\n", buf->cbuf.length);
}

void DATA_reset(Data_t *buf)
{
    //buf->cbuf.read = 0;
    //buf->cbuf.written = 0;
    //buf->data.start_time = 0;
    buf->cbuf.read = 0;
    buf->cbuf.written = 0;
    buf->pNumber = 0;
    buf->wLength = 0;
    buf->rLength = 0;
}

void DATA_add_gps(Data_t *buf, float latitude, float longitude, uint8_t pNumber)
{
    buf->p[pNumber].pLatitude = latitude;
    buf->p[pNumber].pLongitude = longitude;
}

//size_t DATA_add(Data_t *buf, uint32_t time, float pressure, float temp, uint16_t prof_nr, bool finish)
void DATA_add(Data_t *buf, uint32_t time, float pressure, float temperature, uint8_t pNumber)
{
    if (buf->cbuf.written == 0)
    {
        if (pNumber != 0)
        {
            ARTEMIS_DEBUG_PRINTF("DATA :: ERROR, Profile is not zero\n");
        }
        else
        {
            buf->p[buf->pNumber].pStart = time;
            buf->p[buf->pNumber].pIndex = buf->cbuf.written;
        }
    }
    else
    {
        if (pNumber > buf->pNumber)
        {
            buf->pNumber++;
            buf->p[buf->pNumber].pIndex = buf->cbuf.written;
            buf->p[buf->pNumber].pStart = time;
            buf->wLength = 0;
        }
    }

    if (buf->cbuf.written < buf->cbuf.length)
    {
        buf->data.temperature[buf->cbuf.written] = temperature;
        buf->data.pressure[buf->cbuf.written] = pressure;
        buf->p[buf->pNumber].pStop = time;

        //ARTEMIS_DEBUG_PRINTF("DATA :: ADD, Temperature = %.2f, Pressure = %.2f\n", buf->data.temperature[buf->cbuf.written], buf->data.pressure[buf->cbuf.written]);

        buf->wLength++;
        buf->cbuf.written++;
        buf->p[buf->pNumber].pLength = buf->wLength;
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("DATA :: ERROR, Maximum length overflows\n");
    }

    //if(buf->cbuf.written == 0)
    //{
    //    buf->data.start_time = time;
    //}

    //if(buf->cbuf.written < buf->cbuf.length)
    //{
    //    buf->data.pPressure[buf->cbuf.written] = pressure;
    //    buf->data.pTemperature[buf->cbuf.written] = temp;
    //    //buf->data.pTimeOffset[buf->cbuf.written] = time - buf->data.start_time;
    //    buf->data.pTimeOffset[buf->cbuf.written] = time;
    //    buf->cbuf.written++;
    //    return(1);
    //}
    //return(0);
}

void DATA_get_original(Data_t *buf, pData *P, float *pressure, float *temperature, uint8_t pNumber)
{
    if (buf->cbuf.read < buf->cbuf.written)
    {
        *temperature = buf->data.temperature[buf->cbuf.read];
        *pressure = buf->data.pressure[buf->cbuf.read];

        P->pStart = buf->p[pNumber].pStart;
        P->pStop = buf->p[pNumber].pStop;
        P->pLatitude = buf->p[pNumber].pLatitude;
        P->pLongitude = buf->p[pNumber].pLongitude;

        buf->cbuf.read++;
        buf->rLength++;

        if (buf->rLength == buf->p[pNumber].pLength)
        {
            ARTEMIS_DEBUG_PRINTF("\nDATA :: READ, Profile(%u) length (%u) reached\n", pNumber, buf->rLength);
            buf->rLength = 0;
        }
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("DATA :: READ : ERROR, read(%u) overflows\n", buf->cbuf.read);
    }
}

void DATA_get_converted(Data_t *buf, pData *P, uint8_t *pressure, uint16_t *temperature, uint8_t pNumber)
{
    if (buf->cbuf.read < buf->cbuf.written)
    {
        *pressure = module_convert_pressure_to_uint8_t(buf->data.pressure[buf->cbuf.read]);
        *temperature = module_convert_temperature_to_uint16_t(buf->data.temperature[buf->cbuf.read]);

        P->pStart = buf->p[pNumber].pStart;
        P->pStop = buf->p[pNumber].pStop;
        P->pLatitude = buf->p[pNumber].pLatitude;
        P->pLongitude = buf->p[pNumber].pLongitude;

        buf->cbuf.read++;
        buf->rLength++;

        if (buf->rLength == buf->p[pNumber].pLength)
        {
            ARTEMIS_DEBUG_PRINTF("\n\nDATA :: READ, Profile(%u) length (%u) reached\n", pNumber, buf->rLength);
            buf->rLength = 0;
        }
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("DATA :: READ : ERROR, read(%u) overflows\n", buf->cbuf.read);
    }
}

//size_t DATA_get_original(Data_t *buf, uint32_t *time, float *pressure, float *temp)
//{
//    if( buf->cbuf.read < buf->cbuf.written)
//    {
//        *time = buf->data.pTimeOffset[buf->cbuf.read] + buf->data.start_time;
//        *pressure = buf->data.pPressure[buf->cbuf.read];
//        *temp = buf->data.pTemperature[buf->cbuf.read];
//        buf->cbuf.read++;
//        return (1);
//    }
//    return (0);
//}
//
//size_t DATA_get_converted(Data_t *p, uint32_t *start, uint32_t *offset, uint8_t *pressure, int16_t *temp)
//{
//    //ARTEMIS_DEBUG_PRINTF("read = %u, written = %u\n", p->cbuf.read, p->cbuf.written);
//
//    if( p->cbuf.read < p->cbuf.written)
//    {
//        *start = p->data.start_time;
//        *offset = p->data.pTimeOffset[p->cbuf.read];
//        *pressure = module_convert_pressure_to_uint8_t( p->data.pPressure[p->cbuf.read] );
//        *temp = module_convert_temperature_to_int16_t( p->data.pTemperature[p->cbuf.read] );
//        p->cbuf.read++;
//        return (1);
//    }
//    return (0);
//}

uint32_t get_epoch_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    year = year + 2000;
    const uint16_t daysInMonth[] = {0, 31, 59, 90, 120, 151, 181, 211, 242, 272, 303, 333 };
    uint32_t epoch =    (year - 1970) * 86400 * 365 + ((year - 1968) / 4) * 86400 +
                        daysInMonth[month - 1] * 86400 + (day - 1) * 86400 +
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
 * @shit -5° to the left (fitting values within 12 bits (-5 to 35.59)
 */
static float module_convert_uint16_t_to_temperature(uint16_t temp)
{
    int16_t t = temp - 500;
    float fTemp = (float) t;
    fTemp /= 100;
    return fTemp;
}

/**
 * @brief Convert pressure to uint8_t
 * 
 * Converts the pressure value to fit in an unsigned 8-bit uint, precision 0.1.
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

STATIC float module_convert_uint8_t_to_pressure(uint8_t pressure)
{
    return ((float)pressure/10.0f);
}

/**
 * @brief Convert temperature to uint16_t
 * 
 * Converts the S9 OEM Temperature value to fit in a unsigned 16-bit uint.
 * shitfing +5° so that (-5 to 35.59) can fit within 12-bits of 16 bits, precision 0.01
 * temp (uint16_t) = (temp (float) + 5.0) * 100
 * 
 * @param temp Temperature
 * @return int16_t Converted value;
 */
STATIC uint16_t module_convert_temperature_to_uint16_t(float temp)
{
    temp *= 100;
    temp += 500;
    return (uint16_t) (temp);
}

//void create_header_irid(uint8_t *df, pData *P, uint8_t mode_type, uint8_t profNumber, uint8_t pageNumber)
void create_header_irid(uint8_t *df, pData *P, sData *S)
{
    uint8_t buf[IRID_HEADER_LENGTH] = {0};
    uint8_t profNumber = S->profNumber;
    uint8_t pageNumber = S->pageNumber;
    uint8_t modeType = S->modeType;

    /** collect IRID_HEADER_LENGTH=28 bytes for the header and rest will be
        attached as a payload measurement data **/

    /*  1. System ID */
    buf[0] = SYS_get_system_id();

    /*  2. Firmware Version */
    buf[1] = SYS_get_firmware()>>8 ;
    buf[2] = SYS_get_firmware()&0xFF;

    /*  3. Build Year_Date */
    buf[3] = SYS_get_build_year_date()>>8 ;
    buf[4] = SYS_get_build_year_date()&0xFF;

    /*  4. Latitude  */
    int32_t latitude = module_convert_latitude_to_int32_t(P[profNumber].pLatitude);
    buf[5] = latitude>>24;
    buf[6] = latitude>>16;
    buf[7] = latitude>>8;
    buf[8] = latitude&0xFF;

    /*  5. Longitude  */
    int32_t longitude = module_convert_longitude_to_int32_t(P[profNumber].pLongitude);
    buf[9]  = longitude>>24;
    buf[10] = longitude>>16;
    buf[11] = longitude>>8;
    buf[12] = longitude&0xFF;

    /*  6. LCP Variant */
    buf[13] = SYS_get_lcp_variant();

    /*  7. Serial Number */
    uint32_t ser = SYS_get_serial_num();
    buf[14] = ser>>16&0xFF;
    buf[15] = ser>>8&0xFF;
    buf[16] = ser&0xFF;

    /* 8. Profile number */
    buf[17] = profNumber;

    /* 9. Length of measurements in one profile */
    buf[18] = P[profNumber].pLength;

    /* 10. Mode - higher 4bites, Type of measurements -> (Profile_mode=0x01, Park_mode=0x00) */
    /* 11. Page Number - lower 4bits ,(in case of multiple measurements exceede the 340 bytes */
    buf[19] = (modeType<<4&0xF0) | (pageNumber&0xF);

    /* 12. Start time for one profile */
    uint32_t start = P[profNumber].pStart;
    buf[20] = start>>24;
    buf[21] = start>>16;
    buf[22] = start>>8;
    buf[23] = start&0xFF;

    /* 13. Stop time for one profile */
    uint32_t stop = P[profNumber].pStop;
    buf[24] = stop>>24;
    buf[25] = stop>>16;
    buf[26] = stop>>8;
    buf[27] = stop&0xFF;

    /* Copy buf (header) to *df pointer */
    for (uint8_t i=0; i<IRID_HEADER_LENGTH; i++)
    {
        df[i] = buf[i];
        //ARTEMIS_DEBUG_PRINTF("0x%02X, 0x%02X \n", df[i], buf[i]);
    }
}

void create_header_irid_ext(uint8_t *df, pData *P, sData *S)
{
    /* header_irid_ext length IRID_HEADER_LENGTH_EXT = 19 bytes */
    uint8_t buf[IRID_HEADER_LENGTH_EXT] = {0};
    uint8_t profNumber = S->profNumber;
    uint8_t pageNumber = S->pageNumber;
    uint8_t modeType = S->modeType;
    uint8_t mLength = P[profNumber].pLength;

    /* 1. profile number*/
    buf[0] = profNumber;

    /* 2. profile measurement length */
    buf[1] = mLength;

    /* 3. Mode - higher 4bites, Type of measurements -> (Profile_mode=0x01, Park_mode=0x00) */
    /* 4. Page Number - lower 4bits ,(in case of multiple measurements exceede the 340 bytes */
    buf[2] = (modeType<<4&0xF0) | (pageNumber&0xF);

    /* 5. start time */
    uint32_t start = P[profNumber].pStart;
    buf[3] = start >> 24;
    buf[4] = start >> 16;
    buf[5] = start >> 8;
    buf[6] = start & 0xFF;

    /* 6. stop time */
    uint32_t stop = P[profNumber].pStop;
    buf[7] = stop >> 24;
    buf[8] = stop >> 16;
    buf[9] = stop >> 8;
    buf[10] = stop & 0xFF;

    /* 7. Latitude  */
    int32_t latitude = module_convert_latitude_to_int32_t(P[profNumber].pLatitude);
    buf[11] = latitude >> 24;
    buf[12] = latitude >> 16;
    buf[13] = latitude >> 8;
    buf[14] = latitude & 0xFF;

    /* 8. Longitude  */
    int32_t longitude = module_convert_longitude_to_int32_t(P[profNumber].pLongitude);
    buf[15] = longitude  >> 24;
    buf[16] = longitude  >> 16;
    buf[17] = longitude  >> 8;
    buf[18] = longitude  & 0xFF;

    /* copy buff to *df pointer */
    for (uint8_t i=0; i<IRID_HEADER_LENGTH_EXT; i++)
    {
        df[i] = buf[i];
        //ARTEMIS_DEBUG_PRINTF("0x%02X, 0x%02X \n", df[i], buf[i]);
    }
}

uint16_t pack_measurements_irid(Data_t *buf, pData *P, sData *S, uint8_t *rBuf)
{
    uint8_t length = S->mLength;
    uint8_t pNumber = S->profNumber;

    /* local buffer IRID_DATA_OUT=340 to hold the measurements */
    uint8_t df[IRID_DATA_OUT] = {0};

    /* create a park header_irid 28 bytes */
    create_header_irid(df, P, S);

    /* calculate the number of bytes (uint8_t) required for the length
       12 bits for temperature, and 8 bits for pressure = 20 bits
       add more bits for future sensor inclusion.
    */
    uint16_t bytes_length = (length * MEASUREMENT_BITS + 7) / 8 ;
    ARTEMIS_DEBUG_PRINTF("DATA :: (%u) measurements require (%u) bytes\n", length, bytes_length);

    /* start collecting measurements into 312 bytes or less */
    uint16_t Temp = 0;
    uint8_t Pressure = 0;
    /* start at the 28th position */
    uint16_t bytes = IRID_HEADER_LENGTH;
    uint32_t bit_pos = 0;

    for (uint16_t i=0; i<length; i++)
    {
        DATA_get_converted(buf, P, &Pressure, &Temp, pNumber);
        /* get temperature and fit into (12 bits) */
        for (uint8_t j=0; j<12; j++)
        {
            if (Temp & (1 << (11-j)))
            {
                df[bytes] |= (1 << (7 - (bit_pos % 8)));
            }

            bit_pos++;
            if ((bit_pos % 8) == 0)
            {
                bytes++;
            }
        }

        /* get pressure and fit into (8 bits) */
        for (uint8_t j=0; j<8; j++)
        {
            if (Pressure & (1 << (7-j)))
            {
                df[bytes] |= (1 << (7 - (bit_pos % 8)));
            }

            bit_pos++;
            if ((bit_pos % 8) == 0)
            {
                bytes++;
            }
        }
        Temp = 0, Pressure = 0;
    }

    if (bytes_length > IRID_DATA_OUT)
    {
        ARTEMIS_DEBUG_PRINTF("DATA :: ERROR : bytes_length(%u) exceeds IRID_DATA_OUT (%u) bytes\n", bytes_length, IRID_DATA_OUT);
        return 0;
    }
    /* put local measurements into rBuf (out) */
    for (uint16_t i=0; i<(IRID_HEADER_LENGTH+bytes_length); i++)
    {
        rBuf[i] = df[i];
        //ARTEMIS_DEBUG_PRINTF("DATA :: GET, rBuf[%u] = 0x%02X \n", i, rBuf[i]);
    }
    ARTEMIS_DEBUG_PRINTF("DATA :: RETURN Irid: total bytes = (%u)\n", bytes_length + IRID_HEADER_LENGTH);
    /* return number of bytes being transmitted header + payload (28 + bytes_length) */
    return ( IRID_HEADER_LENGTH + bytes_length);
}

uint16_t pack_measurements_irid_ext(Data_t *buf, pData *P, sData *S, uint8_t *rBuf)
{
    uint8_t length = S->mLength;
    uint8_t pNumber = S->profNumber;

    /* local buffer IRID_DATA_OUT=340 to hold the measurements */
    uint8_t df[IRID_DATA_OUT] = {0};

    /* create a park header_irid_ext 19 bytes */
    create_header_irid_ext(df, P, S);

    uint16_t bytes_length = (length * MEASUREMENT_BITS + 7) / 8 ;
    ARTEMIS_DEBUG_PRINTF("DATA :: Extension : (%u) measurements require (%u) bytes\n", length, bytes_length);

    /* start collecting measurements into 312 bytes or less */
    uint16_t Temp = 0;
    uint8_t Pressure = 0;
    /* start at the 19th position */
    uint16_t bytes = IRID_HEADER_LENGTH_EXT;
    uint32_t bit_pos = 0;

    for (uint16_t i=0; i<length; i++)
    {
        DATA_get_converted(buf, P, &Pressure, &Temp, pNumber);
        /* get temperature and fit into (12 bits) */
        for (uint8_t j=0; j<12; j++)
        {
            if (Temp & (1 << (11-j)))
            {
                df[bytes] |= (1 << (7 - (bit_pos % 8)));
            }

            bit_pos++;
            if ((bit_pos % 8) == 0)
            {
                bytes++;
            }
        }

        /* get pressure and fit into (8 bits) */
        for (uint8_t j=0; j<8; j++)
        {
            if (Pressure & (1 << (7-j)))
            {
                df[bytes] |= (1 << (7 - (bit_pos % 8)));
            }

            bit_pos++;
            if ((bit_pos % 8) == 0)
            {
                bytes++;
            }
        }
    }
    ARTEMIS_DEBUG_PRINTF("DATA :: Extension, RETURN Irid_ext : total bytes = (%u)\n", bytes_length + IRID_HEADER_LENGTH_EXT);
    /* return number of bytes being transmitted header + payload (19 + bytes_length */
    return ( IRID_HEADER_LENGTH_EXT + bytes_length );
}

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
