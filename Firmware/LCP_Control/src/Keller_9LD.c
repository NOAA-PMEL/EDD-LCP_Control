/** @file Keller_9LD.c
 *  @brief Keller 9LD Driver
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date September 29, 2020
 *  @version 0.0.1
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */

#include "Keller_9LD.h"



/** @brief Convert bytes to pressure
 * 
 * Takes a bytes array (of 2 bytes) and converts to pressure using:
 * 
 * P [bar] = ( P [u16] – 16384 ) x ( P@49152 – P@16384 ) / 32768 + P@16384
 * 
 * @param *data Pointer to data array 
 * @param *p Pointer to 9LD structure
 * 
 * @return Pressure (kPa)
 */
STATIC float _convert_bytes_to_pressure(uint8_t *data, sKeller_9LD_t *p)
{
    uint16_t p_u16 = (uint16_t) ( (*data++ << 8) | *data);
    float pressure = p_u16 - 16384;
    pressure *= (p->p_max - p->p_min);
    pressure /= 32768;
    pressure += p->p_min;

    return pressure;
}

/** @brief Convert bytes to temperatures
 * 
 * Take a byte array (of 2 bytes) and converts to temperature using:
 * 
 * T[°C] = ( floor( T[u16] / 16 ) – 24 ) x 0.05°C – 50°C
 *
 */
STATIC float _convert_bytes_to_temperature(uint8_t *data)
{
    uint16_t value = (uint16_t) (*data++ << 8 | *data);

    float temperature = (float) (value / 16);
    temperature -= 24;
    temperature = floor(temperature);
    temperature *= 0.05;
    temperature -= 50.0;

    return temperature;
}

/** @brief Convert bytes to User Info
 * 
 * Converts a byte array (4 bytes) to user info and
 * populates the structure
 * 
 * @param *data Pointer to byte array
 * @param *p Pointer to 9LD Structure
 * 
 * @return None
 */
STATIC void _convert_user_information_bytes_to_struct( uint8_t *data, sKeller_9LD_t *p)
{
    p->id = _convert_product_codes(&data[0]);
    _convert_bytes_to_cal_date(&data[6], &p->date.year, &p->date.month, &p->date.day);
    p->p_min = _convert_bytes_to_fp(&data[8]);
    p->p_max = _convert_bytes_to_fp(&data[12]);
}


/** @brief Convert int to float
 * 
 * Converts a (32-bit) 4-byte integer into a float value.
 * 
 * @param *data Pointer to data array
 * 
 * @return float value
 */
STATIC float _convert_bytes_to_fp(uint8_t *data)
{
    union {
        float data_float;
        uint32_t data_u32;
    }cast;
    
    cast.data_u32 = 0;
    uint8_t i;

    for(i=4;i>0;i--)
    {
        cast.data_u32 = cast.data_u32 << 8;
        cast.data_u32 |=  (uint32_t)(*data++);
    }
    
    return cast.data_float;

}

/** Convert bytes to Cal Date
 * 
 * Convert a byte array (2-bytes) to the sensor
 * calibration date using the following byte breakdown:
 * 
 * 0bYYYYYMMM, 0bMDDDDDII
 * byte 0    ,  byte 1 
 * where :
 * YYYY is year -2010
 * MMMM is month
 * DDDDD is day
 * II is ignore
 * 
 * example:
 * 0b00010|1010|11101|00  ->  2|10|29|0
 * 
 * @param *data Pointer to data array
 * @param *year Pointer to year value
 * @param *month Pointer to month value
 * @param *day Pointer to day value
 * 
 * @return None
 */
STATIC void _convert_bytes_to_cal_date(
                                        uint8_t *data,
                                        uint16_t *year,
                                        uint8_t *month,
                                        uint8_t *day
                                        )
{
    *year = 0;
    *month = 0;
    *day = 0;

    *year = (uint16_t)(*data >> 3u) + 2010;
    *month = (uint8_t) ((*data++ & 0x07) << 1);
    *month |= (uint8_t) (*data >> 7);
    *day = (uint8_t) ((*data >> 2) & 0x1F); 

}

/** @brief Convert bytes to Unique ID
 * 
 * Convert a byte array (4-bytes) to the sensor 
 * Unique ID (32-bits)
 *
 * @param *data Pointer to data array
 *  
 * @return Unique ID
 */
STATIC uint32_t _convert_product_codes( uint8_t *data )
{

    uint32_t codes = (uint32_t) (*data++ << 8);
    codes |= (uint32_t) (*data++);
    codes |= (uint32_t) (*data++ << 24);
    codes |= (uint32_t) (*data << 16);

    return codes;
}

STATIC void _gpio_init()
{
    
}

STATIC void _i2c_init(void)
{

}

STATIC void _