/**
 * @file data.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-15
 * 
 * 
 */
#include "data.h"


//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
STATIC float module_convert_uint16_t_to_depth(uint16_t depth);
STATIC float module_convert_uint16_t_to_temperature(uint16_t temp);
static uint16_t module_convert_depth_to_uint16_t(float depth);
static uint16_t module_convert_temperature_to_uint16_t(float temp);


//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void DATA_setbuffer(Data_t *p, uint16_t *pTime, 
                            uint16_t *pDepth, uint16_t *pTemp,
                            size_t length)
{
    p->cbuf.length = length;
    p->cbuf.read = 0;
    p->cbuf.written = 0;

    p->data.pTimeOffset = pTime;
    p->data.pDepth = pDepth;
    p->data.pTemperature = pTemp;
}

void DATA_reset(Data_t *p)
{
    idx = 0;
    p->cbuf.read = 0;
    p->cbuf.written = 0;
    p->data.start_time = 0;
    
}

size_t DATA_add(Data_t *buf, uint32_t time, float depth, float temp)
{

    if(buf->cbuf.written == 0)
    {
        buf->data.start_time = time;        
    }

    if(buf->cbuf.written < buf->cbuf.length) {
        buf->data.pDepth[buf->cbuf.written] = depth;
        buf->data.pTemperature[buf->cbuf.written] = temp;
        buf->data.pTimeOffset[buf->cbuf.written] = time - buf->data.start_time;
        buf->cbuf.written++;
        return(1);
    }
    return(0);
}


size_t DATA_get_original(Data_t *p, uint32_t *time, float *depth, float *temp)
{
    if( p->cbuf.read < p->cbuf.written) 
    {
        *time = p->data.pTimeOffset[p->cbuf.read] + p->data.start_time;
        *depth = module_convert_uint16_t_to_depth(p->data.pDepth[p->cbuf.read]);
        *temp = module_convert_uint16_t_to_temperature(p->data.pTemperature[p->cbuf.read]);
        p->cbuf.read++;
        return 1;
    }

    return 0;
}


size_t DATA_get_converted(Data_t *p, uint32_t *start, uint16_t *offset, uint16_t *depth, uint16_t *temp)
{
    if( p->cbuf.read < p->cbuf.written) 
    {
        *start = p->data.start_time;
        *time = p->data.pTimeOffset[p->cbuf.read];
        *depth = p->data.pDepth[p->cbuf.read];
        *temp = p->data.pTemperature[p->cbuf.read];
        p->cbuf.read++;
        return 1;
    }

    return 0;
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
 * @brief Convert temperature to uint16_t
 * 
 * Converts the S9 OEM Temperature value to fit in a unsigned 16-bit int.
 * 
 * temp (uint16_t) = (temp (float) + 5.0) * 1000
 * 
 * @param temp Temperature
 * @return uint16_t Converted value;
 */
STATIC uint16_t module_convert_temperature_to_uint16_t(float temp)
{
    temp += 5.0f;
    temp *= 1000;

    return (uint16_t) temp;
}