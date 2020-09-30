#include "Keller_9LD.h"




STATIC void _convert_user_information_bytes_to_struct( uint8_t *data, sKeller_9LD_t *p)
{
    p->id = _convert_product_codes(&data[0]);
    _convert_bytes_to_cal_date(&data[6], &p->date.year, &p->date.month, &p->date.day);
    p->p_min = _convert_bytes_to_fp(&data[8]);
    p->p_max = _convert_bytes_to_fp(&data[12]);
}

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

STATIC uint32_t _convert_product_codes( uint8_t *data )
{

    uint32_t codes = (uint32_t) (*data++ << 8);
    codes |= (uint32_t) (*data++);
    codes |= (uint32_t) (*data++ << 24);
    codes |= (uint32_t) (*data << 16);

    return codes;
}