#include "GPS.h"


#include "ublox.h"


void GPS_initialize(void)
{
    UBLOX_initialize( 
                        UBLOX_COM_I2C,
                        UBLOX_MSG_UBX,
                        UBLOX_MSG_UBX,
                        1
                    );
}

bool GPS_Read(GPS_Data_t *data)
{
    bool fix = false;
    UBLOX_Nav_t ublox = {0};

    fix = UBLOX_read_nav(ublox);

    if(fix)
    {
        data->fix = true;
        data->position.lat = ublox.position.lat;
        data->position.lat = ublox.position.lon;
        data->position.alt = ublox.position.alt;

        data->time.year = ublox.time.year;
        data->time.month = ublox.time.month;
        data->time.day = ublox.time.day;
        data->time.hour = ublox.time.hour;
        data->time.min = ublox.time.min;
        data->time.sec = ublox.time.sec;
    }
    return fix;
}
