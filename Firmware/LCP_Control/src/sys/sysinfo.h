#ifndef SYSINFO_H
#define SYSINFO_H

#include <stdint.h>

// #ifdef TEST
#define SERIAL_NUMBER   ( 0x3C )
#define FIRMWARE_MAJOR  ( 0x01 )
#define FIRMWARE_MINOR  ( 0x32 )
#define FIRMWARE_BUILD  ( "a32b89" )
// #endif 

typedef struct sSysInfo_t
{
    uint16_t serial_number;     /**< System Serial Number */
    struct {
        uint8_t major;          /**< Firmware Major Number */
        uint8_t minor;          /**< Firmware Minor Number */
        uint8_t build[6];       /**< Firmware Build Number */
    }firmware;
}SysInfo_t;




uint16_t SYS_get_serial_num(void);
void SYS_get_firmware( uint8_t *major, uint8_t *minor, uint8_t *build);

#endif // SYSINFO_H
