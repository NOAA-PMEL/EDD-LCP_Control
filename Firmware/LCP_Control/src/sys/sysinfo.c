#include "sysinfo.h"
#include <string.h>


static SysInfo_t System = {
    .serial_number = SERIAL_NUMBER,
    .firmware.major = FIRMWARE_MAJOR,
    .firmware.minor = FIRMWARE_MINOR,
    .firmware.build = FIRMWARE_BUILD,
};








uint16_t SYS_get_serial_num(void)
{
    return System.serial_number;
}

void SYS_get_firmware( uint8_t *major, uint8_t *minor, uint8_t *build)
{
    *major = System.firmware.major;
    *minor = System.firmware.minor;
    strncpy(build, System.firmware.build, 6);
}