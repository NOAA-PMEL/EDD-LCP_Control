#ifndef SYSINFO_H
#define SYSINFO_H

#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_ID       ( 0x4D )        /** < NOAA system ID, 0x4D = 77 */

#define FIRMWARE_MAJOR  ( 0x0 )
#define FIRMWARE_MINOR  ( 0x0 )
#define FIRMWARE_PATCH  ( 0x0 )

#define SERIAL_NUMBER   ( 0x4C0000 )    /** < Serial number always starts with L = LCP,
                                              rest two bytes can be used to differentiate
                                              65535 LCPS profilers , LCP variant can be
                                              used as well */

#define LCP_VARIANT     ( 0x00 )        /** < LCP variant defines the number of sensors
                                              attached to the LCP profiler */

static const char *const build_date = __DATE__;
static const char *const build_time = __TIME__;

typedef struct sSysInfo_t
{
    uint8_t system_id ;
    uint32_t serial_number: 24;     /**< System Serial Number */
    struct
    {
        uint8_t major:  4;          /**< Firmware Major Number */
        uint8_t minor:  4;          /**< Firmware Minor Number */
        uint8_t patch:  8;          /**< Firmware Patch Number */
    } firmware;
    uint8_t lcp_variant;
    struct
    {
        uint8_t year:   7;
        uint8_t month:  4;
        uint8_t day:    5;
    } build_date;

} __attribute__((packed)) SysInfo_t ;

uint16_t SYS_get_build_year_date(void);
uint32_t SYS_get_serial_num(void);
uint16_t SYS_get_firmware(void);
uint8_t SYS_get_system_id(void);
uint8_t SYS_get_lcp_variant(void);
bool SYS_lcp_info(void);

#endif // SYSINFO_H
