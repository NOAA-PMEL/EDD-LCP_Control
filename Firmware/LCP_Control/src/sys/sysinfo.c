#include "sysinfo.h"
#include "artemis_rtc.h"
#include "artemis_debug.h"
#include "control.h"
#include "config.h"

static SysInfo_t System = {

    .system_id = SYSTEM_ID,
    .serial_number = SERIAL_NUMBER,
    .firmware.major = FIRMWARE_MAJOR,
    .firmware.minor = FIRMWARE_MINOR,
    .firmware.patch = FIRMWARE_PATCH,
    .lcp_variant = LCP_VARIANT,
};

uint32_t SYS_get_serial_num(void)
{
    return System.serial_number;
}

uint16_t SYS_get_firmware(void)
{
    uint16_t ver = 0x00;
    ver |= System.firmware.major << 12;
    ver |= System.firmware.minor << 8;
    ver |= System.firmware.patch ;

    return ver;
}

uint16_t SYS_get_build_year_date(void)
{
    uint16_t date = 0x0000;

    System.build_date.day = toVal(&build_date[4]);
    System.build_date.month = mthToIndex(&build_date[0]);
    System.build_date.year = toVal(&build_date[9]);

    date |= (System.build_date.year << 9);
    date |= (System.build_date.month << 5);
    date |= (System.build_date.day);

    return date;
}

uint8_t SYS_get_system_id(void)
{
    return System.system_id;
}

uint8_t SYS_get_lcp_variant(void)
{
    return System.lcp_variant;
}

bool SYS_lcp_info(void)
{
    ARTEMIS_DEBUG_PRINTF("\n\nLCP Profiler Information\n");
    ARTEMIS_DEBUG_PRINTF("**************************************\n");

    uint8_t lcp_id = SYS_get_system_id();
    ARTEMIS_DEBUG_PRINTF("\tSystem ID\t: %u\n", lcp_id);

    uint16_t lcp_fw = SYS_get_firmware();
    ARTEMIS_DEBUG_PRINTF("\tLCP Firmware\t: %u.%u.%u-dev\n", lcp_fw>>12&0xF, lcp_fw>>8&0xF, lcp_fw&0xFF);

    uint16_t fw_date = SYS_get_build_year_date();
    ARTEMIS_DEBUG_PRINTF("\tFirmware Date\t: %u.%u.%u\n", fw_date>>5&0xF, fw_date&0x1F,  fw_date>>9&0x7F);
    ARTEMIS_DEBUG_PRINTF("\tFirmware Time\t: %s\n", build_time);

    uint8_t lcp_variant = SYS_get_lcp_variant();
    ARTEMIS_DEBUG_PRINTF("\tLCP Variant\t: %u\n", lcp_variant);

    uint32_t lcp_ser = SYS_get_serial_num();
    ARTEMIS_DEBUG_PRINTF("\tLCP Serial\t: %c%u%u\n", lcp_ser>>16&0xFF, lcp_ser>>8&0xFF, lcp_ser&0xFF);
    ARTEMIS_DEBUG_PRINTF("\n");

    ARTEMIS_DEBUG_PRINTF("**************************************\n");
    ARTEMIS_DEBUG_PRINTF("\tLCP Physical parameters\n");
    ARTEMIS_DEBUG_PRINTF("**************************************\n");
    ARTEMIS_DEBUG_PRINTF("Estimated Mass\t: %.3f kg, %.3f lbs\n", SYSTEM_MASS_EST * 0.453592, SYSTEM_MASS_EST);
    ARTEMIS_DEBUG_PRINTF("Minimum Volume\t: %.3f m^3, %.3f in^3\n", SYSTEM_VOLUME_MIN*0.000016387, SYSTEM_VOLUME_MIN);
    float density = CTRL_calculate_lcp_density(SYSTEM_VOLUME_MIN);
    ARTEMIS_DEBUG_PRINTF("Maximum Density\t: %.3f kg/m^3, %.3f lbs/in^3\n", density, density*0.000036127);
    ARTEMIS_DEBUG_PRINTF("\n");

    return true;
}
