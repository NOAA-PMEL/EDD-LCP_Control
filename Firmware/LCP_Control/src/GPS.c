/**
 * @file GPS.c
 * @brief GPS Driver with UBX Aiding and Sea Navigation Engine Configuration
 */

#include "GPS.h"
#include "ublox.h"
#include "ublox_ubx.h"
#include "artemis_ublox_i2c.h"
#include "artemis_rtc.h"

/************************************************************************
 *                                MACROS
 ************************************************************************/

/* UBX MGA Subtypes & IDs */
#define UBX_MGA_ID_INI              ( 0x40 )
#define UBX_MGA_INI_POS_LLH         ( 0x01 )
#define UBX_MGA_INI_TIME_UTC        ( 0x10 )

/* Dynamic Model Configuration */
#define UBX_CFG_NAV5_MASK_DYN       ( 0x0001 ) // Apply dynamic model parameter
#define UBX_DYN_MODEL_SEA           ( 5 )      // Sea dynamic model

/* Wire Protocol Constants */
#define UBX_CHECKSUM_SIZE           ( 2 )
#define UBX_FRAME_OVERHEAD_SIZE     ( UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE ) // 6 + 2 = 8 bytes

/************************************************************************
 *                               STRUCTS
 ************************************************************************/

#pragma pack(push, 1)

/**
 * @brief UBX-MGA-INI-TIME_UTC Payload (24 bytes)
 */
typedef struct
{
    uint8_t  type;          /**< 0x10 = UTC */
    uint8_t  version;       /**< 0x00 */
    uint8_t  ref;           /**< Reference source (0 = None/Fallback) */
    uint8_t  gnssId;        /**< 0 = GPS timescale */
    uint16_t year;          /**< Gregorian Year (e.g. 2026) */
    uint8_t  month;         /**< Month (1-12) */
    uint8_t  day;           /**< Day (1-31) */
    uint8_t  hour;          /**< Hour (0-23) */
    uint8_t  minute;        /**< Minute (0-59) */
    uint8_t  second;        /**< Second (0-59) */
    uint8_t  reserved1;
    uint32_t ns;            /**< Nanoseconds fraction */
    uint16_t tAccS;         /**< Time accuracy estimate in seconds */
    uint16_t reserved2;
    uint32_t tAccNs;        /**< Nanosecond accuracy */
} ubx_mga_ini_time_utc_t;

/**
 * @brief UBX-MGA-INI-POS_LLH Payload (20 bytes)
 */
typedef struct
{
    uint8_t  type;          /**< 0x01 = Position LLH */
    uint8_t  version;       /**< 0x00 */
    uint8_t  reserved1[2];  /**< Reserved */
    int32_t  lat;           /**< Latitude in deg * 1e7 */
    int32_t  lon;           /**< Longitude in deg * 1e7 */
    int32_t  alt;           /**< Altitude above ellipsoid in cm */
    uint32_t posAcc;        /**< 1-sigma position accuracy estimate in cm */
} ubx_mga_ini_pos_llh_t;

/**
 * @brief UBX-CFG-NAV5 Payload (36 bytes)
 */
typedef struct
{
    uint16_t mask;              /**< Parameters to apply (Bit 0 = dynModel) */
    uint8_t  dynModel;          /**< Dynamic platform model */
    uint8_t  fixMode;           /**< Position fixing mode */
    int32_t  fixedAlt;          /**< Fixed altitude (MSL) for 2D mode [m / 0.01] */
    uint32_t fixedAltVar;       /**< Fixed altitude variance [m^2 / 0.0001] */
    int8_t   minElev;           /**< Minimum elevation [deg] */
    uint8_t  drLimit;           /**< Reserved */
    uint16_t pDop;              /**< Position DOP mask */
    uint16_t tDop;              /**< Time DOP mask */
    uint16_t pAcc;              /**< Position accuracy mask [m] */
    uint16_t tAcc;              /**< Time accuracy mask [m] */
    uint8_t  staticHoldThresh;  /**< Static hold threshold [cm/s] */
    uint8_t  dgpsTimeOut;       /**< DGPS timeout [s] */
    uint8_t  cnoThreshNumSVs;   /**< Number of SVs for C/N0 threshold */
    uint8_t  cnoThresh;         /**< C/N0 threshold [dBHz] */
    uint16_t reserved1;
    uint16_t staticHoldMaxDist; /**< Static hold max distance [m] */
    uint8_t  utcStandard;       /**< UTC standard identifier */
    uint8_t  reserved2[5];
} ubx_cfg_nav5_t;

#pragma pack(pop)

/************************************************************************
 *                         STATIC HELPER FUNCTIONS
 ************************************************************************/

/**
 * @brief Serializes a UBX payload using ublox_ubx utilities and sends over I2C.
 */
static bool ubx_send_packet(uint8_t ubx_class, uint8_t ubx_id, uint8_t *payload, uint16_t payload_len)
{
    ubx_packet_t packet = {0};
    uint8_t tx_msg[64] = {0}; // Sized comfortably for NAV5 (44B) and INI (28-32B)
    uint16_t total_wire_len = payload_len + UBX_FRAME_OVERHEAD_SIZE;

    if (total_wire_len > sizeof(tx_msg))
    {
        return false;
    }

    // 1. Pack structure
    UBX_create_ubx_packet(ubx_class, ubx_id, payload, payload_len, &packet);

    // 2. Generate full wire format (sync + class + id + length + payload + CK_A + CK_B)
    UBX_create_msg_from_packet(&packet, tx_msg);

    // 3. Transmit full packet over I2C including checksum
    return artemis_ublox_i2c_send_msg(tx_msg, total_wire_len, true);
}

/************************************************************************
 *                          GLOBAL FUNCTIONS
 ************************************************************************/

bool GPS_set_dynamic_model_sea(void)
{
    ubx_cfg_nav5_t nav5 = {0};
    nav5.mask     = UBX_CFG_NAV5_MASK_DYN;
    nav5.dynModel = UBX_DYN_MODEL_SEA;

    return ubx_send_packet(UBX_CFG_CLASS, UBX_CFG_NAV5, (uint8_t *)&nav5, sizeof(nav5));
}

bool GPS_inject_rtc_time_with_acc(uint16_t time_accuracy_sec)
{
    rtc_time time;
    if (!artemis_rtc_get_time(&time))
    {
        return false;
    }

    ubx_mga_ini_time_utc_t time_msg = {0};
    time_msg.type     = UBX_MGA_INI_TIME_UTC;
    time_msg.version  = 0x00;
    time_msg.ref      = 0x00;
    time_msg.gnssId   = 0x00;
    
    // Guard against 2-digit vs 4-digit RTC year values
    time_msg.year     = (time.year < 100) ? (uint16_t)(2000 + time.year) : (uint16_t)time.year;
    time_msg.month    = time.month;
    time_msg.day      = time.day;
    time_msg.hour     = time.hour;
    time_msg.minute   = time.min;
    time_msg.second   = time.sec;
    time_msg.ns       = 0;
    time_msg.tAccS    = time_accuracy_sec;
    time_msg.tAccNs   = 0;

    return ubx_send_packet(UBX_MGA_CLASS, UBX_MGA_ID_INI, (uint8_t *)&time_msg, sizeof(time_msg));
}

bool GPS_inject_position(float latitude_deg, float longitude_deg, float altitude_m, float accuracy_km)
{
    if (latitude_deg == 0.0f && longitude_deg == 0.0f)
    {
        return false;
    }

    ubx_mga_ini_pos_llh_t pos_msg = {0};
    pos_msg.type    = UBX_MGA_INI_POS_LLH;
    pos_msg.version = 0x00;
    pos_msg.lat     = (int32_t)(latitude_deg * 1e7f);
    pos_msg.lon     = (int32_t)(longitude_deg * 1e7f);
    pos_msg.alt     = (int32_t)(altitude_m * 100.0f);             // meters -> cm
    pos_msg.posAcc  = (uint32_t)(accuracy_km * 1000.0f * 100.0f); // km -> cm

    return ubx_send_packet(UBX_MGA_CLASS, UBX_MGA_ID_INI, (uint8_t *)&pos_msg, sizeof(pos_msg));
}

bool GPS_initialize(void)
{
    UBLOX_initialize(UBLOX_COM_I2C, UBLOX_MSG_UBX, UBLOX_MSG_UBX, 1);
    return true;
}

void GPS_uninitialize(void)
{
    UBLOX_uninitialize();
}

bool GPS_Read(GPS_Data_t *data)
{
    bool fix = false;
    UBLOX_Nav_t ublox = {0};

    fix = UBLOX_read_nav(&ublox);

    if (fix)
    {
        data->fix = true;
        data->position.lat = ublox.position.lat;
        data->position.lon = ublox.position.lon;
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

void GPS_on(void)
{
    artemis_ublox_i2c_power_on();
}

void GPS_off(void)
{
    // Clean up u-blox peripheral state before dropping power rail
    UBLOX_uninitialize();
    artemis_ublox_i2c_power_off();
}