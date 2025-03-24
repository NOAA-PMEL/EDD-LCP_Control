/**
 * @file ublox_ubx.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-28
 * 
 */
#ifndef UBLOX_UBX_H
#define UBLOX_UBX_H

/************************************************************************
*						STANDARD LIBRARIES
************************************************************************/
#include <stdint.h>

/************************************************************************
*							HEADER FILES
************************************************************************/


/************************************************************************
*							MACROS
************************************************************************/
#define UBX_HEADER_SIZE                 ( 8 )
#define UBX_PAYLOAD_SIZE_MAX            ( 2400 )
#define UBX_MSG_SIZE_MAX                ( UBX_PAYLOAD_SIZE_MAX + UBX_HEADER_SIZE)

#define UBX_HEADER_1                    ( 0xB5 )
#define UBX_HEADER_2                    ( 0x62 )

/************************************************************************
*							ENUM & STRUCTS
************************************************************************/
/**
 * @brief UBX Packet Structure
 * Packet structure for all UBX Packets
 */
typedef struct s_ubx_packet_t
{
  uint8_t cls;          /**< UBX Class */
  uint8_t id;           /**< UBX Class ID */
  uint16_t length;      /**< Payload Length */
  char *payload;        /**< pointer to Payload */
  uint8_t chksumA;      /**< Checksum A */
  uint8_t chksumB;      /**< Checksum B */
  uint16_t rxCount;
}ubx_packet_t;

/**
 * @brief UBX-NAV-PVT Struct
 * Navigation Position Velocity Time Solution
 * p375 of u-blox 8/ u-blox M8 Receiver description manual
 * 
 */
typedef struct s_ubx_nav_pvt_t {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t flags3;
    uint32_t reserved1;
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
}ubx_nav_pvt_t;



/**
 * @brief UBX-CFG-PRT Struct
 * Port Configuration for I2C
 * p252 of u-blox 8/ u-blox M8 Receiver description manual
 */
typedef struct s_ubx_cfg_prt_t {
    uint8_t portID;
    uint8_t reserved1;
    uint16_t txReady;
    uint32_t mode;
    uint32_t reserved2;
    uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
    uint16_t reserved3;
}ubx_cfg_prt_t;


/**
 * @brief UBX-CFG-RATE Struct
 * Navigation/Measurement rate settings
 * p255 of u-blox 8/ u-blox M8 Receiver description manual
 */
typedef struct s_ubx_cfg_rate_t {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
}ubx_cfg_rate_t;


/**
 * @brief ACK-ACK Enum
 * Return from ACK-ACK or ACK-NAK test
 */
typedef enum e_ubx_ack_nak_t {
    UBX_ACK_INVALID_RESPONSE,
    UBX_ACK_NAK_RESPONSE,
    UBX_ACK_ACK_RESPONSE,
}ubx_ack_t;


typedef struct e_ubx_mon_ver_t {
    char *ver;
}ubx_mon_ver_t;


/**
 * @brief UBX Classes
 * 
 */
typedef enum {
    UBX_ACK_CLASS = 0x05,
    UBX_AID_CLASS = 0x0B,
    UBX_CFG_CLASS = 0x06,
    UBX_ESF_CLASS = 0x10,
    UBX_HNR_CLASS = 0x28,
    UBX_INF_CLASS = 0x04,
    UBX_LOG_CLASS = 0x21,
    UBX_MGA_CLASS = 0x13,
    UBX_MON_CLASS = 0x0A,
    UBX_NAV_CLASS = 0x01,
    UBX_RXM_CLASS = 0x02,
    UBX_UPD_CLASS = 0x09
}eUBX_Class_t;


/**
 * @brief UBX-ACK IDs
 * 
 */
typedef enum {
    UBX_ACK_ACK     = 0x01,
    UBX_ACK_NAK     = 0x00
}eUBX_ACK_ID_t;

/**
 * @brief UBX-NAV ID's
 * 
 */
typedef enum eUBX_NAV_ID_t{
    UBX_NAV_AOPSTATUS        = 0x60,
    UBX_NAV_ATT              = 0x05,
    UBX_NAV_CLOCK            = 0x22,
    UBX_NAV_COV              = 0x36,
    UBX_NAV_DGPS             = 0x31,
    UBX_NAV_DOP              = 0x04,
    UBX_NAV_EELL             = 0x3d,
    UBX_NAV_EOE              = 0x61,
    UBX_NAV_GEOFENCE         = 0x39,
    UBX_NAV_HPPOSECEF        = 0x13,
    UBX_NAV_HPPOSLLH         = 0x14,
    UBX_NAV_NMI              = 0x28,
    UBX_NAV_ODO              = 0x09,
    UBX_NAV_ORB              = 0x34,
    UBX_NAV_POSECEF          = 0x01,
    UBX_NAV_POSLLH           = 0x02,
    UBX_NAV_PVT              = 0x07,
    UBX_NAV_RELPOSNED        = 0x3C,
    UBX_NAV_RESETODO         = 0x10, 
    UBX_NAV_SAT              = 0x35,
    UBX_NAV_SBAS             = 0x32,
    UBX_NAV_SLAS             = 0x42,
    UBX_NAV_SOL              = 0x06,
    UBX_NAV_STATUS           = 0x03,
    UBX_NAV_SVINFO           = 0x30,
    UBX_NAV_SVIN             = 0x3B,
    UBX_NAV_TIMEBDS          = 0x24,
    UBX_NAV_TIMEGAL          = 0x25,
    UBX_NAV_TIMEGLO          = 0x23,
    UBX_NAV_TIMEGPS          = 0x20,
    UBX_NAV_TIMELS           = 0x26,
    UBX_NAV_TIMEUTC          = 0x21,
    UBX_NAV_VELECEF          = 0x11,
    UBX_NAV_VELNED           = 0x12,
}UBX_NAV_ID_t;

/**
 * @brief UBX-CFG IDs
 * 
 */
typedef enum eUBX_CFG_ID_t {
    UBX_CFG_ANT              = 0x13,
    UBX_CFG_BATCH            = 0x93,
    UBX_CFG_CFG              = 0x09,
    UBX_CFG_DAT              = 0x06,
    UBX_CFG_DGNSS            = 0x70,
    UBX_CFG_DOSC             = 0x61,
    UBX_CFG_ESFALG           = 0x56,
    UBX_CFG_ESFA             = 0x4C,
    UBX_CFG_ESFG             = 0x4D,
    UBX_CFG_ESFWT            = 0x82,
    UBX_CFG_ESRC             = 0x60,
    UBX_CFG_GEOFENCE         = 0x69,
    UBX_CFG_GNSS             = 0x3E,
    UBX_CFG_HNR              = 0x5C,
    UBX_CFG_INF              = 0x02,
    UBX_CFG_ITFM             = 0x39,
    UBX_CFG_LOGFILTER        = 0x47,
    UBX_CFG_MSG              = 0x01,
    UBX_CFG_NAV5             = 0x24,
    UBX_CFG_NAVX5            = 0x23,
    UBX_CFG_NMEA             = 0x17,
    UBX_CFG_ODO              = 0x1E,
    UBX_CFG_PM2              = 0x3B,
    UBX_CFG_PMS              = 0x86,
    UBX_CFG_PRT              = 0x00,
    UBX_CFG_PWR              = 0x57, 
    UBX_CFG_RATE             = 0x08,
    UBX_CFG_RINV             = 0x34,
    UBX_CFG_RST              = 0x04,
    UBX_CFG_RXM              = 0x11,
    UBX_CFG_SBAS             = 0x16,
    UBX_CFG_SENIF            = 0x88,
    UBX_CFG_SLAS             = 0x8D,
    UBX_CFG_SMGR             = 0x62,
    UBX_CFG_SPT              = 0x64,
    UBX_CFG_TMODE2           = 0x3D,
    UBX_CFG_TMODE3           = 0x71,
    UBX_CFG_TP5              = 0x31,
    UBX_CFG_TXSLOT           = 0x53,
    UBX_CFG_USB              = 0x1B,
}UBX_CFG_ID_t;



/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/
void UBX_create_ubx_packet( 
                        uint8_t ubx_class, 
                        uint8_t ubx_id, 
                        uint8_t *payload, 
                        uint16_t len,
                        ubx_packet_t *packet
                        );
bool UBX_parse_ubx_packet(
                        uint8_t *msg,
                        uint8_t len,
                        ubx_packet_t *packet
                        );
void UBX_create_msg_from_packet(ubx_packet_t *packet, uint8_t *msg);
bool UBX_create_packet_from_msg(uint8_t *msg, uint16_t len, ubx_packet_t *packet);
int16_t UBX_find_start_of_msg(uint8_t *buf, uint16_t len);
ubx_ack_t UBX_check_for_ack(ubx_packet_t *packet, 
                            uint8_t expectedClass, 
                            uint8_t expectedId);

void UBX_parse_cfg(UBX_CFG_ID_t id, ubx_packet_t *packet, void *parsed);
void UBX_parse_nav(UBX_NAV_ID_t id, ubx_packet_t *packet, void *parsed);


#endif // UBLOX_UBX_H