/**
 * @file ublox_ubx.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief UBLOX UBX Command Set.  Used to prep and convert UBX messages.
 * @version 0.1
 * @date 2021-09-28
 * 
 */

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "ublox_ubx.h"

//*****************************************************************************
//
//  Macros & Constants
//
//*****************************************************************************
#define UBX_HEADER_1    ( 0xB5 )
#define UBX_HEADER_2    ( 0x62 )

//*****************************************************************************
//
// Structs, Enums & Typedefs
//
//*****************************************************************************

/**
 * @brief Convert from U32 to I32
 * Also works backwards
 */
typedef union u_u32_to_i32
{
    /* data */
    uint32_t u32;   /**< 32-bit unsigned value */
    int32_t i32;    /**< 32-bit signed value */
}u32_to_i32_t;


/**
 * @brief Convert from U16 to I16
 * Also works backwards
 */
typedef union u_u16_to_i16_t
{
    uint16_t u16;   /**< 16-bit unsigned value */
    int16_t i16;    /**< 16-bit signed value */
}u16_to_i16_t;

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
static ubx_nak_t module_ubx_parse_ack_or_nak_msg(ubx_packet_t *packet, 
                                                uint8_t expectedCls,
                                                uint8_t expectedId);

static void module_ubx_parse_nav_pvt_packet(ubx_packet_t *packet, 
                                            ubx_nav_pvt_t *nav);

static void module_ubx_parse_cfg_prt_packet(ubx_packet_t *packet, 
                                            ubx_cfg_prt_t *prt);

static void module_ubx_parse_cfg_rate_packet(ubx_packet_t *packet, 
                                            ubx_cfg_rate_t *rate);

static uint32_t bytes_to_u32(uint8_t *data);
static int32_t bytes_to_i32(uint8_t *data);
static uint16_t bytes_to_u16(uint8_t *data);
static int16_t bytes_to_i16(uint8_t *data);
static void rmemcpy(uint8_t *dst, void *src, size_t n);

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
/**
 * @brief Creates a UBX Formatted packet
 * 
 * @param ubx_class UBX Class Type
 * @param ubx_id    UBX Class ID Type
 * @param payload   UBX Payload for specific UBX class + id
 * @param len       length of payload
 * @param packet    Pointer to data packet (return)
 */
void UBX_create_ubx_packet( 
                                uint8_t ubx_class, 
                                uint8_t ubx_id, 
                                uint8_t *payload, 
                                uint16_t len,
                                ubx_packet_t *packet
                                )
{
    packet->cls = ubx_class;
    packet->id = ubx_id;
    packet->length = len;
    packet->payload = payload;
    UBX_calc_ubx_checksum(&packet);

}

/**
 * @brief Parse UBX Packet into standard message (char array)
 * 
 * @param msg Pointer to formatted message
 * @param len Length of available message buffer
 * @param packet Packet to parse
 * @return true Valid message parsed
 * @return false Message not valid
 */
bool UBX_parse_ubx_packet(
                        uint8_t *msg,
                        uint8_t len,
                        ubx_packet_t *packet
                        )
{
    bool retVal = False;

    if( (msg[0] == UBX_HEADER_1 ) &&  (msg[1] == UBX_HEADER_2) )
    {
        retVal = true;
    }

    if( retVal == true)
    {
        packet->cls = msg[2];
        packet->id = msg[3]
        packet->length = (msg[5] << 8) | msg[4];
        packet->payload = msg[6];
        packet->chksumA = msg[6 + packet->length];
        packet->chksumB = msg[7 + packet->length];
    }

    return retVal;
}

/**
 * @brief Parse UBX-CFG Class Packet
 * 
 * This is a super function which parses a returned CFG packet into
 * a parsed structure for the specified ID.
 * 
 * Warning: The correct struct MUST be passed in the void *passed pointer
 * for this to work.  This is a dangerous function which has no error checking.
 * 
 * @param id        UBX Class ID 
 * @param packet    UBX Packet to Parse
 * @param parsed    UBX Class ID Structure
 */
void UBX_parse_cfg(UBX_CFG_ID_t id, ubx_packet_t *packet, void *parsed){

    switch(id)
    {
        
        case UBX_CFG_RATE:
            module_ubx_parse_cfg_rate_packet(packet, (ubx_cfg_rate_t*) parsed);
            break;
        case UBX_CFG_CFG:
            break;
        case UBX_CFG_PRT: 
            module_ubx_parse_cfg_prt_packet(packet, (ubx_cfg_prt_t*) parsed);
            break;
        case UBX_CFG_ANT:
        case UBX_CFG_BATCH:
        case UBX_CFG_DAT:
        case UBX_CFG_DGNSS:
        case UBX_CFG_DOSC: 
        case UBX_CFG_ESFALG: 
        case UBX_CFG_ESFA: 
        case UBX_CFG_ESFG: 
        case UBX_CFG_ESFWT:
        case UBX_CFG_ESRC: 
        case UBX_CFG_GEOFENCE:
        case UBX_CFG_GNSS:
        case UBX_CFG_HNR: 
        case UBX_CFG_INF: 
        case UBX_CFG_ITFM:
        case UBX_CFG_LOGFILTER:
        case UBX_CFG_MSG: 
        case UBX_CFG_NAV5: 
        case UBX_CFG_NAVX5: 
        case UBX_CFG_NMEA:
        case UBX_CFG_ODO: 
        case UBX_CFG_PM2: 
        case UBX_CFG_PMS: 
        case UBX_CFG_PWR:  
        case UBX_CFG_RINV:
        case UBX_CFG_RST:
        case UBX_CFG_RXM:
        case UBX_CFG_SBAS:
        case UBX_CFG_SENIF:
        case UBX_CFG_SLAS: 
        case UBX_CFG_SMGR: 
        case UBX_CFG_SPT: 
        case UBX_CFG_TMODE2:
        case UBX_CFG_TMODE3:
        case UBX_CFG_TP5: 
        case UBX_CFG_TXSLOT:
        case UBX_CFG_USB: 
        default:
            /** Not implemented yet */
            break;
    }

}

/**
 * @brief Parse UBX-NAV Class Packet
 * 
 * This is a super function which parses a returned NAV packet into
 * a parsed structure for the specified ID.
 * 
 * Warning: The correct struct MUST be passed in the void *passed pointer
 * for this to work.  This is a dangerous function which has no error checking.
 * 
 * @param id        UBX Class ID 
 * @param packet    UBX Packet to Parse
 * @param parsed    UBX Class ID Structure
 */
void UBX_parse_nav(UBX_NAV_ID_t id, ubx_packet_t *packet, void *parsed){

    switch(id)
    {
        case UBX_NAV_PVT      :
            module_ubx_parse_nav_pvt_packet(packet, (ubx_nav_pvt_t*) parsed);
            break;
        case UBX_NAV_SAT      :
        case UBX_NAV_AOPSTATUS:
        case UBX_NAV_ATT      :
        case UBX_NAV_CLOCK    :
        case UBX_NAV_COV      :
        case UBX_NAV_DGPS     :
        case UBX_NAV_DOP      :
        case UBX_NAV_EELL     :
        case UBX_NAV_EOE      :
        case UBX_NAV_GEOFENCE :
        case UBX_NAV_HPPOSECEF:
        case UBX_NAV_HPPOSLLH :
        case UBX_NAV_NMI      :
        case UBX_NAV_ODO      :
        case UBX_NAV_ORB      :
        case UBX_NAV_POSECEF  :
        case UBX_NAV_POSLLH   :
        case UBX_NAV_RELPOSNED:
        case UBX_NAV_RESETODO : 
        case UBX_NAV_SBAS     :
        case UBX_NAV_SLAS     :
        case UBX_NAV_SOL      :
        case UBX_NAV_STATUS   :
        case UBX_NAV_SVINFO   :
        case UBX_NAV_SVIN     :
        case UBX_NAV_TIMEBDS  :
        case UBX_NAV_TIMEGAL  :
        case UBX_NAV_TIMEGLO  :
        case UBX_NAV_TIMEGPS  :
        case UBX_NAV_TIMELS   :
        case UBX_NAV_TIMEUTC  :
        case UBX_NAV_VELECEF  :
        case UBX_NAV_VELNED   :
        default:
            /** Not implemented yet */
            break;
    }

}

/**
 * @brief Create a message (char array) from a UBX packet
 * 
 * This function creates a character array from a complete
 * UBX packet.  Used to convert a UBX packet into a message
 * to send via I2C.
 * 
 * @param packet Pointer to UBX Packet to create message from.
 * @param msg Formatted char array from UBX packet
 */
void UBX_create_msg_from_packet(ubx_packet_t *packet, uint8_t *msg)
{
    *msg++ = UBX_HEADER_1;
    *msg++ = UBX_HEADER_2;
    *msg++ = packet->cls;
    *msg++ = packet->id;
    *msg++ = (uint8_t) (packet->length >> 8) & 0xFF;
    *msg++ = (uint8_t) (packet->length & 0xFF);
    memcpy(msg, packet->payload, packet->length);
    msg += packet->len;
    *msg += packet->chksumA;
    *msg += packet->chksumB;
}

/**
 * @brief Create a UBX packet from a message (char array)
 * 
 * Used to create a UBX packet from a char array.  This function
 * is typically used to take a I2C read and covert to UBX packet.
 * 
 * @param msg Message (char array) to convert to UBX packet
 * @param len Length of char array
 * @param packet Pointer to packet of converted message
 * @return true Valid packet
 * @return false Invalid packet
 */
bool UBX_create_packet_from_msg(uint8_t *msg, uint16_t len, ubx_packet_t *packet)
{
    bool retVal = false;

    retVal = ( (msg[0] == UBX_HEADER_1) && (msg[1] == UBX_HEADER_2));

    if(len < UBX_HEADER_SIZE)
    {
        retVal = false;
    }

    if(retVal == true)
    {
        packet->cls = msg[2];
        packet->id = msg[3];
        packet->length = (uint16_t) (msg[5]<<8) | (msg[4]);

        if( (packet->length + UBX_HEADER_SIZE) > len )
        {
            /** Message doesn't capture full packet */
            retVal = false;
        } 
        else if(packet->length > 0)
        {
            /** Non-zero length message */
            packet->payload = &msg[6];
            packet->chksumA = *(&msg[6] + packet->length);
            packet->chksumB = *(&msg[6] + packet->length + 1);
        } else {
            /** Zero length message */
            packet->chksumA = msg[6];
            packet->chksumB = msg[7];
        }
    }

    return retVal;
}

/**
 * @brief Find the start of UBX packet in random char array
 * 
 * This function looks for the start of a UBX packet (0xB5 0x62) in a
 * uint8_t array.
 * 
 * @param buf Pointer to data buffer array
 * @param len Length of data buffer
 * @return uint8_t* Pointer to start of UBX message
 */
uint8_t* UBX_find_start_of_msg(uint8_t *buf, uint16_t len)
{
    uint8_t *pStart = NULL;

    for(uint16_t i; i< (len-1); i++)
    {
        if( (buf[i] == UBX_HEADER_1) && (buf[i+1] == UBX_HEADER_2) )
        {
            pStart = (uint8_t*) (buf + i);
            break;
        }        
    }

    return pStart;
}

//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************

/**
 * @brief Parse packet for ACK-ACK/NAK
 * 
 * Parses a packet to see if a valid ACK-ACK or ACK-NAK was sent.
 * 
 * @param packet Pointer to UBX Packet
 * @param expectedCls Expected Class of ACK
 * @param expectedId Expected Class ID of ACK
 * @return ubx_nak_t ACK-ACK, ACK-NAK, or Invalid
 */
static ubx_nak_t module_ubx_parse_ack_or_nak_msg(ubx_packet_t *packet, uint8_t expectedCls, uint8_t expectedId)
{
    ubx_nak_t retVal = UBX_ACK_INVALID;

    if(packet->cls == 0x05)
    {
        if( (packet->payload[0] == expectedCls) && (packet->payload[1] == expectedId))
        {
            if( packet->id == 0x01)
            {
                retVal = UBX_ACK_ACK;
            } else if(packet->id == 0x00)
            {
                retVal = UBX_ACK_NAK;
            }
        }
    }
}

/**
 * @brief Parse UBX-NAV-PVT Packet
 * 
 * Parse a UBX-NAV-PVT packet into a NAV-PVT struct
 * 
 * @param packet Pointer to UBX Packet
 * @param nav Pointer to UBX-NAV-PVT data struct
 */
static void module_ubx_parse_nav_pvt_packet(ubx_packet_t *packet, ubx_nav_pvt_t *nav)
{
    uint8_t *payload = packet->payload;

    nav->iTOW = bytes_to_u32(payload[0]);
    nav->year = bytes_to_u16(payload[4]);
    nav->month = payload[6];
    nav->day = payload[7];
    nav->hour = payload[8];
    nav->min = payload[9];
    nav->sec = payload[10];
    nav->valid = payload[11];
    nav->tAcc = bytes_to_u32(payload[12]);
    nav->nano = bytes_to_i32(payload[16]);
    nav->fixType = payload[20];
    nav->flags = payload[21];
    nav->flags2 = payload[22];
    nav->numSV = payload[23];
    nav->lon = bytes_to_i32(payload[24]);
    nav->lat = bytes_to_i32(payload[28]);
    nav->height = bytes_to_i32(payload[32]);
    nav->hMSL = bytes_to_i32(payload[36]);
    nav->hAcc = bytes_to_u32(payload[40]);
    nav->vAcc = bytes_to_u32(payload[44]);
    nav->velN = bytes_to_i32(payload[48]);
    nav->velE = bytes_to_i32(payload[52]);
    nav->velD = bytes_to_i32(payload[56]);
    nav->gSpeed = bytes_to_i32(payload[60]);
    nav->headMot = bytes_to_i32(payload[64]);
    nav->sAcc = bytes_to_u32(payload[68]);
    nav->headAcc = bytes_to_u32(payload[72]);
    nav->pDOP = bytes_to_u16(payload[76]);
    nav->flags3 = bytes_to_u16(payload[78]);
    nav->reserved1 = bytes_to_u32(payload[80]);
    nav->headVeh = bytes_to_i32(payload[84]);
    nav->magDec = bytes_to_i16(payload[88]);
    nav->magAcc = bytes_to_u16(payload[90]);
}

/**
 * @brief Parse UBX-CFG-PRT Packet
 * 
 * Parse a UBX-CFG-PRT Packet into a CFG-PRT struct
 * 
 * @param packet Pointer to UBX-CFG-PRT Packet
 * @param prt Pointer to CFG-PRT struct
 */
static void module_ubx_parse_cfg_prt_packet(ubx_packet_t *packet, ubx_cfg_prt_t *prt)
{
    uint8_t *payload = packet->payload;

    prt->portID = payload[0];
    prt->reserved1 = payload[1];
    prt-> txReady = bytes_to_u16(&payload[2]);
    prt-> mode = bytes_to_u32(&payload[4]);
    prt-> reserved2 = bytes_to_u32(&payload[8]);
    prt-> inProtoMask = bytes_to_u16(&payload[12]);
    prt-> outProtoMask = bytes_to_u16(&payload[14]);
    prt-> flags = bytes_to_u16(&payload[16]);
    prt-> reserved3 = bytes_to_u16(&payload[18]);

} 

/**
 * @brief Parse UBX-CFG-RATE Packet
 * 
 * Parse a UBX-CFG-RATE Packet into a CFG-RATE struct
 * 
 * @param packet Pointer to UBX-CFG-RATE Packet
 * @param rate Pointer to CFG-Rate struct
 */
static void module_ubx_parse_cfg_rate_packet(ubx_packet_t *packet, ubx_cfg_rate_t *rate)
{
    rate->measRate = bytes_to_u16(&packet->payload[0]);
    rate->navRate = bytes_to_u16(&packet->payload[2]);
    rate->timeRef = bytes_to_u16(&packet->payload[4]);
}

/**
 * @brief Increment UBX Checksum
 * 
 * Increment UBX Checksum with provided value
 * 
 * @param packet Pointer to UBX Packet 
 * @param value Value to add to checksum
 */
static void UBX_checksum_increment(ubx_packet_t *packet, uint8_t value)
{
    packet->chksumA += value;
    packet->chksumB += packet->chksumA;
}

/**
 * @brief UBX Checksum Calculator
 * 
 * Used to calculate UBX checksum when the rest of the packet is complete.
 * 
 * @param packet Pointer to packet to calculate checkum on.
 */
static void module_ublox_calc_ubx_checksum(ubx_packet_t *packet)
{
    packet->chksumA = 0;
    packet->chksumB = 0;
    
    
    UBX_checksum_increment(packet, packet->cls);
    UBX_checksum_increment(packet, packet->id);
    UBX_checksum_increment(packet, (uint8_t)(packet->length & 0xFF));
    UBX_checksum_increment(packet, (uint8_t)(packet->length >> 8));

    for(uint16_t i=0; i<packet->length; i++)
    {
        UBX_checksum_increment(packet, packet->payload[i]);
    }
}

/**
 * @brief Convert byte array to uint32_t
 * 
 * @param data Pointer to data array
 * @return uint32_t 32-bit unsigned int value
 */
static uint32_t bytes_to_u32(uint8_t *data)
{
    uint32_t value;
    for(uint8_t i=0;i<4;i++)
    {
        value[i] = data[4-i-1] << (i*8);
    }

    return value;
}

/**
 * @brief Convert byte array to int32_t
 * 
 * @param data Pointer to data array
 * @return int32_t 32-bit signed int value
 */
static int32_t bytes_to_i32(uint8_t *data)
{
    u32_to_i32_t val;
    val.u32 = bytes_to_u32(data);
    return val.i32;
}

/**
 * @brief Convert byte array to uint16_t
 * 
 * @param data Pointer to data array
 * @return uint16_t 16-bit unsigned int value
 */
static uint16_t bytes_to_u16(uint8_t *data)
{
    uint16_t value;
    return (data[1]<<8 | data[0]);
}

/**
 * @brief Convert byte array to uint16_t
 * 
 * @param data Pointer to data array
 * @return int16_t 16-bit signed int value
 */
static int16_t bytes_to_i16(uint8_t *data)
{
    u16_to_i16_t val;
    val.u16 = bytes_to_u16(data);
    return val.i16;
}

/**
 * @brief Reverse memcpy.  
 * 
 * Reverse memcpy.  Used for little-endian data transfer.
 * 
 * @param dst Pointer to destination array
 * @param src Poniter to source array
 * @param n Length of memory to copy in reverse
 */
static void rmemcpy(uint8_t *dst, void *src, size_t n)
{
    char *d = dst;
    const char *s = src;
    for(size_t i=0; i<n; i++)
    {
        d[i] = s[n -1 -i];
    }
}