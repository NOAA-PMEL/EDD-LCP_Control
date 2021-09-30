

/**
 * @file ublox.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief UBLOX GPS Commands.
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
#include "ublox.h"
#include "ublox_ubx.h"
#include "artemis_ublox_i2c.h"


//*****************************************************************************
//
//  Macros & Constants
//
//*****************************************************************************
#define UBX_HEADER_1    ( 0xB5 )
#define UBX_HEADER_2    ( 0x62 )

static uint8_t ubx_msg[ UBX_MSG_SIZE_MAX ];


//*****************************************************************************
//
// Structs, Enums & Typedefs
//
//*****************************************************************************


//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
bool module_ublox_i2c_initialize(void);
bool module_ublox_cfg_port_for_i2c(uint16_t inConfig, uint16_t outConfig);
bool module_cfg_port_msg(uint8_t cls, uint8_t id, uint8_t rate);
bool module_ublox_read(eUBX_Class_t cls, uint8_t id, void *parsed);
bool module_ublox_read_ubx(
                    eUBX_Class_t cls, 
                    uint8_t id, 
                    uint8_t *payload,
                    uint16_t length, 
                    uint16_t delay, 
                    ubx_packet_t *packet
                    );
void module_ublox_create_msg_from_packet(ubx_packet_t *packet, uint8_t *msg);
bool module_ublox_wait_for_ack_or_nak(  uint8_t expectedClass,
                                        uint8_t expectedId,
                                        uint8_t initial_delay_ms, 
                                        uint8_t attempts  );


//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
/**
 * @brief UBLOX Module Initialize
 * 
 * @param port Port type (I2C, SPI, UART, USB)
 * @param inMsg UBX Messages to accept on input
 * @param outMsg UBX Messages to send as output
 * @param rateHz Rate of field updates
 * @return true Initialization is valid
 * @return false Initialization is invalid
 */
bool UBLOX_initialize(  UBLOX_PortType_t port, 
                        UBLOX_MsgType_t inMsg, 
                        UBLOX_MsgType_t outMsg,
                        uint16_t rateHz)
{

    if(rateHz > 40)
    {
        rateHz = 40;
    }

    uint16_t rate = 1000 / (rateHz * 4);

    if(port == UBLOX_COM_I2C)
    {
        /** Initialize the I2C Module */
        module_ublox_i2c_initialize();

        /** Set port to I2C mode */
        module_ublox_cfg_port_for_i2c((uint8_t) inMsg, (uint8_t) outMsg);

        /** Set the rate */
        module_cfg_port_msg(UBX_NAV_CLASS, UBX_NAV_PVT, rate);
    }
}

/**
 * @brief Read NAV Data
 * 
 * Read the most recent data from the UBLOX GPS
 * 
 * @param data Pointer to NAV Data struct 
 * @return true Valid Data (GPS Fix)
 * @return false Invalid data (No GPS Fix)
 */
bool UBLOX_read_nav(UBLOX_Nav_t *data)
{
    bool retVal = false;
    bool ubloxFix = false;

    ubx_nav_pvt_t nav = {0};

    retVal = module_ublox_read(UBX_NAV_CLASS, UBX_NAV_PVT, &nav);

    if(retVal)
    {
        if(nav.fixType > 2)
        {
            retVal = true;
            
            data->fix = true;
            
            data->position.lat = nav.lat * 1e-7;
            data->position.lon = nav.lon * 1e-7;
            data->position.alt = nav.hMSL;
            
            data->accuracy.hAcc = nav.hAcc;
            data->accuracy.vAcc = nav.vAcc;
            data->accuracy.pDOP = nav.pDOP * 0.01;

            data->time.year = nav.year;
            data->time.month = nav.month;
            data->time.day = nav.day;
            data->time.hour = nav.hour;
            data->time.min = nav.min;
            data->time.sec = nav.sec;
        } else {
            data->fix = false;
            retVal = false;
        }
    }

    return retVal;
}

/**
 * @brief Read CFG-PRT Data
 * 
 * @param prt Pointer to CFG-PRT Data struct
 * @return true Valid read
 * @return false Invalid read
 */
bool UBLOX_read_config(ubx_cfg_prt_t *prt)
{
    bool retVal = false;
    uint8_t payload[20];
    ubx_packet_t packet = {0};

    /** Read UBX */
    retVal = module_ublox_read_ubx(UBX_CFG_CLASS, prt->portID, payload, 20, 0, &packet);


    if(retVal)
    {
        retVal = UBLOX_parse_cfg_prt_packet(&packet, prt);
    }
    return retVal;
}


//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************
/**
 * @brief I2C Initialization
 * 
 * @return true Valid init 
 * @return false Invalid init
 */
bool module_ublox_i2c_initialize(void)
{
    artemis_ublox_i2c_initialize(UBLOX_I2C_ADDR);
    artemis_ublox_i2c_power_on();
}

/**
 * @brief Send UBX Packet to GPS over I2C
 * 
 * @param packet Pointer to UBX Packet
 * @return true Valid send
 * @return false Invalid send
 */
bool module_ublox_send_packet(ubx_packet_t *packet)
{
    UBX_create_msg_from_packet(packet, ubx_msg);
    artemis_ublox_i2c_send_msg(ubx_msg, packet->length + UBX_HEADER_SIZE, false);
}

/**
 * @brief Read UBX Packet from GPS over I2C
 * 
 * @param packet Pointer to UBX Packet
 * @return true Valid Read
 * @return false Invalid Read
 */
bool module_ublox_read_packet(ubx_packet_t *packet)
{
    bool retVal = false;

    /** Read the Data */
    uint16_t msgLen;
    msgLen = artemis_ublox_i2c_read_data(ubx_msg);

    /** Find the start of the UBX message */
    int16_t offset = UBX_find_start_of_msg(ubx_msg, msgLen);
    
    if(offset >= 0)
    {
        /** Error */
      retVal = true;
    }
    
    if(retVal)
    {
        /** Parse the message */
        uint16_t len = msgLen - offset;
        if(len > 0)
        {
          uint8_t *pStart = &ubx_msg[0];
          pStart += offset;
          
            retVal = UBX_parse_ubx_packet(pStart, len, packet);
        }

        return retVal;
    }
}

/**
 * @brief Configure GPS for I2C 
 * 
 * @param inConfig Input message type 
 * @param outConfig Output message type
 * @return true GPS configured for I2C
 * @return false GPS NOT configure for I2C
 */
bool module_ublox_cfg_port_for_i2c(uint16_t inConfig, uint16_t outConfig)
{
    bool retVal = false;
    ubx_packet_t txPacket = {0};
    ubx_packet_t rxPacket = {0};
    uint8_t payload[20] = {0};
    
    /** Set up the payload */
    payload[4] = UBLOX_I2C_ADDR << 1;
    payload[12] = inConfig;
    payload[14] = outConfig;

    /** Create the UBX Packet */
    UBX_create_ubx_packet(UBX_CFG_CLASS, UBX_CFG_PRT, payload, 20, &txPacket);
    
    /** Send packet */
    retVal = module_ublox_send_packet(&txPacket);
    
    /** Query for ACK-ACK or ACK-NAK */
    if(retVal)
    {
        retVal = module_ublox_wait_for_ack_or_nak( UBX_CFG_CLASS, UBX_CFG_PRT, 500, 50);
        
    }

    return retVal;
}

/**
 * @brief Configure Port Messages
 * 
 * @param cls Pointer to UBX Class
 * @param id  Pointer to UBX Class ID
 * @param rate Message rate (ms)
 * @return true Port Message set
 * @return false Port Message NOT set
 */
bool module_cfg_port_msg(uint8_t cls, uint8_t id, uint8_t rate)
{
    bool retVal = false;
    ubx_packet_t txPacket = {0};
    ubx_packet_t rxPacket = {0};
    uint8_t payload[3] = {cls, id, rate};

    /** Create the UBX Packet */
    UBX_create_ubx_packet(UBX_CFG_CLASS, UBX_CFG_MSG, payload, 3, &txPacket);
    
    /** Send packet */
    retVal = module_ublox_send_packet(&txPacket);
    
    /** Query for ACK-ACK or ACK-NAK */
    if(retVal)
    {
        retVal = module_ublox_wait_for_ack_or_nak( UBX_CFG_CLASS, UBX_CFG_PRT, 500, 50);      
    }

//    /** Read buffer & parse for UBX packet */
//    if(retVal)
//    {
//        retVal = module_ublox_read_packet(&rxPacket);
//    }
//
//    /** Check for ACK-ACK or ACK-NAK */
//    if(retVal)
//    {
//        retVal = UBX_check_for_ack(&rxPacket, UBX_CFG_CLASS, UBX_CFG_PRT);
//    }


    return retVal;
}

/**
 * @brief Read UBLOX
 * 
 * Super function to read UBLOX class and id based on specified.
 * 
 * Warning: This function requires the correct data struct to be passed
 * in the void *parsed pointer.  There is no type checking to validate the
 * correct struct was passed, which can have irreversible consequences.
 * 
 * @param cls Class to read
 * @param id Class ID to read
 * @param parsed Poniter to Class ID Data struct
 * @return true Valid Read
 * @return false Invalid Read
 */
bool module_ublox_read(eUBX_Class_t cls, uint8_t id, void *parsed)
{
    bool retVal = false;
    uint16_t delay = 0;
    ubx_packet_t packet = {0};
    
    if(cls == UBX_NAV_CLASS)
    {
      delay = 500;
    }

    /** Read the UBX Packet */
    retVal = module_ublox_read_ubx((uint8_t)cls, id, NULL, 0, delay, &packet);

    if(retVal)
    {
        /** Parse the correct packet type*/
        switch(cls)
        {
            // case UBX_ACK_CLASS:
            //     eUBX_ACK_ID_t id = (eUBX_ACK_ID_t)*pId;
                // UBX_parse_ack_or_nak_msg(&packet, )
            break;
            case UBX_AID_CLASS:

            break;
            case UBX_CFG_CLASS:
                UBX_parse_cfg((UBX_CFG_ID_t) id, &packet, parsed);
            break;
            case UBX_ESF_CLASS:
        
            break;
            case UBX_HNR_CLASS:
        
            break;
            case UBX_INF_CLASS:
        
            break;
            case UBX_LOG_CLASS:
        
            break;
            case UBX_MGA_CLASS:
        
            break;
            case UBX_MON_CLASS:
        
            break;
            case UBX_NAV_CLASS:
                UBX_parse_nav((UBX_NAV_ID_t) id, &packet, parsed);
            break;
            case UBX_RXM_CLASS:
        
            break;
            case UBX_UPD_CLASS:
        
            break;
        }
    }
    return retVal;
}

/**
 * @brief Read UBX-CFG-MSG
 * 
 * Read the UBX-CFG-MSG.  Doesn't really do much at the moment.
 * 
 * @param shortMsg 
 * @param id 
 * @return true 
 * @return false 
 */
bool module_read_cfg_msg(bool shortMsg, uint8_t class, uint8_t id)
{
    char payload[2];
    ubx_packet_t packet = {0};
    bool retVal;

    if(!shortMsg)
    {
        payload[0] = class;
        payload[1] = id;
    }

    retVal = module_ublox_read_ubx(UBX_CFG_CLASS, UBX_CFG_MSG, payload, 2, 0, &packet);

    return retVal;
}

/**
 * @brief Read UBX-CFG-RATE
 * 
 * Read the UBX-CFG-RATE.  Doesn't really do much at the moment
 * 
 * @param rate 
 * @return true 
 * @return false 
 */
bool module_read_cfg_rate(ubx_cfg_rate_t *rate)
{
    char payload[20];
    ubx_packet_t packet = {0};
    bool retVal = false;

    retVal = module_ublox_read_ubx(UBX_CFG_CLASS, UBX_CFG_RATE, payload, 20, 0, &packet);
    if(retVal)
    {
        retVal = UBLOX_parse_cfg_rate_packet(&packet, rate);
    }

    return retVal;
}

/**
 * @brief Read the UBLOX UBX Packet over I2C
 * 
 * Reads the UBX packet from the specified Class & ID
 * 
 * @param cls UBX Class to read
 * @param id UBX Class ID to read
 * @param payload Pointer to payload to send
 * @param length Length of payload
 * @param delay Delay between send & receive (ms)
 * @param packet Pointer to data packet (return values)
 * @return true Valid read
 * @return false Invalid read
 */
bool module_ublox_read_ubx(
                    eUBX_Class_t cls, 
                    uint8_t id, 
                    uint8_t *payload,
                    uint16_t length, 
                    uint16_t delay, 
                    ubx_packet_t *packet
                    )
{
    bool retVal = false;
    ubx_packet_t txPacket = {0};
    int16_t offset;
    
    /** Create Packet */
    UBX_create_ubx_packet((uint8_t) cls, id, payload, length, &txPacket);
    UBX_create_msg_from_packet(&txPacket, ubx_msg);

    /** Send Packet */
//    artemis_ublox_send_packet(ubx_msg, txPacket.length, false);
    module_ublox_send_packet(&txPacket);
    
    /** Sleep if there is a delay */
    while(delay-- > 0)
    {
        am_hal_systick_delay_us(1000);
    }

    /** Read Buffer */
    uint16_t len = artemis_ublox_i2c_read_data(ubx_msg);
    if(len > 0)
    {
        /** Find start of UBX Packet */
        
        offset = UBX_find_start_of_msg(ubx_msg, len);
        if(offset >= 0)
        {
            len -= offset;
            retVal = true;
        } else {
          retVal = false;
        }
    }
    
    if(retVal)
    {   
      uint8_t *pStart = &ubx_msg[0];
      pStart += offset;
        /** Create return Packet */
        retVal = UBX_create_packet_from_msg(pStart , len, packet);
    }
}



bool module_ublox_wait_for_ack_or_nak(  uint8_t expectedClass,
                                        uint8_t expectedId,
                                        uint8_t initial_delay_ms, 
                                        uint8_t attempts  )
{
    bool retVal = false;
    
    ubx_packet_t rxPacket = {0};
    
    /** Delay */
    am_hal_systick_delay_us(initial_delay_ms*1000);
  
    /** Read buffer & parse for UBX packet */
    uint8_t cnt = 0;
    bool endFlag = false;
    while(endFlag == false)
    {
        
        retVal = module_ublox_read_packet(&rxPacket);
        if( (retVal == true) || (++cnt > attempts))
        {
            endFlag = true;
        }
        am_hal_systick_delay_us(10000);
    }

    /** Check for ACK-ACK or ACK-NAK */
    if(retVal)
    {
        ubx_ack_t valid = UBX_ACK_INVALID_RESPONSE;
        valid = UBX_check_for_ack(&rxPacket, expectedClass, expectedId);
        switch(valid)
        {
            case UBX_ACK_ACK_RESPONSE:
                retVal = true;
                break;
            case UBX_ACK_NAK_RESPONSE:
                retVal = false;
                break;
            case UBX_ACK_INVALID_RESPONSE:
            default:
              retVal = false;
              /** Log Error */
              break;
        }
    }

    return retVal;
}
