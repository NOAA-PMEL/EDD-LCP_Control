/**
 * @file ublox.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-09-28
 * 
 * 
 */
#ifndef UBLOX_H
#define UBLOX_H

/************************************************************************
*						STANDARD LIBRARIES
************************************************************************/
#include <stdint.h>

/************************************************************************
*							HEADER FILES
************************************************************************/
#include "ublox_ubx.h"

/************************************************************************
*							MACROS
************************************************************************/
#define UBLOX_I2C_ADDR                  ( 0x42 )

/************************************************************************
*							ENUM & STRUCTS
************************************************************************/

/**
 * @brief UBLOX Com Port Types
 * 
 */
typedef enum eUBLOX_PortType_t{
    UBLOX_COM_I2C   = 0,    /**< I2C Port */
    UBLOX_COM_UART  = 1,    /**< UART Port */
    UBLOX_COM_USB   = 3,    /**< USB Port */
    UBLOX_COM_SPI   = 4,    /**< SPI Port */
}UBLOX_PortType_t;

/**
 * @brief UBLOX Com Message Types
 * 
 */
typedef enum eUBLOX_MsgType_t {
    UBLOX_MSG_UBX = 0x01,       /**< UBX Message */
    UBLOX_MSG_NMEA = 0x02,      /**< NMEA Messages */
    UBLOX_MSG_RTCM2 = 0x04,     /**< RTCM2 Messages */
    UBLOX_MSG_RTCM3 = 0x10      /**< RTCM3 Messages */
}UBLOX_MsgType_t;

/**
 * @brief UBLOX Navigation Data Structure
 * 
 */
typedef struct sUBLOX_Nav_t {
    bool fix;           /**< GPS Fix valid */
    struct{
        float lat;      /**< Latitude (deg)*/
        float lon;      /**< Longitude (deg) */
        float alt;      /**< Altitude (mm) */
    }position;
    struct {
        float hAcc;     /**< Horizontal Accuracy (mm) */
        float vAcc;     /**< Veritcal Accuracy */
        uint16_t pDOP;  /**< Position Dilution of Precision */
    }accuracy;
    struct{
        uint16_t year;  /**< Calendar Year UTC */
        uint8_t month;  /**< Calendar Month UTC */
        uint8_t day;    /**< Calendar Day UTC */
        uint8_t hour;   /**< Hour UTC */    
        uint8_t min;    /**< Minute UTC */
        uint8_t sec;    /**< Second UTC */
    }time;
}UBLOX_Nav_t;



/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/
bool UBLOX_initialize(  UBLOX_PortType_t port, 
                        UBLOX_MsgType_t inMsg, 
                        UBLOX_MsgType_t outMsg,
                        uint16_t rateHz);

bool UBLOX_cfg_prt_for_i2c(uint16_t inConfig, uint16_t outConfig);

bool UBLOX_cfg_port_msg(uint8_t class, uint8_t id, uint8_t rate);

//bool UBLOX_read_ubx(
//                    uint8_t class, 
//                    uint8_t id, 
//                    uint8_t *payload, 
//                    uint16_t delay, 
//                    ubx_packet_t *packet
//                    );

bool UBLOX_read_nav(UBLOX_Nav_t *data);
bool UBLOX_read_config(ubx_cfg_prt_t *prt);


#endif // UBLOX_H