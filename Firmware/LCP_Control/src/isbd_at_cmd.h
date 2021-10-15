/**
 * @file isb_at_cmd.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief Iridium SBD AT Commands
 * @version 0.1
 * @date 2021-09-30
 * 
 */
#ifndef ISBD_AT_CMD_H
#define ISBD_AT_CMD_H
/************************************************************************
*						STANDARD LIBRARIES
************************************************************************/
#include <stdint.h>
#include <stdbool.h>
/************************************************************************
*							HEADER FILES
************************************************************************/


/************************************************************************
*							MACROS
************************************************************************/

#define ISBD_AT_MSG_MAX_LENGTH      ( 270 )
#define ISBD_AT_CHECKSUM_LENGTH     ( 2 )
/************************************************************************
*							ENUM & STRUCTS
************************************************************************/
/**
 * @brief AT Errors
 * 
 */
typedef enum eISBD_AT_Error_t {
    ISBD_AT_ERROR_NONE,
    ISBD_AT_ERROR_INVALID_MSG,
    ISBD_AT_ERROR_INVALID_CRC
}ISBD_AT_Error_t;

/**
 * @brief SBD Command Types
 * 
 */
typedef enum e_isbd_at_cmd_t{
    ISBD_AT_CMD_NONE,       /**< Placeholder */
    ISBD_AT_CMD_AT,         /**< Check alive */
    ISBD_AT_CMD_ATK0,       /**< Turn off flow control */
    ISBD_AT_CMD_CSQ,        /**< Check connection strength */
    ISBD_AT_CMD_SBDWB,      /**< Create a binary SBD message */
    ISBD_AT_CMD_SBDWT,      /**< Create a text SBD message */
    ISBD_AT_CMD_SBDIX,      /**< Initiate SBD session */
    ISBD_AT_CMD_SBDRT,      /**< Download text message */
    ISBD_AT_CMD_SBDRB       /**< Download binary message */
}isbd_at_cmd_t;

/**
 * @brief AT Packet Stucts
 * 
 */
typedef struct s_isbd_at_packet_t
{  
    struct {
        isbd_at_cmd_t ecmd;     /**< SBD Command type */
        uint8_t msg[20];        /**< ASCII command */
        uint8_t expected_cr;    /**< Number of carriage returns expected from cmd*/
    }cmd;
    uint16_t len;           /**< Length of data */
    uint8_t *data;          /**< Pointer to data buffer */
    uint8_t checksumA;      /**< First digit of checksum */
    uint8_t checksumB;      /**< second digit of checksum */
}isbd_at_packet_t;

/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/
bool ISBD_AT_create_packet( isbd_at_cmd_t cmd,
                            uint16_t len,
                            uint8_t *data,
                            isbd_at_packet_t *packet
                            );
bool ISBD_AT_send_packet( isbd_at_packet_t *packet);



#endif