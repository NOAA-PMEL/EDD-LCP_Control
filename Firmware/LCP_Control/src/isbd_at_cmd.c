/**
 * @file isbd_at_cmd.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief
 * @version 0.1
 * @date 2021-09-30
 *
 */
#include "isbd_at_cmd.h"

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <string.h>
#include <stdio.h>

//*****************************************************************************
//
// Project Files
//
//*****************************************************************************

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************
static uint16_t module_isbd_at_calculate_crc(isbd_at_packet_t *packet);
static bool module_isbd_validate_crc(isbd_at_packet_t *packet);

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
/**
 * @brief Create a SBD Packet
 *
 * @param cmd SBD Packet Type
 * @param len Length of data
 * @param data Pointer to data
 * @param packet AT Packet
 * @return true Valid message
 * @return false Invalid message
 */
bool ISBD_AT_create_packet( isbd_at_cmd_t cmd,
                            uint16_t len,
                            uint8_t *data,
                            isbd_at_packet_t *packet
                            )
{
    bool retVal = false;

    if(len <= ISBD_AT_MSG_MAX_LENGTH )
    {
        retVal = true;
    }

    if(retVal)
    {
        packet->cmd.ecmd = cmd;
        switch(packet->cmd.ecmd)
        {
            case ISBD_AT_CMD_AT:
                strncpy((char*)packet->cmd.msg, "AT",2);
                packet->cmd.expected_cr = 3;
                break;
            case ISBD_AT_CMD_ATK0:
                strncpy((char*)packet->cmd.msg, "AT&K0",5);
                packet->cmd.expected_cr = 3;
                break;
            case ISBD_AT_CMD_SBDWB:
                sprintf((char*)packet->cmd.msg, "AT+SBDWB=%u\r", packet->len);
                packet->cmd.expected_cr = 3;
                break;
            case ISBD_AT_CMD_SBDWT:
                sprintf((char*)packet->cmd.msg, "AT+SBDWT=%u\r", packet->len);
                packet->cmd.expected_cr = 3;
                break;
            case ISBD_AT_CMD_SBDIX:
                strncpy((char*)packet->cmd.msg, "AT+SBDIX",8);
                packet->cmd.expected_cr = 5;
                break;
            case ISBD_AT_CMD_SBDRT:
                strncpy((char*)packet->cmd.msg, "AT+SBDRT",8);
                packet->cmd.expected_cr = 4;
                break;
            case ISBD_AT_CMD_SBDRB:
                strncpy((char*)packet->cmd.msg, "AT+SBDRB",8);
                packet->cmd.expected_cr = 4;
                break;
            case ISBD_AT_CMD_NONE:
            default:
                strcpy((char*)packet->cmd.msg, "");
                packet->cmd.expected_cr = 0;
                break;
        }
        if(len > 0)
        {
            packet->data = data;
            packet->len = len;
        } else {
            packet->data = NULL;
            len = 0;
        }
        module_isbd_at_calculate_crc(packet);
    }

    return retVal;
}

bool ISBD_AT_send_packet( isbd_at_packet_t *packet)
{

    return false;
}

/**
 * @brief Calculate the CRC for the SBD packet
 *
 * @param packet Packet to send
 * @return uint16_t CRC value
 */
static uint16_t module_isbd_at_calculate_crc(isbd_at_packet_t *packet)
{
    packet->checksumA = 0;
    packet->checksumB = 0;

    uint16_t checksum = 0;

    for(uint16_t i=0; i<packet->len; i++)
    {
        checksum += packet->data[i];
    }
    packet->checksumA = (uint8_t) (checksum>>8);
    packet->checksumB = (uint8_t) (checksum && 0xFF);

    return checksum;
}

/**
 * @brief Validate CRC sent
 *
 * @param packet SBD Packet
 * @return true CRC is Valid
 * @return false CRC is invalid
 */
static bool module_isbd_validate_crc(isbd_at_packet_t *packet)
{
    uint16_t checksum_from_msg = (uint16_t) (packet->checksumA << 8) | packet->checksumB;
    uint16_t checksum_calculated = module_isbd_at_calculate_crc(packet);

    return (checksum_from_msg == checksum_calculated);
}

