/** @file S9_temp.c
 *  @brief SoundNine OEM Temperature Sensor
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date September 30, 2020
 *  @version 0.0.1
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */
#include "S9_temp.h"






void S9T_init(void)
{

}


void S9T_ON(void)
{

}


void S9T_OFF(void)
{

}


float S9T_Read_T(void)
{

}


float S9T_Read_R(void)
{

}

float S9T_Read(float *t, float *r)
{

}

/** @brief Parse S9 Temperature response
 * 
 * The S9 Temperature sensor returns a data string 
 * in the format:
 * RRRR.RRRR, TT.TTTT\r
 * 
 * Where RRRR.RRRR is the thermistor resistance in Ohms
 * and TT.TTTT is the calculated temperature in degC
 * 
 * @param *data Pointer to data string
 * @param len length of string
 * @param *p Pointer to S9 Temperature structure
 */
STATIC void _parse_msg(char *data, uint8_t len, sS9_t *p)
{
    uint8_t comma, end;
    uint8_t i;

    for(i=0;i<len;i++)
    {
        if(data[i] == ',')
        {
            comma = i;
        }
        else if(data[i] == '\r')
        {
            end = i;
        }
    }

    if(end <= comma)
    {
        p->temperature = NAN;
        p->resistance = NAN;
        return;
    }

    /* Copy Resistance */
    char temp[32];
    strncpy(temp, &data[0],comma);
    p->resistance = atof(temp);

    /* Copy Temperature */
    strncpy(temp, &data[comma+1], end-comma);
    p->temperature = atof(temp);

}


/** @brief Parse version info
 * 
 * When the S9 Temp is sent the "ver" command, 
 * the following response is sent:
 * S9>ver
 * MID=T003
 * C0=0.000855
 * C1=0.000293
 * C2=0.000000
 * C3=0.000000
 * R0=10000.000
 * UID=000000000F0F1A08535722E74FBC90B1
 * S9T0 V0.45
 * OK
 * 
 * This function parses for each individual structure
 * variable and returns.
 * 
 * @param *data Pointer to response string
 * @param *p Pointer to S9 temperature structure
 * 
 */
STATIC void _parse_version(char *data, sS9_t *p )
{
    uint8_t i;
    char temp[255];
    float temp_f; 

    strcpy(temp,data);
    // printf("%s\n", temp);
    char *tok;

    /** @todo strtok needs RTOS case!!! */
    /* Find MID */
    tok = strtok(temp,"=");
    tok = strtok(NULL, "\r");
    strcpy(p->info.MID, tok);

    /* Find C0 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    temp_f = atof(tok);
    p->info.C0 = temp_f;

    /* Find C1 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.C1 = atof(tok);

    /* Find C2 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.C2 = atof(tok);

    /* Find C3 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.C3 = atof(tok);

    /* Find R0 */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    p->info.R0 = atof(tok);

    /* Find UID */
    tok = strtok(NULL, "=");
    tok = strtok(NULL, "\r");
    
    uint8_t len = strlen(tok);
    uint8_t sub_val = 0;
    uint8_t temp_hex[32];
    memset(temp,0,32);

    for(i=0;i<len;i++)
    {
        if( (tok[i] >= '0') && (tok[i] <= '9'))
        {
            temp_hex[i] = tok[i] - '0';
        } else if ( (tok[i] >= 'A') && (tok[i] <= 'F'))
        {
            temp_hex[i] = tok[i] -'A' + 10;
        } else {
            /** @todo - Error condition */
        }
    }
    i=0;
    uint8_t cnt =0;
    while(i < 32)
    {
        p->info.UID[cnt] = temp_hex[i++] << 4;
        p->info.UID[cnt] |= temp_hex[i++];
        cnt++;
    }
    
    /* Find Sensor */
    tok = strtok(NULL, " ");
    strcpy(p->info.sensor, tok);

    /** Find Firmware Major Version */
    tok = strtok(NULL, ".");
    p->info.firmware.major = (uint8_t) atoi(tok);

    /** Find Firmware Minor Version */
    tok = strtok(NULL, "\r");
    p->info.firmware.minor = (uint8_t) atoi(tok);

    /** Find Status */
    tok = strtok(NULL, "\r");
    strcpy(p->info.status, tok);
}