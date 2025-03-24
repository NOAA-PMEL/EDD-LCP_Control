/** @file buffer_c.h
 *  @brief
 *
 *  @author Matt Casari, matthew.casari@noaa.org
 *  @date Dec 4, 2017
 *  @version
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */



#ifndef _BUFFER_C_H_
#define _BUFFER_C_H_

/** Remove STATIC and PERSISTENT values if running TEST */
/** Add the actual values if running release */
#ifdef TEST
#ifndef STATIC
#define STATIC  
#endif
#ifndef PERSISTENT
#define PERSISTENT
#endif
#else
#ifndef STATIC
#define STATIC  static
#endif
#ifndef PERSISTENT
#define PERSISTENT __persistent 
#endif
#endif
/************************************************************************
*							HEADER FILES
************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/************************************************************************
*						STANDARD LIBRARIES
************************************************************************/

/************************************************************************
*							MACROS
************************************************************************/

#define BUFFER_C_SIZE				(256)
#define ACTUAL_BUFFER_C_SIZE		(BUFFER_C_SIZE + 1)



typedef enum {
    BUFFER_C_OK 		= 0x00u,	/**< Buffer is OK */
    BUFFER_C_ERROR 		= 0x01u,	/**< Buffer Failed */
    BUFFER_C_FULL 		= 0x02u,	/**< Buffer is Full */
    BUFFER_C_EMPTY 		= 0x04u,	/**< Buffer is Empty */
    BUFFER_C_STRING_TOO_LONG = 0x08u,	/**< String is too long to use*/
    BUFFER_C_NO_STRING = 0x10u,			/**< No String to return */

}eBufferU32Status_t;

typedef struct _CircularBufferU32_t
{
    char buffer[ACTUAL_BUFFER_C_SIZE];
    uint16_t size;
    uint16_t read;
    uint16_t write;
    bool EndlineFlag;
}sCircularBufferU32_t;


/************************************************************************
*					GLOBAL FUNCTION PROTOTYPES
************************************************************************/
/** @brief Clear the buffer
 *
 *	Clear the char buffer
 *
 *  @param *buf pointer to the char buffer
 *
 *  @return None
 */
void BufferU32_Clear(sCircularBufferU32_t *buf);

uint8_t BufferU32_Get_Size(sCircularBufferU32_t *buf);

/** @brief Add char to buffer
 *
 *	Adds a single char to the buffer
 *
 *  @param *buf pointer to the char buffer
 *  @param value char value to add
 *
 *  @return result
 */
 eBufferU32Status_t BufferU32_put(sCircularBufferU32_t *buf, uint32_t value);

/** @brief Get char from buffer
 *
 *	Retreives one char from the buffer
 *
 *  @param *buf pointer to the char buffer
 *  @param *value pointer to the char variable
 *
 *  @return result
 */
 eBufferU32Status_t BufferU32_get(sCircularBufferU32_t *buf, uint32_t *value);





#ifdef TEST
extern uint16_t BufferU32_Scan(sCircularBufferU32_t *buf,char val);
extern void BufferU32_Size(sCircularBufferU32_t *buf);
extern uint16_t BufferU32_NextIndex(uint16_t idx);
extern uint16_t BufferU32_PrevIndex(uint16_t idx);
#endif

#endif

