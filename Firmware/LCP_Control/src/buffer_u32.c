/** @file BufferU32.c
 *  @brief Character circular buffer library
 *
 *  @author Matt Casari, matthew.casari@noaa.gov
 *  @date Dec 4, 2015
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

#include "buffer_u32.h"

/*******************************************************************************
*					STATIC FUNCTIONS PROTOTYPES
*******************************************************************************/
STATIC uint16_t BufferU32_Scan(sCircularBufferU32_t *buf,uint32_t val);
STATIC void BufferU32_Size(sCircularBufferU32_t *buf);
STATIC uint16_t BufferU32_NextIndex(uint16_t idx);
STATIC uint16_t BufferU32_PrevIndex(uint16_t idx);
/*******************************************************************************
*					GLOBAL FUNCTIONS
*******************************************************************************/
void BufferU32_Clear(sCircularBufferU32_t *buf)
{
        buf->EndlineFlag = false;
	buf->read = 0;
	buf->write = 0;
	memset(buf,0,ACTUAL_BUFFER_C_SIZE);


	return;
}
uint8_t BufferU32_Get_Size(sCircularBufferU32_t *buf)
{
  return buf->size;
}

eBufferU32Status_t BufferU32_put(sCircularBufferU32_t *buf, uint32_t val){
	eBufferU32Status_t result = BUFFER_C_ERROR;

	/** Determine the index to write to */
	uint16_t NextWrite = BufferU32_NextIndex(buf->write);

	/** If the next index is equal to the read index, the buffer is full */
	if(NextWrite == buf->read){
		result = BUFFER_C_FULL;
	} else {
		/** Write the value to the buffer */
		buf->buffer[buf->write] = val;
		/** Update the index to the next value */
		buf->write = NextWrite;
		/** Update the buffer size */
		BufferU32_Size(buf);
		/** Update the result */
		result = BUFFER_C_OK;
	}
        if(val == '\n')
        {
         buf->EndlineFlag = true;
        }
	return result;
}

eBufferU32Status_t BufferU32_get(sCircularBufferU32_t *buf, uint32_t *value){
	eBufferU32Status_t result = BUFFER_C_ERROR;

	/** If buffer read index is not == to write index, then there is data */
	/** otherwise it is empty */
	if(buf->read != buf->write){
		/** Read the buffer Value */
		*value = buf->buffer[buf->read];
		/** Clear the current buffer value */
		buf->buffer[buf->read] = 0;
		/** Update the Index */
		buf->read = BufferU32_NextIndex(buf->read);
		/** Update the buffer size */
		BufferU32_Size(buf);
		/** Update the result */
		result = BUFFER_C_OK;

	} else {
		result = BUFFER_C_EMPTY;
	}
	return result;
}






/*******************************************************************************
*					STATIC FUNCTIONS
*******************************************************************************/
/** @brief Scan buffer for value
 *
 * Scans the buffer for a specified value, returns the number of characters from
 * start of string through location of specified value.
 *
 * @param *buf Pointer to the Circular Buffer
 * @param val Value to scan for
 *
 * @return len Length of string
 */
STATIC uint16_t BufferU32_Scan(sCircularBufferU32_t *buf,uint32_t val){
	uint16_t CountIdx = 0;
	uint16_t ReadIdx = buf->read;
	uint16_t WriteIdx = buf->write;

	while(ReadIdx != WriteIdx){
		CountIdx++;
		if(buf->buffer[ReadIdx] == val){
			break;
		}
		ReadIdx = BufferU32_NextIndex(ReadIdx);

	}

	if(ReadIdx == WriteIdx){
		CountIdx = 0;
	}

	return CountIdx;

 }


/** @brief Compute the buffer size
 *
 * Computes the size of the allocated buffer and saves it
 * in the structure
 *
 * @param *buf Pointer to the Circular Buffer
 *
 * @return None
 */
STATIC void BufferU32_Size(sCircularBufferU32_t *buf){
	if(buf->write == buf->read){
	  buf->size = 0;
	} else if(buf->write > buf->read){
	  buf->size = (buf->write - buf->read);
	} else {
	  buf->size = BUFFER_C_SIZE - buf->read + buf->write + 1;
	}
  }


/** @brief Compute the next index
 *
 * Computes the next index value for the buffer
 *
 * @param idx Index to increment
 *
 * @return Compensated Index
 */
STATIC uint16_t BufferU32_NextIndex(uint16_t idx){

	if(++idx == ACTUAL_BUFFER_C_SIZE){
		idx= 0;
	}
	return idx;
  }

/** @brief Compute the previous index
 *
 * Computes the previous index value for the buffer
 *
 * @param idx Index to decrement
 *
 * @return Compensated Index
 */
STATIC uint16_t BufferU32_PrevIndex(uint16_t idx){
	if(idx == 0){
		idx = ACTUAL_BUFFER_C_SIZE -1;
	} else {
		idx--;
	}
	return idx;
}
