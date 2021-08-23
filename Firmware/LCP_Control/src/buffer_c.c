/** @file BufferC.c
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

#include "buffer_c.h"

/*******************************************************************************
*					STATIC FUNCTIONS PROTOTYPES
*******************************************************************************/
STATIC uint16_t BufferC_Scan(sCircularBufferC_t *buf,char val);
STATIC void BufferC_Size(sCircularBufferC_t *buf);
STATIC uint16_t BufferC_NextIndex(uint16_t idx);
STATIC uint16_t BufferC_PrevIndex(uint16_t idx);
/*******************************************************************************
*					GLOBAL FUNCTIONS
*******************************************************************************/
void BufferC_Clear(sCircularBufferC_t *buf)
{
        buf->EndlineFlag = false;
	buf->read = 0;
	buf->write = 0;
	memset(buf,0,ACTUAL_BUFFER_C_SIZE);


	return;
}
uint8_t BufferC_Get_Size(sCircularBufferC_t *buf)
{
  return buf->size;
}

eBufferCStatus_t BufferC_putc(sCircularBufferC_t *buf, char val){
	eBufferCStatus_t result = BUFFER_C_ERROR;
	
	/** Determine the index to write to */
	uint16_t NextWrite = BufferC_NextIndex(buf->write);
	
	/** If the next index is equal to the read index, the buffer is full */
	if(NextWrite == buf->read){
		result = BUFFER_C_FULL;
	} else {
		/** Write the value to the buffer */
		buf->buffer[buf->write] = val;
		/** Update the index to the next value */
		buf->write = NextWrite;
		/** Update the buffer size */
		BufferC_Size(buf);
		/** Update the result */
		result = BUFFER_C_OK;
	}
        if(val == '\n')
        {
         buf->EndlineFlag = true; 
        }
	return result;
}

eBufferCStatus_t BufferC_getc(sCircularBufferC_t *buf, char *value){
	eBufferCStatus_t result = BUFFER_C_ERROR;

	/** If buffer read index is not == to write index, then there is data */
	/** otherwise it is empty */
	if(buf->read != buf->write){
		/** Read the buffer Value */
		*value = buf->buffer[buf->read];
		/** Clear the current buffer value */
		buf->buffer[buf->read] = 0;
		/** Update the Index */
		buf->read = BufferC_NextIndex(buf->read);
		/** Update the buffer size */
		BufferC_Size(buf);
		/** Update the result */
		result = BUFFER_C_OK;

	} else {
		result = BUFFER_C_EMPTY;
	}
	return result;
}


eBufferCStatus_t BufferC_Backspace(sCircularBufferC_t *buf){
	eBufferCStatus_t result = BUFFER_C_ERROR;

	if(buf->write == buf->read){
		result = BUFFER_C_EMPTY;
	} else {
		buf->write = BufferC_PrevIndex(buf->write);
		buf->buffer[buf->write] = 0;
		BufferC_Size(buf);
		result = BUFFER_C_OK;
	}

	return result;
}




eBufferCStatus_t BufferC_puts(sCircularBufferC_t *buf, char *str, uint16_t len){
	eBufferCStatus_t result = BUFFER_C_ERROR;

	uint16_t i;

	if(len >= (BUFFER_C_SIZE - buf->size)) {
		result = BUFFER_C_STRING_TOO_LONG;
	} else {
		
		for(i=0;i<len;i++){
			result = BufferC_putc(buf,str[i]);
		}
	}
	return result;
}

uint16_t BufferC_gets(sCircularBufferC_t *buf, 
                              char *str, 
                              uint16_t maxLen){
//	eBufferCStatus_t result = BUFFER_C_ERROR;
	uint16_t len = BufferC_Get_Size(buf);
	uint16_t i;
	
    
    if(len > 0)
    {
      if(len > maxLen)
      {
        len = maxLen;
      }
      for(i=0;i<len;i++)
      {
        BufferC_getc(buf, str++);
      }
    }
//	if(len == 0){
//		result = BUFFER_C_NO_STRING ;
//	}else if(len >= maxLen){
//		result = BUFFER_C_STRING_TOO_LONG;
//	} else {
//		for(i=0;i<len;i++){
//			BufferC_getc(buf,str++);
//		}
//                *str = 0;
//		result = BUFFER_C_OK;
//	}
	return len;
}


void BufferC_remove_leading_whitespace( sCircularBufferC_t *buf)
{
  char temp;
  uint16_t r_len, n_len;
  while(true)
  {
    if(buf->size == 0)
    {
      break;
    }
    r_len = BufferC_Scan(buf, '\r');
    n_len = BufferC_Scan(buf, '\n');
    
    if(r_len != 1 && n_len != 1)
    {
      break;
    }
    else
    {
      BufferC_getc(buf, &temp);
    }  
  }
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
STATIC uint16_t BufferC_Scan(sCircularBufferC_t *buf,char val){
	uint16_t CountIdx = 0;
	uint16_t ReadIdx = buf->read;
	uint16_t WriteIdx = buf->write;

	while(ReadIdx != WriteIdx){
		CountIdx++;
		if(buf->buffer[ReadIdx] == val){
			break;
		}
		ReadIdx = BufferC_NextIndex(ReadIdx);
		
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
STATIC void BufferC_Size(sCircularBufferC_t *buf){
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
STATIC uint16_t BufferC_NextIndex(uint16_t idx){
	
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
STATIC uint16_t BufferC_PrevIndex(uint16_t idx){
	if(idx == 0){
		idx = ACTUAL_BUFFER_C_SIZE -1;
	} else {
		idx--;
	}
	return idx;
}