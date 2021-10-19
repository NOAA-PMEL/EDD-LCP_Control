/**
 * @file piston.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-14
 * 
 * 
 */
#ifndef PISTON_H
#define PISTON_H


/**********************************************************************************
 * Includes
 *********************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
/**********************************************************************************
 * Configuration Constants
 *********************************************************************************/


/**********************************************************************************
 * MACROS
 *********************************************************************************/
#define PISTON_I2C_ADDR         ( 0x6C )        /**< I2C Address */


#define PISTON_I2C_MEM_ADDR_VOLUME  ( 0x00 )    /**< Memory Address of Volume (double) */
#define PISTON_I2C_MEM_ADDR_EXT_RET ( 0x60 )
#define PISTON_I2C_MEM_ADDR_AT_ZERO ( 0x68 )
#define PISTON_I2C_MEM_ADDR_AT_FULL ( 0x69 )
#define PISTON_I2C_MEM_ADDR_TRV_ENG ( 0x61 )


#define PISTON_VOLUME_DIFF_MAX      ( 0.008f )
/**********************************************************************************
 * Typdefs
 *********************************************************************************/
typedef struct sPiston_t
{
    struct {
        uint16_t rate;
        bool data_valid;
        SemaphoreHandle_t semaphore;
    }rtos;
    double setpoint;        /**< User setpoint for volume */
    double volume;           /**< Most recent volume estimate */
    bool at_zero;           /**< At Zero flag */
    bool at_full;           /**< At Full extent flag */
}Piston_t;

/**********************************************************************************
 * Function Prototypes
 *********************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

void task_move_piston_to_zero(void);
void task_move_piston_to_full(void);
void task_move_piston_to_volume(void);

bool PIS_set_volume(double volume);
void PIS_move_to_zero(void);
void PIS_move_to_full(void);
void PIS_extend(void);
void PIS_retract(void);
void PIS_stop(void);
void PIS_is_at_zero(void);
void PIS_move_to_volume(float volume);


/**********************************************************************************
 * Unit Test Variables & Static Prototpyes
 *********************************************************************************/
#ifdef TEST
#ifdef DOXYGEN_IGNORE_THIS


#endif // DOXYGEN_IGNORE_THIS
#endif


#ifdef __cplusplus
} // extern "C"
#endif 



#endif // PISTON_H
