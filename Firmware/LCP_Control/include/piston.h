/**
 * @file piston.h
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief Artemis main board piston control interface
 * @version 0.4
 * @date 2026-10-31
 */
#ifndef PISTON_H
#define PISTON_H

/**********************************************************************************
 * Includes
 *********************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "task.h"
#include "semphr.h"
#include "rtos.h"

/**********************************************************************************
 * MACROS & I2C REGISTERS
 *********************************************************************************/
#define PISTON_I2C_ADDR                 ( 0x24 )        /**< 7-bit Slave I2C Address */

/* MEM WRITE ADDRESSES */
#define PISTON_I2C_W_SET_VOLUME         ( 0x00 )
#define PISTON_I2C_W_VAR_WRITE          ( 0x07 )
#define PISTON_I2C_W_VAR_WRITE_KEY      ( 0xA5 )
#define PISTON_I2C_W_SET_LENGTH         ( 0x20 )
#define PISTON_I2C_W_RESET              ( 0x77 )
#define PISTON_I2C_W_RESET_KEY          ( 0xCA )

/* MEM READ and WRITE ADDRESSES */
#define PISTON_I2C_RW_TRV_DIR           ( 0x60 )
#define PISTON_I2C_RW_TRV_ENG           ( 0x61 )
#define PISTON_I2C_RW_TRV_SPD           ( 0x62 )
#define PISTON_I2C_RW_TRV_USER_OR       ( 0x63 )
#define PISTON_I2C_RW_MOV_ZERO          ( 0x64 )
#define PISTON_I2C_RW_MOV_FULL          ( 0x65 )
#define PISTON_I2C_RW_RST_FULL          ( 0x66 )
#define PISTON_I2C_RW_PST_CAL           ( 0x67 )

/* MEM READ ADDRESSES */
#define PISTON_I2C_R_VOLUME_TOTAL       ( 0x08 )
#define PISTON_I2C_R_VOLUME_HOUSE       ( 0x0C )
#define PISTON_I2C_R_VOLUME_S_PST       ( 0x18 )
#define PISTON_I2C_R_VOLUME_L_PST       ( 0x1C )
#define PISTON_I2C_R_LENGTH_TOTAL       ( 0x28 )
#define PISTON_I2C_R_LENGTH_S_PST       ( 0x38 )
#define PISTON_I2C_R_LENGTH_L_PST       ( 0x3C )

#define PISTON_I2C_R_PST_ENC_COUNTS     ( 0x50 )        /**< 4-byte int32_t ticks */
#define PISTON_I2C_R_PST_ENC_COUNTS_MAX ( 0x54 )        /**< 4-byte int32_t max ticks */

#define PISTON_I2C_R_TRV_ZERO           ( 0x68 )
#define PISTON_I2C_R_TRV_FULL           ( 0x69 )
#define PISTON_I2C_R_TRV_MIN            ( 0x6A )
#define PISTON_I2C_R_TRV_MAX            ( 0x6B )
#define PISTON_I2C_R_TRV_FRST           ( 0x6C )

#define PISTON_I2C_R_SYS_ID             ( 0xE8 )
#define PISTON_I2C_R_YEAR_BUILD         ( 0xF8 )
#define PISTON_I2C_R_FIRMWARE_MAJ       ( 0xFA )
#define PISTON_I2C_R_FIRMWARE_MIN       ( 0xFB )
#define PISTON_I2C_R_FIRMWARE_BUILD     ( 0xFC )

/* VOLUME AND LENGTH TOLERANCE DEAD-BANDS */
#define PISTON_VOLUME_DIFF_MAX          ( 0.05f )
#define PISTON_LENGTH_DIFF_MAX          ( 0.01f )      /**< Accommodates ~0.011" unbraked coast */

/**********************************************************************************
 * Typedefs
 *********************************************************************************/
typedef struct sPiston_t
{
    struct {
        uint16_t rate;
        bool data_valid;
        SemaphoreHandle_t semaphore;
    } rtos;
    float setpoint_v;           /**< User setpoint for volume (in^3) */
    float setpoint_l;           /**< User setpoint for length (in) */
    float volume;               /**< Most recent volume estimate */
    float length;               /**< Most recent length estimate */
    bool at_zero;               /**< At Zero flag */
    bool at_full;               /**< At Full extent flag */
} Piston_t;

/**********************************************************************************
 * Function Prototypes
 *********************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

bool PIS_initialize(void);
void PIS_uninitialize(void);
bool PIS_set_volume(float volume);
bool PIS_set_length(float length);
void PIS_move_to_zero(void);
void PIS_move_to_full(void);
void PIS_reset_to_full(void);
void PIS_extend(void);
void PIS_retract(void);
void PIS_stop(void);
bool PIS_is_at_zero(void);
bool PIS_is_at_full(void);
bool PIS_move_to_volume(float volume);
bool PIS_move_to_length(float length);
float PIS_get_length(void);
float PIS_get_volume(void);

void PIS_calibration(bool cal);
bool PIS_calibration_check(void);
void PIS_Reset(void);

// FreeRTOS accessor functions
bool PIS_Get_Volume(float *volume);
bool PIS_Get_Length(float *length);
void PIS_task_move_length(TaskHandle_t *xPiston);
void PIS_task_move_volume(TaskHandle_t *xPiston);
void PIS_task_move_full(TaskHandle_t *xPiston);
void PIS_task_reset_full(TaskHandle_t *xPiston);
void PIS_task_move_zero(TaskHandle_t *xPiston);

void task_move_piston_to_zero(void);
void task_move_piston_to_full(void);
void task_reset_piston_to_full(void);
void task_move_piston_to_volume(void);
void task_move_piston_to_length(void);
void PIS_set_piston_rate(uint8_t rate);
bool PIS_task_delete(TaskHandle_t *xPiston);
bool PIS_taskStatus(void);

#ifdef __cplusplus
}
#endif 

#endif // PISTON_H