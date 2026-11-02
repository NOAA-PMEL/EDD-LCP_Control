#ifndef _ARTEMIS_PISTON_H
#define _ARTEMIS_PISTON_H

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
// Configuration Constants
//
//*****************************************************************************
#define ARTEMIS_PISTON_I2C_ADDRESS          ( 0x6C )
#define ARTEMIS_PISTON_I2C_DATA_LEN_REG     ( 0x01 )
#define ARTEMIS_PISTON_I2C_DATA_REG         ( 0x02 )

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
// Function Prototypes - Core Driver
//
//*****************************************************************************
void artemis_piston_i2c_initialize(uint8_t i2c_addr);
void artemis_piston_i2c_uninitialize(void);
void artemis_piston_i2c_power_on(void);
void artemis_piston_i2c_power_off(void);

void artemis_piston_i2c_send_msg(uint8_t *msg, uint16_t len, bool stop);
uint16_t artemis_piston_i2c_read_data(uint8_t *pBuf);
void artemis_piston_i2c_read(uint8_t addr, uint8_t *data, uint16_t len);
void artemis_piston_set_write_mode(bool state);

//*****************************************************************************
//
// Function Prototypes - Bus Hardening & Freeze Workarounds
//
//*****************************************************************************

/**
 * @brief Yields task execution during the slave's internal 3.2-second blocking delay.
 */
void artemis_piston_blackout_wait(void);

/**
 * @brief Unlocks SCL/SDA by clocking 9 cycles with a STOP condition if the slave stalls.
 */
void artemis_piston_bus_clear(void);

/**
 * @brief Performs non-blocking read polling with exponential backoff and bus recovery.
 *
 * @param addr Slave register offset to read from.
 * @param data Destination buffer to receive data.
 * @param len Number of bytes to read.
 * @param max_retries Maximum polling attempts before declaring an error.
 * @return true if valid data was received, false if slave stayed unresponsive.
 */
bool artemis_piston_i2c_read_retry(uint8_t addr, uint8_t *data, uint16_t len, uint8_t max_retries);

/**
 * @brief Issues a travel stop command and invokes blackout wait to protect the I2C bus.
 */
void artemis_piston_stop_and_wait(void);

/**
 * @brief Long-window supervisory poll loop for homing sequences across multiple slave freezes.
 *
 * @param timeout_ms Total timeout allowance (recommended 20000ms - 25000ms).
 * @return true if homing completed successfully, false if timed out.
 */
bool artemis_piston_wait_homing_complete(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // _ARTEMIS_PISTON_H