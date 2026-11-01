/**
 * @file temperature.h
 * @brief High-level abstraction header for temperature sensors.
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************
- Typedefs & Structures
*********************************************************************************/

typedef enum eTemperatureSensor_t {
    TEMP_Undefined,
    TEMP_SoundNine_OEM,
} TemperatureSensor_t;

typedef struct sTemperature_t {   
    TemperatureSensor_t sensor;
    struct {
        uint8_t uart;           /**< UART Port */
    } uart;
} Temperature_t;

typedef struct sTemperature_Measurement_t {
    float temperature;
    float resistance;
} Temperature_Measurement_t;

/**********************************************************************************
- Function Prototypes
*********************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Initialize temperature sensor subsystem.
 * @param sensor Target sensor type.
 * @return true if initialized successfully.
 */
bool TEMP_initialize(TemperatureSensor_t sensor);

/** 
 * @brief FreeRTOS-aware initialization routine.
 * Powers rail, enables MAX14830 port, waits for POST, and silences streaming.
 */
void TEMP_initialize_RTOS(void);

/** 
 * @brief FreeRTOS teardown routine.
 * Disables the MAX14830 port and turns off sensor power.
 */
void TEMP_uninitialize_RTOS(void);

/** 
 * @brief Provide power to the temperature sensor rail.
 */
void TEMP_Power_ON(void);

/** 
 * @brief Cut power to the temperature sensor rail.
 */
void TEMP_Power_OFF(void);

/** 
 * @brief Stop autonomous auto-sampling stream.
 * @return true on command transmission and FIFO flush.
 */
bool TEMP_stop_sampling(void);

/** 
 * @brief Flushes the MAX14830 RX FIFO and clears the overrun status flag.
 */
void TEMP_flush_rx(void);

/** 
 * @brief Read temperature sensor on demand.
 * @param data Destination pointer for parsed measurement results.
 */
void TEMP_Read(Temperature_Measurement_t *data);

#ifdef __cplusplus
}
#endif

#endif // TEMPERATURE_H