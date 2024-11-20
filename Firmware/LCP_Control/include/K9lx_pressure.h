/**! @file K9lx_pressure.h
 * @brief Maxim SPI-to-UART Converter
 *
 * @author Basharat Martin basharat.martin@noaa.gov
 * @date Feb 23, 2023
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the Keller 9LXe, circuit board (9L140) Pressure sensor over RS-485
 *
 *
 *
 * @bug  No known bugs
 */
#ifndef K9LX_H_PRESSURE
#define K9LX_H_PRESSURE

#include "stdint.h"
#include "artemis_max14830.h"
#include "bsp_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    K9LX_MODE_PR   = 0,        /**< Mode: Vented Gauge. Zero at atmospheric pressure */
    K9LX_MODE_PA   = 1,        /**< Mode: Sealed Gauge. Zero at 1.0 bar abs */
    K9LX_MODE_PAA  = 2,        /**< Mode: Absolute.  Zero at vacuum */
    K9LX_MODE_NONE = 3         /**< Mode: not defined */
} eK9LX_PMode_t;

typedef struct
{
    e_uart_t port;
    uint32_t baudrate;
    const am_hal_gpio_pincfg_t *pin_config;
    uint32_t pin_number;
} K9lx_init_param;

typedef struct s_module_scaling_t
{
    float low;
    float high;
    float diff;
} K9lx_scaling_t;

typedef struct s_module_manufacturer_t
{
    uint32_t custom_id;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    eK9LX_PMode_t mode;
} K9lx_manufacturer_t;

typedef struct s_module_t
{
    //module_buffer_t txbuffer;
    //module_buffer_t rxbuffer;
    struct{
        struct{
            const am_hal_gpio_pincfg_t *pin;
            uint32_t pin_number;
        }power;
        struct {
            e_uart_t port;
            uint32_t baudrate;
        }uart;
    }device;

    K9lx_scaling_t scaling;
    K9lx_manufacturer_t manufacturer;
} K9lx_t;

typedef struct s_k9lx_data_pt
{
    float pressure;
    float temperature;
} K9lx_data_pt;

typedef union
{
    struct {
        uint8_t u[4];
    };
    float f;
} u32_to_float_t;

/* Functions */
void K9lx_init(K9lx_init_param *p);
void K9lx_power_on(void);
void K9lx_power_off(void);
void K9lx_read_P(float *pressure);
void K9lx_read_PT(float *pressure, float *temperature);

#ifdef __cplusplus
}
#endif

#endif // K9LX_H_PRESSURE
