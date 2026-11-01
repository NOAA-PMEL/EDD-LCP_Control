/**
 * @file temperature.c
 * @brief Temperature sensor interface for SoundNine OEM on MAX14830 COM0
 */

#include "temperature.h"
#include "S9_temperature.h"
#include "am_bsp_pins.h"
#include "MAX14830.h"

#include "FreeRTOS.h"
#include "task.h"

//*****************************************************************************
// Static Variables & Configurations
//*****************************************************************************

static Temperature_t module = {
    .sensor = TEMP_SoundNine_OEM,
};

static S9_init_param sParam = {
    .port       = MAX14830_COM_PORT0,
    .baudrate   = MAX14830_COM_BAUDRATE_9600,
    .pin_config = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_COM0_POWER_PIN,
    .pin_number = AM_BSP_GPIO_COM0_POWER_PIN
};

static S9_init_param *pS;

//*****************************************************************************
// Global Functions
//*****************************************************************************

bool TEMP_initialize(TemperatureSensor_t sensor)
{
    module.sensor = sensor;

    switch (sensor)
    {
        case TEMP_SoundNine_OEM:
            pS = &sParam;
            S9T_init(pS);
            break;
        case TEMP_Undefined:
        default:
            break;
    }

    return true;
}

void TEMP_initialize_RTOS(void)
{
    // 1. Energize the sensor power rail
    S9T_enable();

    // 2. Enable MAX14830 Port 0 and configure 9600 baud
    MAX14830_port_enable((eMAX18430_ComPort_t)sParam.port);
    MAX14830_Set_baudrate((eMAX18430_ComPort_t)sParam.port, sParam.baudrate);

    // 3. Sensor POST delay: Wait 350 ms for SoundNine MCU to boot and stabilize
    vTaskDelay(pdMS_TO_TICKS(350));

    // 4. Halt default continuous streaming and flush power-up UART garbage
    TEMP_stop_sampling();
}

void TEMP_uninitialize_RTOS(void)
{
    MAX14830_port_disable((eMAX18430_ComPort_t)sParam.port);
    TEMP_Power_OFF();
}

void TEMP_Power_ON(void)
{
    S9T_enable();
}

void TEMP_Power_OFF(void)
{
    S9T_disable();
}

void TEMP_Read(Temperature_Measurement_t *data)
{
    float temperature = 0.0f;
    float resistance  = 0.0f;

    S9T_Read(&temperature, &resistance);

    data->temperature = temperature;
    data->resistance  = resistance;
}

bool TEMP_stop_sampling(void)
{
    // Clear any partial command line in the S9 receive buffer
    MAX14830_write((eMAX18430_ComPort_t)sParam.port, (uint8_t *)"\r\n", 2);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Send the stop command with carriage return
    MAX14830_write((eMAX18430_ComPort_t)sParam.port, (uint8_t *)"stop\r", 5);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Flush any leftover frames out of the MAX14830 FIFO
    TEMP_flush_rx();

    return true;
}

void TEMP_flush_rx(void)
{
    // 1. Reset RX FIFO for UART Port 0
    MAX14830_reset_rx_fifo((eMAX18430_ComPort_t)sParam.port);

    // 2. Read Line Status Register (LSR) to clear OE (Overrun Error) flag
    uint8_t lsr = 0;
    MAX14830_read_register((eMAX18430_ComPort_t)sParam.port, MAX14830_REG_LSR, &lsr);
}