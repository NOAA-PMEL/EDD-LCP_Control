/**
 * @file temperature.c
 * @brief Temperature sensor subsystem interface
 */

#include "temperature.h"
#include "S9_temperature.h"
#include "am_bsp_pins.h"
#include "MAX14830.h"

/* FreeRTOS includes for vTaskDelay and pdMS_TO_TICKS */
#include "FreeRTOS.h"
#include "task.h"

static Temperature_t module = {
    .sensor = TEMP_SoundNine_OEM,
};

static S9_init_param sParam = {
    .port        = MAX14830_COM_PORT0,
    .baudrate    = MAX14830_COM_BAUDRATE_9600,
    .pin_config  = &g_AM_BSP_GPIO_COM0_POWER_PIN,
    .pin_number  = AM_BSP_GPIO_COM0_POWER_PIN
};
static S9_init_param *pS;

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

void TEMP_Power_ON(void)
{
    S9T_enable();
}

void TEMP_Power_OFF(void)
{
    S9T_disable();
}

void TEMP_initialize_RTOS(void)
{
    uint8_t flush_buf[32];

    /* 1. Configure MAX14830 COM port clock and baudrate first */
    MAX14830_port_enable((eMAX18430_ComPort_t)sParam.port);
    MAX14830_Set_baudrate((eMAX18430_ComPort_t)sParam.port, sParam.baudrate);

    /* 2. Energize sensor power rail */
    TEMP_Power_ON();

    /* 3. Essential: Wait for SoundNine internal MCU to complete boot sequence */
    vTaskDelay(pdMS_TO_TICKS(150));

    /* 4. Send stop and rate 0 to halt autonomous streaming */
    _module_s9_stop_sampling_RTOS();
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 5. Drain any autonomous bytes that leaked into the FIFO during warm-up */
    while (MAX14830_UART_Read((eMAX18430_ComPort_t)sParam.port, flush_buf) > 0);

    /* 6. If task_temperature() uses per-sample power gating, shut the rail 
     * off here so task_temperature() starts with the rail cleanly unpowered. */
    TEMP_Power_OFF();
}

void TEMP_uninitialize_RTOS(void)
{
    MAX14830_port_disable((eMAX18430_ComPort_t)sParam.port);
    TEMP_Power_OFF();
}

void TEMP_Read(Temperature_Measurement_t *data)
{
    float temperature = 0.0f;
    float resistance = 0.0f;

    S9T_Read(&temperature, &resistance);

    data->temperature = temperature;
    data->resistance = resistance;
}

void TEMP_flush_rx(void)
{
    uint8_t dummy[32];
    while (MAX14830_UART_Read((eMAX18430_ComPort_t)sParam.port, dummy) > 0);
}