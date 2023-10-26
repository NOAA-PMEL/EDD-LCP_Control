#include "temperature.h"
#include "S9_temperature.h"
#include "am_bsp_pins.h"
#include "MAX14830.h"

static Temperature_t module = {
    .sensor = TEMP_SoundNine_OEM,
};

static S9_init_param sParam = {
    .port       =   MAX14830_COM_PORT0,
    .baudrate   =   MAX14830_COM_BAUDRATE_9600,
    .pin_config =   &g_AM_BSP_GPIO_COM0_POWER_PIN,
    .pin_number =   AM_BSP_GPIO_COM0_POWER_PIN
};
static S9_init_param *pS;

void TEMP_initialize(TemperatureSensor_t sensor){

    module.sensor = sensor;

    switch (sensor)
    {
        case TEMP_SoundNine_OEM:
            pS = &sParam;
            S9T_init(pS);
            break;
        case TEMP_Undefined:
            break;
        default:
            break;
    }
}

void TEMP_Power_ON(void)
{
    S9T_ON();
}

void TEMP_Power_OFF(void)
{
    S9T_OFF();
}

void TEMP_Read(Temperature_Measurement_t *data)
{
    float temperature, resistance = 0;

    S9T_Read(&temperature, &resistance);

    data->temperature = temperature;
    data->resistance = resistance;
}

