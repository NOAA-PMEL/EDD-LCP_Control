/**! @file K9lx_pressure.c
 * @brief Keller 9LXe Pressure Sensor Driver over MAX14830 RS-485
 *
 * @author Basharat Martin basharat.martin@noaa.gov
 * @date Feb 23, 2023
 * @version 1.0.1
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 */

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "artemis_debug.h"
#include "MAX14830.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#include "buffer_c.h"
#include "K9lx_pressure.h"

#define K9LX_ADDR                   0x01
#define K9LX_BUFFER_LENGTH          0x0D
#define K9LX_RECV_ERROR             0x80

#define K9LX_READ_REG_CMD           0x03
#define K9LX_WRITE_REG_CMD          0x06
#define K9LX_ECHO_CMD               0x08
#define K9LX_WRITE_REGS_CMD         0x10

#define K9LX_FL_CH0                 0x0000
#define K9LX_FL_P1                  0x0002
#define K9LX_FL_P2                  0x0004
#define K9LX_FL_T                   0x0006
#define K9LX_FL_TOB1                0x0008
#define K9LX_FL_TOB2                0x000A

#define K9LX_FL1_P1                 0x0100
#define K9LX_FL1_TOB1               0x0102
#define K9LX_FL1_P2                 0x0104
#define K9LX_FL1_TOB2               0x0106

#define K9LX_INT16_CH0              0x0010
#define K9LX_INT16_P1               0x0011
#define K9LX_INT16_P2               0x0012
#define K9LX_INT16_T                0x0013
#define K9LX_INT16_TOB1             0x0014
#define K9LX_INT16_TOB2             0x0015

#define K9LX_INT32_CH0              0x0020
#define K9LX_INT32_P1               0x0022
#define K9LX_INT32_P2               0x0024
#define K9LX_INT32_T                0x0026
#define K9LX_INT32_TOB1             0x0028
#define K9LX_INT32_TOB2             0x002A

#define K9LX_STATUS                 0x020C
#define K9LX_SER_NUM_H              0x0202
#define K9LX_SER_NUM_L              0x0203
#define K9LX_FIRM_VER0              0x020E
#define K9LX_FIRM_VER1              0x020F
#define K9LX_P_MODE                 0x0209

#define K9LX_UART                   0x0200
#define K9LX_UART_9K                (0x0)
#define K9LX_UART_115K              (0x1)

#define CMD_LENGTH                  8

#define FREERTOS

static K9lx_t k9lx;
static K9lx_t *pK9lx = &k9lx;

static void module_k9lx_read_sensor_p(float *pressure);
static void module_k9lx_read_sensor_pt(K9lx_data_pt *data);
static void module_k9lx_device_info(uint8_t port);
static uint16_t module_k9lx_crc16 (uint8_t *crc_h, uint8_t *crc_l, uint8_t *pData, uint8_t len);
static int8_t module_k9lx_read_reg(uint8_t port, uint16_t reg_add, uint8_t reg_num, uint8_t *pData);
static int8_t module_k9lx_read_reg_RTOS(uint8_t port, uint16_t reg_add, uint8_t reg_num, uint8_t *pData);
static int8_t module_k9lx_data_integrity (uint8_t *Data, uint8_t len);

void K9lx_init(K9lx_init_param *p)
{
    pK9lx->device.uart.port = p->port;
    pK9lx->device.uart.baudrate = p->baudrate;

    pK9lx->device.power.pin = p->pin_config;
    pK9lx->device.power.pin_number = p->pin_number;

    am_hal_gpio_pinconfig(pK9lx->device.power.pin_number, *pK9lx->device.power.pin);

    MAX14830_Set_baudrate((eMAX18430_ComPort_t)pK9lx->device.uart.port, pK9lx->device.uart.baudrate);

    K9lx_power_on();

    /* 250ms cold boot margin for Keller internal processor */
    am_util_delay_ms(250);

    module_k9lx_device_info(pK9lx->device.uart.port);
    ARTEMIS_DEBUG_PRINTF("K9lx Pressure Sensor is initialized\n\n");
    K9lx_power_off();
}

void K9lx_power_on(void)
{
    am_hal_gpio_output_clear(pK9lx->device.power.pin_number);
}

void K9lx_power_off(void)
{
    am_hal_gpio_output_set(pK9lx->device.power.pin_number);
}

void K9lx_ON(void)
{
    K9lx_power_on();
}

void K9lx_OFF(void)
{
    K9lx_power_off();
}

static uint16_t module_k9lx_crc16 (uint8_t *crc_h, uint8_t *crc_l, uint8_t *pData, uint8_t len)
{
    uint16_t crc16 = 0xFFFF;
    uint8_t n, m, x;

    m = len;
    x = 0;

    while (m > 0)
    {
        crc16 ^= pData[x];
        for (n = 0; n < 8; n++)
        {
            if (crc16 & 1)
            {
                crc16 >>= 1;
                crc16 ^= 0xA001;
            }
            else
            {
                crc16 >>= 1;
            }
        }
        m--;
        x++;
    }

    *crc_h = (crc16 >> 8) & 0xFF;
    *crc_l = crc16 & 0xFF;

    return crc16;
}

void K9lx_read_P(float *pressure)
{
    float p = 0.0f;
    module_k9lx_read_sensor_p(&p);
    *pressure = p;
}

void K9lx_read_PT(float *pressure, float *temperature)
{
    K9lx_data_pt data = {0};
    module_k9lx_read_sensor_pt(&data);
    *pressure = data.pressure;
    *temperature = data.temperature;
}

static void module_k9lx_read_sensor_p(float *pressure)
{
    uint8_t ret = 0;
    uint8_t p[4] = {0};
    u32_to_float_t pressure_bar;

#ifdef FREERTOS
    ret = module_k9lx_read_reg_RTOS(pK9lx->device.uart.port, K9LX_FL1_P1, 2, p);
#else
    ret = module_k9lx_read_reg(pK9lx->device.uart.port, K9LX_FL1_P1, 2, p);
#endif

    if (ret == 0)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            pressure_bar.u[3 - i] = p[i];
        }
        *pressure = pressure_bar.f;
    }
    else
    {
        *pressure = 0.0f;
    }
}

static void module_k9lx_read_sensor_pt(K9lx_data_pt *data)
{
    uint8_t ret = 0;
    uint8_t pt[8] = {0};
    u32_to_float_t pressure_bar;
    u32_to_float_t temperature_c;

#ifdef FREERTOS
    ret = module_k9lx_read_reg_RTOS(pK9lx->device.uart.port, K9LX_FL1_P1, 4, pt);
#else
    ret = module_k9lx_read_reg(pK9lx->device.uart.port, K9LX_FL1_P1, 4, pt);
#endif

    if (ret == 0)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            pressure_bar.u[3 - i] = pt[i];
        }
        for (uint8_t i = 4; i < 8; i++)
        {
            temperature_c.u[7 - i] = pt[i];
        }
        data->pressure = pressure_bar.f;
        data->temperature = temperature_c.f;
    }
    else
    {
        data->pressure = 0.0f;
        data->temperature = 0.0f;
    }
}

static int8_t module_k9lx_data_integrity (uint8_t *Data, uint8_t len)
{
    /* Modbus frames for Keller cannot exceed K9LX_BUFFER_LENGTH */
    if (len < 5 || len > K9LX_BUFFER_LENGTH)
    {
        return -1;
    }

    uint8_t lBuf[K9LX_BUFFER_LENGTH] = {0};
    uint8_t crc_h = 0, crc_l = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        lBuf[i] = Data[i];
    }

    if (K9LX_RECV_ERROR & lBuf[1])
    {
        return -1;
    }

    module_k9lx_crc16(&crc_h, &crc_l, lBuf, len - 2);
    if (crc_l == lBuf[len - 2] && crc_h == lBuf[len - 1])
    {
        return 0;
    }

    return -1;
}

static int8_t module_k9lx_read_reg(uint8_t port, uint16_t reg_add, uint8_t reg_num, uint8_t *pData)
{
    uint8_t cmd[CMD_LENGTH] = {0};
    uint8_t rxlen = 0;
    uint8_t rData[16] = {0};
    uint8_t crc_h = 0, crc_l = 0;

    cmd[0] = K9LX_ADDR;
    cmd[1] = K9LX_READ_REG_CMD;
    cmd[2] = (reg_add >> 8) & 0xFF;
    cmd[3] = reg_add & 0xFF;
    cmd[4] = 0x00;
    cmd[5] = reg_num;

    module_k9lx_crc16(&crc_h, &crc_l, cmd, 6);
    cmd[6] = crc_l;
    cmd[7] = crc_h;

    MAX14830_UART_Write((eMAX18430_ComPort_t)port, cmd, CMD_LENGTH);

    /* 
     * Keller Modbus frame format:
     * Addr (1) + Func (1) + ByteCount (1) + Data (reg_num * 2) + CRC (2) = 5 + (reg_num * 2)
     */
    const uint8_t expected_len = 5 + (reg_num * 2);
    uint8_t chunk[16] = {0};
    uint32_t attempts = 0;
    const uint32_t MAX_ATTEMPTS = 15; // 15 x 10ms = 150ms timeout window

    /* Bare-metal polling loop using am_util_delay_ms since FreeRTOS scheduler isn't started */
    while (rxlen < expected_len && attempts++ < MAX_ATTEMPTS)
    {
        uint16_t len = MAX14830_UART_Read((eMAX18430_ComPort_t)port, chunk);
        if (len > 0)
        {
            for (uint8_t i = 0; i < len && (rxlen + i) < sizeof(rData); i++)
            {
                rData[rxlen + i] = chunk[i];
            }
            rxlen += len;
        }
        else
        {
            am_util_delay_ms(10);
        }
    }

    int8_t ret = module_k9lx_data_integrity(rData, rxlen);
    if (ret == 0 && rxlen >= expected_len)
    {
        for (uint8_t i = 0; i < rxlen - 5; i++)
        {
            pData[i] = rData[i + 3];
        }
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("K9lX:: Read ERROR (%d, rxlen=%u)\n", ret, rxlen);
    }

    return ret;
}

static int8_t module_k9lx_read_reg_RTOS(uint8_t port, uint16_t reg_add, uint8_t reg_num, uint8_t *pData)
{
    uint8_t cmd[CMD_LENGTH] = {0};
    uint8_t rxlen = 0;
    uint8_t rData[32] = {0}; // Expand from 16 to 32 bytes for safety
    uint8_t crc_h = 0, crc_l = 0;

    cmd[0] = K9LX_ADDR;
    cmd[1] = K9LX_READ_REG_CMD;
    cmd[2] = (reg_add >> 8) & 0xFF;
    cmd[3] = reg_add & 0xFF;
    cmd[4] = 0x00;
    cmd[5] = reg_num;

    module_k9lx_crc16(&crc_h, &crc_l, cmd, 6);
    cmd[6] = crc_l;
    cmd[7] = crc_h;

    MAX14830_UART_Write((eMAX18430_ComPort_t)port, cmd, CMD_LENGTH);

    const uint8_t expected_len = 5 + (reg_num * 2);
    uint8_t chunk[32] = {0};
    uint8_t sweeps = 0;

    while (rxlen < expected_len && sweeps++ < 4)
    {
        uint32_t len = MAX14830_UART_Read((eMAX18430_ComPort_t)port, chunk);
        if (len > 0)
        {
            for (uint32_t i = 0; i < len && rxlen < sizeof(rData); i++)
            {
                rData[rxlen++] = chunk[i];
            }
        }

        if (rxlen < expected_len)
        {
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }

    int8_t ret = module_k9lx_data_integrity(rData, rxlen);
    if (ret == 0 && rxlen >= expected_len)
    {
        for (uint8_t i = 0; i < (expected_len - 5); i++)
        {
            pData[i] = rData[i + 3];
        }
    }
    else
    {
        static uint8_t err_count = 0;
        if (err_count++ % 10 == 0)
        {
            ARTEMIS_DEBUG_PRINTF("K9lX:: RTOS Read ERROR (%d, rxlen=%u)\n", ret, rxlen);
        }
    }

    return ret;
}

static void module_k9lx_device_info(uint8_t port)
{
    uint8_t firmware[4] = {0};
    uint8_t serial[4] = {0};
    uint32_t serial_nr = 0;
    uint8_t pt[8] = {0};

    u32_to_float_t pressure_bar;
    u32_to_float_t temperature_c;
    int8_t ret = 0;

    ARTEMIS_DEBUG_PRINTF("\nK9LX Pressure Sensor\n");
    ARTEMIS_DEBUG_PRINTF("**************************************\n");

    /* 1. Firmware version - read first, print entire line once */
    ret = module_k9lx_read_reg(port, K9LX_FIRM_VER0, 2, firmware);
    if (ret == 0)
    {
        ARTEMIS_DEBUG_PRINTF("\tFirmware Ver\t: %u.%u.%u.%u\n",
                             firmware[0], firmware[1], firmware[2], firmware[3]);
    }

    /* 2. Serial number - read first, print entire line once */
    ret = module_k9lx_read_reg(port, K9LX_SER_NUM_H, 2, serial);
    if (ret == 0)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            serial_nr |= (serial[i] << (8 * (3 - i)));
        }
        ARTEMIS_DEBUG_PRINTF("\tSerial Number\t: %u\n", serial_nr);
    }

    /* 3. Pressure & Temperature snapshot */
    ret = module_k9lx_read_reg(port, K9LX_FL1_P1, 4, pt);
    if (ret == 0)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            pressure_bar.u[3 - i] = pt[i];
        }
        for (uint8_t i = 4; i < 8; i++)
        {
            temperature_c.u[7 - i] = pt[i];
        }
        ARTEMIS_DEBUG_PRINTF("\tPressure\t: %0.5f bar\n", pressure_bar.f);
        ARTEMIS_DEBUG_PRINTF("\tTemperature\t: %0.3f °C, %0.3f °F\n",
                             temperature_c.f, ((9.0f / 5.0f) * temperature_c.f) + 32.0f);
    }
}