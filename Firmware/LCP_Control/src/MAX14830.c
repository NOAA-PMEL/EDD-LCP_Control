/**
 * @file MAX14830.c
 * @brief Maxim MAX14830 Quad SPI-to-UART Driver (Consolidated Stable Version)
 */

#include "MAX14830.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "artemis_spi.h"
#include "artemis_debug.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

#include "am_bsp_pins.h"
#include "buffer_c.h"

/** FIFO Data Registers */
#define MAX14830_RHR                ( 0x00 )
#define MAX14830_THR                ( 0x00 )

/** Interrupt Registers */
#define MAX14830_REG_IRQEN          ( 0x01 )
#define MAX14830_REG_ISR            ( 0x02 )
#define MAX14830_REG_LSRINTEN       ( 0x03 )
#define MAX14830_REG_LSR            ( 0x04 )
#define MAX14830_REG_SPCLCHRINTEN   ( 0x05 )
#define MAX14830_REG_SPCLCHARINT    ( 0x06 )
#define MAX14830_REG_STSINTEN       ( 0x07 )
#define MAX14830_REG_STSINT         ( 0x08 )

/** UART Mode Registers */
#define MAX14830_REG_MODE1          ( 0x09 )
#define MAX14830_REG_MODE2          ( 0x0A )
#define MAX14830_REG_LCR            ( 0x0B )
#define MAX14830_REG_RXTIMEOUT      ( 0x0C )
#define MAX14830_REG_HDPIXDELAY     ( 0x0D )
#define MAX14830_REG_IRDA           ( 0x0E )

/** FIFO Control Registers */
#define MAX14830_REG_FLOWLVL        ( 0x0F )
#define MAX14830_REG_FIFOTRGLVL     ( 0x10 )
#define MAX14830_REG_TXFIFOLVL      ( 0x11 )
#define MAX14830_REG_RXFIFOLVL      ( 0x12 )

/** Flow Control Registers */
#define MAX14830_REG_FLOWCTRL       ( 0x13 )

/** GPIO Registers */
#define MAX14830_REG_GPIOCONFIG     ( 0x18 )
#define MAX14830_REG_GPIODATA       ( 0x19 )

/** Clock Configuration Registers */
#define MAX14830_REG_PLLCONFIG      ( 0x1A )
#define MAX14830_REG_BRGCONFIG      ( 0x1B )
#define MAX14830_REG_DIVLSB         ( 0x1C )
#define MAX14830_REG_DIVMSB         ( 0x1D )
#define MAX14830_REG_CLKSOURCE      ( 0x1E )

/** Global Registers */
#define MAX14830_REG_GLOBALRQ       ( 0x1F )

/** IRQEn Register Bits */
#define MAX14830_IRQ_RFIFOEMTY      ( 1u << 6 )

/** LSR Register Bits */
#define MAX14830_LSR_RXNOISE        ( 1u << 5 )
#define MAX14830_LSR_RXBREAK        ( 1u << 4 )
#define MAX14830_LSR_FRAMEERR       ( 1u << 3 )
#define MAX14830_LSR_RXPARITY       ( 1u << 2 )
#define MAX14830_LSR_RXOVERRUN      ( 1u << 1 )
#define MAX14830_LSR_RTIMEOUT       ( 1u )

/** MODE1 Register Bits */
#define MAX14830_MODE1_IRQ_SEL      ( 1u << 7 )
#define MAX14830_MODE1_TRNSCV_CTRL  ( 1u << 4 )

/** MODE2 Register Bits */
#define MAX14830_MODE2_ECHO_SUPRS   ( 1u << 7 )
#define MAX14830_MODE2_RX_EMTY_INV  ( 1u << 3 )
#define MAX14830_MODE2_FIFO_RST     ( 1u << 1 )

/** LCR Register Bits */
#define MAX14830_LCR_STOPBITS_1     ( 0u )
#define MAX14830_LCR_LENGTH_8       ( 0x03 )

#define MAX14830_FLOWLVL_HALT(words)    ((words / 8) & 0x0f)
#define MAX14830_FLOWLVL_RES(words)     (((words / 8) & 0x0f) << 4)
#define MAX14830_FIFOTRIGLVL_TX(words)  ((words / 8) & 0x0f)
#define MAX14830_FIFOTRIGLVL_RX(words)  (((words / 8) & 0x0f) << 4)

/** CLKSource Register Bits */
#define MAX14830_CLK_PLL_BYPASS     ( 1u << 3 )
#define MAX14830_CLK_CRYSTAL_EN     ( 1u << 1 )

/** BRGConfig Register Bits */
#define MAX14830_BRG_CLK_DISABLE    ( 1u << 6 )

/** SPI Command Flags */
#define MAX14830_SPI_WRITE_BIT      ( 1u << 7 )
#define MAX14830_SPI_READ_BIT       ( 0x7F )

#define MAX14830_XTAL_FREQ          ( 4000000u )
#define MAX14830_NUM_SERIAL_PORTS   ( 4 )

static SemaphoreHandle_t xSpiMutex = NULL;
static EventGroupHandle_t gEventHandle = NULL;

static void *pIomHandle;
static am_hal_iom_config_t IomConfig = {0};

#define XFER_DATA_SIZE ( 64 )
static uint8_t xfer_data[XFER_DATA_SIZE];

sCircularBufferC_t txBuf[MAX14830_NUM_SERIAL_PORTS];
sCircularBufferC_t rxBuf[MAX14830_NUM_SERIAL_PORTS];

static void module_MAX14830_conf(void);
static void module_MAX14830_chip_enable(void);
static void module_MAX14830_chip_disable(void);
static void module_MAX14830_Power_On(void);
static void module_MAX14830_Power_Off(void);

static uint32_t module_MAX14830_Read(
                    eMAX18430_ComPort_t port,
                    uint8_t reg,
                    uint16_t rxlen,
                    uint8_t *pData);

static void module_MAX14830_Write(
                    eMAX18430_ComPort_t port,
                    uint8_t reg,
                    uint8_t *data,
                    uint16_t len);

static void module_MAX14830_RegUpdate(eMAX18430_ComPort_t port,
                    uint8_t reg, uint8_t bit, uint8_t val);

void module_MAX14830_Handle_IRQ(void);

bool MAX14830_initialize(void)
{
    IomConfig.eInterfaceMode       = AM_HAL_IOM_SPI_MODE;
    IomConfig.ui32ClockFreq        = AM_HAL_IOM_100KHZ;
    IomConfig.eSpiMode             = AM_HAL_IOM_SPI_MODE_0;
    IomConfig.pNBTxnBuf            = NULL;
    IomConfig.ui32NBTxnBufLength   = 0;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_initialize(3, &pIomHandle));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_configure(pIomHandle, &IomConfig));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_enable(pIomHandle));

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCK, g_AM_BSP_GPIO_IOM3_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MISO, g_AM_BSP_GPIO_IOM3_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MOSI, g_AM_BSP_GPIO_IOM3_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_CS, g_AM_BSP_GPIO_IOM3_CS));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NIRQ, g_AM_BSP_GPIO_S2U_NIRQ));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NRESET, g_AM_BSP_GPIO_S2U_NRESET));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_ON, g_AM_BSP_GPIO_S2U_ON));

    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_S2U_NIRQ));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_S2U_NIRQ));

    NVIC_SetPriority(GPIO_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(GPIO_IRQn);
    am_hal_interrupt_master_enable();

    module_MAX14830_Power_On();
    module_MAX14830_conf();

    return true;
}

void MAX14830_uninitialize(void)
{
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_disable(pIomHandle));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_uninitialize(pIomHandle));
}

static void module_MAX14830_conf(void)
{
    uint8_t regVal = MAX14830_CLK_CRYSTAL_EN | MAX14830_CLK_PLL_BYPASS;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_CLKSOURCE, &regVal, 1);

    regVal = MAX14830_MODE1_IRQ_SEL | MAX14830_MODE1_TRNSCV_CTRL | 0x20;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE1, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_MODE1, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_MODE1, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_MODE1, &regVal, 1);

    regVal = MAX14830_MODE2_RX_EMTY_INV | MAX14830_MODE2_FIFO_RST;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE2, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_MODE2, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_MODE2, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_MODE2, &regVal, 1);

    module_MAX14830_RegUpdate(MAX14830_COM_PORT0, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);
    module_MAX14830_RegUpdate(MAX14830_COM_PORT1, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);
    module_MAX14830_RegUpdate(MAX14830_COM_PORT2, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);
    module_MAX14830_RegUpdate(MAX14830_COM_PORT3, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);

    /* Port 0 (S9 standard UART): NO Echo Suppression */
    regVal = MAX14830_MODE2_RX_EMTY_INV;
   module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE2, &regVal, 1);

    /* Port 3 (Keller RS-485): Echo Suppression ENABLED */
    regVal = MAX14830_MODE2_ECHO_SUPRS | MAX14830_MODE2_RX_EMTY_INV;
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_MODE2, &regVal, 1);

    regVal = MAX14830_FLOWLVL_HALT(96) | MAX14830_FLOWLVL_RES(48);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_FLOWLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_FLOWLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_FLOWLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_FLOWLVL, &regVal, 1);

    regVal = MAX14830_LCR_LENGTH_8 | MAX14830_LCR_STOPBITS_1;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_LCR, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_LCR, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_LCR, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_LCR, &regVal, 1);

    regVal = MAX14830_FIFOTRIGLVL_RX(64) | MAX14830_FIFOTRIGLVL_TX(64);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_FIFOTRGLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_FIFOTRGLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_FIFOTRGLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_FIFOTRGLVL, &regVal, 1);

    regVal = MAX14830_GPIO_PIN_TO_BIT(2);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_GPIOCONFIG, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_GPIODATA, &regVal, 1);

    module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_ISR, 1, &regVal);
    module_MAX14830_Read(MAX14830_COM_PORT3, MAX14830_REG_ISR, 1, &regVal);

    regVal = MAX14830_IRQ_RFIFOEMTY;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_IRQEN, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_IRQEN, &regVal, 1);

    for (uint8_t i = 0; i < 4; i++)
    {
        module_MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, 1, &regVal);
        regVal |= MAX14830_BRG_CLK_DISABLE;
        module_MAX14830_Write((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, &regVal, 1);
    }
}

void MAX14830_disable_direct(void)
{
    module_MAX14830_Power_Off();
    am_hal_iom_disable(pIomHandle);
}

void MAX14830_enable_direct(void)
{
    am_hal_iom_enable(pIomHandle);
    module_MAX14830_Power_On();
    module_MAX14830_conf();
}

void MAX14830_disable(void)
{
    module_MAX14830_Power_Off();
    am_hal_iom_disable(pIomHandle);
}

void MAX14830_enable(void)
{
    am_hal_iom_enable(pIomHandle);
    module_MAX14830_Power_On();
    module_MAX14830_conf();

    if (gEventHandle == NULL)
    {
        gEventHandle = xEventGroupCreate();
    }

    if (xSpiMutex == NULL)
    {
        xSpiMutex = xSemaphoreCreateMutex();
    }
}

static void module_MAX14830_RegUpdate(eMAX18430_ComPort_t port,
                        uint8_t reg, uint8_t bit, uint8_t val)
{
    uint8_t curVal = 0x00;
    module_MAX14830_Read(port, reg, 1, &curVal);
    uint8_t update = curVal & bit;
    if (update == val)
    {
        return;
    }
    curVal ^= bit;
    module_MAX14830_Write(port, reg, &curVal, 1);
}

void am_gpio_isr(void)
{
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_S2U_NIRQ));
    module_MAX14830_Handle_IRQ();
}

static void module_MAX14830_chip_enable(void)
{
    am_hal_gpio_output_clear(AM_BSP_GPIO_IOM3_CS);
}

static void module_MAX14830_chip_disable(void)
{
    am_hal_gpio_output_set(AM_BSP_GPIO_IOM3_CS);
}

void MAX14830_port_enable(eMAX18430_ComPort_t port)
{
    uint32_t state = 0;
    am_hal_gpio_state_read(AM_BSP_GPIO_S2U_ON, AM_HAL_GPIO_OUTPUT_READ, &state);

    if (state == 1)
    {
        MAX14830_enable();
    }

    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg &= ~MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);
}

void MAX14830_port_enable_direct(eMAX18430_ComPort_t port)
{
    uint32_t state = 0;
    am_hal_gpio_state_read(AM_BSP_GPIO_S2U_ON, AM_HAL_GPIO_OUTPUT_READ, &state);
    if (state == 1)
    {
        MAX14830_enable_direct();
    }

    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg &= ~MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);
}

void MAX14830_port_disable_direct(eMAX18430_ComPort_t port)
{
    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg |= MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        reg = 0;
        module_MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, 1, &reg);
        if ((reg & MAX14830_BRG_CLK_DISABLE) > 0)
        {
            cnt++;
        }
    }

    if (cnt == MAX14830_NUM_SERIAL_PORTS)
    {
        MAX14830_disable_direct();
    }
}

void MAX14830_port_disable(eMAX18430_ComPort_t port)
{
    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg |= MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

    char discard;
    sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];

    while (BufferC_Get_Size(pBuf) > 0)
    {
        BufferC_getc(pBuf, &discard);
    }

    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        reg = 0;
        module_MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, 1, &reg);
        if ((reg & MAX14830_BRG_CLK_DISABLE) > 0)
        {
            cnt++;
        }
    }

    if (cnt == MAX14830_NUM_SERIAL_PORTS)
    {
        MAX14830_disable();
    }
}

void module_MAX14830_Handle_IRQ(void)
{
    uint8_t rxData[128] = {0};
    uint8_t irq = 0;
    module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_GLOBALRQ, 1, &irq);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Loop through and service any port with an active interrupt */
    for (uint8_t p = 0; p < 4; p++)
    {
        if ((~irq & 0x0F) & (1u << p))
        {
            uint8_t rxlen, isr_status, lsr_status, regVal;
            uint8_t i = 0;

            do
            {
                module_MAX14830_Read((eMAX18430_ComPort_t)p, MAX14830_REG_ISR, 1, &isr_status);
                module_MAX14830_Read((eMAX18430_ComPort_t)p, MAX14830_REG_RXFIFOLVL, 1, &rxlen);

                if (!isr_status && !rxlen)
                {
                    if (gEventHandle != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
                    {
                        xEventGroupSetBitsFromISR(gEventHandle, (1u << p), &xHigherPriorityTaskWoken);
                    }
                    break;
                }

                if (rxlen > 0)
                {
                    while (rxlen > 0)
                    {
                        module_MAX14830_Read((eMAX18430_ComPort_t)p, MAX14830_RHR, 1, &regVal);
                        module_MAX14830_Read((eMAX18430_ComPort_t)p, MAX14830_REG_LSR, 1, &lsr_status);

                        if (lsr_status & MAX14830_LSR_RXOVERRUN)
                        {
                            module_MAX14830_Read((eMAX18430_ComPort_t)p, MAX14830_REG_RXFIFOLVL, 1, &rxlen);
                            module_MAX14830_Read((eMAX18430_ComPort_t)p, MAX14830_RHR, rxlen, rxData);
                            while (rxlen > 0)
                            {
                                BufferC_putc(&rxBuf[p], rxData[i++]);
                                rxlen--;
                            }
                            i = 0;
                            break;
                        }

                        if (lsr_status & (MAX14830_LSR_RXNOISE | MAX14830_LSR_RXBREAK | MAX14830_LSR_RXPARITY | MAX14830_LSR_FRAMEERR | MAX14830_LSR_RTIMEOUT))
                        {
                            break;
                        }

                        BufferC_putc(&rxBuf[p], regVal);
                        rxlen--;
                    }
                }
            } while (1);
        }
    }

    if (xHigherPriorityTaskWoken != pdFALSE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

uint32_t MAX14830_Set_baudrate(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate)
{
    float D = MAX14830_XTAL_FREQ / (16 * (float)baudrate);
    uint32_t DIV = (uint32_t)trunc(D);
    uint32_t FRACT = (uint32_t)round(16 * (D - DIV));
    FRACT = FRACT & 0x0000000F;
    uint32_t DIVMSB = (DIV & 0x00000100) >> 8;
    uint32_t DIVLSB = (DIV & 0x000000FF);

    module_MAX14830_Write(port, MAX14830_REG_DIVLSB, (uint8_t *)&DIVLSB, 1);
    module_MAX14830_Write(port, MAX14830_REG_DIVMSB, (uint8_t *)&DIVMSB, 1);
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, (uint8_t *)&FRACT, 1);
    return ((16 * MAX14830_XTAL_FREQ) / (16 * (16 * DIV + FRACT)));
}

void MAX14830_UART_Write(eMAX18430_ComPort_t port, uint8_t *data, uint16_t len)
{
    sCircularBufferC_t *pBuf = &txBuf[(uint8_t)port];

    for (uint8_t i = 0; i < len; i++)
    {
        BufferC_putc(pBuf, *data++);
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xSpiMutex != NULL)
    {
        while (xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(200UL)) != pdPASS);
    }

    uint32_t xLen = 0;
    while (BufferC_Get_Size(pBuf))
    {
        xLen = BufferC_gets(pBuf, (char *)xfer_data, XFER_DATA_SIZE);
        module_MAX14830_Write(port, MAX14830_THR, xfer_data, xLen);
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xSpiMutex != NULL)
    {
        xSemaphoreGive(xSpiMutex);
    }
}

void MAX14830_UART_Write_direct(eMAX18430_ComPort_t port, uint8_t *data, uint16_t len)
{
    sCircularBufferC_t *pBuf = &txBuf[(uint8_t)port];

    for (uint8_t i = 0; i < len; i++)
    {
        BufferC_putc(&txBuf[(uint8_t)port], *data++);
    }

    uint32_t xLen = 0;
    while (BufferC_Get_Size(pBuf))
    {
        xLen = BufferC_gets(pBuf, (char *)xfer_data, XFER_DATA_SIZE);
        module_MAX14830_Write(port, MAX14830_THR, xfer_data, xLen);
    }
}

uint32_t MAX14830_UART_Read(eMAX18430_ComPort_t port, uint8_t *pData)
{
    uint32_t len = 0;
    sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];
    EventBits_t target_bit = (1u << (uint8_t)port);

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && gEventHandle != NULL)
    {
        /* 1. Wait for ISR notification if buffer is empty */
        if (BufferC_Get_Size(pBuf) == 0)
        {
            xEventGroupClearBits(gEventHandle, target_bit);
            EventBits_t bits = xEventGroupWaitBits(gEventHandle, target_bit, pdTRUE, pdFALSE, pdMS_TO_TICKS(350UL));
            if (bits & target_bit)
            {
                vTaskDelay(pdMS_TO_TICKS(10UL));
            }
        }

        /* 2. Hardware FIFO fallback: If buffer is still empty, poll MAX14830 directly */
        if (BufferC_Get_Size(pBuf) == 0)
        {
            uint8_t fifo_level = 0;
            module_MAX14830_Read(port, MAX14830_REG_RXFIFOLVL, 1, &fifo_level);
            if (fifo_level > 0)
            {
                uint8_t hw_data[32];
                uint8_t read_bytes = (fifo_level > 32) ? 32 : fifo_level;
                module_MAX14830_Read(port, MAX14830_RHR, read_bytes, hw_data);
                for (uint8_t b = 0; b < read_bytes; b++)
                {
                    BufferC_putc(pBuf, hw_data[b]);
                }
            }
        }

        /* 3. Drain circular buffer */
        while (BufferC_Get_Size(pBuf) > 0)
        {
            BufferC_getc(pBuf, (char *)pData);
            pData++;
            len++;
        }
    }
    else
    {
        /* Pre-scheduler bootup fallback */
        am_util_delay_ms(25);
        while (BufferC_Get_Size(pBuf) > 0)
        {
            BufferC_getc(pBuf, (char *)pData);
            pData++;
            len++;
        }
    }

    return len;
}

uint16_t MAX14830_UART_Read_direct(eMAX18430_ComPort_t port, uint8_t *pData)
{
    uint16_t len = 0;
    sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];

    while (BufferC_Get_Size(pBuf) > 0)
    {
        BufferC_getc(pBuf, (char *)pData);
        pData++;
        len++;
    }

    return len;
}

uint32_t MAX14830_UART_Read_bytes_waiting(eMAX18430_ComPort_t port)
{
    return BufferC_Get_Size(&rxBuf[(uint8_t)port]);
}

static void module_MAX14830_Power_On(void)
{
    am_hal_gpio_output_set(AM_BSP_GPIO_S2U_NRESET);
    am_hal_gpio_output_clear(AM_BSP_GPIO_S2U_ON);
}

static void module_MAX14830_Power_Off(void)
{
    am_hal_gpio_output_clear(AM_BSP_GPIO_S2U_NRESET);
    am_hal_gpio_output_set(AM_BSP_GPIO_S2U_ON);
}

static uint32_t module_MAX14830_Read(
                                    eMAX18430_ComPort_t port,
                                    uint8_t reg,
                                    uint16_t rxlen,
                                    uint8_t *pData)
{
    uint32_t aligned_rx[36];
    module_MAX14830_chip_enable();

    uint8_t cmd = (((uint8_t)port & 0x03) << 5) | (reg & 0x1F);

    am_hal_iom_transfer_t transfer =
    {
        .uPeerInfo = {
            .ui32SpiChipSelect  = 0,
            .ui32I2CDevAddr     = 0,
        },
        .ui32InstrLen           = 1,
        .ui32Instr              = cmd,
        .ui32NumBytes           = rxlen,
        .eDirection             = AM_HAL_IOM_RX,
        .pui32TxBuffer          = NULL,
        .pui32RxBuffer          = aligned_rx,
        .bContinue              = false,
        .ui8RepeatCount         = 0,
        .ui8Priority            = 1,
    };

    am_hal_iom_blocking_transfer(pIomHandle, &transfer);
    module_MAX14830_chip_disable();

    memcpy(pData, aligned_rx, rxlen);
    return 0;
}

static void module_MAX14830_Write(
                                eMAX18430_ComPort_t port,
                                uint8_t reg,
                                uint8_t *data,
                                uint16_t len)
{
    uint32_t aligned_tx[36];
    assert(reg <= 0x1F);

    memcpy(aligned_tx, data, len);
    module_MAX14830_chip_enable();

    uint8_t cmd = MAX14830_SPI_WRITE_BIT | (((uint8_t)port & 0x03) << 5) | (reg & 0x1F);

    am_hal_iom_transfer_t transfer =
    {
        .uPeerInfo = {
            .ui32SpiChipSelect  = 0,
            .ui32I2CDevAddr     = 0,
        },
        .ui32InstrLen           = 1,
        .ui32Instr              = cmd,
        .ui32NumBytes           = len,
        .eDirection             = AM_HAL_IOM_TX,
        .pui32TxBuffer          = aligned_tx,
        .pui32RxBuffer          = NULL,
        .bContinue              = false,
        .ui8RepeatCount         = 0,
        .ui8Priority            = 1,
    };

    am_hal_iom_blocking_transfer(pIomHandle, &transfer);
    module_MAX14830_chip_disable();
}