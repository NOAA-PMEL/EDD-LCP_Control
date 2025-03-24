/**
 * @file MAX14830.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-08-11
 * 
 * @co-author Basharat Martin (basharat.martin@noaa.gov)
 *
 */

#include "MAX14830.h"

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Artemis specific files
//
//*****************************************************************************
#include "artemis_spi.h"
#include "artemis_debug.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "am_bsp_pins.h"
#include "buffer_c.h"

//*****************************************************************************
//
//  Register Defines
//
//*****************************************************************************

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
#define MAX14830_REG_XON1           ( 0x14 )
#define MAX14830_REG_XON2           ( 0x15 )
#define MAX14830_REG_XOFF1          ( 0x16 )
#define MAX14830_REG_XOFF2          ( 0x17 )

/** GPIO Registers */
#define MAX14830_REG_GPIOCONFIG     ( 0x18 )
#define MAX14830_REG_GPIODATA       ( 0x19 )

/** Clock Configuration Registers*/
#define MAX14830_REG_PLLCONFIG      ( 0x1A )
#define MAX14830_REG_BRGCONFIG      ( 0x1B )
#define MAX14830_REG_DIVLSB         ( 0x1C )
#define MAX14830_REG_DIVMSB         ( 0x1D )
#define MAX14830_REG_CLKSOURCE      ( 0x1E )

/** Global Registers */
#define MAX14830_REG_GLOBALRQ       ( 0x1F )
#define MAX14830_REG_GLOBLCOMND     ( 0x1F )

/** Synchronization Registers */
#define MAX14830_REG_TXSYNCH        ( 0x20 )
#define MAX14830_REG_SYNCHDELAY1    ( 0x21 )
#define MAX14830_REG_SYNCHDELAY2    ( 0x22 )

/** Timer Registers */
#define MAX14830_REG_TIMER1         ( 0x23 )
#define MAX14830_REG_TIMER2         ( 0x24 )

/** Revision Register */
#define MAX14830_REG_REVID          ( 0x25 )

/** IRQEn Register Bits */
#define MAX14830_IRQ_CTSIEN         ( 1u << 7 )
#define MAX14830_IRQ_RFIFOEMTY      ( 1u << 6 )
#define MAX14830_IRQ_TFIFOEMTY      ( 1u << 5 )
#define MAX14830_IRQ_TFIFOTRG       ( 1u << 4 )
#define MAX14830_IRQ_RFIFOTRG       ( 1u << 3 )
#define MAX14830_IRQ_STS            ( 1u << 2 )
#define MAX14830_IRQ_SPCLCHR        ( 1u << 1 )
#define MAX14830_IRQ_LSRERR         ( 1u )

/** LSRIntEn Register Bits */
#define MAX14830_LSR_INT_NOISEINT   ( 1u << 5 )
#define MAX14830_LSR_INT_RBREAKI    ( 1u << 4 )
#define MAX14830_LSR_INT_FRAMEERR   ( 1u << 3 )
#define MAX14830_LSR_INT_PARITY     ( 1u << 2 )
#define MAX14830_LSR_INT_ROVERR     ( 1u << 1 )
#define MAX14830_LSR_INT_RTIMEOUT   ( 1u )

/** LSR Register Bits */
#define MAX14830_LSR_CTS            ( 1u << 7 )
#define MAX14830_LSR_RXNOISE        ( 1u << 5 )
#define MAX14830_LSR_RXBREAK        ( 1u << 4 )
#define MAX14830_LSR_FRAMEERR       ( 1u << 3 )
#define MAX14830_LSR_RXPARITY       ( 1u << 2 )
#define MAX14830_LSR_RXOVERRUN      ( 1u << 1 )
#define MAX14830_LSR_RTIMEOUT       ( 1u )

/** SpclChrIntEn Register Bits */
#define MAX14830_SCI_MLTDRP         ( 1u << 5 )
#define MAX14830_SCI_BREAK          ( 1u << 4 )
#define MAX14830_SCI_XOFF2          ( 1u << 3 )
#define MAX14830_SCI_XOFF1          ( 1u << 2 )
#define MAX14830_SCI_XON2           ( 1u << 1 )
#define MAX14830_SCE_XON1           ( 1u )

/** STS Register Bits */
#define MAX14830_STS_CLKRDY         ( 1u << 5 )
#define MAX14830_STS_GPI3           ( 1u << 3 )
#define MAX14830_STS_GPI2           ( 1u << 2 )
#define MAX14830_STS_GPI1           ( 1u << 1 )
#define MAX14830_REG_GPI0           ( 1u )

/** MODE1 Register Bits */
#define MAX14830_MODE1_IRQ_SEL      ( 1u << 7 )
#define MAX14830_MODE1_TRNSCV_CTRL  ( 1u << 4 )
#define MAX14830_MODE1_RTS_HIZ      ( 1u << 3 )
#define MAX14830_MODE1_TX_HIZ       ( 1u << 2 )
#define MAX14830_MODE1_TX_DISABL    ( 1u << 1 )
#define MAX14830_MODE1_RX_DISABL    ( 1u )

/** MODE2 Register Bits */
#define MAX14830_MODE2_ECHO_SUPRS       ( 1u << 7 )
#define MAX14830_MODE2_MULTI_DROP       ( 1u << 6 )
#define MAX14830_MODE2_LOOPBACK         ( 1u << 5 )
#define MAX14830_MODE2_SPECIAL_CHR      ( 1u << 4 )
#define MAX14830_MODE2_RX_EMTY_INV      ( 1u << 3 )
#define MAX14830_MODE2_RX_TRIG_INV      ( 1u << 2 )
#define MAX14830_MODE2_FIFO_RST         ( 1u << 1 )
#define MAX14830_MODE2_RST              ( 1u )

/** LCR Register Bits */
#define MAX14830_LCR_RTS                ( 1u << 7 )
#define MAX14830_LCR_TX_BREAK           ( 1u << 6 )
#define MAX14830_LCR_FORCE_PARITY       ( 1u << 5 )
#define MAX14830_LCR_EVEN_PARITY        ( 1u << 4 )
#define MAX14830_LCR_PARITY_EN          ( 1u << 3 )
#define MAX14830_LCR_STOPBITS_1         ( 0u )
#define MAX14830_LCR_STOPBITS_2         ( 1u << 2 )
#define MAX14830_LCR_LENGTH_5           ( 0x00 )
#define MAX14830_LCR_LENGTH_6           ( 0x01 )
#define MAX14830_LCR_LENGTH_7           ( 0x02 )
#define MAX14830_LCR_LENGTH_8           ( 0x03 )

/** HDplxDelay Register Bits */
#define MAX14830_HDD_SETUP_MASK         ( 0x0F << 4 )
#define MAX14830_HDD_HOLD_MASK          ( 0x0F )

/** IrDA Register Bits */
#define MAX14830_IRDA_TX_INV            ( 1u << 5 )
#define MAX14830_IRDA_RX_INV            ( 1u << 4 )
#define MAX14830_IRDA_MIR               ( 1u << 3 )
#define MAX14830_IRDA_RTS_INV           ( 1u << 2 )
#define MAX14830_IRDA_SIR               ( 1u << 1 )
#define MAX14830_IRDA_IRDA_EN           ( 1u )

/** FlowLvl Register Bits */
#define MAX14830_IRDA_RESUME_MASK       ( 0x0F << 4 )
#define MAX14830_IRDA_HALT_MASK         ( 0x0F )

/** FIFOTrigLvl Register Bits */
#define MAX14830_IRDA_RX_TRIG           ( 0x0F << 4 )
#define MAX14830_IRDA_TX_TRIG           ( 0x0F )

/** FlowCtrl Register Bits */
#define MAX14830_FC_TX_XON1_XOFF1       ( 1u << 7 )
#define MAX14830_FC_TX_XON2_XOFF2       ( 1u << 6 )
#define MAX14830_FC_RX_XON1_XOFF1       ( 1u << 5 )
#define MAX14830_FC_RX_XON2_XOFF2       ( 1u << 4 )
#define MAX14830_FC_SW_FLOW_EN          ( 1u << 3 )
#define MAX14830_FC_SW_GPI_ADDR         ( 1u << 2 )
#define MAX14830_FC_AUTO_CTS            ( 1u << 1 )
#define MAX14830_FC_AUTO_RTS            ( 1u )

/* Flow control trigger level register masks */
#define MAX14830_FLOWLVL_HALT_MASK      (0x000f) /* Flow control halt level */
#define MAX14830_FLOWLVL_RES_MASK       (0x00f0) /* Flow control resume level */
#define MAX14830_FLOWLVL_HALT(words)    ((words / 8) & 0x0f)
#define MAX14830_FLOWLVL_RES(words)     (((words / 8) & 0x0f) << 4)

/* FIFO interrupt trigger level register masks */
#define MAX14830_FIFOTRIGLVL_TX_MASK    (0x0f) /* TX FIFO trigger level */
#define MAX14830_FIFOTRIGLVL_RX_MASK    (0xf0) /* RX FIFO trigger level */
#define MAX14830_FIFOTRIGLVL_TX(words)  ((words / 8) & 0x0f)
#define MAX14830_FIFOTRIGLVL_RX(words)  (((words / 8) & 0x0f) << 4)

/** PLLConfig Register Bits */
#define MAX14830_PLL_PREDIV(x)          ( x & 0x1F )
#define MAX14830_PLL_MULT_6             ( 0u  )
#define MAX14830_PLL_MULT_48            ( 1u << 6 )
#define MAX14830_PLL_MULT_96            ( 1u << 7 )
#define MAX14830_PLL_MULT_144           ( 3u << 6 )

/** CLKSource Register Bits */
#define MAX14830_CLK_PLL_BYPASS         ( 1u << 3 )
#define MAX14830_CLK_PLL_EN             ( 1u << 2 )
#define MAX14830_CLK_CRYSTAL_EN         ( 1u << 1 )

/** BRGConfig Register Bits */
#define MAX14830_BRG_CLK_DISABLE        ( 1u << 6 )
#define MAX14830_BRG_4X_MODE            ( 1u << 5 )
#define MAX14830_BRG_2X_MODE            ( 1u << 4 )

/** Write Bit */
#define MAX14830_SPI_WRITE_BIT          ( 1u << 7)
#define MAX14830_SPI_READ_BIT           ( 0x7F )

/** System Settings */
#define MAX14830_XTAL_FREQ              ( 4000000u )

#define MAX14830_NUM_SERIAL_PORTS       ( 4 )
#define MAX14830_WRITE_TASK_PRIORITY    ( 5 )
#define STACK_SIZE                      ( 128 )


//*****************************************************************************
//
// FreeRTOS variables, tasks and semaphores
//
//*****************************************************************************
static SemaphoreHandle_t xSpiMutex = NULL;
static volatile bool rx_bytes = false;
static EventGroupHandle_t gEventHandle = NULL;

//*****************************************************************************
//
// Ambiq IOM static variables
//
//*****************************************************************************
static void *pIomHandle;
static am_hal_iom_config_t IomConfig = {0};
const uint8_t CTSIEn        = (1u << 7);
const uint8_t RFifoEmtyIEn  = (1u << 6);
const uint8_t TFifoEmtyIEn  = (1u << 5);

volatile bool xfer_complete = false;
volatile uint32_t txn_stat = 0;

#define XFER_DATA_SIZE      ( 64 )
static uint8_t xfer_data[XFER_DATA_SIZE];

//*****************************************************************************
//
// Circular buffer defines
//
//*****************************************************************************
sCircularBufferC_t txBuf[MAX14830_NUM_SERIAL_PORTS];
sCircularBufferC_t rxBuf[MAX14830_NUM_SERIAL_PORTS];

//
//void xfer_complete_callback(void *pCallbackCtxt, uint32_t transactionStatus){
//    (void)pCallbackCtxt;
//    xfer_complete = true;
//    txn_stat = transactionStatus;
//}


//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
static void module_MAX14830_conf(void);
static void module_MAX14830_chip_enable(void);
static void module_MAX14830_chip_disable(void);
static void module_MAX14830_Power_On(void);
static void module_MAX14830_Power_Off(void);
static uint8_t module_MAX14830_FastRead(void);
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

/* RTOS Tasks */
void module_MAX14830_Write_Task(void *pvParameters);
void module_MAX14830_Read_Task(void  *pvParameters);
void module_MAX14830_Handle_IRQ(void);
void module_MAX14830_Handle_IRQ_RTOS(void);
void module_MAX14830_Handle_Rx(uint8_t port);
void module_MAX14830_RTOS_ISR(uint8_t irq);

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
/**
 * @brief Initialize the MAX14830 IC
 * 
 */
bool MAX14830_initialize(void)
{
    IomConfig.eInterfaceMode       = AM_HAL_IOM_SPI_MODE;
    IomConfig.ui32ClockFreq        = AM_HAL_IOM_100KHZ;
    IomConfig.eSpiMode             = AM_HAL_IOM_SPI_MODE_0;
    IomConfig.pNBTxnBuf            = NULL;
    IomConfig.ui32NBTxnBufLength   = 0;

    /** Initialize the SPI Configurations */
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_initialize(3, &pIomHandle));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_configure(pIomHandle, &IomConfig));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_enable(pIomHandle));

    /* SPI3 module */
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCK, g_AM_BSP_GPIO_IOM3_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MISO, g_AM_BSP_GPIO_IOM3_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MOSI, g_AM_BSP_GPIO_IOM3_MOSI));

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_CS, g_AM_BSP_GPIO_IOM3_CS));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NIRQ, g_AM_BSP_GPIO_S2U_NIRQ));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NRESET, g_AM_BSP_GPIO_S2U_NRESET));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_ON, g_AM_BSP_GPIO_S2U_ON));

    /*Enable interrupt for IRQ*/
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_S2U_NIRQ));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_S2U_NIRQ));

    NVIC_SetPriority(GPIO_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(GPIO_IRQn);
    am_hal_interrupt_master_enable();

    /* Turn on */
    module_MAX14830_Power_On();
    /* MAX14830 register configuration */
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
    ///* Wait for IRQ to goes high */
    //uint8_t ret = 0;
    //do {
    //    am_hal_gpio_state_read(AM_BSP_GPIO_S2U_NIRQ, AM_HAL_GPIO_INPUT_READ, &ret);
    //}while(ret==0);
    //am_hal_gpio_state_read(AM_BSP_GPIO_S2U_NIRQ, AM_HAL_GPIO_INPUT_READ, &ret);
    //ARTEMIS_DEBUG_PRINTF("MAX14830: IRQ = %u, ready to be programmed\n", ret);

    /** Set the MAX14830 Clock Source to enable Crystal , this is done only by port0 */
    uint8_t regVal = MAX14830_CLK_CRYSTAL_EN | MAX14830_CLK_PLL_BYPASS;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_CLKSOURCE, &regVal, 1);

    /* enable IRQ bit and automatic transceiver direction */
    regVal = MAX14830_MODE1_IRQ_SEL | MAX14830_MODE1_TRNSCV_CTRL | 0x20;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE1, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_MODE1, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_MODE1, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_MODE1, &regVal, 1);

    /* enable first received byte through IRQ interrupt, and reset FIFO*/
    regVal = MAX14830_MODE2_RX_EMTY_INV | MAX14830_MODE2_FIFO_RST;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE2, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_MODE2, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_MODE2, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_MODE2, &regVal, 1);

    //module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_MODE2, 1, &regVal);
    //ARTEMIS_DEBUG_PRINTF("MAX14830: FIFO REG = 0x%02X\n", regVal);

    /* Update FIFO */
    module_MAX14830_RegUpdate(MAX14830_COM_PORT0, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);
    module_MAX14830_RegUpdate(MAX14830_COM_PORT1, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);
    module_MAX14830_RegUpdate(MAX14830_COM_PORT2, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);
    module_MAX14830_RegUpdate(MAX14830_COM_PORT3, MAX14830_REG_MODE2, MAX14830_MODE2_FIFO_RST, 0);

    /* Echo supress for RS-485 half-duplex*/
    regVal = MAX14830_MODE2_ECHO_SUPRS | MAX14830_MODE2_RX_EMTY_INV;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE2, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_MODE2, &regVal, 1);

    /* Configure flow control levels */
    /* Flow control halt level 96, resume level 48 , from linux max310x */
    regVal = MAX14830_FLOWLVL_HALT(96) | MAX14830_FLOWLVL_RES(48);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_FLOWLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_FLOWLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_FLOWLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_FLOWLVL, &regVal, 1);

    /* Set 8N1 */
    regVal = MAX14830_LCR_LENGTH_8 | MAX14830_LCR_STOPBITS_1;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_LCR, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_LCR, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_LCR, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_LCR, &regVal, 1);

    /* RX TX fifo trigger level */
    regVal = MAX14830_FIFOTRIGLVL_RX(64) | MAX14830_FIFOTRIGLVL_TX(64);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_FIFOTRGLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT1, MAX14830_REG_FIFOTRGLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT2, MAX14830_REG_FIFOTRGLVL, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_FIFOTRGLVL, &regVal, 1);

    /* Enable power to the port 0/1, 2/3 , apparently they are sharing the pins*/
    regVal = MAX14830_GPIO_PIN_TO_BIT(2);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_GPIOCONFIG, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_GPIODATA, &regVal, 1);
    //regVal = MAX14830_GPIO_PIN_TO_BIT(9);
    //module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_GPIOCONFIG, &regVal, 1);
    //module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_GPIODATA, &regVal, 1);

    /* Clear ISR register for existing used ports, 0 and 3 */
    module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_ISR, 1, &regVal);
    module_MAX14830_Read(MAX14830_COM_PORT3, MAX14830_REG_ISR, 1, &regVal);
    //ARTEMIS_DEBUG_PRINTF("MAX14830 INIT: ISR STATUS REG = 0x%02X\n", regVal);
    //module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_ISR, 1, &regVal);
    //ARTEMIS_DEBUG_PRINTF("MAX14830 INIT: ISR STATUS REG = 0x%02X\n", regVal);

    /* Enable Interrupt only to receive first byte */
    regVal = MAX14830_IRQ_RFIFOEMTY ; //| MAX14830_IRQ_TFIFOEMTY | MAX14830_IRQ_CTSIEN;
    module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_IRQEN, &regVal, 1);
    module_MAX14830_Write(MAX14830_COM_PORT3, MAX14830_REG_IRQEN, &regVal, 1);

    //module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_GPIOCONFIG, 1, &regVal);
    //ARTEMIS_DEBUG_PRINTF("MAX14830: REG = 0x%02X\n", regVal);
    //module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_GPIODATA, 1, &regVal);
    //ARTEMIS_DEBUG_PRINTF("MAX14830: REG = 0x%02X\n", regVal);
    //module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_CLKSOURCE, 1, &regVal);
    //ARTEMIS_DEBUG_PRINTF("MAX14830: REG = 0x%02X\n", regVal);


    /* disable all the UART clocks initially */
    for(uint8_t i=0; i<4; i++)
    {
        module_MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, 1, &regVal);
        regVal |= MAX14830_BRG_CLK_DISABLE;
        module_MAX14830_Write((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, &regVal, 1);
    }
}

void MAX14830_disable_direct(void)
{
    /* Turn off */
    module_MAX14830_Power_Off();
    am_hal_iom_disable(pIomHandle);
}

void MAX14830_enable_direct(void)
{
    /* turn on */
    am_hal_iom_enable(pIomHandle);
    module_MAX14830_Power_On();
    /* MAX14830 register configuration */
    module_MAX14830_conf();
}

void MAX14830_disable(void)
{
    /* Turn off and disable the iom */
    module_MAX14830_Power_Off();
    am_hal_iom_disable(pIomHandle);
}

void MAX14830_enable(void)
{
    /* turn on */
    am_hal_iom_enable(pIomHandle);
    module_MAX14830_Power_On();
    /* MAX14830 register configuration */
    module_MAX14830_conf();

    /* create global event handler for eventGroup */
    if (gEventHandle != NULL)
    {
        ARTEMIS_DEBUG_PRINTF("MAX14830 :: gEventHandle, alive\n");
    }
    else
    {
        gEventHandle = xEventGroupCreate();
        if( gEventHandle != NULL )
        {
            ARTEMIS_DEBUG_PRINTF("MAX14830 :: gEventHandle, created\n");
        }
    }

    /* create SpiMutex semaphore for tasks */
    if (xSpiMutex != NULL)
    {
        ARTEMIS_DEBUG_PRINTF("MAX14830 :: xSpiMutex, alive\n");
    }
    else
    {
        xSpiMutex = xSemaphoreCreateMutex();
        if( xSpiMutex != NULL )
        {
            ARTEMIS_DEBUG_PRINTF("MAX14830 :: xSpiMutex, created\n");
        }
    }
}

static void module_MAX14830_RegUpdate(eMAX18430_ComPort_t port,
                        uint8_t reg, uint8_t bit, uint8_t val)
{
    uint8_t curVal = 0x00;
    module_MAX14830_Read(port, reg, 1, &curVal);
    uint8_t update = curVal&bit;
    if ( update == val )
    {
        /* no need to change */
        return;
    }
    else
    {
        /* toggle the bit, and write */
        curVal ^= bit;
	    module_MAX14830_Write(port, reg, &curVal, 1);
    }
}

void am_gpio_isr(void)
{
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_S2U_NIRQ));
    //uint8_t irq = module_MAX14830_FastRead();
    //module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_GLOBALRQ, 1, &irq);
    //ARTEMIS_DEBUG_PRINTF("\nGPIO_IRQ FAST IRQ=0x%02X\n", irq);
    //ARTEMIS_DEBUG_PRINTF("\nInterrupt\n");
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

/**
 * @brief Enable the MAX14830 Port
 * 
 * @param port Port to enable
 */
void MAX14830_port_enable(eMAX18430_ComPort_t port)
{
    /** If the device is off, turn it on */
    uint32_t state = 0;
    am_hal_gpio_state_read(AM_BSP_GPIO_S2U_ON, AM_HAL_GPIO_OUTPUT_READ, &state);

    if(state == 1)
    {
        MAX14830_enable();
        ARTEMIS_DEBUG_PRINTF("MAX14830 :: Power ON\n");
    }

    /** Enable the Port Clock */
    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg &= ~MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

    ///** Restart the Read Task if currently suspended */
    //switch(eTaskGetState(data_read_int_handle))
    //{
    //    case eReady:
    //        break;
    //    case eRunning:
    //    case eBlocked:
    //        break;
    //    case eSuspended:
    //        vTaskResume(data_read_int_handle);
    //        break;
    //    case eDeleted:
    //        break;
    //    default:
    //        break;
    //}
    //vTaskResume(data_read_int_handle);
}

void MAX14830_port_enable_direct(eMAX18430_ComPort_t port)
{
    /** If the device is off, turn it on */
    uint32_t state = 0;
    am_hal_gpio_state_read(AM_BSP_GPIO_S2U_ON, AM_HAL_GPIO_OUTPUT_READ, &state);
    if(state == 1)
    {
        MAX14830_enable_direct();
        ARTEMIS_DEBUG_PRINTF("MAX14830 :: Direct Power ON\n");
    }

    /** Enable the Port Clock */
    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg &= ~MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);
}

void MAX14830_port_disable_direct(eMAX18430_ComPort_t port)
{
    /** Disable the Port Clock */
    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg |= MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

    /** Check to see if all ports are inactive */
    uint8_t cnt = 0;
    for(uint8_t i=0; i<4; i++)
    {
        reg = 0;
        module_MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, 1, &reg);
        if( (reg & MAX14830_BRG_CLK_DISABLE) > 0)
        {
            cnt++;
        }
    }

    /** If all the ports are deactivated, shut down the power for lower power ops */
    if(cnt == MAX14830_NUM_SERIAL_PORTS)
    {
        MAX14830_disable_direct();
        ARTEMIS_DEBUG_PRINTF("MAX14830 :: Direct Power OFF\n");
    }
}

/**
 * @brief Disable selected MAX14830 Port
 * 
 * @param port Port to disable
 */
void MAX14830_port_disable(eMAX18430_ComPort_t port)
{
    /** Disable the Port Clock */
    uint8_t reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    reg |= MAX14830_BRG_CLK_DISABLE;
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

    char discard;
    sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];

    /* read MAX14830 remaining bytes before turning it off */
    while( (BufferC_Get_Size(pBuf) > 0) )
    {
        BufferC_getc(pBuf, &discard);
        //ARTEMIS_DEBUG_PRINTF("%c", discard);
    }
    //ARTEMIS_DEBUG_PRINTF("\n\n");

    /** Check to see if all ports are inactive */
    uint8_t cnt = 0;
    for(uint8_t i=0; i<4; i++)
    {
        reg = 0;
        module_MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_REG_BRGCONFIG, 1, &reg);
        if( (reg & MAX14830_BRG_CLK_DISABLE) > 0)
        {
            cnt++;
        }
    }

    /** If all the ports are deactivated, shut down the power for lower power ops */
    if(cnt == MAX14830_NUM_SERIAL_PORTS)
    {
        MAX14830_disable();
        ARTEMIS_DEBUG_PRINTF("MAX14830 :: Power OFF\n");
    }
}

/**
 * @brief Handle IRQ of Max14830
 * 
 * @param no
 */

void module_MAX14830_Handle_IRQ()
{
    /** Fast read does some weird stuff when it comes to reading K9lx sensor RS485,
        so read the global register */

    uint8_t rxData[128] = {0};
    uint8_t i=0;
    uint8_t irq, port;
    module_MAX14830_Read(MAX14830_COM_PORT0, MAX14830_REG_GLOBALRQ, 1, &irq);

    /** Walk through IRQ to find out which port has the data*/
    for (uint8_t i=0; i<4; i++)
    {
        if ( (~irq&0x0F) & (1u << i))
        {
            port = i;
        }
    }

    if (port<0 && port>3)
    {
        ARTEMIS_DEBUG_PRINTF("MAX14830 :: PORT ERROR, port=%i\n", port);
        return;
    }

    /* looks fine here */
    uint8_t rxlen, isr_status, lsr_status, regVal;
    do
    {
        module_MAX14830_Read((eMAX18430_ComPort_t)port, MAX14830_REG_ISR, 1, &isr_status);
        module_MAX14830_Read((eMAX18430_ComPort_t)port, MAX14830_REG_RXFIFOLVL, 1, &rxlen);
        //ARTEMIS_DEBUG_PRINTF("\nrxlen = %u\n", rxlen);

        if (!isr_status && !rxlen)
        {
            //ARTEMIS_DEBUG_PRINTF("\nbreaking .. rxlen = %u\n", rxlen);
            rx_bytes = true;
            if ( gEventHandle != NULL )
            {
                BaseType_t xHigherPriorityTaskWoken, xResult;
                xHigherPriorityTaskWoken = pdFALSE;
                xResult = xEventGroupSetBitsFromISR(gEventHandle, (uint8_t)port + 0x01, &xHigherPriorityTaskWoken);
                if (xResult != pdFAIL)
                {
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            }
            break;
        }

        if (rxlen > 0)
        {
            /* Receive bytes */
            while (rxlen > 0)
            {
                module_MAX14830_Read((eMAX18430_ComPort_t)port, MAX14830_RHR, 1, &regVal);
                module_MAX14830_Read((eMAX18430_ComPort_t)port, MAX14830_REG_LSR, 1, &lsr_status);

                /* if overrun, then collect all the bytes at once */
                if (lsr_status & MAX14830_LSR_RXOVERRUN)
                {
                    ARTEMIS_DEBUG_PRINTF("MAX14830 :: OVERFlow happened\n");
                    module_MAX14830_Read((eMAX18430_ComPort_t)port, MAX14830_REG_RXFIFOLVL, 1, &rxlen);
                    module_MAX14830_Read((eMAX18430_ComPort_t)port, MAX14830_RHR, rxlen, rxData);
                    while (rxlen > 0)
                    {
                        BufferC_putc(&rxBuf[(uint8_t)port], rxData[i]);
                        i++;
                        //ARTEMIS_DEBUG_PRINTF("%c", *rxData);
                        rxlen--;
                    }
                    i=0;
                    break;
                }

                /* check data error on each received byte , how to handle, read again ? */
                if (lsr_status & (  MAX14830_LSR_RXNOISE    |
                                    MAX14830_LSR_RXBREAK    |
                                    MAX14830_LSR_RXPARITY   |
                                    MAX14830_LSR_FRAMEERR   |
                                    MAX14830_LSR_RTIMEOUT   ))
                {
                    ARTEMIS_DEBUG_PRINTF("MAX14830 :: BYTE ERROR, lsr_status = 0x%02X\n", lsr_status);
                    break;
                }

                BufferC_putc(&rxBuf[(uint8_t)port], regVal);
                //ARTEMIS_DEBUG_PRINTF("%c", regVal);
                //ARTEMIS_DEBUG_PRINTF("\n0x%02X\n", regVal);
                rxlen--;
            }
        }
    } while(1);
}

/**
 * @brief RTOS Write Task
 * 
 * @param pvParameters 
 */
void module_MAX14830_Write_Task(void  *pvParameters)
{
    eMAX18430_ComPort_t port = (eMAX18430_ComPort_t) pvParameters;
    sCircularBufferC_t *pBuf = &txBuf[(uint8_t)port];

    /* write spi burst*/
    uint8_t txlen = 0;
    uint8_t sData[32] = {0};

    while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 200UL )) != pdPASS);
    txlen = BufferC_gets(pBuf, (char*)sData, 32);
    module_MAX14830_Write(port, MAX14830_THR, sData, txlen);
    xSemaphoreGive(xSpiMutex);
    //ARTEMIS_DEBUG_PRINTF("\nWRITE TASK is deleted port (%u)\n", (uint8_t) port);
    vTaskDelete(NULL);
}

/**
 * D = fRef / (16 * BaudRate)
 * DIV = TRUNC(D)
 * FRACT, is a 4-bit nibble, which is programmed into BRGConfig[3:0]
 * FRACT = ROUND(16 x (D-DIV)).
 * 
 * 
 */
uint32_t MAX14830_Set_baudrate(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate )
{
    float D = MAX14830_XTAL_FREQ / (16 * (float)baudrate );
    uint32_t DIV = (uint32_t)trunc(D) ;
    uint32_t FRACT = (uint32_t) round(16 * (D - DIV));
    FRACT = FRACT & 0x0000000F;
    uint32_t DIVMSB = (DIV & 0x00000100) >> 8;
    uint32_t DIVLSB = (DIV & 0x000000FF);

    /** Send the BRGConfig */
    module_MAX14830_Write(port, MAX14830_REG_DIVLSB, (uint8_t *)&DIVLSB, 1);
    module_MAX14830_Write(port, MAX14830_REG_DIVMSB, (uint8_t *)&DIVMSB, 1);
    module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, (uint8_t *)&FRACT, 1);
    return ((16*MAX14830_XTAL_FREQ) / (16 * (16*DIV + FRACT)) );
}

/**
 * @brief Write UART RTOS
 * 
 * @param port Port to write to
 * @param data Pointer to data array to write
 * @param len Length of data array
 */
void MAX14830_UART_Write(eMAX18430_ComPort_t port, uint8_t *data, uint16_t len)
{
    uint8_t taskDesc[configMAX_TASK_NAME_LEN];
    sprintf((char*)taskDesc, "S2U Write CH%u", (uint8_t) port);

    /** Put data into circular buffer */
    for(uint8_t i=0; i < len; i++)
    {
        BufferC_putc(&txBuf[(uint8_t) port], *data++);
    }

    /** Ensure the port is enabled */
    //MAX14830_port_enable(port);

    /** Create Task */
    xTaskCreate(module_MAX14830_Write_Task,
                (char*)taskDesc,
                STACK_SIZE,
                (void*) port,
                tskIDLE_PRIORITY + MAX14830_WRITE_TASK_PRIORITY,
                NULL);

    //ARTEMIS_DEBUG_PRINTF("MAX14830 task created = port (%u)\n", (uint8_t) port);
}

/**
 * @brief UART Write (non-RTOS)
 * 
 * @param port Port to write to
 * @param data Pointer to data array
 * @param len Length of the data array
 */
void MAX14830_UART_Write_direct(eMAX18430_ComPort_t port, uint8_t *data, uint16_t len)
{
    sCircularBufferC_t *pBuf = &txBuf[(uint8_t)port];

    /** Put data into circular buffer */
    for(uint8_t i=0; i < len; i++)
    {
        BufferC_putc(&txBuf[(uint8_t) port], *data++);
    }
    /** Ensure the port is enabled */
    //MAX14830_port_enable_direct(port);

    /** Transfer bytes in one go*/
    uint32_t xLen = 0;
    while(BufferC_Get_Size(pBuf))
    {
        xLen = BufferC_gets(pBuf, (char*)xfer_data, XFER_DATA_SIZE);
        module_MAX14830_Write(port, MAX14830_THR, xfer_data, xLen);
    }
}

/**
 * @brief UART Read - RTOS
 * 
 * @param port Port to read from
 * @param pData Pointer to data array
 * @return uint32_t Number of characters read
 */
uint32_t MAX14830_UART_Read(eMAX18430_ComPort_t port, uint8_t *pData)
{
    uint32_t len=0;
    sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];

    while (1)
    {
        uint8_t Port = xEventGroupWaitBits(gEventHandle, 0x07, pdTRUE, pdFALSE, xDelay2000ms);
        Port = (Port-1);
        //ARTEMIS_DEBUG_PRINTF("Port (%u)\n", Port);
        if (Port == (uint8_t)port)
        {
            vTaskDelay(pdMS_TO_TICKS(10UL));
            //while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 500UL )) != pdPASS);
            while( (BufferC_Get_Size(pBuf) > 0) )
            {
                BufferC_getc(pBuf, (char*)pData);
                pData++;
                len++;
            }
            //xSemaphoreGive(xSpiMutex);
            break;
        }
    }
    return len;
}

/**
 * @brief UART Read (non-RTOS)
 * 
 * @param port Port to read from
 * @param pData Pointer to data array
 * @return len, length of the data
 */
uint16_t MAX14830_UART_Read_direct(eMAX18430_ComPort_t port, uint8_t *pData)
{
    uint16_t len=0;
    sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];

    /** wait for an interrupt and wait for 10ms max,
        based on 48MHz cpu clock */

    uint32_t cnt = 0;
    while(!rx_bytes && cnt++ < 500000);

    while( (BufferC_Get_Size(pBuf) > 0) )
    {
        BufferC_getc(pBuf, (char*)pData);
        pData++;
        len++;
    }

    rx_bytes = false;
    return len;
}

/**
 * @brief Find the number of bytes in waiting
 * 
 * @param port Port to check
 * @return uint32_t Number of bytes in waiting
 */
uint32_t MAX14830_UART_Read_bytes_waiting(eMAX18430_ComPort_t port)
{
    uint32_t len = BufferC_Get_Size(&rxBuf[(uint8_t)port]);
    return len;
}

//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************
/**
 * @brief Power the MAX14830 ON
 * 
 */
static void module_MAX14830_Power_On(void)
{
    //am_hal_iom_enable(&pIomHandle);
    am_hal_gpio_output_set(AM_BSP_GPIO_S2U_NRESET);
    am_hal_gpio_output_clear(AM_BSP_GPIO_S2U_ON);
}

/**
 * @brief Power the MAX14830 OFF
 * 
 */
static void module_MAX14830_Power_Off(void)
{
    //am_hal_iom_disable(&pIomHandle);
    am_hal_gpio_output_clear(AM_BSP_GPIO_S2U_NRESET);
    am_hal_gpio_output_set(AM_BSP_GPIO_S2U_ON);
}

/**
 * @brief Fast read
 * 
 * @return uint8_t
 */
static uint8_t module_MAX14830_FastRead(void)
{
    uint32_t write_byte = 0x00;
    uint32_t read_byte = 0x00;
    am_hal_iom_transfer_t xfer;

    xfer.uPeerInfo.ui32SpiChipSelect = 0;
    xfer.ui32InstrLen = 0;
    xfer.ui32Instr = 0;
    xfer.ui32NumBytes = 1;
    xfer.eDirection = AM_HAL_IOM_FULLDUPLEX;
    xfer.pui32TxBuffer = &write_byte;
    xfer.pui32RxBuffer = &read_byte;
    xfer.bContinue = false;
    xfer.ui8RepeatCount = 0;
    xfer.ui8Priority = 1;
    xfer.ui32PauseCondition = 0;
    xfer.ui32StatusSetClr = 0;

    /** Chip Select */
    module_MAX14830_chip_enable();
    am_hal_iom_spi_blocking_fullduplex(pIomHandle, &xfer);
    /** Chip Deselect */
    module_MAX14830_chip_disable();

    //ARTEMIS_DEBUG_PRINTF("FAST READ: status = %u, rb = %u\n", status, read_byte);
    return read_byte;
}

/**
 * @brief MAX14830 Read
 * 
 * @param port  Port to read from
 * @param ui32Instr Instruction 
 * @param ui32NumBytes Number of Bytes
 * @param pData Pointer to data array
 * @return uint32_t Number of bytes read
 */
static uint32_t module_MAX14830_Read(
                                    eMAX18430_ComPort_t port,
                                    uint8_t reg,
                                    uint16_t rxlen,
                                    uint8_t *pData)
{
    /** Chip select */
    module_MAX14830_chip_enable();

    //ARTEMIS_DEBUG_PRINTF("\nREAD port=%d\n", port);
    /** Prep read cmd byte */
    uint8_t cmd = MAX14830_SPI_READ_BIT & (port << 5);
    cmd |= reg;


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
        .pui32RxBuffer          = (uint32_t *)pData,
        .bContinue              = false,
        .ui8RepeatCount         = 0,
        .ui8Priority            = 1,
    };

    /** Execute the transction over IOM */
    am_hal_iom_blocking_transfer(pIomHandle, &transfer);
    /** Chip Deselect */
    module_MAX14830_chip_disable();

    return 0;
}

/**
 * @brief Write to port
 * 
 * @param port Port to write to
 * @param reg Register to write
 * @param data Pointer to data array
 * @param len Length of data array
 */
static void module_MAX14830_Write(
                                eMAX18430_ComPort_t port,
                                uint8_t reg,
                                uint8_t *data,
                                uint16_t len)
{
    assert(reg <= 0x1F);
    /** Chip Select */
    module_MAX14830_chip_enable();

    /** Prep Write Byte */
    uint8_t cmd = MAX14830_SPI_WRITE_BIT | (port << 5);
    cmd |= reg;

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
        .pui32TxBuffer          = (uint32_t*)data,
        .pui32RxBuffer          = NULL,
        .bContinue              = false,
        .ui8RepeatCount         = 0,
        .ui8Priority            = 1,
    };

    am_hal_iom_blocking_transfer(pIomHandle, &transfer);
    /** After transfer, turn Chip Select off */
    module_MAX14830_chip_disable();
}
