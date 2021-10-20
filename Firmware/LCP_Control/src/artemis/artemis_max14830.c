#include "artemis_max14830.h"

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
#include "task.h"
#include "event_groups.h"
#include "semphr.h"


//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "am_bsp_pins.h"
#include "buffer_c.h"

//*****************************************************************************
//
//  Macros
//
//*****************************************************************************
//#define BITBANG_SINCE_AMBIQ_DOESNT_WANT_TO_HELP_ME  ( 1 )
#define MAX14830_IOM_MODULE     ( 3 )


/** System Settings */
#define MAX14830_XTAL_FREQ              ( 4000000u )

//*****************************************************************************
//
//  Register Defines
//
//*****************************************************************************
/** FIFO Data Registers */
#define MAX14830_RHR   ( 0x00 )
#define MAX14830_THR  ( 0x00 )

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
#define MAX14830_IRQ_CTSIEN       ( 1u << 7 )
#define MAX14830_IRQ_RFIFOEMTY    ( 1u << 6 )
#define MAX14830_IRQ_TFIFOEMTY    ( 1u << 5 )
#define MAX14830_IRQ_TFIFOTRG     ( 1u << 4 )
#define MAX14830_IRQ_RFIFOTRG     ( 1u << 3 )
#define MAX14830_IRQ_STS          ( 1u << 2 )
#define MAX14830_IRQ_SPCLCHR      ( 1u << 1 )
#define MAX14830_IRQ_LSRERR       ( 1u )

/** LSRIntEn Register Bits */
#define MAX14830_LSR_INT_NOISEINT     ( 1u << 5 )
#define MAX14830_LSR_INT_RBREAKI      ( 1u << 4 )
#define MAX14830_LSR_INT_FRAMEERR     ( 1u << 3 )
#define MAX14830_LSR_INT_PARITY       ( 1u << 2 )
#define MAX14830_LSR_INT_ROVERR       ( 1u << 1 )
#define MAX14830_LSR_INT_RTIMEOUT     ( 1u )

/** LSR Register Bits */
#define MAX14830_LSR_CTS              ( 1u << 7 )
#define MAX14830_LSR_RXNOISE          ( 1u << 5 )
#define MAX14830_LSR_RXBREAK          ( 1u << 4 )
#define MAX14830_LSR_FRAMEERR         ( 1u << 3 )
#define MAX14830_LSR_RXPARITY         ( 1u << 2 )
#define MAX14830_LSR_RXOVERRUN        ( 1u << 1 )
#define MAX14830_LSR_RTIMEOUT         ( 1u )

/** SpclChrIntEn Register Bits */
#define MAX14830_SCI_MLTDRP          ( 1u << 5 )
#define MAX14830_SCI_BREAK           ( 1u << 4 )
#define MAX14830_SCI_XOFF2           ( 1u << 3 )
#define MAX14830_SCI_XOFF1           ( 1u << 2 )
#define MAX14830_SCI_XON2            ( 1u << 1 )
#define MAX14830_SCE_XON1            ( 1u )

/** STS Register Bits */
#define MAX14830_STS_CLKRDY          ( 1u << 5 )
#define MAX14830_STS_GPI3             ( 1u << 3 )
#define MAX14830_STS_GPI2             ( 1u << 2 )
#define MAX14830_STS_GPI1             ( 1u << 1 )
#define MAX14830_REG_GPI0             ( 1u )

/** MODE1 Register Bits */
#define MAX14830_MODE1_IRQ_SEL         ( 1u << 7 )
#define MAX14830_MODE1_TRNSCV_CTRL     ( 1u << 4 )
#define MAX14830_MODE1_RTS_HIZ          ( 1u << 3 )
#define MAX14830_MODE1_TX_HIZ           ( 1u << 2 )
#define MAX14830_MODE1_TX_DISABL        ( 1u << 1 )
#define MAX14830_MODE1_RX_DISABL        ( 1u )

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
#define MAX14830_HDD_HOLD_MASK           ( 0x0F )

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
#define MAX14830_FIFOTRIGLVL_RX_TRIG    ( 0x0F << 4 )
#define MAX14830_FIFOTRIGLVL_TX_TRIG    ( 0x0F )

/** FlowCtrl Register Bits */
#define MAX14830_FC_TX_XON1_XOFF1       ( 1u << 7 )
#define MAX14830_FC_TX_XON2_XOFF2       ( 1u << 6 )
#define MAX14830_FC_RX_XON1_XOFF1       ( 1u << 5 )
#define MAX14830_FC_RX_XON2_XOFF2       ( 1u << 4 )
#define MAX14830_FC_SW_FLOW_EN          ( 1u << 3 )
#define MAX14830_FC_SW_GPI_ADDR         ( 1u << 2 )
#define MAX14830_FC_AUTO_CTS            ( 1u << 1 )
#define MAX14830_FC_AUTO_RTS            ( 1u )

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


/* Flow control trigger level register masks */
#define MAX14830_FLOWLVL_HALT_MASK       (0x000f) /* Flow control halt level */
#define MAX14830_FLOWLVL_RES_MASK        (0x00f0) /* Flow control resume level */
#define MAX14830_FLOWLVL_HALT(words)     ((words / 8) & 0x0f)
#define MAX14830_FLOWLVL_RES(words)      (((words / 8) & 0x0f) << 4)

/* FIFO interrupt trigger level register masks */
#define MAX14830_FIFOTRIGLVL_TX_MASK     (0x0f) /* TX FIFO trigger level */
#define MAX14830_FIFOTRIGLVL_RX_MASK     (0xf0) /* RX FIFO trigger level */
#define MAX14830_FIFOTRIGLVL_TX(words)   ((words / 8) & 0x0f)
#define MAX14830_FIFOTRIGLVL_RX(words)   (((words / 8) & 0x0f) << 4)

/* Misc definitions */
#define MAX14830_FIFO_SIZE               (128)
#define MAX14830_REV_MASK                (0xfc)

/** Write Bit */
#define MAX14830_SPI_WRITE_BIT          ( 1u << 7)


/** System Settings */
#define MAX14830_XTAL_FREQ              ( 4000000u )

//

#define MAX14830_NUM_SERIAL_PORTS       ( 4 )
#define MAX14830_WRITE_TASK_PRIORITY    ( 5 )
#define STACK_SIZE                      ( 128 )

#define ARTEMIS_MAX14830_BUFFER_LENGTH  ( 256 )

//*****************************************************************************
//
// Structs
//
//*****************************************************************************
typedef uint8_t module_buffer_t[256];


#ifndef BITBANG_SINCE_AMBIQ_DOESNT_WANT_TO_HELP_ME
typedef struct s_module_t
{
    artemis_spi_t spi;
    module_buffer_t txbuffer;
    module_buffer_t rxbuffer;
} module_t;
//static module_t module;
#else
typedef struct s_module_bb_t
{
  artemis_spi_bb_t spi;
  module_buffer_t txbuffer;
  module_buffer_t rxbuffer;
}module_t;

#endif

static module_t module;
//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
static void module_max14830_init(void);
static void module_max14830_Power_On(void);
static void module_max14830_Power_Off(void);
static uint32_t module_max14830_FastRead(void);
static uint32_t module_max14830_Read(
                    eMAX18430_ComPort_t port,
                    uint32_t ui32Instr,
                    uint32_t ui32NumBytes,
                    char *pData );


static void module_max14830_Write(
                    eMAX18430_ComPort_t port, 
                    uint8_t reg, 
                    char *data, 
                    uint32_t len
                    );

uint8_t module_max14830_port_from_pin(uint8_t pin);
//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void artemis_max14830_init(void)
{
  #ifndef BITBANG_SINCE_AMBIQ_DOESNT_WANT_TO_HELP_ME
  
    artemis_spi_t *spi = &module.spi;

#if MAX14830_IOM_MODULE == 0
    spi->chipselect = AM_BSP_IOM0_CS_CHNL;
    spi->iom.module = 0;
#elif MAX14830_IOM_MODULE == 1  
    spi->chipselect = AM_BSP_IOM1_CS_CHNL;
    spi->iom.module = 1;
#elif MAX14830_IOM_MODULE == 2  
    spi->chipselect = AM_BSP_IOM2_CS_CHNL;
    spi->iom.module = 2;
#elif MAX14830_IOM_MODULE == 3  
    spi->chipselect = AM_BSP_IOM3_CS_CHNL;
    spi->iom.module = ARTEMIS_IOM_MODULE_SPI3;
#elif MAX14830_IOM_MODULE == 4  
    spi->chipselect = AM_BSP_IOM4_CS_CHNL;
    spi->iom.module = 4;
#elif MAX14830_IOM_MODULE == 5
    spi->chipselect = AM_BSP_IOM5_CS_CHNL;
    spi->iom.module = 5;
#endif
  
  
    
    spi->iom.config.eInterfaceMode = AM_HAL_IOM_SPI_MODE;
    spi->iom.config.ui32ClockFreq = AM_HAL_IOM_100KHZ;
    spi->iom.config.eSpiMode = AM_HAL_IOM_SPI_MODE_0;
    artemis_iom_initialize(&spi->iom);
    
#if MAX14830_IOM_MODULE == 0
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCK, g_AM_BSP_GPIO_IOM0_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MISO, g_AM_BSP_GPIO_IOM0_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MOSI, g_AM_BSP_GPIO_IOM0_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_CS, g_AM_BSP_GPIO_IOM0_CS));
#elif MAX14830_IOM_MODULE == 1    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK, g_AM_BSP_GPIO_IOM1_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, g_AM_BSP_GPIO_IOM1_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MOSI, g_AM_BSP_GPIO_IOM1_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS, g_AM_BSP_GPIO_IOM1_CS));
#elif MAX14830_IOM_MODULE == 2    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCK, g_AM_BSP_GPIO_IOM2_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_MISO, g_AM_BSP_GPIO_IOM2_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_MOSI, g_AM_BSP_GPIO_IOM2_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_CS, g_AM_BSP_GPIO_IOM2_CS));
#elif MAX14830_IOM_MODULE == 3    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCK, g_AM_BSP_GPIO_IOM3_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MISO, g_AM_BSP_GPIO_IOM3_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MOSI, g_AM_BSP_GPIO_IOM3_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_CS, g_AM_BSP_GPIO_IOM3_CS));
#elif MAX14830_IOM_MODULE == 4    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCK, g_AM_BSP_GPIO_IOM4_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_MISO, g_AM_BSP_GPIO_IOM4_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_MOSI, g_AM_BSP_GPIO_IOM4_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_CS, g_AM_BSP_GPIO_IOM4_CS));
#elif MAX14830_IOM_MODULE == 5    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_SCK, g_AM_BSP_GPIO_IOM5_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_MISO, g_AM_BSP_GPIO_IOM5_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_MOSI, g_AM_BSP_GPIO_IOM5_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_CS, g_AM_BSP_GPIO_IOM5_CS));

#endif
    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NIRQ, g_AM_BSP_GPIO_S2U_NIRQ));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NRESET, g_AM_BSP_GPIO_S2U_NRESET));
    
    
  #else
    artemis_spi_bb_t *spi = &module.spi;
    spi->chipselect = AM_BSP_IOM3_CS_CHNL;
    spi->MISO = AM_BSP_GPIO_IOM3_MISO;
    spi->MOSI = AM_BSP_GPIO_IOM3_MOSI;
    spi->SCLK = AM_BSP_GPIO_IOM3_SCK;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCK, g_AM_BSP_GPIO_IOM3_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MISO, g_AM_BSP_GPIO_IOM3_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MOSI, g_AM_BSP_GPIO_IOM3_MOSI));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_CS, g_AM_BSP_GPIO_IOM3_CS));

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NIRQ, g_AM_BSP_GPIO_S2U_NIRQ));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NRESET, g_AM_BSP_GPIO_S2U_NRESET));
  #endif
    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_ON, g_AM_BSP_GPIO_S2U_ON));

  module_max14830_Power_Off();
  module_max14830_Power_On(); 
  module_max14830_init();
}

void artemis_max14830_enable(eMAX18430_ComPort_t port)
{
  module_max14830_Power_On();
}

void artemis_max14830_disable(eMAX18430_ComPort_t port)
{
  
}

void artemis_max14830_Set_baudrate(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate )
{
  float D = MAX14830_XTAL_FREQ / (16 * baudrate );
  uint32_t DIV = (uint32_t)trunc(D) ;
  uint32_t FRACT = (uint32_t) round(16 * (D - DIV));
  FRACT = FRACT & 0x0000000F;
  uint32_t DIVMSB = (DIV & 0x00000100) >> 8;
  uint32_t DIVLSB = (DIV & 0x000000FF);

  /** Send the BRGConfig */
  module_max14830_Write(port, MAX14830_REG_DIVLSB, (char*)&DIVLSB, 1);
  module_max14830_Write(port, MAX14830_REG_DIVMSB, (char*)&DIVMSB, 1);
  module_max14830_Write(port, MAX14830_REG_BRGCONFIG, (char*)&FRACT, 1);
}

void artemis_max14830_UART_Write(eMAX18430_ComPort_t port, char *data, uint32_t len)
{
//  artemis_max14830_enable(port);
//  module_max14830_Write(port, MAX14830_THR, data, len);
//  artemis_max14830_disable(port);
  uint8_t txDatalen = 0;
  
  uint8_t txFifoLen;
  module_max14830_Read(port, MAX14830_REG_TXFIFOLVL, 1, (char*)&txFifoLen);
  
  uint8_t txFifoEmptyLen = MAX14830_FIFO_SIZE - txFifoLen;
//  if(txFifoEmptyLen > len)
//  {
//    txDataLen = len;
//  } else {
//    tx
//  }
  
  for(uint32_t i=0; i<len; i++)
  {
    module_max14830_Write(port, MAX14830_THR, data++, 1);
  }
  
  
//  Max14830RegRead(channel, MAX14830_TXFIFOLVL_REG, &txFifoDataLen);
//        txFifoEmptyLen = MAX14830_FIFO_SIZE - txFifoDataLen;
//
//        if (txFifoEmptyLen > len)
//        {
//            transmitTxDatalen = len;
//        }
//        else
//        {
//            transmitTxDatalen = txFifoEmptyLen;
//        }
//
//        writeCount = transmitTxDatalen;
//        do  
//        {   
//            /* transmit character */
//            Max14830RegWrite(channel, MAX14830_THR_REG, *buf++);
//            transmitTxDatalen--;
//        } while(transmitTxDatalen != 0);
        
        
  
}

uint32_t artemis_max14830_UART_Read(eMAX18430_ComPort_t port, char *pData, uint32_t max_len)
{
  uint32_t data[64];
  
  uint8_t len;
  do
  {
    module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, 1, &len);
    if(len > 0)
    {
      module_max14830_Read(port, MAX14830_RHR, 1, pData++);
    }
  } while(len > 0);
//  artemis_max14830_enable(port);
//  module_max14830_Read(port, 1, 32, pData);
//  artemis_max14830_disable(port);

}

uint32_t artemis_max14830_UART_Read_bytes_waiting(eMAX18430_ComPort_t port)
{
  
}

void artemis_max14830_gpio_configure_output(uint8_t pin, eMAX14830_GPIO_Output_t type)
{
  uint8_t port = module_max14830_port_from_pin(pin);
  uint8_t data;
 
  module_max14830_Read(port, MAX14830_REG_GPIOCONFIG, 1, &data);
  /** Set to output */
  data |= MAX14830_GPIO_PIN_TO_BIT(pin);
  
  if(type)
  {
    data |= (MAX14830_GPIO_PIN_TO_BIT(pin) << 4);
  }
  module_max14830_Write(
                      port,  
                      MAX14830_REG_GPIOCONFIG, 
                      &data, 
                      1);
  
}



void artemis_max14830_gpio_set(uint8_t pin)
{
  uint8_t port = module_max14830_port_from_pin(pin);  
  uint8_t data;
  module_max14830_Read(port, MAX14830_REG_GPIODATA, 1, &data);
  data |= MAX14830_GPIO_PIN_TO_BIT(pin);
  module_max14830_Write(
                      port,  
                      MAX14830_REG_GPIODATA, 
                      &data, 
                      1);
  
}

void artemis_max14830_gpio_clear(uint8_t pin)
{
  uint8_t port = module_max14830_port_from_pin(pin);
  uint8_t data;
  module_max14830_Read(port, MAX14830_REG_GPIODATA, 1, &data);
  data &= ~MAX14830_GPIO_PIN_TO_BIT(pin);
  module_max14830_Write(
                      port,  
                      MAX14830_REG_GPIODATA, 
                      &data, 
                      1);
  
}

//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************
static void module_max14830_init(void)
{
  eMAX18430_ComPort_t channel = MAX14830_COM_PORT0;
  char data = MAX14830_MODE2_RST;
  
  for(uint8_t i=0; i<4; i++)
  {
    channel = (eMAX18430_ComPort_t)i;
      switch(channel)
      {
      case 0:
        channel = MAX14830_COM_PORT0;
        break;
      case 1:
        channel = MAX14830_COM_PORT1;
        break;
      case 2:
        channel = MAX14830_COM_PORT2;
        break;
      case 3:
        channel = MAX14830_COM_PORT3;
        break;
      default:
        break;
      }
    //  Max14830RegWrite(channel, MAX14830_MODE2_REG, MAX14830_MODE2_RST_BIT);
      data = MAX14830_MODE2_RST;
//      module_max14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
      module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
      
      data = 0u;
//      module_max14830_Write(MAX14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
      module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);

      // Wait until reset is cleared
      uint32_t resetTimeout = 128;
      while(--resetTimeout)
      {
        am_hal_systick_delay_us(50);
//        module_max14830_Read(MAX14830_COM_PORT0, MAX14830_REG_DIVLSB, 1, &data);
        module_max14830_Read(channel, MAX14830_REG_DIVLSB, 1, &data);
        if(data == 1)
        {
          break;
        }
      }
      
      data = 0;
      module_max14830_Write(channel, MAX14830_REG_IRQEN, &data, 1);
      data = MAX14830_CLK_PLL_BYPASS | MAX14830_CLK_CRYSTAL_EN;
      module_max14830_Write(channel, MAX14830_REG_CLKSOURCE, &data, 1);
  


  

  

  
///* Disable all interrupts */
//        Max14830RegWrite(channel, MAX14830_IRQEN_REG, 0);
//
//        /* Disable PLL and Use crystal clock. */
//        Max14830RegWrite(channel, MAX14830_CLKSRC_REG, MAX14830_CLKSRC_PLLBYP_BIT | MAX14830_CLKSRC_CRYST_BIT);
//
//        /* Set reference clock to crystal clock */
//        channel->uartRefClock = channel->clockFreq;
//
//        /* set the baud rate */
  artemis_max14830_Set_baudrate(channel, 9600);
//
//        /* configure LCR register, 8N1 mode by default */
  data = MAX14830_LCR_LENGTH_8;
  module_max14830_Write(channel, MAX14830_REG_LCR, &data, 1);
//
//        if (channel->rs485Mode == TRUE)
//        {
//            /* Enable auto transmit and receive for RS485 */
//            Max14830RegWrite(channel, MAX14830_MODE1_REG, MAX14830_MODE1_TRNSCVCTRL_BIT);
//            Max14830RegWrite(channel, MAX14830_MODE2_REG, MAX14830_MODE2_ECHOSUPR_BIT);
//            Max14830RegWrite(channel, MAX14830_HDPIXDELAY_REG, 0x11);
//        }
//
  
  /* RMW IRQ With interrupt out */
  module_max14830_Read(channel, MAX14830_REG_MODE1, 1, &data);
  data |= MAX14830_MODE1_IRQ_SEL;
  module_max14830_Write(channel, MAX14830_REG_MODE1, &data, 1);
  
  

    /* Reset FIFOs and enable echo suppression */
    module_max14830_Read(channel, MAX14830_REG_MODE2, 1, &data);
    data |= MAX14830_MODE2_FIFO_RST;
    module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
  
//
//        /* configure FIFO trigger level register */
//        /* RX FIFO trigger for 16 words, TX FIFO trigger for 64 words */
//        regValue = MAX14830_FIFOTRIGLVL_RX(16) | MAX14830_FIFOTRIGLVL_TX(64);
//        Max14830RegWrite(channel, MAX14830_FIFOTRIGLVL_REG, regValue);
  
  data = MAX14830_FIFOTRIGLVL_RX(16) | MAX14830_FIFOTRIGLVL_TX(64);
  module_max14830_Write(channel, MAX14830_REG_FIFOTRGLVL, &data, 1);
  
//
//        /* configure flow control levels */
//        regValue = MAX14830_FLOWLVL_RES(48) | MAX14830_FLOWLVL_HALT(96);
//        Max14830RegWrite(channel, MAX14830_FLOWLVL_REG, regValue);
  data = MAX14830_FLOWLVL_RES(48) | MAX14830_FLOWLVL_HALT(96);
  module_max14830_Write(channel, MAX14830_REG_FLOWLVL, &data, 1);
  
//
//        /* clear timeout register */
//        Max14830RegWrite(channel, MAX14830_RXTO_REG, 0);
  data = 0u;
  module_max14830_Write(channel, MAX14830_REG_RXTIMEOUT, &data, 0);
  
//
//        /* configure LSR interrupt enable register */
//        /* enable RX timeout interrupt */
//        regValue = MAX14830_LSR_RXTO_BIT | MAX14830_LSR_RXOVR_BIT | MAX14830_LSR_RXPAR_BIT | 
//                   MAX14830_LSR_FRERR_BIT | MAX14830_LSR_RXNOISE_BIT;
//        Max14830RegWrite(channel, MAX14830_LSR_IRQEN_REG, regValue);
  
  data = (MAX14830_LSR_RTIMEOUT |
          MAX14830_LSR_RXOVERRUN |
          MAX14830_LSR_RXPARITY |
          MAX14830_LSR_FRAMEERR |
          MAX14830_LSR_RXNOISE );
  module_max14830_Write(channel, MAX14830_REG_LSRINTEN, &data, 1);
  
//
//        /* clear FIFO reset */
//        Max14830RegRead(channel, MAX14830_MODE2_REG, &regValue);
//        regValue &= ~MAX14830_MODE2_FIFORST_BIT;
//        Max14830RegWrite(channel, MAX14830_MODE2_REG, regValue);
  module_max14830_Read(channel, MAX14830_REG_MODE2, 1, &data);
  data &= ~MAX14830_MODE2_FIFO_RST;
  module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
  
//
//        /* Set Rx FIFO empty INT invert */
//        Max14830RegRead(channel, MAX14830_MODE2_REG, &regValue);
//        regValue |= MAX14830_MODE2_RXEMPTINV_BIT;
//        Max14830RegWrite(channel, MAX14830_MODE2_REG, regValue);
  module_max14830_Read(channel, MAX14830_REG_MODE2, 1, &data);
  data |= MAX14830_MODE2_RX_EMTY_INV;
  module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
  
//
//        /* get invalid data */
//        Max14830RegRead(channel, MAX14830_RHR_REG, &regValue);
  module_max14830_Read(channel, MAX14830_RHR, 1, &data);
  
//
//        /* clear IRQ status register by reading it */
//        Max14830RegRead(channel, MAX14830_IRQSTS_REG, &regValue);
  module_max14830_Read(channel, MAX14830_REG_ISR, 1, &data);
  
//
//        /* disable auto flow control */
//        Max14830RegWrite(channel, MAX14830_FLOWCTRL_REG, 0);
  data = 0;
  module_max14830_Write(channel, MAX14830_REG_FLOWCTRL, &data, 1);
  
//  /** Invert Tx & RX */
//  data = MAX14830_IRDA_TX_INV | MAX14830_IRDA_RX_INV;
//  module_max14830_Write(channel, MAX14830_REG_IRDA, &data, 1);
  
//
//        /* enable RX interrupts */
//        regValue = MAX14830_IRQ_RXEMPTY_BIT | MAX14830_IRQ_LSR_BIT;
//        Max14830RegWrite(channel, MAX14830_IRQEN_REG, regValue);
  data = MAX14830_IRQ_RFIFOEMTY  | MAX14830_IRQ_LSRERR;
  module_max14830_Write(channel, MAX14830_REG_IRQEN, &data, 1);
//
//        rxSemName[5] = '0' + channel->bUnit;
//        rxSemName[6] = '0' + channel->sUnit;
//        status = SemaphoreBinaryCreate((const char *)rxSemName, QUEUE_TYPE_FIFO, SEM_LOCKED, &channel->rxSem);
//        if (status != SUCCESS)
//        {
//            printk("%s: failed to create semaphore %d\n", __FUNCTION__, status);
//        }
//
//        /* enable the channel */
//        channel->isEnabled = TRUE;
//    }
  }
}
static void module_max14830_Power_On(void)
{
  am_hal_gpio_output_set(AM_BSP_GPIO_S2U_NRESET);
  am_hal_gpio_output_clear(AM_BSP_GPIO_S2U_ON);
  
}
static void module_max14830_Power_Off(void)
{
  am_hal_gpio_output_clear(AM_BSP_GPIO_S2U_NRESET);
  am_hal_gpio_output_set(AM_BSP_GPIO_S2U_ON);
}

static uint32_t module_max14830_FastRead(void);
static uint32_t module_max14830_Read(
                    eMAX18430_ComPort_t port,
                    uint32_t ui32Instr,
                    uint32_t ui32NumBytes,
                    char *pData );



static uint32_t module_max14830_Read(
                    eMAX18430_ComPort_t port,
                    uint32_t ui32Instr,
                    uint32_t ui32NumBytes,
                    char *pData )

{
  uint8_t hi = 0;
    uint8_t lo = 0;
    artemis_stream_t txstream = {0};
    artemis_stream_t rxstream = {0};

    artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_MAX14830_BUFFER_LENGTH);
    artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_MAX14830_BUFFER_LENGTH);
    
    
    artemis_stream_reset(&txstream);
    artemis_stream_reset(&rxstream);
    
    uint32_t inst = (uint32_t) port  & 0x03;
    inst = inst << 5;
    inst |= ui32Instr;
    
    
    artemis_stream_put(&txstream, inst); //ui32Instr ); 
    artemis_spi_send(&module.spi, false, &txstream);
    artemis_spi_receive(&module.spi, true, &rxstream, ui32NumBytes);

    artemis_stream_read(&rxstream, (char*)pData, ui32NumBytes);

#ifdef MAX14830_TEST
    printf("Read Data =", *pData++);
    for(uint32_t i=0;i<ui32NumBytes;i++)
    {
      if(i%8 == 0)
      {
        printf("\n0x%02x", *pData++);
      } else {
        printf(", 0x%02x", *pData++);
      }
        
    }
    printf("\n");
#endif
  
}

static void module_max14830_Write(
                    eMAX18430_ComPort_t port, 
                    uint8_t reg, 
                    char *data, 
                    uint32_t len
                    )
{
  
  
  artemis_stream_t txstream = {0};
  uint8_t cmd = reg | 0x80;
  
  cmd |= (port << 5);
  
  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_MAX14830_BUFFER_LENGTH);
  artemis_stream_reset(&txstream);
  artemis_stream_put(&txstream, cmd);
  
  artemis_stream_write(&txstream, data, len);
//  artemis_stream_put(&txstream, port | 0x80);
//  reg;
//  artemis_stream_put(&txstream, reg);
//  for(uint32_t i=0;i<len;i++)
//  {
//    artemis_stream_put(&txstream, *data++);
//  }
  
  artemis_spi_send(&module.spi, true, &txstream);
}


uint8_t module_max14830_port_from_pin(uint8_t pin)
{
  uint8_t port;
  if(pin < 4)
  {
    port = MAX14830_COM_PORT0;
  }
  else if (pin < 8)
  {
    port = MAX14830_COM_PORT1;
  } 
  else if (pin < 12)
  {
    port = MAX14830_COM_PORT2;
  }
  else if (pin < 16)
  {
    port = MAX14830_COM_PORT3;
  }
  else
  {
    // @todo: ERROR
  }
  return port;
}