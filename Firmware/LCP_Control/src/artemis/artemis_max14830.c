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

//*****************************************************************************
//
//  Register Defines
//
//*****************************************************************************
/** FIFO Data Registers */
#define MAX14830_REG_RHR            ( 0x00 )
#define MAX14830_REG_THR            ( 0x00 )

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
#define MAX14830_SCI_MLTDRP           ( 1u << 5 )
#define MAX14830_SCI_BREAK            ( 1u << 4 )
#define MAX14830_SCI_XOFF2            ( 1u << 3 )
#define MAX14830_SCI_XOFF1            ( 1u << 2 )
#define MAX14830_SCI_XON2             ( 1u << 1 )
#define MAX14830_SCE_XON1             ( 1u )

/** STS Register Bits */
#define MAX14830_STS_CLKRDY           ( 1u << 5 )
#define MAX14830_STS_GPI3             ( 1u << 3 )
#define MAX14830_STS_GPI2             ( 1u << 2 )
#define MAX14830_STS_GPI1             ( 1u << 1 )
#define MAX14830_REG_GPI0             ( 1u )

/** MODE1 Register Bits */
#define MAX14830_MODE1_IRQ_SEL        ( 1u << 7 )
#define MAX14830_MODE1_TRNSCV_CTRL    ( 1u << 4 )
#define MAX14830_MODE1_RTS_HIZ        ( 1u << 3 )
#define MAX14830_MODE1_TX_HIZ         ( 1u << 2 )
#define MAX14830_MODE1_TX_DISABL      ( 1u << 1 )
#define MAX14830_MODE1_RX_DISABL      ( 1u )

/** MODE2 Register Bits */
#define MAX14830_MODE2_ECHO_SUPRS     ( 1u << 7 )
#define MAX14830_MODE2_MULTI_DROP     ( 1u << 6 )
#define MAX14830_MODE2_LOOPBACK       ( 1u << 5 )
#define MAX14830_MODE2_SPECIAL_CHR    ( 1u << 4 )
#define MAX14830_MODE2_RX_EMTY_INV    ( 1u << 3 )
#define MAX14830_MODE2_RX_TRIG_INV    ( 1u << 2 )
#define MAX14830_MODE2_FIFO_RST       ( 1u << 1 )
#define MAX14830_MODE2_RST            ( 1u )

/** LCR Register Bits */
#define MAX14830_LCR_RTS              ( 1u << 7 )
#define MAX14830_LCR_TX_BREAK         ( 1u << 6 )
#define MAX14830_LCR_FORCE_PARITY     ( 1u << 5 )
#define MAX14830_LCR_EVEN_PARITY      ( 1u << 4 )
#define MAX14830_LCR_PARITY_EN        ( 1u << 3 )
#define MAX14830_LCR_STOPBITS_1       ( 0u )
#define MAX14830_LCR_STOPBITS_2       ( 1u << 2 )
#define MAX14830_LCR_LENGTH_5         ( 0x00 )
#define MAX14830_LCR_LENGTH_6         ( 0x01 )
#define MAX14830_LCR_LENGTH_7         ( 0x02 )
#define MAX14830_LCR_LENGTH_8         ( 0x03 )

/** HDplxDelay Register Bits */
#define MAX14830_HDD_SETUP_MASK       ( 0x0F << 4 )
#define MAX14830_HDD_HOLD_MASK        ( 0x0F )

/** IrDA Register Bits */
#define MAX14830_IRDA_TX_INV          ( 1u << 5 )
#define MAX14830_IRDA_RX_INV          ( 1u << 4 )
#define MAX14830_IRDA_MIR             ( 1u << 3 )
#define MAX14830_IRDA_RTS_INV         ( 1u << 2 )
#define MAX14830_IRDA_SIR             ( 1u << 1 )
#define MAX14830_IRDA_IRDA_EN         ( 1u )

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
#define MAX14830_CLK_TO_RTS         	( 1u << 7 )
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
#define MAX14830_SPI_WRITE_CMD           ( 0x80 )
#define MAX14830_SPI_READ_CMD            ( 0x7F )

/** System Settings */
#define MAX14830_XTAL_FREQ              ( 4000000u )
#define MAX14830_IOM_MODULE             ( 3 )
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
typedef struct s_module_t
{
    artemis_spi_t spi;
    module_buffer_t txbuffer;
    module_buffer_t rxbuffer;
} module_t;

static module_t module;

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************

static void module_max14830_init(void);
static void module_max14830_Power_On(void);
static void module_max14830_Power_Off(void);

static void module_max14830_Read(emax18430_ComPort_t port, uint8_t reg, uint8_t *rData, uint8_t len);
static void module_max14830_Write(emax18430_ComPort_t port, uint8_t reg, uint8_t *sData, uint8_t len);

static uint8_t module_max14830_FastRead(void);
uint8_t module_max14830_port_from_pin(uint8_t pin);
static void module_max14830_chip_enable(void);
static void module_max14830_chip_disable(void);

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void artemis_max14830_init(void)
{
    artemis_spi_t *spi = &module.spi;

    spi->chipselect = AM_BSP_IOM3_CS_CHNL;
    spi->iom.module = ARTEMIS_IOM_MODULE_SPI3;
    spi->iom.config.eInterfaceMode = AM_HAL_IOM_SPI_MODE;
    spi->iom.config.ui32ClockFreq = AM_HAL_IOM_100KHZ;
    //spi->iom.config.ui32ClockFreq = AM_HAL_IOM_1MHZ;
    spi->iom.config.eSpiMode = AM_HAL_IOM_SPI_MODE_0;

    artemis_iom_initialize(&spi->iom);

    /* SPI3 module */
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCK, g_AM_BSP_GPIO_IOM3_SCK));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MISO, g_AM_BSP_GPIO_IOM3_MISO));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MOSI, g_AM_BSP_GPIO_IOM3_MOSI));

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_CS, g_AM_BSP_GPIO_IOM3_CS));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NIRQ, g_AM_BSP_GPIO_S2U_NIRQ));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_NRESET, g_AM_BSP_GPIO_S2U_NRESET));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_S2U_ON, g_AM_BSP_GPIO_S2U_ON));
    module_max14830_chip_disable();

    module_max14830_Power_Off();
    module_max14830_Power_On();
    module_max14830_init();
}

static void module_max14830_chip_enable(void)
{
	am_hal_gpio_output_clear(AM_BSP_GPIO_IOM3_CS);
}

static void module_max14830_chip_disable(void)
{
	am_hal_gpio_output_set(AM_BSP_GPIO_IOM3_CS);
}

void artemis_max14830_enable(emax18430_ComPort_t port)
{
	module_max14830_Power_On();
}

void artemis_max14830_disable(emax18430_ComPort_t port)
{
	module_max14830_Power_Off();
}

uint32_t artemis_max14830_Set_baudrate(emax18430_ComPort_t port, emax14830_Baudrate_t baudrate)
{
	float D = MAX14830_XTAL_FREQ / (16 * baudrate );
	uint32_t DIV = (uint32_t)trunc(D) ;
	uint32_t FRACT = (uint32_t) round(16 * (D - DIV));
	FRACT = FRACT & 0x0000000F;
	uint32_t DIVMSB = (DIV & 0x00000100) >> 8;
	uint32_t DIVLSB = (DIV & 0x000000FF);

	/** Send the BRGConfig */
	module_max14830_Write(port, MAX14830_REG_DIVLSB, (uint8_t*)&DIVLSB, 1);
	module_max14830_Write(port, MAX14830_REG_DIVMSB, (uint8_t*)&DIVMSB, 1);
	module_max14830_Write(port, MAX14830_REG_BRGCONFIG, (uint8_t*)&FRACT, 1);

	//am_util_stdio_printf ("\n Baudrate = %u \n ", (16*MAX14830_XTAL_FREQ) / (16 * (16*DIV + FRACT)));
	return ((16*MAX14830_XTAL_FREQ) / (16 * (16*DIV + FRACT)) );
}

void artemis_max14830_UART_Write(emax18430_ComPort_t port, uint8_t *sData, uint8_t len)
{

	uint8_t txFifoLen = 0 ;
	uint8_t dataLen = 0;
	uint8_t sBuffer [STACK_SIZE] = {0};
	uint8_t sBufferLen = 0;

	for (sBufferLen=0; sBufferLen<len; sBufferLen++){
		sBuffer[sBufferLen] = sData[sBufferLen];
	}
	module_max14830_Write(port, MAX14830_REG_THR, sBuffer, sBufferLen);

    uint8_t ISR_status = 0x00;
    uint8_t LSR_status = 0x00;
    uint8_t txlen = 0;

    do {
		module_max14830_Read(port, MAX14830_REG_TXFIFOLVL, &txlen, 1);
    } while (txlen);

	//am_util_stdio_printf ("\nPORT:: %d,  WRITE DONE ...\n", port);



	//if (txFifoLen == 0){
	//	// send bytes in case of TX Fifo is ready for now !!!
	//	module_max14830_Write(port, MAX14830_REG_THR, sBuffer, sBufferLen);
	//}
	//else {
	//	// TODO: wait for txfifo to be empty for now
	//	// do nothing for now
	//}

////  artemis_max14830_enable(port);
////  module_max14830_Write(port, MAX14830_REG_THR, data, len);
////  artemis_max14830_disable(port);
//  uint8_t txDatalen = 0;
//  uint8_t txFifoLen;
//  module_max14830_Read(port, MAX14830_REG_TXFIFOLVL, &txFifoLen, 1);
//
//  uint8_t txFifoEmptyLen = MAX14830_FIFO_SIZE - txFifoLen;
////  if(txFifoEmptyLen > len)
////  {
////    txDataLen = len;
////  } else {
////    tx
////  }
//
//  for(uint32_t i=0; i<len; i++)
//  {
//    module_max14830_Write(port, MAX14830_REG_THR, sData++, 1);
//  }
//
//
////  Max14830RegRead(channel, MAX14830_TXFIFOLVL_REG, &txFifoDataLen);
////        txFifoEmptyLen = MAX14830_FIFO_SIZE - txFifoDataLen;
////
////        if (txFifoEmptyLen > len)
////        {
////            transmitTxDatalen = len;
////        }
////        else
////        {
////            transmitTxDatalen = txFifoEmptyLen;
////        }
////
////        writeCount = transmitTxDatalen;
////        do
////        {
////            /* transmit character */
////            Max14830RegWrite(channel, MAX14830_REG_THR_REG, *buf++);
////            transmitTxDatalen--;
////        } while(transmitTxDatalen != 0);
//
}

void artemis_max14830_UART_Read(emax18430_ComPort_t port, uint8_t *rxData, uint8_t *rxLen)
{
    uint16_t len = 0;
    uint16_t txlen = 0;
    uint8_t rxBuff[STACK_SIZE] = {0};
    uint8_t ISR_status = 0;
    uint8_t LSR_status = 0;
    uint8_t data = 0;
    bool flag = false;

    /* TODO:  implement the Interrupt way of getting data from RXFifo */
    do
    {
        module_max14830_Read(port, MAX14830_REG_LSR, &LSR_status, 1);
        module_max14830_Read(port, MAX14830_REG_ISR, &ISR_status, 1);
        /* Keep checking for RXFIFO Trigger level */
        if (LSR_status&MAX14830_LSR_RXOVERRUN)
        {
            /* batch read data */
            do
            {
                module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);
                module_max14830_Read(port, MAX14830_REG_RHR, rxBuff, len);
                for (uint8_t i=0; i<len; i++)
                {
                    rxData[*rxLen+i] = rxBuff[i];
                }
                *rxLen += len;
            } while (len);
            flag = true;
            break;
        }
    } while ( !(LSR_status&0x01) );

    do
    {
        module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);
        if ( len > 0 )
        {
            uint16_t i=0;
            do
            {
                module_max14830_Read(port, MAX14830_REG_RHR, &data, 1);
                module_max14830_Read(port, MAX14830_REG_LSR, &LSR_status, 1);
                module_max14830_Read(port, MAX14830_REG_ISR, &ISR_status, 1);
                module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);
                rxData[i] = data;
                i++;
                *rxLen = i;
            } while (len);

            module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);
            if (!len)
            {
                flag = true;
            }
            else
            {
                // collect more bytes
            }
        }
    } while (!flag);

    //am_util_stdio_printf ("\nPORT: %d,  READ DONE ...\n", port);

    //if (len >0)
    //{

    //    module_max14830_Read(port, MAX14830_REG_RHR, rxBuff, len);

    //    for (uint16_t i=0; i<len; i++)
    //    {
    //        rxData[i] = rxBuff[i];
    //    }

    //    *rxLen = len;

    //    /* collect remaing bytes */
    //    do {

    //        module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);
    //        module_max14830_Read(port, MAX14830_REG_RHR, rxBuff, len);
    //        module_max14830_Read(port, MAX14830_REG_ISR, &status, 1);

    //        //am_util_stdio_printf ("status = 0x%02X , length = %u \n ", status,  len);

    //        for (uint16_t i=1; i<len; i++)
    //        {
    //            //am_util_stdio_printf ("%d ", rxBuff[i]);
    //            rxData[(*rxLen)++] = rxBuff[i-1];
    //        }
    //        *rxLen +=len;

    //        am_util_stdio_printf ("status = 0x%02X , length = %u \n ", status,  *rxLen);

    //    } while ( true );
    //
    //    am_util_stdio_printf ("status = 0x%02X , length = %u \n ", status,  *rxLen);

    //    for (uint16_t i=0; i<*rxLen; i++)
    //    {
    //        am_util_stdio_printf ("%d ", rxData[i]);
    //    }

    //    am_util_stdio_printf ("\n DONE\n\n");

    //    //module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);

    //    //if (len > 0)
    //    //{
    //    //    while (len--)
    //    //    {
    //    //        module_max14830_Read(port, MAX14830_REG_RHR, &data, 1);
    //    //        am_util_stdio_printf ("%d \n", data);
    //    //    }
    //    //}
    //    break;
    //}

    ///* check RXFIFO if there is data, wait for 1 seconds */
    //while (len==0 && counter<1000000)
    //{
    //	module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);
    //    am_hal_systick_delay_us(1);
    //	counter++;
    //}

    ///* collect remaining bytes within 1 second */
    ////am_util_delay_ms(10);

    //counter = 0;
    //while (len > 0 && counter<1000)
    //{
    //	module_max14830_Read(port, MAX14830_REG_RHR, rxBuff, len);
    //	for (uint16_t i=0; i<len; i++){
    //		rxData[i+j] = rxBuff[i];
    //		//am_util_stdio_printf ("%d ", rxData[i]);
    //	}
    //	j += len;
    //    am_hal_systick_delay_us(100);
    //	module_max14830_Read(port, MAX14830_REG_RXFIFOLVL, &len, 1);
    //    counter++;
    //}
    //*rxLen = j;

    //artemis_max14830_enable(port);
    //module_max14830_Read(port, 1, 32, pData);
    //artemis_max14830_disable(port);
}

uint32_t artemis_max14830_UART_Read_bytes_waiting(emax18430_ComPort_t port)
{
    // does this need to be implemented ?
    return 0;
}

void artemis_max14830_gpio_configure_output(uint8_t pin, emax14830_GPIO_Output_t type)
{
    uint8_t port = module_max14830_port_from_pin(pin);
    uint8_t data;

    module_max14830_Read(port, MAX14830_REG_GPIOCONFIG, &data, 1);
    /** Set to output */
    data |= max14830_GPIO_PIN_TO_BIT(pin);

    if(type)
    {
        data |= (max14830_GPIO_PIN_TO_BIT(pin) << 4);
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
    module_max14830_Read(port, MAX14830_REG_GPIODATA, &data, 1);
    data |= max14830_GPIO_PIN_TO_BIT(pin);
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
    module_max14830_Read(port, MAX14830_REG_GPIODATA, &data, 1);
    data &= ~max14830_GPIO_PIN_TO_BIT(pin);
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
    /* Hardware Reset */
    am_hal_gpio_output_clear(AM_BSP_GPIO_S2U_NRESET);
    am_hal_systick_delay_us(10);
    am_hal_gpio_output_set(AM_BSP_GPIO_S2U_NRESET);

    /* Wait for IRQ to goes high */
    uint32_t ret = 0;
    do {
        //ARTEMIS_DEBUG_PRINTF("IRQ is %u \n", ret);
        am_hal_gpio_state_read(AM_BSP_GPIO_S2U_NIRQ, AM_HAL_GPIO_INPUT_READ, &ret);
    }while(ret==0);

    am_hal_gpio_state_read(AM_BSP_GPIO_S2U_NIRQ, AM_HAL_GPIO_INPUT_READ, &ret);
    ARTEMIS_DEBUG_PRINTF("Max14830 IRQ is %u, ready to be programmed\n", ret);

    /* Software Reset */
    uint8_t data = MAX14830_MODE2_RST;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
    data = 0x00;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);

    /* FIFO Reset for all ports */
    data = MAX14830_MODE2_FIFO_RST;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
    module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_MODE2, &data, 1);
    module_max14830_Write(max14830_COM_PORT2, MAX14830_REG_MODE2, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_MODE2, &data, 1);
    data = 0x00;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
    module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_MODE2, &data, 1);
    module_max14830_Write(max14830_COM_PORT2, MAX14830_REG_MODE2, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_MODE2, &data, 1);

    /* enable the ClkSource , bypass the PLL,
    NOTE :: Clock Source is only enabled via Port0  */
    data = MAX14830_CLK_PLL_BYPASS | MAX14830_CLK_CRYSTAL_EN;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_CLKSOURCE, &data, 1);

    /* Setup Modes */
    data = MAX14830_MODE1_IRQ_SEL | MAX14830_MODE1_TRNSCV_CTRL;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE1, &data, 1);
    module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_MODE1, &data, 1);
    module_max14830_Write(max14830_COM_PORT2, MAX14830_REG_MODE1, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_MODE1, &data, 1);

    /* Enable Interrupts on RxFifo and TxFifo */
    data = MAX14830_IRQ_RFIFOEMTY | MAX14830_IRQ_TFIFOEMTY;
    //data = 0x01;
    module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_IRQEN, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_IRQEN, &data, 1);

    //data = 0xFF;
    //module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_LSRINTEN, &data, 1);
    //data = 0xFF;
    //module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_STSINTEN, &data, 1);

    data = 0x02;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_RXTIMEOUT, &data, 1);
    module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_RXTIMEOUT, &data, 1);
    //module_max14830_Write(max14830_COM_PORT2, MAX14830_REG_RXTIMEOUT, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_RXTIMEOUT, &data, 1);

    //data = 0x3F;
    ////module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_LSRINTEN, &data, 1);
    //module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_LSRINTEN, &data, 1);
    ////module_max14830_Write(max14830_COM_PORT2, MAX14830_REG_LSRINTEN, &data, 1);
    //module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_LSRINTEN, &data, 1);

    /* configure LCR register, Set 8N1 mode as default for all ports
    and enable RTS on port2 and port3 */
    data = MAX14830_LCR_LENGTH_8 | MAX14830_LCR_RTS;
    //data = MAX14830_LCR_LENGTH_8;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_LCR, &data, 1);
    module_max14830_Write(max14830_COM_PORT1, MAX14830_REG_LCR, &data, 1);
    module_max14830_Write(max14830_COM_PORT2, MAX14830_REG_LCR, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_LCR, &data, 1);

    /* RX TX fifo trigger level */
    data = MAX14830_FIFOTRIGLVL_RX(64) | MAX14830_FIFOTRIGLVL_TX(64);
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_FIFOTRGLVL, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_FIFOTRGLVL, &data, 1);

    /* enable RS-485 for Port3,
     * echo suppress  */
    data = MAX14830_MODE2_ECHO_SUPRS | MAX14830_MODE2_RX_EMTY_INV;
    //data = MAX14830_MODE2_ECHO_SUPRS ;
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_MODE2, &data, 1);


    /* enable RS-232 for Port0 */
    data = max14830_GPIO_PIN_TO_BIT(2);
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_GPIOCONFIG, &data, 1);
    module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_GPIODATA, &data, 1);

    data = max14830_GPIO_PIN_TO_BIT(9);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_GPIOCONFIG, &data, 1);
    module_max14830_Write(max14830_COM_PORT3, MAX14830_REG_GPIODATA, &data, 1);
}

//  emax18430_ComPort_t channel = max14830_COM_PORT0;
//  uint8_t data = MAX14830_MODE2_RST;
//
//  for(uint8_t i=0; i<4; i++)
//  {
//    channel = (emax18430_ComPort_t)i;
//      switch(channel)
//      {
//      case 0:
//        channel = max14830_COM_PORT0;
//        break;
//      case 1:
//        channel = max14830_COM_PORT1;
//        break;
//      case 2:
//        channel = max14830_COM_PORT2;
//        break;
//      case 3:
//        channel = max14830_COM_PORT3;
//        break;
//      default:
//        break;
//      }
//    //  Max14830RegWrite(channel, MAX14830_MODE2_REG, MAX14830_MODE2_RST_BIT);
//      data = MAX14830_MODE2_RST;
////      module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
//      module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
//
//      data = 0u;
////      module_max14830_Write(max14830_COM_PORT0, MAX14830_REG_MODE2, &data, 1);
//      module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
//
//      // Wait until reset is cleared
//      uint32_t resetTimeout = 128;
//      while(--resetTimeout)
//      {
//        am_hal_systick_delay_us(50);
////        module_max14830_Read(max14830_COM_PORT0, MAX14830_REG_DIVLSB, 1, &data);
//        module_max14830_Read(channel, MAX14830_REG_DIVLSB, 1, &data);
//        if(data == 1)
//        {
//          break;
//        }
//      }
//
//      data = 0;
//      module_max14830_Write(channel, MAX14830_REG_IRQEN, &data, 1);
//      data = MAX14830_CLK_PLL_BYPASS | MAX14830_CLK_CRYSTAL_EN;
//      module_max14830_Write(channel, MAX14830_REG_CLKSOURCE, &data, 1);
//
//
/////* Disable all interrupts */
////        Max14830RegWrite(channel, MAX14830_IRQEN_REG, 0);
////
////        /* Disable PLL and Use crystal clock. */
////        Max14830RegWrite(channel, MAX14830_CLKSRC_REG, MAX14830_CLKSRC_PLLBYP_BIT | MAX14830_CLKSRC_CRYST_BIT);
////
////        /* Set reference clock to crystal clock */
////        channel->uartRefClock = channel->clockFreq;
////
////        /* set the baud rate */
//  artemis_max14830_Set_baudrate(channel, 9600);
////
////        /* configure LCR register, 8N1 mode by default */
//  data = MAX14830_LCR_LENGTH_8;
//  module_max14830_Write(channel, MAX14830_REG_LCR, &data, 1);
////
////        if (channel->rs485Mode == TRUE)
////        {
////            /* Enable auto transmit and receive for RS485 */
////            Max14830RegWrite(channel, MAX14830_MODE1_REG, MAX14830_MODE1_TRNSCVCTRL_BIT);
////            Max14830RegWrite(channel, MAX14830_MODE2_REG, MAX14830_MODE2_ECHOSUPR_BIT);
////            Max14830RegWrite(channel, MAX14830_HDPIXDELAY_REG, 0x11);
////        }
////
//
//  /* RMW IRQ With interrupt out */
//  module_max14830_Read(channel, MAX14830_REG_MODE1, 1, &data);
//  data |= MAX14830_MODE1_IRQ_SEL;
//  module_max14830_Write(channel, MAX14830_REG_MODE1, &data, 1);
//
//
//
//    /* Reset FIFOs and enable echo suppression */
//    module_max14830_Read(channel, MAX14830_REG_MODE2, 1, &data);
//    data |= MAX14830_MODE2_FIFO_RST;
//    module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
//
////
////        /* configure FIFO trigger level register */
////        /* RX FIFO trigger for 16 words, TX FIFO trigger for 64 words */
////        regValue = MAX14830_FIFOTRIGLVL_RX(16) | MAX14830_FIFOTRIGLVL_TX(64);
////        Max14830RegWrite(channel, MAX14830_FIFOTRIGLVL_REG, regValue);
//
//  data = MAX14830_FIFOTRIGLVL_RX(16) | MAX14830_FIFOTRIGLVL_TX(64);
//  module_max14830_Write(channel, MAX14830_REG_FIFOTRGLVL, &data, 1);
//
////
////        /* configure flow control levels */
////        regValue = MAX14830_FLOWLVL_RES(48) | MAX14830_FLOWLVL_HALT(96);
////        Max14830RegWrite(channel, MAX14830_FLOWLVL_REG, regValue);
//  data = MAX14830_FLOWLVL_RES(48) | MAX14830_FLOWLVL_HALT(96);
//  module_max14830_Write(channel, MAX14830_REG_FLOWLVL, &data, 1);
//
////
////        /* clear timeout register */
////        Max14830RegWrite(channel, MAX14830_RXTO_REG, 0);
//  data = 0u;
//  module_max14830_Write(channel, MAX14830_REG_RXTIMEOUT, &data, 0);
//
////
////        /* configure LSR interrupt enable register */
////        /* enable RX timeout interrupt */
////        regValue = MAX14830_LSR_RXTO_BIT | MAX14830_LSR_RXOVR_BIT | MAX14830_LSR_RXPAR_BIT |
////                   MAX14830_LSR_FRERR_BIT | MAX14830_LSR_RXNOISE_BIT;
////        Max14830RegWrite(channel, MAX14830_LSR_IRQEN_REG, regValue);
//
//  data = (MAX14830_LSR_RTIMEOUT |
//          MAX14830_LSR_RXOVERRUN |
//          MAX14830_LSR_RXPARITY |
//          MAX14830_LSR_FRAMEERR |
//          MAX14830_LSR_RXNOISE );
//  module_max14830_Write(channel, MAX14830_REG_LSRINTEN, &data, 1);
//
////
////        /* clear FIFO reset */
////        Max14830RegRead(channel, MAX14830_MODE2_REG, &regValue);
////        regValue &= ~MAX14830_MODE2_FIFORST_BIT;
////        Max14830RegWrite(channel, MAX14830_MODE2_REG, regValue);
//  module_max14830_Read(channel, MAX14830_REG_MODE2, 1, &data);
//  data &= ~MAX14830_MODE2_FIFO_RST;
//  module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
//
////
////        /* Set Rx FIFO empty INT invert */
////        Max14830RegRead(channel, MAX14830_MODE2_REG, &regValue);
////        regValue |= MAX14830_MODE2_RXEMPTINV_BIT;
////        Max14830RegWrite(channel, MAX14830_MODE2_REG, regValue);
//  module_max14830_Read(channel, MAX14830_REG_MODE2, 1, &data);
//  data |= MAX14830_MODE2_RX_EMTY_INV;
//  module_max14830_Write(channel, MAX14830_REG_MODE2, &data, 1);
//
////
////        /* get invalid data */
////        Max14830RegRead(channel, MAX14830_REG_RHR_REG, &regValue);
//  module_max14830_Read(channel, MAX14830_REG_RHR, 1, &data);
//
////
////        /* clear IRQ status register by reading it */
////        Max14830RegRead(channel, MAX14830_IRQSTS_REG, &regValue);
//  module_max14830_Read(channel, MAX14830_REG_ISR, 1, &data);
//
////
////        /* disable auto flow control */
////        Max14830RegWrite(channel, MAX14830_FLOWCTRL_REG, 0);
//  data = 0;
//  module_max14830_Write(channel, MAX14830_REG_FLOWCTRL, &data, 1);
//
////  /** Invert Tx & RX */
////  data = MAX14830_IRDA_TX_INV | MAX14830_IRDA_RX_INV;
////  module_max14830_Write(channel, MAX14830_REG_IRDA, &data, 1);
//
////
////        /* enable RX interrupts */
////        regValue = MAX14830_IRQ_RXEMPTY_BIT | MAX14830_IRQ_LSR_BIT;
////        Max14830RegWrite(channel, MAX14830_IRQEN_REG, regValue);
//  data = MAX14830_IRQ_RFIFOEMTY  | MAX14830_IRQ_LSRERR;
//  module_max14830_Write(channel, MAX14830_REG_IRQEN, &data, 1);
////
////        rxSemName[5] = '0' + channel->bUnit;
////        rxSemName[6] = '0' + channel->sUnit;
////        status = SemaphoreBinaryCreate((const char *)rxSemName, QUEUE_TYPE_FIFO, SEM_LOCKED, &channel->rxSem);
////        if (status != SUCCESS)
////        {
////            printk("%s: failed to create semaphore %d\n", __FUNCTION__, status);
////        }
////
////        /* enable the channel */
////        channel->isEnabled = TRUE;
////    }
//  }
//}

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

static uint8_t module_max14830_FastRead(void)
{
    uint8_t write_byte = 0x00;
    uint8_t read_byte = 0x00;
    uint32_t status;

    am_hal_iom_transfer_t xfer;
    xfer.uPeerInfo.ui32SpiChipSelect = 1;
    xfer.ui32InstrLen = 0;
    xfer.ui32Instr = 0;
    xfer.ui32NumBytes = 1;
    xfer.eDirection = AM_HAL_IOM_FULLDUPLEX;
    xfer.pui32TxBuffer = (uint32_t *)&write_byte;
    xfer.pui32RxBuffer = (uint32_t *)&read_byte;
    xfer.bContinue = false;
    xfer.ui8RepeatCount = 0;
    xfer.ui8Priority = 1;
    xfer.ui32PauseCondition = 0;
    xfer.ui32StatusSetClr = 0;

    /* CS high */
    module_max14830_chip_enable();
    status = am_hal_iom_spi_blocking_fullduplex(module.spi.iom.handle, &xfer);
    /* CS low */
    module_max14830_chip_disable();

    //ARTEMIS_DEBUG_PRINTF("FAST READ: status = 0x%02X , READByte = 0x%02X \n", status, read_byte);
    return read_byte;
}

static void module_max14830_Read(emax18430_ComPort_t port, uint8_t reg, uint8_t *rData, uint8_t len)
{
    /* CS high */
    module_max14830_chip_enable();

    artemis_stream_t txstream = {0};
    artemis_stream_t rxstream = {0};
    uint8_t cmd = MAX14830_SPI_READ_CMD & (port << 5);

    cmd |= reg;

    artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_MAX14830_BUFFER_LENGTH);
    artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_MAX14830_BUFFER_LENGTH);
    artemis_stream_reset(&txstream);
    artemis_stream_reset(&rxstream);

    artemis_stream_put(&txstream, cmd);
    artemis_spi_send(&module.spi, true, &txstream);
    artemis_spi_receive(&module.spi, true, &rxstream, len);
    artemis_stream_read(&rxstream, rData, len);

    //am_util_stdio_printf ("SPI received = ");
    //for (uint8_t i=0; i<len; i++){
    //	am_util_stdio_printf ("0x%02X ", rData[i]);
    //}
    //am_util_stdio_printf ("\n");

    /* CS low */
    module_max14830_chip_disable();
}

static void module_max14830_Write(emax18430_ComPort_t port, uint8_t reg, uint8_t *sData, uint8_t len)
{
    /* CS high */
    module_max14830_chip_enable();

    artemis_stream_t txstream = {0};
    uint8_t cmd = MAX14830_SPI_WRITE_CMD | (port << 5);

    cmd |= reg;

    artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_MAX14830_BUFFER_LENGTH);
    artemis_stream_reset(&txstream);
    artemis_stream_put(&txstream, cmd);

    //am_util_stdio_printf ("Sending = len is %d -> ", len);

    for(uint8_t i=0; i<len; i++){
        artemis_stream_put(&txstream, *sData++);
    }

    //am_util_stdio_printf ("0x%02X ", cmd);
    //for (uint8_t i=0; i<len; i++){
    //	am_util_stdio_printf ("0x%02X ", sData[i]);
    //}
    //am_util_stdio_printf ("\n");

    /* send command */
    artemis_spi_send(&module.spi, true, &txstream);

    /* CS low */
    module_max14830_chip_disable();
}

uint8_t module_max14830_port_from_pin(uint8_t pin)
{
    uint8_t port;
    if(pin < 4){
        port = max14830_COM_PORT0;
    }
    else if (pin < 8){
        port = max14830_COM_PORT1;
    }
    else if (pin < 12){
        port = max14830_COM_PORT2;
    }
    else if (pin < 16){
        port = max14830_COM_PORT3;
    }
    else{
        //@todo: ERROR
    }
    return port;
}
