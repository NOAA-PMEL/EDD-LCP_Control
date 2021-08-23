/**! @file MAX14830.h
 * @brief Maxim SPI-to-UART Converter
 *
 * @author Matt Casari, matthew.casari@noaa.gov
 * @date August 11, 2021
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the MAX14830 SPI to UART Converter for 4 channels
 *
 *
 * @bug  No known bugs
 */
#ifndef _MAX14830_H
#define _MAX14830_H


#include <stdint.h>

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


/** System Settings */
#define MAX14830_XTAL_FREQ              ( 4000000u )


typedef enum
{
  MAX14830_COM_PORT0    = 0,
  MAX14830_COM_PORT1    = 1,
  MAX14830_COM_PORT2    = 2,
  MAX14830_COM_PORT3    = 3  
}eMAX18430_ComPort_t;


typedef enum
{
  MAX14830_COM_BAUDRATE_1200    = 1200,
  MAX14830_COM_BAUDRATE_2400    = 2400,
  MAX14830_COM_BAUDRATE_4800    = 4800,
  MAX14830_COM_BAUDRATE_9600    = 9600,
  MAX14830_COM_BAUDRATE_19200   = 19200,
  MAX14830_COM_BAUDRATE_28800   = 28800,
  MAX14830_COM_BAUDRATE_38400   = 38400,
  MAX14830_COM_BAUDRATE_57600   = 57600,
  MAX14830_COM_BAUDRATE_115200  = 115200,
  MAX14830_COM_BAUDRATE_230400  = 230400
}eMAX14830_Baudrate_t;

//typedef struct
//{
//  
//  
//}sMAX14830_t;


void MAX14830_init(void);

void MAX14830_enable(eMAX18430_ComPort_t port);

void MAX14830_disable(eMAX18430_ComPort_t port);

void MAX14830_Set_baudrate(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate );
void MAX14830_UART_Write(eMAX18430_ComPort_t port, char *data, uint32_t len);
void MAX14830_UART_Write_direct(eMAX18430_ComPort_t port, char *data, uint32_t len);
uint32_t MAX14830_UART_Read(eMAX18430_ComPort_t port, char *pData, uint32_t max_len);
uint32_t MAX14830_UART_Read_bytes_waiting(eMAX18430_ComPort_t port);
uint32_t MAX14830_UART_Read_direct(eMAX18430_ComPort_t port, char *pData, uint32_t max_len);
#endif // _MAX14830_H