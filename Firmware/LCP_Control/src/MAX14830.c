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

//

#define MAX14830_NUM_SERIAL_PORTS       ( 4 )
#define MAX14830_WRITE_TASK_PRIORITY    ( 5 )
#define STACK_SIZE                      ( 128 )


//*****************************************************************************
//
// FreeRTOS variables, tasks and semaphores
//
//*****************************************************************************
TaskHandle_t data_read_int_handle;
SemaphoreHandle_t xSpiMutex = NULL;

//*****************************************************************************
//
// Ambiq IOM static variables
//
//*****************************************************************************
static am_hal_iom_config_t IomConfig = {
  .eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
  .ui32ClockFreq        = AM_HAL_IOM_500KHZ,
  .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
  .pNBTxnBuf            = NULL,
  .ui32NBTxnBufLength       = 0
};

static void *pIomHandle;
const uint8_t CTSIEn        = (1u << 7);
const uint8_t RFifoEmtyIEn  = (1u << 6);
const uint8_t TFifoEmtyIEn  = (1u << 5);

volatile bool xfer_complete = false;
volatile uint32_t txn_stat = 0;

#define XFER_DATA_SIZE      ( 64 )
static char xfer_data[XFER_DATA_SIZE];


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
static void module_MAX14830_Power_On(void);
static void module_MAX14830_Power_Off(void);
static uint32_t module_MAX14830_FastRead(void);
static uint32_t module_MAX14830_Read(
                    eMAX18430_ComPort_t port,
                    uint32_t ui32Instr,
                    uint32_t ui32NumBytes,
                    char *pData );


static void module_MAX14830_Write(
                    eMAX18430_ComPort_t port, 
                    uint8_t reg, 
                    char *data, 
                    uint32_t len
                    );

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void MAX14830_init(void)
{
  uint32_t status = AM_HAL_STATUS_FAIL;

  /** Initialize the Power Pin */
  am_hal_gpio_pinconfig(BSP_S2U_ON, g_LCP_BSP_S2U_ON);
  module_MAX14830_Power_Off();
  module_MAX14830_Power_On();
  
  /** Initialize the Chip Select */
  am_hal_gpio_pinconfig(BSP_S2U_SPI_CS, g_LCP_BSP_S2U_SPI_CS);
  am_hal_gpio_output_set(BSP_S2U_SPI_CS);
  
  /** Initialize the IRQ Input */
  am_hal_gpio_pinconfig(BSP_S2U_SPI_NIRQ, g_LCP_BSP_S2U_SPI_NIRQ);
  
  /** Initialize the RESET line */
  am_hal_gpio_pinconfig(BSP_S2U_SPI_NRESET, g_LCP_BSP_S2U_SPI_NRESET);
  am_hal_gpio_output_clear(BSP_S2U_SPI_NRESET);
  
  
  /** Initialize the SPI Configurations */
  status = am_hal_iom_initialize(3, &pIomHandle);
  printf("INIT: status=%d\n", status);
  status = am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false);
  printf("PWRC: status=%d\n", status);
  status = am_hal_iom_configure(pIomHandle, &IomConfig);
  printf("CONF: status=%d\n", status);
  status = am_hal_iom_enable(pIomHandle);
  printf("ENAB: status=%d\n", status);
  /** Initialie the SPI - MOSI */
  status = am_hal_gpio_pinconfig(BSP_S2U_SPI_MOSI, g_LCP_BSP_S2U_SPI_MOSI);
  printf("MOSI: status=%d\n", status);
  /** Initialie the SPI - MISO */
  status = am_hal_gpio_pinconfig(BSP_S2U_SPI_MISO, g_LCP_BSP_S2U_SPI_MISO);
  printf("MISO: status=%d\n", status);
  /** Initialie the SPI - SCLK */
  status = am_hal_gpio_pinconfig(BSP_S2U_SPI_SCK, g_LCP_BSP_S2U_SPI_SCK);
  printf("SCK : status=%d\n", status);
  
  /** Clear the IOM register-access interrupts */
  am_hal_iom_interrupt_clear(pIomHandle, AM_HAL_IOM_INT_ALL);
  am_hal_iom_interrupt_enable(pIomHandle, AM_HAL_IOM_INT_ERR);
  
  /** Set the NVIC for the IO Master */
//  NVIC_EnableIRQ(IOMSTR3_IRQn);

  /** Set the MAX14830 Clock Source to enable Crystal */
  char temp = MAX14830_CLK_CRYSTAL_EN;
//  module_MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_CLKSOURCE, &temp, 1);

  /** Start the Read Task */
//  MAX14830_disable(MAX14830_COM_PORT0);
}

void MAX14830_enable(eMAX18430_ComPort_t port)
{
  /** If the device is off, turn it on */
  uint32_t state = 0;
  am_hal_gpio_state_read(BSP_S2U_ON, AM_HAL_GPIO_OUTPUT_READ, &state);
  if(state == 1)
  {
    module_MAX14830_Power_On();
    am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false);
  }

  /** Enable the Port Clock */
  char reg = 0;
  module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
  reg &= MAX14830_BRG_CLK_DISABLE;
  module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

  /** Restart the Read Task if currently suspended */
  switch(eTaskGetState(data_read_int_handle))
  {
    case eReady:
      break;
    case eRunning:
    case eBlocked:
      break;
    case eSuspended:
      vTaskResume(data_read_int_handle);
      break;
    case eDeleted:
      break;
    default:
      break;
  }
  vTaskResume(data_read_int_handle);

}

void MAX14830_enable_direct(eMAX18430_ComPort_t port)
{
  /** If the device is off, turn it on */
  uint32_t state = 0;
  am_hal_gpio_state_read(BSP_S2U_ON, AM_HAL_GPIO_OUTPUT_READ, &state);
  if(state == 1)
  {
    module_MAX14830_Power_On();
    am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false);
  }

  /** Enable the Port Clock */
  char reg = 0;
  module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
  reg &= MAX14830_BRG_CLK_DISABLE;
  module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);


}


void MAX14830_disable(eMAX18430_ComPort_t port)
{
  /** Disable the Port Clock */
  char reg = 0;
  module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
  reg |= MAX14830_BRG_CLK_DISABLE;
  module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

  /** Check to see if all ports are inactive */
  uint8_t cnt = 0;
  for(uint8_t i=0; i<4; i++)
  {
    reg = 0;
    module_MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    if( (reg & MAX14830_BRG_CLK_DISABLE) > 0)
    {
      cnt++;
    }
  }

  /** If all the ports are deactivated, shut down the power for lower power ops */
  if(cnt ==  MAX14830_NUM_SERIAL_PORTS)
  {
    am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
//    am_hal_gpio_output_set(BSP_S2U_ON); //, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    module_MAX14830_Power_Off();
  }

}


void MAX14830_Interrupt_Task(void *pvParameters)
{   
    char data[256];
    char *pData = &data[0];

    /** Fast read to the IRQ location */
    while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 500UL )) != pdPASS);
    uint32_t irq = module_MAX14830_FastRead();
    xSemaphoreGive(xSpiMutex);
    

    /** Walk through IRQs to find out which channels has data */
    for(uint8_t i=0; i<4; i++)
    {
      if(irq | (1u<<i))
      {
        /** Read the number of words in the FIFO (RxFIFOLvl) register */
        uint32_t len=0;
        while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 500UL )) != pdPASS);
        module_MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_RHR, len, pData);
        xSemaphoreGive(xSpiMutex);

        /** Push the data into a circular buffer */
        for(uint16_t idx=0; idx<256; idx++)
        {
          BufferC_putc(&rxBuf[i], *pData++);
        }
      }
    }
}

void module_MAX14830_Write_Task(void  *pvParameters)
{

  eMAX18430_ComPort_t port = (eMAX18430_ComPort_t) pvParameters;
  char data = 0;

  sCircularBufferC_t *pBuf = &txBuf[(uint8_t)port];

  while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 500UL )) != pdPASS);
  while(BufferC_Get_Size(pBuf))
  {
    BufferC_getc(pBuf, &data);
    module_MAX14830_Write(port, MAX14830_THR, &data, 1);
  }
  xSemaphoreGive(xSpiMutex);
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
void MAX14830_Set_baudrate(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudrate )
{

//  uint32_t status = AM_HAL_STATUS_FAIL;
  
//  module_MAX14830_FastRead();

  float D = MAX14830_XTAL_FREQ / (16 * baudrate );
  uint32_t DIV = (uint32_t)trunc(D) ;
  uint32_t FRACT = (uint32_t) round(16 * (D - DIV));
  FRACT = FRACT & 0x0000000F;
  uint32_t DIVMSB = (DIV & 0x00000100) >> 8;
  uint32_t DIVLSB = (DIV & 0x000000FF);

  /** Send the BRGConfig */
  
  module_MAX14830_Write(port, MAX14830_REG_DIVLSB, (char*)&DIVLSB, 1);
  module_MAX14830_Write(port, MAX14830_REG_DIVMSB, (char*)&DIVMSB, 1);
  module_MAX14830_Write(port, MAX14830_REG_BRGCONFIG, (char*)&FRACT, 1);
  
}

void MAX14830_UART_Write(eMAX18430_ComPort_t port, char *data, uint32_t len)
{
  
  char taskDesc[configMAX_TASK_NAME_LEN];
  sprintf(taskDesc, "S2U Write CH%u", (uint8_t) port);

  /** Put data into circular buffer */
  for(uint8_t i=0; i < len; i++)
  {
    BufferC_putc(&txBuf[(uint8_t) port], *data++);
  }

  /** Ensure the port is enabled */
  MAX14830_enable(port);

  /** Create Task */
  xTaskCreate( 
    module_MAX14830_Write_Task,   
    taskDesc, 
    STACK_SIZE, 
    (void*) port, 
    tskIDLE_PRIORITY + MAX14830_WRITE_TASK_PRIORITY, 
    NULL);

}

void MAX14830_UART_Write_direct(eMAX18430_ComPort_t port, char *data, uint32_t len)
{
  char taskDesc[configMAX_TASK_NAME_LEN];
  sprintf(taskDesc, "S2U Write CH%u", (uint8_t) port);

  /** Put data into circular buffer */
  for(uint8_t i=0; i < len; i++)
  {
    BufferC_putc(&txBuf[(uint8_t) port], *data++);
  }

  /** Ensure the port is enabled */
  MAX14830_enable_direct(port);

  sCircularBufferC_t *pBuf = &txBuf[(uint8_t)port];
  char cData;
  uint32_t xLen = 0;
  while(BufferC_Get_Size(pBuf))
  {
    xLen = BufferC_gets(pBuf, xfer_data, XFER_DATA_SIZE);
    module_MAX14830_Write(port, MAX14830_THR, xfer_data, xLen); 
  }
//  while(BufferC_Get_Size(pBuf))
//  {
//    BufferC_getc(pBuf, &cData);
//    module_MAX14830_Write(port, MAX14830_THR, &cData, 1);
//  }
}  

uint32_t MAX14830_UART_Read(eMAX18430_ComPort_t port, char *pData, uint32_t max_len)
{
  uint32_t len=0;
  sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];

  while( (BufferC_Get_Size(pBuf) > 0) && (len < max_len) )
  {
    BufferC_getc(pBuf, pData);
    pData++;
    len++;
  }
  return len;
}

uint32_t MAX14830_UART_Read_direct(eMAX18430_ComPort_t port, char *pData, uint32_t max_len)
{
  
  
  uint32_t len=0;
  sCircularBufferC_t *pBuf = &rxBuf[(uint8_t)port];
  
  
//  module_MAX14830_FastRead();

  module_MAX14830_Read(MAX14830_COM_PORT0,1, 36, pData);
  
  printf("%s\n", pData);
  
}

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
static void module_MAX14830_Power_On(void)
{
  am_hal_iom_enable(&pIomHandle);
  am_hal_gpio_output_set(BSP_S2U_SPI_NRESET);
  am_hal_gpio_output_clear(BSP_S2U_ON);
  
}

static void module_MAX14830_Power_Off(void)
{
  am_hal_gpio_output_clear(BSP_S2U_SPI_NRESET);
  am_hal_gpio_output_set(BSP_S2U_ON);
}

static void MAX14830_CS_Set(void)
{
  am_hal_gpio_output_clear(BSP_S2U_SPI_CS);
}

static void MAX14830_CS_Clear(void)
{
  am_hal_gpio_output_set(BSP_S2U_SPI_CS);
}

static uint32_t module_MAX14830_FastRead(void)
{
  uint32_t write_byte = 0u;
  uint32_t read_byte = 0u;
  uint32_t status; 
  
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
  MAX14830_CS_Set();
  
  status = am_hal_iom_spi_blocking_fullduplex(pIomHandle, &xfer);

  /** Chip Deselect */
  MAX14830_CS_Clear();
  
  printf("FAST READ: status = %ul, rb = %u\n", status, read_byte);
  return read_byte;

}



static uint32_t
module_MAX14830_Read(
   eMAX18430_ComPort_t port,
    uint32_t ui32Instr,
    uint32_t ui32NumBytes,
    char *pData )
{
    am_hal_iom_transfer_t Transaction;

    /** Chip select */
    MAX14830_CS_Set();
      
    //
    // Create the transaction.
    //
    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = ui32Instr;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32RxBuffer   = (uint32_t*)pData;
    Transaction.uPeerInfo.ui32SpiChipSelect = 0;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Execute the transction over IOM.
    //
    if (am_hal_iom_blocking_transfer(pIomHandle, &Transaction))
    {
      /** Chip Deselect */
      MAX14830_CS_Clear();
      return -1;
    }
      /** Chip Deselect */
      MAX14830_CS_Clear();
      return 0;
    }

static void module_MAX14830_Write(
  eMAX18430_ComPort_t port, 
  uint8_t reg, 
  char *data, 
  uint32_t len
  )
{
  assert(reg <= 0x1F);
 

  /** Enable */
  module_MAX14830_Power_On();
//  am_hal_iom_enable(&pIomHandle);

  /** Chip Select */
  MAX14830_CS_Set();
  
  /** Prep Write Byte */
  char write_byte= MAX14830_SPI_WRITE_BIT | reg;
  switch(port)
  {
    case MAX14830_COM_PORT0:
      break;
    case MAX14830_COM_PORT1:
      write_byte|= (1u << 5);
      break;
    case MAX14830_COM_PORT2:
      write_byte|= (1u << 6);
      break;
    case MAX14830_COM_PORT3:
      write_byte|= (3u << 5);
      break;
    default:
      /** Error */
      break;
  }

  am_hal_iom_transfer_t transfer = 
  {
    .uPeerInfo = {
      .ui32SpiChipSelect    = 0,
      .ui32I2CDevAddr       = 0,
    },
    .ui32InstrLen         = 0,
    .ui32Instr            = 0,
    .ui32NumBytes         = 1,
    .eDirection           = AM_HAL_IOM_TX,
    .pui32TxBuffer        = (uint32_t*) &write_byte,
    .pui32RxBuffer        = NULL,
    .bContinue            = false,
    .ui8RepeatCount       = 0,
    .ui8Priority          = 1,
    .ui32PauseCondition   = 0,
    .ui32StatusSetClr     = 0
  };

  am_hal_iom_blocking_transfer(pIomHandle, &transfer);
 
  
  transfer.pui32TxBuffer = (uint32_t*) data;
  transfer.ui32NumBytes = len;
  

  am_hal_iom_blocking_transfer(pIomHandle, &transfer);
  
  /** After transfer, turn Chip Select off */
  MAX14830_CS_Clear();
  
  /** Disable */
//  module_MAX14830_Power_Off();
//  am_hal_iom_disable(&pIomHandle);
  
}