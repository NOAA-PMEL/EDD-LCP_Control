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

#define MAX14830_NUM_SERIAL_PORTS       ( 4 )
#define MAX14830_WRITE_TASK_PRIORITY    ( 5 )
#define STACK_SIZE                      ( 128 )
TaskHandle_t data_read_int_handle;
SemaphoreHandle_t xSpiMutex = NULL;


static am_hal_iom_config_t IomConfig = {
  .eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
  .ui32ClockFreq        = AM_HAL_IOM_250KHZ,
  .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
};

static void *pIomHandle;
const uint8_t CTSIEn        = (1u << 7);
const uint8_t RFifoEmtyIEn  = (1u << 6);
const uint8_t TFifoEmtyIEn  = (1u << 5);

volatile bool xfer_complete = false;
volatile uint32_t txn_stat = 0;

#define XFER_DATA_SIZE      ( 64 )
static char xfer_data[XFER_DATA_SIZE];

sCircularBufferC_t txBuf[MAX14830_NUM_SERIAL_PORTS];
sCircularBufferC_t rxBuf[MAX14830_NUM_SERIAL_PORTS];

void xfer_complete_callback(void *pCallbackCtxt, uint32_t transactionStatus){
    (void)pCallbackCtxt;
    xfer_complete = true;
    txn_stat = transactionStatus;
}


static void MAX14830_Power_On(void)
{
  am_hal_iom_enable(&pIomHandle);
  am_hal_gpio_output_set(BSP_S2U_SPI_NRESET);
  am_hal_gpio_output_clear(BSP_S2U_ON);
  
}

static void MAX14830_Power_Off(void)
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

static uint32_t MAX14830_FastRead(void)
{
  uint32_t write_byte = 0u;
  uint32_t read_byte = 0u;
  am_hal_iom_transfer_t transfer = 
  { 
    .uPeerInfo = {
      .ui32SpiChipSelect    = 0,
      .ui32I2CDevAddr       = 0,
    },
    .ui32InstrLen         = 0,
    .ui32Instr            = 0,
    .ui32NumBytes         = 1,
    .eDirection           = AM_HAL_IOM_FULLDUPLEX,
    .pui32TxBuffer        = &write_byte,
    .pui32RxBuffer        = &read_byte,
    .bContinue            = false,
    .ui8RepeatCount       = 0,
    .ui8Priority          = 1,
    .ui32PauseCondition   = 0,
    .ui32StatusSetClr     = 0
  };
  
  /** Enable */
//  MAX14830_Power_On();
//  am_hal_iom_enable(&pIomHandle);

  /** Chip Select */
  MAX14830_CS_Set();
  
  am_hal_iom_spi_blocking_fullduplex(pIomHandle, &transfer);
  
  /** Chip Deselect */
  MAX14830_CS_Clear();
  
  /** Disable */
//  MAX14830_Power_Off();

  return read_byte;

}

static uint32_t MAX14830_Read(eMAX18430_ComPort_t port, uint8_t reg, uint32_t len, char *data)
{

  assert(reg <= 0x1F);
  uint32_t write_byte = reg;
  uint32_t read_bytes[256];
  uint32_t interrupts = 0u;
  memset(read_bytes, 0, 256);
  memset(data, 0, len);


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
    .eDirection           = AM_HAL_IOM_FULLDUPLEX,
    .pui32TxBuffer        = &write_byte,
    .pui32RxBuffer        = read_bytes,
    .bContinue            = false,
    .ui8RepeatCount       = 0,
    .ui8Priority          = 1,
    .ui32PauseCondition   = 0,
    .ui32StatusSetClr     = 0
  };
  
  /** Chip Select */
  MAX14830_CS_Set();
  
  am_hal_iom_spi_blocking_fullduplex(pIomHandle, &transfer);

  transfer = (am_hal_iom_transfer_t) {
    .uPeerInfo = {
      .ui32SpiChipSelect    = 0,
      .ui32I2CDevAddr       = 0,
    },
    .ui32InstrLen         = 0,
    .ui32Instr            = 0,
    .ui32NumBytes         = len,
    .eDirection           = AM_HAL_IOM_FULLDUPLEX,
    .pui32TxBuffer        = (uint32_t*)data,
    .pui32RxBuffer        = (uint32_t*)data,
    .bContinue            = false,
    .ui8RepeatCount       = 0,
    .ui8Priority          = 1,
    .ui32PauseCondition   = 0,
    .ui32StatusSetClr     = 0
  };

  am_hal_iom_spi_blocking_fullduplex(pIomHandle, &transfer);
  
  /** Chip Deselect */
  MAX14830_CS_Clear();

  return interrupts;
}

static void 
MAX14830_Write(
  eMAX18430_ComPort_t port, 
  uint8_t reg, 
  char *data, 
  uint32_t len
  )
{
  assert(reg <= 0x1F);
 

  /** Enable */
  MAX14830_Power_On();
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
//  MAX14830_Power_Off();
//  am_hal_iom_disable(&pIomHandle);
  
}



void MAX14830_init(void)
{
  uint32_t status = AM_HAL_STATUS_FAIL;

  /** Initialize the Power Pin */
  am_hal_gpio_pinconfig(BSP_S2U_ON, g_LCP_BSP_S2U_ON);
  MAX14830_Power_Off();
  MAX14830_Power_On();
  
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
  NVIC_EnableIRQ(IOMSTR3_IRQn);

  /** Set the MAX14830 Clock Source to enable Crystal */
  char temp = MAX14830_CLK_CRYSTAL_EN;
  MAX14830_Write(MAX14830_COM_PORT0, MAX14830_REG_CLKSOURCE, &temp, 1);

  /** Start the Read Task */

}

void MAX14830_enable(eMAX18430_ComPort_t port)
{
  /** If the device is off, turn it on */
  uint32_t state = 0;
  am_hal_gpio_state_read(BSP_S2U_ON, AM_HAL_GPIO_OUTPUT_READ, &state);
  if(state == 1)
  {
    MAX14830_Power_On();
    am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false);
  }

  /** Enable the Port Clock */
  char reg = 0;
  MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
  reg &= MAX14830_BRG_CLK_DISABLE;
  MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

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
    MAX14830_Power_On();
    am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false);
  }

  /** Enable the Port Clock */
  char reg = 0;
  MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
  reg &= MAX14830_BRG_CLK_DISABLE;
  MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);


}


void MAX14830_disable(eMAX18430_ComPort_t port)
{
  /** Disable the Port Clock */
  char reg = 0;
  MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
  reg |= MAX14830_BRG_CLK_DISABLE;
  MAX14830_Write(port, MAX14830_REG_BRGCONFIG, &reg, 1);

  /** Check to see if all ports are inactive */
  uint8_t cnt = 0;
  for(uint8_t i=0; i<4; i++)
  {
    reg = 0;
    MAX14830_Read(port, MAX14830_REG_BRGCONFIG, 1, &reg);
    if( (reg & MAX14830_BRG_CLK_DISABLE) > 0)
    {
      cnt++;
    }
  }

  /** If all the ports are deactivated, shut down the power for lower power ops */
  if(cnt ==  MAX14830_NUM_SERIAL_PORTS)
  {
    am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
//    am_hal_gpio_output_set(BSP_S2U_ON); //, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    MAX14830_Power_Off();
  }

}

void MAX14830_Interrupt_Task(void *pvParameters)
{   
    char data[256];
    char *pData = &data[0];

    /** Fast read to the IRQ location */
    while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 500UL )) != pdPASS);
    uint32_t irq = MAX14830_FastRead();
    xSemaphoreGive(xSpiMutex);
    

    /** Walk through IRQs to find out which channels has data */
    for(uint8_t i=0; i<4; i++)
    {
      if(irq | (1u<<i))
      {
        /** Read the number of words in the FIFO (RxFIFOLvl) register */
        uint32_t len=0;
        while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 500UL )) != pdPASS);
        MAX14830_Read((eMAX18430_ComPort_t)i, MAX14830_RHR, len, pData);
        xSemaphoreGive(xSpiMutex);

        /** Push the data into a circular buffer */
        for(uint16_t idx=0; idx<256; idx++)
        {
          BufferC_putc(&rxBuf[i], *pData++);
        }
      }
    }
}

void MAX14830_Write_Task(void  *pvParameters)
{

  eMAX18430_ComPort_t port = (eMAX18430_ComPort_t) pvParameters;
  char data = 0;

  sCircularBufferC_t *pBuf = &txBuf[(uint8_t)port];

  while( xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS( 500UL )) != pdPASS);
  while(BufferC_Get_Size(pBuf))
  {
    BufferC_getc(pBuf, &data);
    MAX14830_Write(port, MAX14830_THR, &data, 1);
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
  
//  MAX14830_FastRead();

  float D = MAX14830_XTAL_FREQ / (16 * baudrate );
  uint32_t DIV = (uint32_t)trunc(D) ;
  uint32_t FRACT = (uint32_t) round(16 * (D - DIV));
  FRACT = FRACT & 0x0000000F;
  uint32_t DIVMSB = (DIV & 0x00000100) >> 8;
  uint32_t DIVLSB = (DIV & 0x000000FF);

  /** Send the BRGConfig */
  
  MAX14830_Write(port, MAX14830_REG_DIVLSB, (char*)&DIVLSB, 1);
  MAX14830_Write(port, MAX14830_REG_DIVMSB, (char*)&DIVMSB, 1);
  MAX14830_Write(port, MAX14830_REG_BRGCONFIG, (char*)&FRACT, 1);
  
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
    MAX14830_Write_Task,   
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
    MAX14830_Write(port, MAX14830_THR, xfer_data, xLen); 
  }
//  while(BufferC_Get_Size(pBuf))
//  {
//    BufferC_getc(pBuf, &cData);
//    MAX14830_Write(port, MAX14830_THR, &cData, 1);
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
  
  
  MAX14830_FastRead();

  MAX14830_Read(MAX14830_COM_PORT0,0, 256, pData);
  
}

uint32_t MAX14830_UART_Read_bytes_waiting(eMAX18430_ComPort_t port)
{
  uint32_t len = BufferC_Get_Size(&rxBuf[(uint8_t)port]);
  return len;
}