#include "artemis_max14830.h"


/** Simplified COMMS protocol from Keller */
/** 
    Communication Protocol
    D-Line OEM-transmitter samples only on request.
    The idle state is the sleep mode to save power.
    Sequence for data acquisition:
    1. Request measurement
    2 bytes from master
    2. Await the end of conversion (three ways)
    - Simple delay of 8 ms
    - Polling of the �Busy?� flag [5] in the
    status byte (only one byte reading needed)
    - Event triggering by the additional �EOC�
    handshake pin (goes to VDD)
    3. Read out measurement results
    1 byte from master, 3�5 bytes from slave
    4. Interpretation of new data
    P [bar] = P min�P max 16384�49152
    T [�C] = -50�150 �C 384�64384
*/

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
#include "artemis_pa9ld.h"
#include "artemis_i2c.h"
#include "artemis_debug.h"
#include "artemis_stream.h"

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
#define ARTEMIS_PA9LD_BUFFER_LENGTH     ( 16 ) 
#define ARTEMIS_PA9LD_I2C_ADDRESS       ( 0x40 )


/** PA9LD Command Bytes */
#define PA9LD_CMD_REQUEST_MEASURE       ( 0xAC )

/** PA9LD MTP Address */
#define PA9LD_MTP_CUST_ID0              ( 0x00 )
#define PA9LD_MTP_CUST_ID1              ( 0x01 )
#define PA9LD_MTP_SCALING_0             ( 0x12 )
#define PA9LD_MTP_SCALING_1             ( 0x13 )
#define PA9LD_MTP_SCALING_2             ( 0x14 )
#define PA9LD_MTP_SCALING_3             ( 0x15 )
#define PA9LD_MTP_SCALING_4             ( 0x16 )

#define PA9LD_STATUS_BIT                ( 0x20 )

//*****************************************************************************
//
//  Register Defines
//
//*****************************************************************************


//*****************************************************************************
//
// Structs, Enums & Typedefs
//
//*****************************************************************************
typedef uint8_t module_buffer_t[ARTEMIS_PA9LD_BUFFER_LENGTH];
typedef struct s_module_t
{
    artemis_i2c_t i2c;
    module_buffer_t txbuffer;
    module_buffer_t rxbuffer;
    struct {
      uint32_t pin;
      am_hal_gpio_pincfg_t *pinConfig;
    }power;
    module_scaling_t scaling;
    module_manufacturer_t manufacturer;
} module_t;

typedef struct s_pa9ld_data_t
{
  float pressure;
  float temperature;
}pa9ld_data_t;

typedef union {
  uint32_t u32;
  float f;
}u32_to_float_t;

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************
static module_t module;

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************


//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
static void module_pa9ld_read_sensor(pa9ld_data_t *data);
static float module_pa9ld_convert_pressure(uint32_t u32Pressure);
static float module_pa9ld_convert_temperature(uint32_t u32Temp);
static bool module_pa9ld_read_status(void);
static int32_t module_pa9ld_read_with_status(artemis_i2c_t *i2c, 
                                             artemis_stream_t *rxstream, 
                                             uint32_t numBytes, 
                                             uint32_t attempts);
static uint32_t module_pa9ld_read_unique_product_code(void);
static void module_pa9ld_convert_memory_manufacturer(uint8_t *data, module_manufacturer_t *m);
static void module_pa9ld_convert_memory_pressure( uint8_t *data, 
                                                  module_scaling_t *scaling);
static float module_pa9ld_convert_memory_pressure_value(uint8_t *data);
static void module_pa9ld_read_memory_pressure(module_scaling_t *scaling);
static void module_pa9ld_read_memory(module_scaling_t *scaling, 
                                module_manufacturer_t *manufacturer);

void artemis_pa9ld_initialize(const am_hal_gpio_pincfg_t *power, 
                              const uint32_t power_pin)
{
    artemis_i2c_t *i2c = &module.i2c;
    
    module.manufacturer.custom_id = 0;
    module.scaling.high = 0.0f;
    module.scaling.low = 0.0f;
    module.power.pinConfig = (am_hal_gpio_pincfg_t*)power;
    module.power.pin = power_pin;

    i2c->address = ARTEMIS_PA9LD_I2C_ADDRESS;
    
    i2c->iom.module = 4;
    i2c->iom.config.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
    i2c->iom.config.ui32ClockFreq = AM_HAL_IOM_100KHZ;
    artemis_iom_initialize(&i2c->iom);

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCL, g_AM_BSP_GPIO_IOM4_SCL));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SDA, g_AM_BSP_GPIO_IOM4_SDA));
    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.power.pin, *module.power.pinConfig));

    /** Turn PA-9LD On */
    artemis_pa9ld_power_off();
    artemis_pa9ld_power_on();
    
    /** Read the module serial number & scaling coefficients */    
    module_pa9ld_read_memory(&module.scaling, &module.manufacturer);
    
    
    
}

void artemis_pa9ld_power_on(void)
{
  am_hal_gpio_output_clear(module.power.pin);
}

void artemis_pa9ld_power_off(void)
{
  am_hal_gpio_output_set(module.power.pin);
}

void artemis_pa9ld_get_calibration(module_manufacturer_t *manufacturer, 
                                  module_scaling_t *scaling )
{
  manufacturer = &module.manufacturer;
  scaling = &module.scaling;
}
void artemis_pa9ld_read(float *pressure, float *temperature)
{
  pa9ld_data_t data;
  
  module_pa9ld_read_sensor(&data);
  
  *pressure = data.pressure;
  *temperature = data.temperature;
  
}

//void artemis_pa9ld_read_memory();
//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************
static void module_pa9ld_read_sensor(pa9ld_data_t *data)
{
  artemis_stream_t rxstream = {0};
  artemis_stream_t txstream = {0};
  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  
  artemis_i2c_t *i2c = &module.i2c;

  /** Retrieve Measurement Data */
  artemis_stream_put(&txstream, PA9LD_CMD_REQUEST_MEASURE);
  artemis_i2c_send(i2c, true, &txstream);
  
  /** @todo Add fail-out mechanism */
//  while(!module_pa9ld_read_status());
//  
//  artemis_i2c_receive(i2c, true, &rxstream, 5);
  module_pa9ld_read_with_status(i2c, &rxstream, 5, 10);
//  module_pa9ld_read_with_status(&rxstream, 5, 10);
  
  uint8_t temp;
  /** Skip the status byte */
//  artemis_stream_get(&rxstream, &temp);

  /** Get the 2 bytes of Pressure */
  artemis_stream_get(&rxstream, &temp);
  uint32_t pressure = temp << 8;
  
  artemis_stream_get(&rxstream, &temp);
  pressure |= temp;
  
//  printf("px = %u, ", pressure);
    
  /** Get the 2 bytes of Pressure */
  artemis_stream_get(&rxstream, &temp);
  uint32_t temperature = temp << 8;
  
  artemis_stream_get(&rxstream, &temp);
  temperature |= temp;
//  printf("tx = %u\n", temperature);
  
  /** Convert the pressure */
  data->pressure = module_pa9ld_convert_pressure(pressure);
  
  /** Convert the temperature */
  data->temperature = module_pa9ld_convert_temperature(temperature);
}



static float module_pa9ld_convert_pressure(uint32_t u32Pressure)
{
  if(u32Pressure < 16384)
  {
    u32Pressure = 0;
  } else {
    u32Pressure -= 16384;
  }
//  u32Pressure -= 16384;
//  
//  int32 pTemp = (int32_t)u32Pressure;
//  pTemp -= -16384;
  float fPressure =  (float) (u32Pressure);
  fPressure *= module.scaling.diff;
  fPressure /= 32768;
  fPressure += module.scaling.low;
  
  return fPressure;
}

static float module_pa9ld_convert_temperature(uint32_t u32Temp)
{
  u32Temp = u32Temp >> 4;
  float fTemp = (float)(u32Temp);
  fTemp -= 24;
  fTemp *= 0.05;
  fTemp -= 50;
  
  return fTemp;
}

//static bool module_pa9ld_read_status(void)
//{
//  artemis_stream_t rxstream = {0};
//  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
//  
//  artemis_i2c_t *i2c = &module.i2c;
//  artemis_i2c_receive(i2c, true, &rxstream, 1);
//  
//  uint8_t data;
//  artemis_stream_get(&rxstream, &data);
//  data = data & PA9LD_STATUS_BIT;
//  return data;
//}

static uint32_t module_pa9ld_read_unique_product_code(void)
{
  artemis_stream_t rxstream = {0};
  artemis_stream_t txstream = {0};
  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  artemis_i2c_t *i2c = &module.i2c;
  uint32_t id0;
  uint32_t id1;
  
  /** Retrieve Cust_ID0 */
  artemis_stream_put(&txstream, PA9LD_MTP_CUST_ID0);
  artemis_i2c_send(i2c, true, &txstream);
  /** @todo Add fail-out mechanism */
//  while(!module_pa9ld_read_status());
//  
//  artemis_i2c_receive(i2c, true, &rxstream, 3);
  module_pa9ld_read_with_status(i2c, &rxstream, 3, 10);
  uint8_t temp;
  
  artemis_stream_get(&rxstream, &temp);
  id0 = temp << 8;
  artemis_stream_get(&rxstream, &temp);
  id0 |= temp;
  
  /** Retrieve Cust_ID1 */
  artemis_stream_put(&txstream, PA9LD_MTP_CUST_ID1);
  artemis_i2c_send(i2c, true, &txstream);
  /** @todo Add fail-out mechanism */
//  while(!module_pa9ld_read_status());
//  
//  artemis_i2c_receive(i2c, true, &rxstream, 3);
  module_pa9ld_read_with_status(i2c, &rxstream, 3, 10);
  
  artemis_stream_get(&rxstream, &temp);
  id1 = temp << 8;
  artemis_stream_get(&rxstream, &temp);
  id1 |= temp;
  
  
  
  /** Create Product Code */
  return ((id1 << 16) | id0 );
  
}

static int32_t module_pa9ld_read_with_status(artemis_i2c_t *i2c, artemis_stream_t *rxstream, uint32_t numBytes, uint32_t attempts)
{
  int32_t result = -1;
  
  uint8_t temp;
  do{
      artemis_stream_reset(rxstream);
      artemis_i2c_receive(i2c, true, rxstream, numBytes);
      artemis_stream_get(rxstream, &temp);
    
  }while((temp & PA9LD_STATUS_BIT) && (--attempts > 0));
//  printf("%u, %ul\n", temp & PA9LD_STATUS_BIT, attempts);
  
  if( !(temp & PA9LD_STATUS_BIT))
  {
    result = 0;
  }
  
  return result;
}

//static float module_pa9ld_read_low_pressure_limit(void)
//{
//  artemis_stream_t rxstream = {0};
//  artemis_stream_t txstream = {0};
//  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
//  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
//  
//  artemis_i2c_t *i2c = &module.i2c;
//  uint32_t scale1;
//  uint32_t scale2;
//  uint8_t temp;
//  
//  /** Retrieve Scaling1 */
//  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_1);
//  artemis_i2c_send(i2c, true, &txstream);
//  am_hal_systick_delay_us(10000);
//  module_pa9ld_read_with_status(i2c, &rxstream, 3, 10);
//  
//  artemis_stream_get(&rxstream, &temp);
//  scale1 = temp << 8;
//  artemis_stream_get(&rxstream, &temp);
//  scale1 |= temp;
//  
//  
//  /** Reset the buffers */
//  artemis_stream_reset(&rxstream);
//  artemis_stream_reset(&txstream);
//  
//  /** Retrieve Scaling2 */
//  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_2);
//  artemis_i2c_send(i2c, true, &txstream);
//  /** @todo Add fail-out mechanism */
////  while(!module_pa9ld_read_status());
////  
////  artemis_i2c_receive(i2c, true, &rxstream, 3);
////  do{
////      artemis_stream_reset(&rxstream);
////      artemis_i2c_receive(i2c, true, &rxstream, 3);
////      artemis_stream_get(&rxstream, &temp);
////  }while((temp & PA9LD_STATUS_BIT));
//  module_pa9ld_read_with_status(i2c, &rxstream, 3, 10);
////  artemis_stream_get(&rxstream, &temp);
//  artemis_stream_get(&rxstream, &temp);
//  scale2 = temp << 8;
//  artemis_stream_get(&rxstream, &temp);
//  scale2 |= temp;
//  
//  
//  uint32_t pressure =  ((scale2 << 16) | scale1);
//  
//  u32_to_float_t low_p;
//  low_p.u32 = pressure;
//  
//  return low_p.f;
//}


//static float module_pa9ld_read_high_pressure_limit(void)
//{
//  artemis_stream_t rxstream = {0};
//  artemis_stream_t txstream = {0};
//  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
//  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
//  
//  artemis_i2c_t *i2c = &module.i2c;
//  uint32_t scale3;
//  uint32_t scale4;
//  
//  /** Retrieve Scaling1 */
//  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_1);
//  artemis_i2c_send(i2c, true, &txstream);
//  /** @todo Add fail-out mechanism */
////  while(!module_pa9ld_read_status());
//  
////  artemis_i2c_receive(i2c, true, &rxstream, 3);
//  module_pa9ld_read_with_status(i2c, &rxstream, 3, 10);
//  uint8_t temp;
//  
////  artemis_stream_get(&rxstream, &temp);
//  artemis_stream_get(&rxstream, &temp);
//  scale3 = temp << 8;
//  artemis_stream_get(&rxstream, &temp);
//  scale3 |= temp;
//  
//  artemis_stream_reset(&rxstream);
//  artemis_stream_reset(&txstream);
//  
//  /** Retrieve Scaling2 */
//  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_2);
//  artemis_i2c_send(i2c, true, &txstream);
//  /** @todo Add fail-out mechanism */
////  while(!module_pa9ld_read_status());
//  
////  artemis_i2c_receive(i2c, true, &rxstream, 3);
//  module_pa9ld_read_with_status(i2c, &rxstream, 3, 10);
//  
////  artemis_stream_get(&rxstream, &temp);
//  artemis_stream_get(&rxstream, &temp);
//  scale4 = temp << 8;
//  artemis_stream_get(&rxstream, &temp);
//  scale4 |= temp;
//  
//  
//  uint32_t pressure =  ((scale4 << 16) | scale3);
//  
//  u32_to_float_t high_p;
//  high_p.u32 = pressure;
//  
//  return high_p.f;
//  
//}

static void module_pa9ld_convert_memory_manufacturer(uint8_t *data, module_manufacturer_t *m)
{
  uint16_t scale0;
  
  /** Read the Scaling0 Bytes */
  scale0 = (uint16_t) *data << 8;
  data++;
  scale0 |= *data;
  
  uint16_t temp2;
  temp2 = scale0 & 0xF800;
  temp2 = temp2 >> 11;
  temp2 += 2010;
  m->year = temp2;
  m->month = (scale0 & 0x0780) >> 7;
  m->day = (scale0 &  0x007C) >> 2;
  m->mode = (ePA9LD_PMode_t) (scale0 & 0x0003);
}

static float module_pa9ld_convert_memory_pressure_value(uint8_t *data)
{
  uint32_t pressure = (uint32_t) *data++ << 24;
  pressure |= (*data++ << 16);
  pressure |= (*data++ << 8);
  pressure |= (*data);
  
  union Data {
    float f;
    uint32_t u32;
  };
  
  union Data p;
  
  p.u32 = pressure;
  
  return p.f;
}

static void module_pa9ld_convert_memory_pressure( uint8_t *data, 
                                                  module_scaling_t *scaling)
{
  
  /** Convert the low pressure scaling */
  scaling->low = module_pa9ld_convert_memory_pressure_value(data);
  data += 4;
  
  /** Convert the low pressure scaling */
  scaling->high = module_pa9ld_convert_memory_pressure_value(data);
  
  /** Calculate diff */
  scaling->diff = scaling->high - scaling->low;

}

static void module_pa9ld_read_memory_pressure(module_scaling_t *scaling)
{
  /** Prep for I2C messages */
  artemis_stream_t rxstream = {0};
  artemis_stream_t txstream = {0};
  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  
  artemis_i2c_t *i2c = &module.i2c;
  
  /** Read low values */
  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_1);
  artemis_i2c_send(i2c, true, &txstream);
  am_hal_systick_delay_us(8000);
  module_pa9ld_read_with_status(i2c, &rxstream, 3, 20);
  
  uint8_t data[4];
  uint8_t *pData = &data[0];
  artemis_stream_read(&rxstream, pData, 2);
  pData += 2;
// 
  
  artemis_stream_reset(&rxstream);
  artemis_stream_reset(&txstream);
  
  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_2);
  artemis_i2c_send(i2c, true, &txstream);
  am_hal_systick_delay_us(8000);
  module_pa9ld_read_with_status(i2c, &rxstream, 3, 20);
  
  
  artemis_stream_read(&rxstream, pData, 2);
  
  
  scaling->low = module_pa9ld_convert_memory_pressure_value(data);
  
  /** Read high values */
  
  artemis_stream_reset(&rxstream);
  artemis_stream_reset(&txstream);
  
  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_3);
  artemis_i2c_send(i2c, true, &txstream);
  am_hal_systick_delay_us(8000);
  module_pa9ld_read_with_status(i2c, &rxstream, 3, 20);
  
//  uint8_t data[4];
  pData = &data[0];
  artemis_stream_read(&rxstream, pData, 2);
  pData += 2;
// 
  
  artemis_stream_reset(&rxstream);
  artemis_stream_reset(&txstream);
  
  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_4);
  artemis_i2c_send(i2c, true, &txstream);
  am_hal_systick_delay_us(8000);
  module_pa9ld_read_with_status(i2c, &rxstream, 3, 20);
  
  
  artemis_stream_read(&rxstream, pData, 2);
  
  
  scaling->high = module_pa9ld_convert_memory_pressure_value(data);
  
  scaling->diff = scaling->high - scaling->low;
  

  
}
static void module_pa9ld_read_memory(module_scaling_t *scaling, 
                                module_manufacturer_t *manufacturer)
{
  /** Prep for I2C messages */
  artemis_stream_t rxstream = {0};
  artemis_stream_t txstream = {0};
  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
  
  artemis_i2c_t *i2c = &module.i2c;
  
  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_0);
  artemis_i2c_send(i2c, true, &txstream);
  module_pa9ld_read_with_status(i2c, &rxstream, 10, 20);
  
  uint8_t data[10];
  uint8_t *pData = &data[0];
  artemis_stream_read(&rxstream, pData, 10);
  
  /** Convert the manufacturer data */
  module_pa9ld_convert_memory_manufacturer(pData, manufacturer);
  pData++;
  
  
  /** Read the Pressure Scaling Values */
  module_pa9ld_read_memory_pressure(scaling);
  /** Converte the pressure scaling values */
//  module_pa9ld_convert_memory_pressure(pData, scaling);
  
  /** Read the Custom ID */
  manufacturer->custom_id = module_pa9ld_read_unique_product_code();

}
    
    
//static void module_pa9ld_read_manufacturer(module_manufacturer_t *manufacturer)
//{
//    /** Read the Scaling0 Data */
//  artemis_stream_t rxstream = {0};
//  artemis_stream_t txstream = {0};;
//  artemis_stream_setbuffer(&rxstream, module.rxbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
//  artemis_stream_setbuffer(&txstream, module.txbuffer, ARTEMIS_PA9LD_BUFFER_LENGTH);
//  
//  artemis_i2c_t *i2c = &module.i2c;
//  uint16_t scale0;
//
//  /** Retrieve Scaling1 */
//  artemis_stream_put(&txstream, PA9LD_MTP_SCALING_0);
//  artemis_i2c_send(i2c, true, &txstream);
//  /** @todo Add fail-out mechanism */
//   am_hal_systick_delay_us(10000);
//  module_pa9ld_read_with_status(i2c, &rxstream, 3, 10);
//  uint8_t temp;
//  
////  /** Read but skip the status byte */
////  artemis_stream_get(&rxstream, &temp);
//  
//  /** Read the Scaling0 Bytes */
//  artemis_stream_get(&rxstream, &temp);
//  scale0 = temp << 8;
//  artemis_stream_get(&rxstream, &temp);
//  scale0 |= temp;
//  
//  uint16_t temp2;
//  temp2 = scale0 & 0xF800;
//  temp2 = temp2 >> 11;
//  temp2 += 2010;
//  manufacturer->year = temp2;
////  manufacturer->year = ((scale0 & 0xF800) >> 10 + 2010);
//  manufacturer->month = (scale0 & 0x0780) >> 7;
//  manufacturer->day = (scale0 &  0x007C) >> 2;
//  manufacturer->mode = (ePA9LD_PMode_t) (scale0 & 0x0003);
//  
//  
//  manufacturer->custom_id = module_pa9ld_read_unique_product_code();
//}