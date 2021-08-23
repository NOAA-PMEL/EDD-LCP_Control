//*****************************************************************************
//
//  am_bsp_pins.c
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @ingroup BSP
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_bsp.h"

//*****************************************************************************
//
//  BUTTON0 pin: Labeled BTN2 on the Apollo3 EVB.
//
//*****************************************************************************
//const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_BUTTON0 =
//{
//    .uFuncSel            = AM_HAL_PIN_16_GPIO,
//    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
//    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
//};
//
////*****************************************************************************
////
////  BUTTON1 pin: Labeled BTN3 on the Apollo3 EVB.
////
////*****************************************************************************
//const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_BUTTON1 =
//{
//    .uFuncSel            = AM_HAL_PIN_18_GPIO,
//    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
//    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
//};
//
////*****************************************************************************
////
////  BUTTON2 pin: Labeled BTN4 on the Apollo3 EVB.
////
////*****************************************************************************
//const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_BUTTON2 =
//{
//    .uFuncSel            = AM_HAL_PIN_19_GPIO,
//    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
//    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
//};


//*****************************************************************************
//
//  IRIDIUM_COM_UART_TX pin: This pin is the IRIDIUM COM_UART transmit pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_IRIDIUM_COM_UART_TX = 
{
  .uFuncSel             = AM_HAL_PIN_14_UART1TX,
  .eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
  
};


//*****************************************************************************
//
//  IRIDIUM_COM_UART_RX pin: This pin is the IRIDIUM COM_UART receive pin.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_IRIDIUM_COM_UART_RX =
{
    .uFuncSel            = AM_HAL_PIN_15_UART1RX
};


//*****************************************************************************
//
//  IRIDIUM_GPIO_ON pin: This pin is the IRIDIUM Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_IRIDIUM_ON =
{
    .uFuncSel            = AM_HAL_PIN_4_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  IRIDIUM_GPIO_RING_IND pin: This pin is the IRIDIUM Ring Indicator pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_IRIDIUM_GPIO_RING_IND =
{
    .uFuncSel            = AM_HAL_PIN_22_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};

//*****************************************************************************
//
//  IRIDIUM_GPIO_NET_AVAIL pin: This pin is the IRIDIUM Network Available pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_IRIDIUM_NET_AVAIL =
{
    .uFuncSel            = AM_HAL_PIN_24_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};


//*****************************************************************************
//
//  GPS_GPIO_ON pin: This pin is the GPS Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPS_ON =
{
    .uFuncSel            = AM_HAL_PIN_26_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  GPS_SDA pin: GPS I2C data signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPS_I2C_SDA =
{
    .uFuncSel            = AM_HAL_PIN_9_M1SDAWIR3,
    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .uIOMnum             = 0
};

//*****************************************************************************
//
//  GPS_SCL pin: GPS I2C clk signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPS_I2C_SCL =
{
    .uFuncSel            = AM_HAL_PIN_8_M1SCL,
    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .uIOMnum             = 0
};


//*****************************************************************************
//
//  GPS_GPIO_GPIO pin: This pin is the GPS EXTINT pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPS_GPIO =
{
    .uFuncSel            = AM_HAL_PIN_10_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};






//*****************************************************************************
//
//  ACCEL_SCK pin: Accelermoter SPI SCK signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_ACCEL_SPI_SCK =
{
    .uFuncSel            = AM_HAL_PIN_5_M0SCK,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .uIOMnum             = 0
};

//*****************************************************************************
//
//  ACCEL_MOSI pin: Accelerometer SPI MOSI signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_ACCEL_SPI_MOSI =
{
    .uFuncSel            = AM_HAL_PIN_7_M0MOSI,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .uIOMnum             = 0
};


//*****************************************************************************
//
//  ACCEL_MISO pin: Accelerometer SPI MISO signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_ACCEL_SPI_MISO =
{
    .uFuncSel            = AM_HAL_PIN_6_M0MISO,
    .uIOMnum             = 0
};


//*****************************************************************************
//
//  ACCEL CS pin: Accelerometer chip select.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_ACCEL_SPI_CS =
{
    .uFuncSel            = AM_HAL_PIN_36_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  ACCEL_INT pin: This pin is the Accelerometer Interrupt pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_ACCEL_INT =
{
    .uFuncSel            = AM_HAL_PIN_37_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};


#ifdef INCLUDE_CRYPTO


#endif

#ifdef INCLUDE_ADC24


#endif


//*****************************************************************************
//
//  PRES_ON pin: This pin is the Pressure Sensor Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_PRES_ON =
{
    .uFuncSel            = AM_HAL_PIN_44_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};


//*****************************************************************************
//
//  PRES_SDA pin: Pressure Sensor I2C data signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_PRES_I2C_SDA =
{
    .uFuncSel            = AM_HAL_PIN_40_M4SDAWIR3,
    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .uIOMnum             = 0
};

//*****************************************************************************
//
//  PRES_SCL pin: Pressure Sensor I2C clk signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_PRES_I2C_SCL =
{
    .uFuncSel            = AM_HAL_PIN_39_M4SCL,
    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .uIOMnum             = 0
};



//*****************************************************************************
//
//  S2U_ON pin: This pin is the Serial-to-UART Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_S2U_ON =
{
    .uFuncSel            = AM_HAL_PIN_45_GPIO,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  S2U_SPI_MISO pin: SPI-to-UART SPI MISO signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_S2U_SPI_MISO =
{
    .uFuncSel            = AM_HAL_PIN_43_M3MISO,
//    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_24K,
    .uIOMnum             = 3
};

//*****************************************************************************
//
//  S2U_SPI_MOSI pin: SPI-to-UART SPI MOSI signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_S2U_SPI_MOSI =
{
    .uFuncSel            = AM_HAL_PIN_38_M3MOSI,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .uIOMnum             = 3
};

//*****************************************************************************
//
//  S2U_SPI_SCK pin: SPI-to-UART SPI SCK signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_S2U_SPI_SCK =
{
    .uFuncSel            = AM_HAL_PIN_42_M3SCK,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .uIOMnum             = 3
};

//*****************************************************************************
//
//  S2U_SPI_CS pin: SPI-to-UART SPI MOSI Chip Select.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_S2U_SPI_CS =
{
    .uFuncSel            = AM_HAL_PIN_17_GPIO,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  S2U_SPI_NIRQ pin: This pin is the SPI-to-UART Interrupt (negative) pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_S2U_SPI_NIRQ =
{
    .uFuncSel            = AM_HAL_PIN_18_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};


//*****************************************************************************
//
//  S2U_SPI_NRESET pin: This pin is the SPI-to-UART Reset pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_S2U_SPI_NRESET =
{
    .uFuncSel            = AM_HAL_PIN_19_GPIO,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
//    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};



//*****************************************************************************
//
//  COM0_POWER_ON pin: This pin is COM0 Output port Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_COM0_POWER_ON =
{
    .uFuncSel            = AM_HAL_PIN_11_GPIO,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  COM1_POWER_ON pin: This pin is COM1 Output port Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_COM1_POWER_ON =
{
    .uFuncSel            = AM_HAL_PIN_12_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  COM2_POWER_ON pin: This pin is COM2 Output port Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_COM2_POWER_ON =
{
    .uFuncSel            = AM_HAL_PIN_13_GPIO,
    
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  COM3_POWER_ON pin: This pin is COM3 Output port Power ON pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_COM3_POWER_ON =
{
    .uFuncSel            = AM_HAL_PIN_34_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};



#ifdef INCLUDE_IMU

#endif


//*****************************************************************************
//
//  LED1 pin.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_LED1 =
{
    .uFuncSel            = AM_HAL_PIN_31_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
//    .ePowerSw            = AM_HAL_GPIO_PIN_POWERSW_VSS
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL
};

//*****************************************************************************
//
//  LED2 pin.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_LED2 =
{
    .uFuncSel            = AM_HAL_PIN_32_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  LED3 pin.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_LED3 =
{
    .uFuncSel            = AM_HAL_PIN_29_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  SC_NSHDN pin: This pin is Super Capacitor Shutdown (negative) pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_SC_NSHDN =
{
    .uFuncSel            = AM_HAL_PIN_0_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  SC_PGOOD pin: This pin is Super Capacitor Power Good pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_SC_PGOOD =
{
    .uFuncSel            = AM_HAL_PIN_28_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};


//*****************************************************************************
//
//  SC_ON pin: This pin is Super Capacitor Power On pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_SC_ON =
{
    .uFuncSel            = AM_HAL_PIN_1_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};



//*****************************************************************************
//
//  CONSOLE_TX pin: This pin is the Console UART transmit pin.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_UART_CONSOLE_TX =
{
    .uFuncSel            = AM_HAL_PIN_48_UART0TX,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  CONSOLE_RX pin: This pin is the Console UART receive pin.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_UART_CONSOLE_RX =
{
    .uFuncSel            = AM_HAL_PIN_49_UART0RX
};


#ifdef INCLUDE_ADC_VOLT_BUS
//*****************************************************************************
//
//  BUS_VOLT_ENABLE pin: This pin is Bus Voltage Read Enable pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_BUS_VOLT_ENABLE =
{
    .uFuncSel            = AM_HAL_PIN_23_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  BUS_VOLTAGE pin: This pin is the Bus Voltage Read Pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_BUS_VOLTAGE =
{
    .uFuncSel            = AM_HAL_PIN_35_ADCSE7,
};
#endif // INCLUDE_ADC_VOLT_BUS





//*****************************************************************************
//
//  PWR_CTRL_ENABLE pin: This pin is the Power Control(piston) Enable pin
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_PWR_CTRL_EN =
{
    .uFuncSel            = AM_HAL_PIN_2_GPIO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};


//*****************************************************************************
//
//  PRES_SDA pin: Pressure Sensor I2C data signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_I2C_CTRL_I2C_SDA =
{
    .uFuncSel            = AM_HAL_PIN_25_M2SDAWIR3,
    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .uIOMnum             = 0
};

//*****************************************************************************
//
//  PRES_SCL pin: Pressure Sensor I2C clk signal.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_I2C_CTRL_SCL =
{
    .uFuncSel            = AM_HAL_PIN_27_M2SCL,
    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .uIOMnum             = 0
};















//*****************************************************************************
//
//  ITM_SWO pin: ITM Serial Wire Output.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_ITM_SWO =
{
//    .uFuncSel            = AM_HAL_PIN_41_SWO,
    .uFuncSel           = AM_HAL_PIN_33_SWO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

//*****************************************************************************
//
//  SWDCK pin: Cortex Serial Wire DCK.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_SWDCK =
{
    .uFuncSel            = AM_HAL_PIN_20_SWDCK
};

//*****************************************************************************
//
//  SWDIO pin: Cortex Serial Wire DIO.
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_LCP_BSP_GPIO_SWDIO =
{
    .uFuncSel            = AM_HAL_PIN_21_SWDIO
};

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
