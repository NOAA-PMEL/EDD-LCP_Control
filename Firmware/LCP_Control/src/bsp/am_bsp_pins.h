//*****************************************************************************
//
//  am_bsp_pins.h
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @addtogroup apollo3_bsp BSP for the Apollo3 EVB.
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

#ifndef AM_BSP_PINS_H
#define AM_BSP_PINS_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//  IRIDIUM_TX pin: Iridium UART Tx Pin 
//
//*****************************************************************************
#define BSP_IRIDIUM_COM_UART_TX     ( 14 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IRIDIUM_COM_UART_TX;

//*****************************************************************************
//
//  IRIDIUM_RX pin: Iridium UART Rx Pin 
//
//*****************************************************************************
#define BSP_IRIDIUM_COM_UART_RX     ( 15 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IRIDIUM_COM_UART_RX;

//*****************************************************************************
//
//  IRIDIUM_ON pin: Iridium ON Pin 
//
//*****************************************************************************
#define BSP_IRIDIUM_GPIO_ON     ( 4 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IRIDIUM_ON;

//*****************************************************************************
//
//  IRIDIUM_RING_IND pin: Iridium Ring Indicator Pin
//
//*****************************************************************************
#define BSP_IRIDIUM_GPIO_RING_IND   ( 22 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IRIDIUM_RING_IND;

//*****************************************************************************
//
//  IRIDIUM_NET_AVAIL pin: Iridium Network Available Pin 
//
//*****************************************************************************
#define BSP_IRIDIUM_GPIO_NET_AVAIL ( 24 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IRIDIUM_NET_AVAIL;


//*****************************************************************************
//
//  GPS_POWER pin: GPS Power Pin 
//
//*****************************************************************************
#define BSP_GPS_GPIO_ON         ( 26 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_GSP_ON;

//*****************************************************************************
//
//  GPS_SDA pin: GPS I2C SDA Pin
//
//*****************************************************************************
#define BSP_GPS_I2C_SDA     ( 9 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_GPS_I2C_SDA;

//*****************************************************************************
//
//  GPS_SCL pin: GPS I2C SCL Pin
//
//*****************************************************************************
#define BSP_GPS_I2C_SCL     ( 8 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_GPS_I2C_SCL;

//*****************************************************************************
//
//  GPS_GPIO pin: GPS GPIO Pin
//
//*****************************************************************************
#define BSP_GPS_GPIO     ( 10 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_GPS_GPIO;

//*****************************************************************************
//
//  ACCEL_SCK pin: Accelerometer SCK (clock) Pin
//
//*****************************************************************************
#define BSP_ACCEL_SPI_SCK     ( 5 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ACCEL_SPI_SCK;

//*****************************************************************************
//
//  ACCEL_MOSI pin: Accelerometer MOSI(Data Out) Pin
//
//*****************************************************************************
#define BSP_ACCEL_SPI_MOSI     ( 7 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ACCEL_SPI_MOSI;


//*****************************************************************************
//
//  ACCEL_MISO pin: Accelerometer MISO (Data In) Pin
//
//*****************************************************************************
#define BSP_ACCEL_SPI_MISO     ( 6 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ACCEL_SPI_MISO;



//*****************************************************************************
//
//  ACCEL_CS pin: Accelerometer CS (Chip select) Pin
//
//*****************************************************************************
#define BSP_ACCEL_SPI_CS     ( 36 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ACCEL_SPI_CS;


//*****************************************************************************
//
//  ACCEL_INT pin: Accelerometer INT (interrupt) Pin
//
//*****************************************************************************
#define BSP_ACCEL_INT     ( 37 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ACCEL_INT;


#ifdef INCLUDE_CRYPTO
//*****************************************************************************
//
//  CRYPTO_ON pin: Crypto Power Pin
//
//*****************************************************************************
#define BSP_CRYPTO_ON     ( 16 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_CRYPTO_ON;


//*****************************************************************************
//
//  CRYPTO_SCL pin: Crypto SCL (Clock) Pin
//
//*****************************************************************************
#define BSP_CRYPTO_I2C_SCL     ( 8 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_CRYPTO_I2C_SCL;


//*****************************************************************************
//
//  CRYPTO_SCL pin: Crypto SDA (Data) Pin
//
//*****************************************************************************
#define BSP_CRYPTO_I2C_SDA     ( 9 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_CRYPTO_I2C_SDA;
#endif 


#ifdef INCLUDE_ADC24
//*****************************************************************************
//
//  ADC24_ON pin: ADC24 (24bit Analog-to-digital converter) Power Pin
//
//*****************************************************************************
#define BSP_ADC24_ON     ( 44 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ADC24_ON;


//*****************************************************************************
//
//  ADC24_SCL pin: ADC24 (24bit Analog-to-digital converter) I2C SCL (Clock) Pin
//
//*****************************************************************************
#define BSP_ADC24_I2C_SCL     ( 8 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ADC24_I2C_SCL;


//*****************************************************************************
//
//  ADC24_SDA pin: ADC24 (24bit Analog-to-digital converter) I2C SDA (Data) Pin
//
//*****************************************************************************
#define BSP_ADC24_I2C_SDA     ( 9 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ADC24_I2C_SDA;


//*****************************************************************************
//
//  ADC24_NDREADY pin: ADC24 Data Ready (inverted) Pin
//
//*****************************************************************************
#define BSP_ADC24_NDREADY     ( 3 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_ADC24_NDREADY;


#endif

//*****************************************************************************
//
//  PRES_ON pin: Pressure Sensor Power Pin
//
//*****************************************************************************
#define BSP_PRES_ON     ( 44 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_PRES_ON;

//*****************************************************************************
//
//  PRES_I2C_SCL pin: Pressure Sensor I2C SCL (Clock) Pin
//
//*****************************************************************************
#define BSP_PRES_I2C_SCL     ( 39 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_PRES_I2C_SCL;

//*****************************************************************************
//
//  PRES_I2C_SDA pin: Pressure Sensor I2C SDA (Data) Pin
//
//*****************************************************************************
#define BSP_PRES_I2C_SDA     ( 40 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_PRES_I2C_SDA;


//*****************************************************************************
//
//  S2U_ON pin: SPI-to-UART Power Pin
//
//*****************************************************************************
#define BSP_S2U_ON     ( 45 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_S2U_ON;

//*****************************************************************************
//
//  S2U_SPI_MISO pin: SPI-to-UART SPI MISO (Data In) Pin
//
//*****************************************************************************
#define BSP_S2U_SPI_MISO     ( 43 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_S2U_SPI_MISO;

//*****************************************************************************
//
//  S2U_SPI_MOSI pin: SPI-to-UART SPI MOSI (Data Out) Pin
//
//*****************************************************************************
#define BSP_S2U_SPI_MOSI     ( 38 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_S2U_SPI_MOSI;


//*****************************************************************************
//
//  S2U_SPI_SCK pin: SPI-to-UART SPI SCK (Clock) Pin
//
//*****************************************************************************
#define BSP_S2U_SPI_SCK     ( 42 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_S2U_SPI_SCK;

//*****************************************************************************
//
//  S2U_SPI_NCS pin: SPI-to-UART SPI CS (Chip Select, Negative) Pin
//
//*****************************************************************************
#define BSP_S2U_SPI_CS     ( 17 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_S2U_SPI_CS;


//*****************************************************************************
//
//  S2U_SPI_NIRQ pin: SPI-to-UART SPI IRQ (Interrupt, Negative) Pin
//
//*****************************************************************************
#define BSP_S2U_SPI_NIRQ     ( 18 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_S2U_SPI_NIRQ;


//*****************************************************************************
//
//  S2U_SPI_NRST pin: SPI-to-UART SPI RESET (Reset, Negative) Pin
//
//*****************************************************************************
#define BSP_S2U_SPI_NRESET     ( 19 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_S2U_SPI_NRESET;


//*****************************************************************************
//
//  COM0_POWER_ON: COM0 Power Output Pin
//
//*****************************************************************************
#define BSP_COM0_POWER_ON     ( 11 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_COM0_POWER_ON;


//*****************************************************************************
//
//  COM1_POWER_ON: COM1 Power Output Pin
//
//*****************************************************************************
#define BSP_COM1_POWER_ON     ( 12 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_COM1_POWER_ON;


//*****************************************************************************
//
//  COM2_POWER_ON: COM2 Power Output Pin
//
//*****************************************************************************
#define BSP_COM2_POWER_ON     ( 13 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_COM2_POWER_ON;


//*****************************************************************************
//
//  COM3_POWER_ON: COM3 Power Output Pin
//
//*****************************************************************************
#define BSP_COM3_POWER_ON     ( 34 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_COM3_POWER_ON;


//*****************************************************************************
//
//  IMU_ON: Inertial Measurement Unit (IMU) Power Output Pin
//
//*****************************************************************************
#define BSP_IMU_POWER_ON     ( 16 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IMU_POWER_ON;


//*****************************************************************************
//
//  IMU_I2C_SDA: Inertial Measurement Unit (IMU) I2C SDA (Data) Pin
//
//*****************************************************************************
#define BSP_IMU_I2C_SDA     ( 9 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IMU_I2C_SDA;


//*****************************************************************************
//
//  IMU_I2C_SCL: Inertial Measurement Unit (IMU) I2C SCL (Data) Pin
//
//*****************************************************************************
#define BSP_IMU_I2C_SCL     ( 8 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_IMU_I2C_SCL;



//*****************************************************************************
//
//  LED1: Debug LED1 Pin
//
//*****************************************************************************
#define BSP_GPIO_LED1     ( 31 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_GPIO_LED1;


//*****************************************************************************
//
//  LED2: Debug LED2 Pin
//
//*****************************************************************************
#define BSP_GPIO_LED2     ( 32 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_GPIO_LED2;


//*****************************************************************************
//
//  LED1: Debug LED3 Pin
//
//*****************************************************************************
#define BSP_GPIO_LED3     ( 29 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_GPIO_LED3;



//*****************************************************************************
//
//  SC_NSHDN: Super Capacitor Shutdown (negative) Pin
//
//*****************************************************************************
#define BSP_SC_NSDHN         ( 0 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_SC_NSHDN;


//*****************************************************************************
//
//  SC_PGOOD: Super Capacitor Power Good Pin
//
//*****************************************************************************
#define BSP_SC_PGOOD         ( 0 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_SC_PGOOD;


//*****************************************************************************
//
//  SC_ON: Super Capacitor Power On Pin
//
//*****************************************************************************
#define BSP_SC_ON         ( 1 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_SC_ON;



//*****************************************************************************
//
//  CONSOLE_TX: Console UART Tx Pin 
//
//*****************************************************************************
#define BSP_UART_CONSOLE_TX         ( 48 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_UART_CONSOLE_TX;

//*****************************************************************************
//
//  CONSOLE_RX: Console UART Rx Pin 
//
//*****************************************************************************
#define BSP_UART_CONSOLE_RX         ( 49 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_UART_CONSOLE_RX;


////*****************************************************************************
////
////  CONSOLE_BOOT: Console Bootloader Pin 
////
////*****************************************************************************
//#define BSP_UART_CONSOLE_BOOT         ( 48 )
//extern const am_hal_gpio_pincfg_t   g_LCP_BSP_UART_CONSOLE_BOOT;




//*****************************************************************************
//
//  BUS_VOLT_ENABLE: Bus Voltage Enable (for reading voltage) Pin 
//
//*****************************************************************************
#define BSP_BUS_VOLT_ENABLE         ( 23 )
extern const am_hal_gpio_pincfg_t   g_LCP_BU_VOLT_ENABLE;


//*****************************************************************************
//
//  BUS_VOLTAGE: Bus Voltage read Pin
//
//*****************************************************************************
#define BSP_BUS_VOLTAGE         ( 35 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_BUS_VOLTAGE;



//*****************************************************************************
//
//  PWR_CTRL_EN: Power Control Enable Pin 
//
//*****************************************************************************
#define BSP_I2C_CTRL_EN         ( 2 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_I2C_CTRL_EN;

//*****************************************************************************
//
//  PWR_CTRL_SDA: Power Control Board I2C SDA (Data) Pin 
//
//*****************************************************************************
#define BSP_I2C_CTRL_SDA         ( 25 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_I2C_CTRL_SDA;

//*****************************************************************************
//
//  PWR_CTRL_SCL: Power Control I2C SCL (Clock) Pin 
//
//*****************************************************************************
#define BSP_I2C_CTRL_SCL         ( 27 )
extern const am_hal_gpio_pincfg_t   g_LCP_BSP_I2C_CTRL_SCL;




//*****************************************************************************
//
//  ITM_SWO pin: ITM Serial Wire Output.
//
//*****************************************************************************
#define AM_BSP_GPIO_ITM_SWO             41
extern const am_hal_gpio_pincfg_t       g_LCP_BSP_GPIO_ITM_SWO;

//*****************************************************************************
//
//  SWDCK pin: Cortex Serial Wire DCK.
//
//*****************************************************************************
#define AM_BSP_GPIO_SWDCK               20
extern const am_hal_gpio_pincfg_t       g_LCP_BSP_GPIO_SWDCK;

//*****************************************************************************
//
//  SWDIO pin: Cortex Serial Wire DIO.
//
//*****************************************************************************
#define AM_BSP_GPIO_SWDIO               21
extern const am_hal_gpio_pincfg_t       g_LCP_BSP_GPIO_SWDIO;


#ifdef __cplusplus
}
#endif

#endif // AM_BSP_PINS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
