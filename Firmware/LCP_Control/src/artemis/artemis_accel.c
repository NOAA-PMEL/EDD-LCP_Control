/**! @file artemis_accel.c
 * @brief STM Accelerometer 
 *
 * @author Basharat Martin basharat.martin@noaa.gov
 * @date April 24, 2023
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the STM LIS2DW12 Acceleration sensor over SPI
 * 
 *
 * @bug  No known bugs
 *
 **/

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
#include "artemis_stream.h"
#include "artemis_accel.h"

//*****************************************************************************
//
// Register Mapping
//
//*****************************************************************************

#define ACCEL_WRITE_CMD      0x00
#define ACCEL_READ_CMD       0x80

#define ACCEL_OUT_T_L        0x0D   // R, Temperature Sensor LSB
#define ACCEL_OUT_T_H        0x0E   // R, Temperature Sensor MSB 
#define ACCEL_DEV_ID         0x0F   // R

#define ACCEL_CTRL_1         0x20   // R/W
#define ACCEL_CTRL_2         0x21   // R/W
#define ACCEL_CTRL_3         0x22   // R/W
#define ACCEL_CTRL_4_INT1    0x23   // R/W
#define ACCEL_CTRL_5_INT2    0x24   // R/W
#define ACCEL_CTRL_6         0x25   // R/W

#define ACCEL_OUT_T          0x26   // R, Tempearture Sensor
#define ACCEL_STATUS         0x27   // R, Status data register

#define ACCEL_OUT_X_L        0x28   // R, Output registers
#define ACCEL_OUT_X_H        0x29   //
#define ACCEL_OUT_Y_L        0x2A   //
#define ACCEL_OUT_Y_H        0x2B   //
#define ACCEL_OUT_Z_L        0x2C   //
#define ACCEL_OUT_Z_H        0x2D   //

#define ACCEL_FIFO_CTRL      0x2E   // R/W
#define ACCEL_FIFO_SAMPLES   0x2F   // R, Unread samples stored in FIFO

#define ACCEL_TAP_THS_X      0x30   // R/W, Tap threshold
#define ACCEL_TAP_THS_Y      0x31   // R/W, Tap threshold
#define ACCEL_TAP_THS_Z      0x32   // R/W, Tap threshold
#define ACCEL_INT_DUR        0x33   // R/W, Interrupt Duration
#define ACCEL_WAKE_UP_THS    0x34   // R/W, Tap/double-tap selection,
                                    //      inactivity enable,
                                    //      wakeup threshold

#define ACCEL_WAKE_UP_DUR    0x35   // R/W, Wakeup duration
#define ACCEL_FREE_FALL      0x36   // R/W, Free-Fall Configuration

#define ACCEL_STATUS_DUP     0x37   // R, Status Register
#define ACCEL_WAKE_UP_SRC    0x38   // R, Wakeup source
#define ACCEL_TAP_SRC        0x39   // R, Tap source
#define ACCEL_SIXD_SRC       0x3A   // R, 6D source

#define ACCEL_ALL_INT_SRC    0x3B   // R,
#define ACCEL_X_OFS_USR      0x3C   // R/W,
#define ACCEL_Y_OFS_USR      0x3D   // R/W,
#define ACCEL_Z_OFS_USR      0x3E   // R/W,
#define ACCEL_CTRL7          0x3F   // R/W,

//*****************************************************************************
//
// Structs
//
//*****************************************************************************

typedef uint8_t module_buffer_t[128];

typedef struct s_module_t
{
	artemis_spi_t spi;
	module_buffer_t txbuffer;
	module_buffer_t rxbuffer;
} module_t;

static module_t module;

#define ACCEL_BUFFER         128

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************

//static void module_max14830_init(void);
//static void module_max14830_Power_On(void);
//static void module_max14830_Power_Off(void);
//static void module_max14830_Read(eMAX18430_ComPort_t port, uint8_t reg, uint8_t *rData, uint8_t len);
//static void module_max14830_Write(eMAX18430_ComPort_t port, uint8_t reg, uint8_t *sData, uint8_t len);
//static void module_max14830_FastRead(void);
//uint8_t module_max14830_port_from_pin(uint8_t pin);
//static void module_max14830_chip_enable(void);
//static void module_max14830_chip_disable(void);

static void artemis_accel_read_reg(uint8_t reg, uint8_t *rData, uint8_t len);
static void artemis_accel_write_reg(uint8_t reg, uint8_t *rData, uint8_t len);
static void module_accel_chip_enable(void);
static void module_accel_chip_disable(void);

void accel_tempearture(int16_t *tempearture);
void accel_xyz(int16_t *xyz);

void artemis_accel_init(void)
{
	artemis_spi_t *spi = &module.spi;

	spi->chipselect = AM_BSP_IOM0_CS_CHNL;
	spi->iom.module = ARTEMIS_IOM_MODULE_SPI0;
	spi->iom.config.eInterfaceMode = AM_HAL_IOM_SPI_MODE;
	spi->iom.config.ui32ClockFreq = AM_HAL_IOM_100KHZ;
	spi->iom.config.eSpiMode = AM_HAL_IOM_SPI_MODE_0;

	artemis_iom_initialize(&spi->iom);

	/* SPI0 module */
	ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCK, g_AM_BSP_GPIO_IOM0_SCK));
	ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MISO, g_AM_BSP_GPIO_IOM0_MISO));
	ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MOSI, g_AM_BSP_GPIO_IOM0_MOSI));

	ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_ACCEL_SPI_CS, g_AM_BSP_GPIO_ACCEL_SPI_CS));
	ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_ACCEL_INT, g_AM_BSP_GPIO_ACCEL_INT));

    uint8_t str[2];

    //artemis_accel_read_reg(ACCEL_DEV_ID, str, 1);
    artemis_accel_read_reg(ACCEL_CTRL_1, str, 1);
    //artemis_accel_read_reg(ACCEL_OUT_X_L, str, 2);
    //artemis_accel_read_reg(ACCEL_OUT_Y_L, str, 2);
    //artemis_accel_read_reg(ACCEL_OUT_Z_L, str, 2);
    //artemis_accel_read_reg(ACCEL_OUT_T_L, str, 2);
    //uint8_t data = 0x91;
    //artemis_accel_write_reg(ACCEL_CTRL_1, &data, 1);
    //artemis_accel_read_reg(ACCEL_CTRL_1, str, 1);

    uint8_t temp = 0;
    artemis_accel_read_reg(ACCEL_OUT_T, &temp, 1);

    //int16_t temp = 0;
    //accel_tempearture(&temp);

    am_util_stdio_printf("ACCEL temp = %0.5f\n", (float)temp);

    //int16_t xyz[3] = {0};
    //accel_xyz(xyz);

    //am_util_stdio_printf("ACCEL xyz = %d, %d, %d\n", xyz[0], xyz[1], xyz[2]);

}

void accel_xyz(int16_t *xyz)
{
    // read two bytes (16 bits) consecutively, 
    uint8_t rData[6] = {0};
    int16_t l_xyz[3] = {0};

    artemis_accel_read_reg(ACCEL_OUT_X_L, rData, 6);

    l_xyz[0] = (rData[1] << 8) | rData[0] ;
    l_xyz[1] = (rData[3] << 8) | rData[2] ;
    l_xyz[2] = (rData[5] << 8) | rData[4] ;


    am_util_stdio_printf("ACCEL xyz = %d, %d, %d\n", l_xyz[0], l_xyz[1], l_xyz[2]);
    xyz = l_xyz;
}

void accel_tempearture(int16_t *temperature)
{
    // read two bytes (16 bits) consecutively, 
    // temperature LSB fist then MSB
    uint8_t rData[2] = {0};
    int16_t val = 0;

    artemis_accel_read_reg(ACCEL_OUT_T_L, rData, 2);

    val  = (int16_t) rData[1];
    val <<= 4;
    val |= (int16_t) rData[0];

    am_util_stdio_printf("ACCEL temp = %d\n", val);
    *temperature = val;
}

static void artemis_accel_write_reg(uint8_t reg, uint8_t *sData, uint8_t len)
{
	artemis_stream_t txstream = {0};
	uint8_t cmd = ACCEL_WRITE_CMD;

	cmd |= reg;

	artemis_stream_setbuffer(&txstream, module.txbuffer, ACCEL_BUFFER);
	artemis_stream_reset(&txstream);

	artemis_stream_put(&txstream, cmd);

	for(uint8_t i=0; i<len; i++){
		artemis_stream_put(&txstream, *sData++);
	}

	/* CS high */
    module_accel_chip_enable();
	artemis_spi_send(&module.spi, true, &txstream);
	/* CS low */
    module_accel_chip_disable();
}

static void artemis_accel_read_reg(uint8_t reg, uint8_t *rData, uint8_t len)
{
	artemis_stream_t txstream = {0};
	artemis_stream_t rxstream = {0};
	uint8_t cmd = ACCEL_READ_CMD;

	cmd |= reg;

	artemis_stream_setbuffer(&txstream, module.txbuffer, ACCEL_BUFFER);
	artemis_stream_setbuffer(&rxstream, module.rxbuffer, ACCEL_BUFFER);
	artemis_stream_reset(&txstream);
	artemis_stream_reset(&rxstream);

	artemis_stream_put(&txstream, cmd);

	/* CS high */
    module_accel_chip_enable();
	artemis_spi_send(&module.spi, true, &txstream);
	artemis_spi_receive(&module.spi, true, &rxstream, len);
	artemis_stream_read(&rxstream, rData, len);
	/* CS low */
    module_accel_chip_disable();
}

static void module_accel_chip_enable(void)
{
	am_hal_gpio_output_clear(AM_BSP_GPIO_ACCEL_SPI_CS);
}

static void module_accel_chip_disable(void)
{
	am_hal_gpio_output_set(AM_BSP_GPIO_ACCEL_SPI_CS);
}

