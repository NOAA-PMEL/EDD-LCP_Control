/**! @file pressure_k9lx.c
 * @brief Maxim SPI-to-UART Converter
 *
 * @author Basharat Martin basharat.martin@noaa.gov
 * @date Feb 23, 2023
 * @version 1.0.0
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Controls the Keller 9LXe, circuit board (9L140) Pressure sensor over RS-485
 * 
 *
 * @bug  No known bugs
 *
 **/

/** Simplified COMMS protocol from Keller */
/** 
	TODO:  organize this, 
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

#include "artemis_max14830.h"
#include "artemis_debug.h"
#include "artemis_stream.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
//#include "FreeRTOS.h"
//#include "task.h"
//#include "event_groups.h"
//#include "semphr.h"

//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "buffer_c.h"
#include "K9lx_pressure.h"

//*****************************************************************************
//
//  Macros
//
//*****************************************************************************

#define K9LX_ADDR					0x01	//	0x01 default device address for Modbus
#define K9LX_BUFFER_LENGTH			0x0D	//	13 bytes for 9LXe sensor
#define K9LX_RECV_ERROR				0x80	/*	return with exception error 
												function code 7th bit is high */
/** K9LX Command Bytes */
#define K9LX_READ_REG_CMD			0x03	//	Modbus read command F3:
#define K9LX_WRITE_REG_CMD			0x06	//	Modbus write single register command F6:
#define K9LX_ECHO_CMD				0x08	//	Modbus echo test F8:
#define K9LX_WRITE_REGS_CMD			0x10	//	Modbus write multiple registers command F16:

/*** Modbus Register Map ***/
// 32bit floating point addresses Big-Endian
#define K9LX_FL_CH0					0x0000	//	return calculated value (customer specific)
#define K9LX_FL_P1					0x0002	//	return Pressure of sensor1 in bar (HWord : LWord)
#define K9LX_FL_P2					0x0004	//	return Pressure of sensor2 in bar (HWord : LWord)
#define K9LX_FL_T					0x0006	//	return Temperature in °C (HWord : LWord)
#define K9LX_FL_TOB1				0x0008	//	return Temperature of sensor1 in °C (HWord : LWord)
#define K9LX_FL_TOB2				0x000A	//	return Tempearture of sensor2 in °C (HWord : LWord)

// 32bit floating point addresses, accessing data in one cylce 8 bytes
#define K9LX_FL1_P1					0x0100	//	return Pressure of sensor1 in bar (HWord : LWord)
#define K9LX_FL1_TOB1				0x0102	//	return Temperature of sensor1 in °C (HWord : LWord)
#define K9LX_FL1_P2					0x0104	//	return Pressure of sensor2 in bar (HWord : LWord)
#define K9LX_FL1_TOB2				0x0106	//	return Temperature of sensor2 in °C (HWord : LWord)

// 16bit int addresses 
#define K9LX_INT16_CH0				0x0010	//	return calculated value (customer specific) 1/100 (LSB)
#define K9LX_INT16_P1				0x0011	//	return Pressure of sensor1 in bar (1/100 LSB)
#define K9LX_INT16_P2				0x0012	//	return Pressure of sensor2 in bar (1/100 LSB)
#define K9LX_INT16_T				0x0013	//	return Temperature in °C (1/100 LSB)
#define K9LX_INT16_TOB1				0x0014	//	return Temperature of sensor1 in °C (1/100 LSB)
#define K9LX_INT16_TOB2				0x0015	//	return Tempearture of sensor2 in °C (1/100 LSB)

// 32bit int addresses 
#define K9LX_INT32_CH0				0x0020	//	return calculated value (customer specific) 1/100000 (LSB) 
#define K9LX_INT32_P1				0x0022	//	return Pressure of sensor1 in Pa (1/100000 LSB)
#define K9LX_INT32_P2				0x0024	//	return Pressure of sensor2 in Pa (1/100000 LSB)
#define K9LX_INT32_T				0x0026	//	return Temperature in °C (1/100 LSB)
#define K9LX_INT32_TOB1				0x0028	//	return Temperature of sensor1 in °C (1/100 LSB)
#define K9LX_INT32_TOB2				0x002A	//	return Tempearture of sensor2 in °C (1/100 LSB)

/* Modbus , Device Configuration Registers */
#define K9LX_STATUS					0x020C	//	Read-Only, returns the device status,
#define K9LX_STATUS_CH0_ERROR		(1<<0)	//	0	: 	CH0 error
#define K9LX_STATUS_P1_ERROR		(1<<1)	//	1	:	P1 error
#define K9LX_STATUS_P2_ERROR		(1<<2)	//	2	:	P2 error
#define K9LX_STATUS_T_ERROR			(1<<3)	//	3	:	T error
#define K9LX_STATUS_TOB1_ERROR		(1<<4)	//	4	:	TOB1 error
#define K9LX_STATUS_TOB2_ERROR		(1<<5)	//	5	:	TOB2 error
#define K9LX_STATUS_ERR2_ERROR		(1<<6)	//	6	:	computation error
#define K9LX_STATUS_NSTD_ERROR		(1<<7)	//	7	:	STD error, either in power-up mode or Standard

#define K9LX_SER_NUM_H				0x0202	//	Serial number Higher bits
#define K9LX_SER_NUM_L				0x0203	//	Serial number lower bits
#define K9LX_FIRM_VER0				0x020E	//	Read-Only Firmware version 0 -> Class:Group
#define K9LX_FIRM_VER1				0x020F	//	Read-Only Firmware version 1 -> Year:Week
#define K9LX_P_MODE					0x0209	//	Read-Only : returns type of Sensor and Calibration
											//	Bit 3...0 : P1 = 0:PR(relative), 1:PA(absolute) 15:not available
											//	Bit 7...4 : P2 = 0:PR(relative), 1:PA(absolute) 15:not available

#define K9LX_UART					0x0200	//	R/W : UART settings : Bit 3...0: 0: 9600 1:115200
#define K9LX_UART_96K				(0x0)	//	9600 
#define K9LX_UART_115K				(0x1)	//	115200 

#define K9LX_CFG_P					0x0204	//  R : Active Pressure channel Bit 1:P1, Bit 2:P2
#define K9LX_CFG_P_P1				(1<<1)	//	P1 : bit1
#define K9LX_CFG_P_P2				(1<<2)	//	P2 : bit2

#define K9LX_CFG_T					0x0204	//  R : Active Temperature channel Bit 3:T, Bit 4:TOB1, Bit 5:TOB2
#define K9LX_CFG_T_T				(1<<3)	//	T
#define K9LX_CFG_T_TOB1				(1<<4)	//	TOB1
#define K9LX_CFG_T_TOB2				(1<<5)	//	TOB2

#define CMD_LENGTH					8		//	command size is 8 bytes

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

typedef uint8_t module_buffer_t[K9LX_BUFFER_LENGTH];
typedef struct s_module_t
{
	module_buffer_t txbuffer;
	module_buffer_t rxbuffer;
	struct {
		uint32_t pin;
		am_hal_gpio_pincfg_t *pinConfig;
	} power;

	module_scaling_t scaling;
	module_manufacturer_t manufacturer;

} module_t;

typedef struct s_k9lx_data_t
{
	float pressure;
	float temperature;
} k9lx_data_t;

typedef union {
	struct {
		uint8_t u[4];
	};
	float f;
} u32_to_float_t;

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
static void module_k9lx_read_sensor(k9lx_data_t *data);
static float module_k9lx_convert_pressure(uint32_t u32Pressure);
static float module_k9lx_convert_temperature(uint32_t u32Temp);
static bool module_k9lx_read_status(void);
static void module_k9lx_device_info(uint8_t port);
static uint16_t module_k9lx_crc16 (uint8_t *crc_h, uint8_t *crc_l, uint8_t *pData, uint8_t len);
static int8_t module_k9lx_read_reg(uint8_t port, uint16_t reg_add, uint8_t reg_num, uint8_t *pData);
static int8_t module_k9lx_data_integrity (uint8_t *Data, uint8_t len);

void K9lx_init(eMAX18430_ComPort_t port, eMAX14830_Baudrate_t baudRate)
{
	eMAX14830_Baudrate_t br = baudRate;
	/* set max port and baudrate */ 
	artemis_max14830_Set_baudrate(port, br);

	module.power.pinConfig = (am_hal_gpio_pincfg_t*)&g_AM_BSP_GPIO_COM3_POWER_PIN;
	module.power.pin = AM_BSP_GPIO_COM3_POWER_PIN;

	/** Initialize the COM Port Power Pin */
	ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.power.pin, *module.power.pinConfig));

	/** Turn K-9LX On */
	K9lx_power_off();
	K9lx_power_on();

	/* wait for device to get initialized*/
	am_util_delay_ms(500);

	/** Read the module Firmware version, P-Mode, serial number, Active channels */ 
	module_k9lx_device_info(port);
	//K9lx_power_off();
}
	
void K9lx_power_on(void)
{
	am_hal_gpio_output_clear(module.power.pin);
}

void K9lx_power_off(void)
{
	am_hal_gpio_output_set(module.power.pin);
}

static uint16_t module_k9lx_crc16 (uint8_t *crc_h, uint8_t *crc_l, uint8_t *pData, uint8_t len){

	uint16_t crc16 = 0xFFFF;
	uint8_t n, m, x;

	m = len;
	x = 0;

	// loop over all bits
	while(m>0){
		crc16 ^= pData[x];
		for(n=0; n<8; n++){
			if(crc16 & 1){
				crc16 >>= 1;
				crc16 ^= 0xA001;
			}
			else {
				crc16 >>= 1;
			}
		}
		m--;
		x++;
	}

	// result
	*crc_h = (crc16 >>8 ) & 0xFF;
	*crc_l = crc16 & 0xFF;

	return crc16 ;
}

void K9lx_read(float *pressure, float *temperature)
{
	k9lx_data_t data;
	module_k9lx_read_sensor(&data);
	*pressure = data.pressure;
	*temperature = data.temperature;
}

//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************
static void module_k9lx_read_sensor(k9lx_data_t *data)
{
	uint8_t ret = 0;
	uint8_t pt[8] = {0};
	u32_to_float_t pressure_bar;
	u32_to_float_t temperature_c;

	// TODO: port = 3
	ret = module_k9lx_read_reg(3, K9LX_FL1_P1, 4, pt);
	if (ret == 0) {
		for (uint8_t i=0; i<4; i++){
			pressure_bar.u[3-i] = pt[i];
		}
		for (uint8_t i=4; i<8; i++){
			temperature_c.u[7-i] = pt[i];
		}
	}

	/** Convert the pressure */
	//data->pressure = module_k9lx_convert_pressure(pressure_bar.f);
	data->pressure = pressure_bar.f;

	/** Convert the temperature */
	//data->temperature = module_k9lx_convert_temperature(temperature_c.f);
	data->temperature = temperature_c.f;
}

static float module_k9lx_convert_pressure(uint32_t u32Pressure)
{
	if(u32Pressure < 16384)
	{
		u32Pressure = 0;
	} 
	else {
		u32Pressure -= 16384;
	}
	//u32Pressure -= 16384;
	//
	//int32 pTemp = (int32_t)u32Pressure;
	//pTemp -= -16384;
	float fPressure =  (float) (u32Pressure);
	fPressure *= module.scaling.diff;
	fPressure /= 32768;
	fPressure += module.scaling.low;

	return fPressure;
}

static float module_k9lx_convert_temperature(uint32_t u32Temp)
{
	u32Temp = u32Temp >> 4;
	float fTemp = (float)(u32Temp);
	fTemp -= 24;
	fTemp *= 0.05;
	fTemp -= 50;

	return fTemp;
}

//static bool module_k9lx_read_status(void)
//{
	//artemis_stream_t rxstream = {0};
	//artemis_stream_setbuffer(&rxstream, module.rxbuffer, K9LX_BUFFER_LENGTH);
	//
	//artemis_i2c_t *i2c = &module.i2c;
	//artemis_i2c_receive(i2c, true, &rxstream, 1);
	//
	//uint8_t data;
	//artemis_stream_get(&rxstream, &data);
	//data = data & K9LX_STATUS_BIT;
	//return data;
//}

//static int32_t module_k9lx_read_with_status(artemis_i2c_t *i2c, artemis_stream_t *rxstream, uint32_t numBytes, uint32_t attempts)
//{
//	int32_t result = -1;
//	uint8_t temp;
//
//	do{
//		artemis_stream_reset(rxstream);
//		artemis_i2c_receive(i2c, true, rxstream, numBytes);
//		artemis_stream_get(rxstream, &temp);
//
//	}while((temp & K9LX_STATUS_BIT) && (--attempts > 0));
//
//	//printf("%u, %ul\n", temp & K9LX_STATUS_BIT, attempts);
//
//	if( !(temp & K9LX_STATUS_BIT))
//	{
//		result = 0;
//	}
//	return result;
//}

static int8_t module_k9lx_data_integrity (uint8_t *Data, uint8_t len){

	/*	check received data's Function's code
	 *	7th bit of second byte will tell the error has been received
	 *	third byte would be the error code, follow the MODBUS communication protocol v3.7
	 *	check the CRC on the received bytes
	 * 
	 */

	// copy the Data into the local buffer
	uint8_t lBuf[K9LX_BUFFER_LENGTH] = {0};
	uint8_t crc_h, crc_l = 0;

	for (uint8_t i=0; i<len; i++){
		lBuf[i] = Data[i];
	}

	// check the function code error bit (7th bit)
	if (K9LX_RECV_ERROR & lBuf[1]){
		// TODO : error has been occured, handle it further with the error code on third-byte
		return -1;
	}
	else {
		// check the CRC16, send only the useful bytes, last two bytes are CRC-bytes
		module_k9lx_crc16 (&crc_h, &crc_l, lBuf, len-2);
		if (crc_l == lBuf[len-2] && crc_h == lBuf[len-1]) {
			// received data is correct
			return 0;
		}
		else {
			// TODO: handle the error code
			return -1;
		}
	}
	// unknown error
	return -2;
}

static int8_t module_k9lx_read_reg(uint8_t port, uint16_t reg_add, uint8_t reg_num, uint8_t *pData)
{
	/** Prep for MAX SPI messages */
	uint8_t cmd[CMD_LENGTH] = {0};
	uint8_t rxData[K9LX_BUFFER_LENGTH] = {0};
	uint8_t rxLen = 0;
	uint8_t crc_h, crc_l = 0;

	cmd[0] = K9LX_ADDR;
	cmd[1] = K9LX_READ_REG_CMD;
	cmd[2] = (reg_add >> 8) & 0xFF;
	cmd[3] = reg_add & 0xFF;
	cmd[4] = 0x00;
	cmd[5] = reg_num ; // read number of registers

	/* calculate CRC16 */ 
	module_k9lx_crc16 (&crc_h, &crc_l, cmd, 6);

	/* MODBUS */
	cmd[6] = crc_l;
	cmd[7] = crc_h;

	artemis_max14830_UART_Write(port, cmd, CMD_LENGTH);
	artemis_max14830_UART_Read(port, rxData, &rxLen);

	// check data integrity 
	int8_t ret = module_k9lx_data_integrity (rxData,rxLen);

	if (ret == 0 && rxLen > 0){
		for(uint8_t i=0; i<rxLen-5; i++){
			pData[i] = rxData[i+3];
			//am_util_stdio_printf ("%d ", pData[i]);
		}
	}
	else {
		//am_util_stdio_printf ("Received ERROR (%d) at keller \n", ret);
	}
	//am_util_stdio_printf ("\n");

	return ret;
}

static void module_k9lx_device_info(uint8_t port)
{
	uint8_t firmware[4] = {0};
	uint8_t serial[4] = {0};
	uint32_t serial_nr = 0;

	uint8_t pr[4] = {0};
	uint8_t tp[4] = {0};
	uint8_t pt[8] = {0};

	k9lx_data_t values;
	u32_to_float_t pressure_bar;
	u32_to_float_t pressure_pascal;
	u32_to_float_t temperature_c;

	int32_t pressure_p = 0;

	int8_t ret = 0;

	ARTEMIS_DEBUG_PRINTF("\nK9LX Pressure Sensor\n");
	ARTEMIS_DEBUG_PRINTF("*****************************\n");

	//Get the Firmware version : 5.20.12.28
	ARTEMIS_DEBUG_PRINTF("\tFirmware Ver\t: ");
	ret = module_k9lx_read_reg(port, K9LX_FIRM_VER0, 2, firmware);
	if (ret == 0) {
		for (uint8_t i=0; i<4; i++){
			am_util_stdio_printf ("%u.", firmware[i]);
		}
	}

	// Get the Serial number : ? 
	ARTEMIS_DEBUG_PRINTF("\n\tSerial Number\t: ");
	ret = module_k9lx_read_reg(port, K9LX_SER_NUM_H, 2, serial);
	if (ret == 0) {
		for (uint8_t i=0; i<4; i++){
			serial_nr |= (serial[i] << (8*(3-i)) );
		}
		ARTEMIS_DEBUG_PRINTF("%u\n", serial_nr);
	}

	//ARTEMIS_DEBUG_PRINTF("\nPressure : ");
	//ret = module_k9lx_read_reg(port, K9LX_FL_P1, 2, pr);
	//if (ret == 0) {
	//	for (uint8_t i=0; i<4; i++){
	//		pressure_bar.u[3-i] = pr[i];
	//	}
	//	ARTEMIS_DEBUG_PRINTF("%0.5f bar", pressure_bar.f);
	//}

	//ARTEMIS_DEBUG_PRINTF("\nTemperature : ");
	//ret = module_k9lx_read_reg(port, K9LX_FL_TOB1, 2, tp);
	//if (ret == 0) {
	//	for (uint8_t i=0; i<4; i++){
	//		temperature_c.u[3-i] = tp[i];
	//	}
	//	ARTEMIS_DEBUG_PRINTF("%0.5f\n", temperature_c.f);
	//}


	/* 32-bit floating point , access P and T in one cycle 8 bytes */
	ARTEMIS_DEBUG_PRINTF("\n\tPressure\t: ");
	ret = module_k9lx_read_reg(port, K9LX_FL1_P1, 4, pt);
	if (ret == 0) {
		for (uint8_t i=0; i<4; i++){
			pressure_bar.u[3-i] = pt[i];
		}
		ARTEMIS_DEBUG_PRINTF("%0.5f bar", pressure_bar.f);
		ARTEMIS_DEBUG_PRINTF("\n\tTemperature\t: ");
		for (uint8_t i=4; i<8; i++){
			temperature_c.u[7-i] = pt[i];
		}
		ARTEMIS_DEBUG_PRINTF("%0.5f °C", temperature_c.f);
	}

	ARTEMIS_DEBUG_PRINTF("\n\tPressure\t: ");
	ret = module_k9lx_read_reg(port, K9LX_INT32_P1, 2, pr);
	if (ret == 0) {
		for (uint8_t i=0; i<4; i++){
			//pressure_pascal.u[3-i] = pr[i];
			pressure_p |= pr[i] << (8*(3-i));
		}
		ARTEMIS_DEBUG_PRINTF("%d pascal", pressure_p);
	}

	ARTEMIS_DEBUG_PRINTF("\n");
	//am_util_delay_ms(1000);

	//am_util_delay_ms(100);
	//module_k9lx_read_reg(port, K9LX_FIRM_VER1, 1, firmware1);

}

