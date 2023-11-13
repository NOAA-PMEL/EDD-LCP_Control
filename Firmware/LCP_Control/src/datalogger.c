/**! @file OpenLog Datalogger Qwiic i2c interface
 * @brief Sparkfuns openlog datalogger
 *
 * @author Basharat Martin basharat.martin@noaa.gov
 * @date October 12, 2023
 * @version 1.0.0
 *
 * Note : followed the source files provided by joseph Kurina - NOAA Affiliate <joseph.kurina@noaa.gov>
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note interfaces with i2c protocol, can be connected with either I2C_1 or I2C_4 (debug)
 * 
 *
 **/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "datalogger.h"
#include "buffer_c.h"
#include "artemis_debug.h"

static void datalogger_i2c_read(uint8_t offset, uint8_t offsetlen,
                                uint8_t *pBuf, uint16_t size);
static void datalogger_i2c_write(uint8_t offset, uint8_t offsetlen,
                                uint8_t *pBuf, uint16_t size);
static void datalogger_write_sync(void);

static module_d module;
static uint16_t sps_count = 0;
static uint16_t park_count = 0;

bool datalogger_init(uint8_t iomNo)
{
    artemis_i2c_t *i2c = &module.i2c;

    i2c->address = LOGGER_I2C_ADDRESS;
    i2c->iom.config.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
    i2c->iom.config.ui32ClockFreq = AM_HAL_IOM_400KHZ;
    i2c->iom.config.pNBTxnBuf = NULL;
    i2c->iom.config.ui32NBTxnBufLength = 0;
    i2c->iom.module = iomNo;
    artemis_iom_initialize(&i2c->iom);

    if (iomNo == 1)
    {
        module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_I2C_1_PWR;
        module.power.pin = AM_BSP_GPIO_I2C_1_PWR;
        ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL, g_AM_BSP_GPIO_IOM1_SCL));
        ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA, g_AM_BSP_GPIO_IOM1_SDA));
    }
    else if (iomNo == 4)
    {
        module.power.pinConfig = (am_hal_gpio_pincfg_t *)&g_AM_BSP_GPIO_PRES_ON;
        module.power.pin = AM_BSP_GPIO_PRES_ON;
        ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCL, g_AM_BSP_GPIO_IOM4_SCL));
        ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SDA, g_AM_BSP_GPIO_IOM4_SDA));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("ERROR:: Datalogger init -> Selete the iom number (1, 4)\n");
        return false;
    }

    ARTEMIS_DEBUG_HALSTATUS(am_hal_gpio_pinconfig(module.power.pin, *module.power.pinConfig));
    datalogger_power_on();
    // print device information
    datalogger_device_info();
    datalogger_power_off();

    return true;
}

void datalogger_device_info(void)
{
    ARTEMIS_DEBUG_PRINTF("\nDatalogger Qwiic Device Info\n");
    ARTEMIS_DEBUG_PRINTF("*******************************\n");
    ARTEMIS_DEBUG_PRINTF("Device Unique ID\t: ");

    uint8_t id = datalogger_get_id();
    ARTEMIS_DEBUG_PRINTF("0x%02X", id);
    ARTEMIS_DEBUG_PRINTF("\n");

    ARTEMIS_DEBUG_PRINTF("Device FW Version\t: ");
    uint16_t fw = datalogger_fw_version();
    ARTEMIS_DEBUG_PRINTF("%u.%u", fw>>8&0xff, fw&0xff);
    ARTEMIS_DEBUG_PRINTF("\n");

    ARTEMIS_DEBUG_PRINTF("Device Status Info\t: ");
    uint8_t status = datalogger_status();
    if (status & 0x01)
    {
        ARTEMIS_DEBUG_PRINTF("SD init Good");
        ARTEMIS_DEBUG_PRINTF("\n");
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("SD init Not Good");
        ARTEMIS_DEBUG_PRINTF("\n");
    }
}

void datalogger_predeploy_mode(SensorGps_t *gps, bool check)
{
    char *dirname = "pre-deploy_mode";
    rtc_time time;

    int32_t lat = (int32_t) (gps->latitude * 10000000);
    int32_t lon = (int32_t) (gps->longitude * 10000000);
    int32_t alt = (int32_t) (gps->altitude * 10000000);

    /* if initialize is true, then create a directory and files*/
    datalogger_cd("..");
    datalogger_mkdir(dirname);
    datalogger_cd(dirname);

    /* get time-stamp */
    artemis_rtc_get_time(&time);
    char filename[64] = {0};
    sprintf (filename, "pds_%02d.%02d.20%02d.txt", time.month, time.day, time.year);

    datalogger_createfile(filename);
    datalogger_openfile(filename);

    char data[256] = {0};
    datalogger_writefile("\n******************************\n");
    datalogger_writefile("\nLCP System Check information\n");
    datalogger_writefile("\n******************************\n\n");

    sprintf (data, "Check\t: %s\nTime\t: %02d:%02d:%02d\nDate\t: %02d.%02d.20%02d\nLatitude\t: %ld\nLongitude\t: %ld\nAltitude\t: %ld\n",
                    check == true ? "OK" : "Failed", time.hour, time.min, time.sec, time.month, time.day, time.year, lat, lon, alt);

    for (uint16_t i=0; i< strlen(data); i++)
    {
        ARTEMIS_DEBUG_PRINTF("%c", data[i]);
    }

    datalogger_writefile(data);
    datalogger_write_sync();

    ///* read file*/
    //uint32_t size = datalogger_filesize(filename);
    //ARTEMIS_DEBUG_PRINTF("file size = %u\n", size);

    //char buf[512];
    //datalogger_readfile(filename, buf, size);

    //for (uint32_t i=0; i<size; i++)
    //{
    //    ARTEMIS_DEBUG_PRINTF("%c", buf[i]);
    //}
    //ARTEMIS_DEBUG_PRINTF("\n");
}

char *datalogger_profile_create_file(uint16_t sps_nr)
{
    char *dirname = "profile_mode";
    rtc_time time;

    /* if initialize is true, then create a directory and files*/
    datalogger_cd("..");
    datalogger_mkdir(dirname);
    datalogger_cd(dirname);

    /* get time-stamp */
    artemis_rtc_get_time(&time);

    static char filename[64] = {0};

    sprintf (filename, "%d_sps_%02d.%02d.20%02d.txt", sps_nr, time.month, time.day, time.year);
    datalogger_createfile(filename);
    datalogger_openfile(filename);
    datalogger_writefile("S.No.\t| Depth(m)\t| Temperature(°C) | Volume(in3)\t| Time-stamp\t\n");
    datalogger_writefile("\n============================================================================\n\n");
    datalogger_write_sync();
    ARTEMIS_DEBUG_PRINTF("%s file created\n", filename);
    sps_count = 0;

    return filename;
}

void datalogger_profile_mode(char *filename, float depth, float temp, float volume, rtc_time *time)
{
    char *dirname = "profile_mode";
    //rtc_time time;

    /* if initialize is true, then create a directory and files*/
    datalogger_cd("..");
    //datalogger_mkdir(dirname);
    datalogger_cd(dirname);

    ///* get time-stamp */
    //artemis_rtc_get_time(&time);

    sps_count++;
    datalogger_openfile(filename);

    int32_t Depth  =  (int32_t) (depth  * 10000);
    int32_t Temp   =  (int32_t) (temp   * 10000);
    int32_t Volume =  (int32_t) (volume * 10000);

    char data[128] = {0};
    sprintf (data, "\n%u\t  %ld.%04ld\t  %ld.%04ld\t   %ld.%04ld\t  %02d:%02d:%02d\n",
                    sps_count, Depth/10000, Depth%10000, Temp/10000, Temp%10000,
                    Volume/10000, Volume%10000, time->hour, time->min, time->sec);

    datalogger_writefile(data);
    datalogger_write_sync();
}

char *datalogger_park_create_file(uint16_t park_nr)
{
    char *dirname = "park_mode";
    rtc_time time;

    /* if initialize is true, then create a directory and files*/
    datalogger_cd("..");
    datalogger_mkdir(dirname);
    datalogger_cd(dirname);

    /* get time-stamp */
    artemis_rtc_get_time(&time);

    static char filename[64] = {0};

    sprintf (filename, "%d_park_%02d.%02d.20%02d.txt", park_nr, time.month, time.day, time.year);
    datalogger_createfile(filename);
    datalogger_openfile(filename);
    datalogger_writefile("S.No.\t| Depth(m)\t| Temperature(°C)\t| Time-stamp\t\n");
    datalogger_writefile("\n===========================================================\n\n");
    datalogger_write_sync();
    ARTEMIS_DEBUG_PRINTF("%s file created\n", filename);
    park_count = 0;

    return filename;
}

void datalogger_park_mode(char *filename, float depth, float temp, rtc_time *time)
{
    char *dirname = "park_mode";
    //rtc_time time;

    /* if initialize is true, then create a directory and files*/
    datalogger_cd("..");
    datalogger_cd(dirname);

    ///* get time-stamp */
    //artemis_rtc_get_time(&time);

    park_count++;
    datalogger_openfile(filename);

    int32_t Depth  =  (int32_t) (depth  * 10000);
    int32_t Temp   =  (int32_t) (temp   * 10000);

    char data[64] = {0};
    sprintf (data, "\n%u\t  %ld.%04ld\t  %ld.%04ld\t %02d:%02d:%02d\n",
                    park_count, Depth/10000, Depth%10000, Temp/10000, Temp%10000,
                    time->hour, time->min, time->sec);

    datalogger_writefile(data);
    datalogger_write_sync();

}

void datalogger_surface_mode(float temp, float depth);

void datalogger_log_init(void)
{
    uint8_t cmd = LOGGER_INIT_LOG;
    uint8_t initlog = 0x00;
    datalogger_i2c_write(cmd, 1, &initlog, 1);
}


void datalogger_mkdir(char *dirname)
{
    uint8_t cmd = LOGGER_MKDIR;
    uint8_t len = strlen(dirname);
    datalogger_i2c_write(cmd, 1, (uint8_t *)dirname, len);
}

void datalogger_cd(char *dirname)
{
    uint8_t cmd = LOGGER_CD;
    uint8_t len = strlen(dirname);
    datalogger_i2c_write(cmd, 1, (uint8_t *)dirname, len);
}

void datalogger_createfile(char *filename)
{
    uint8_t cmd = LOGGER_CREATE_FILE;
    uint8_t len = strlen(filename);
    datalogger_i2c_write(cmd, 1, (uint8_t *)filename, len);
}

void datalogger_openfile(char *filename)
{
    uint8_t cmd = LOGGER_OPEN_FILE;
    uint8_t len = strlen (filename);
    datalogger_i2c_write(cmd, 1, (uint8_t *)filename, len);
}

uint32_t datalogger_filesize(char *filename)
{
    uint32_t size = 0;
    uint8_t bytes[4] = {0};

    uint8_t cmd = LOGGER_FILE_SIZE;
    uint8_t len = strlen (filename);
    datalogger_i2c_write(cmd, 1, (uint8_t *)filename, len);

    /* read 4 bytes for file size */
    datalogger_i2c_read(0, 0, bytes, 4);
    size |= bytes[0]<<24 | bytes [1]<<16 | bytes[2]<<8 | bytes[3] ;
    return size;
}

void datalogger_writefile(char *contents)
{
    uint8_t cmd = LOGGER_WRITE_FILE;
    uint32_t len = strlen (contents);

    uint8_t i = 0;
    uint32_t j = 0;

    if (len > 31)
    {
        while (len >0)
        {
            for (i=0; i<LOGGER_BUFFER_SIZE-1; i++)
            {
                module.txbuffer[i] = contents[i+j];
                len--;
                if (len == 0)
                {
                    break;
                }
            }
            // send 31 bytes to the SD-card
            datalogger_i2c_write(cmd, 1, module.txbuffer, i);
            j += i;
        } 
    }
    else if (len < 32 && len > 0)
    {
        datalogger_i2c_write(cmd, 1, (uint8_t *)contents, len);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("ERROR:: writing file content is 0\n");
    }
}

void datalogger_readfile(char *filename, char *contents, uint32_t size)
{
    uint8_t cmd = LOGGER_READ_FILE;
    uint8_t len = strlen (filename);

    datalogger_i2c_write(cmd, 1, (uint8_t *)filename, len);

    uint32_t bytes = size;
    uint32_t j = 0;
    uint8_t i = 0;

    while (bytes > 0)
    {
        /* read 32 bytes from the SD card at a time */
        datalogger_i2c_read(0, 0, module.rxbuffer, LOGGER_BUFFER_SIZE);
        for (i=0; i<LOGGER_BUFFER_SIZE; i++)
        {
            contents[i+j] = (char) module.rxbuffer[i];
            bytes--;
            if (bytes == 0)
            {
                break;
            }
        }
        j += i;
    }
}

uint32_t datalogger_rmdir(char *dirname)
{
    uint32_t nr_files = datalogger_rmfile(dirname, true);
    return nr_files;
}

uint32_t datalogger_rmfile(char *filename, bool recursive)
{
    uint32_t nr_files = 0;
    uint8_t bytes[4] = {0};

    if (recursive == true)
    {
        uint8_t cmd = LOGGER_RM_RECUR;
        uint8_t len = strlen (filename);
        datalogger_i2c_write(cmd, 1, (uint8_t *)filename, len);

        /* read 4 bytes for file size */
        datalogger_i2c_read(0, 0, bytes, 4);
        nr_files |= bytes[0]<<24 | bytes [1]<<16 | bytes[2]<<8 | bytes[3] ;
        return nr_files;
    }
    else
    {
        uint8_t cmd = LOGGER_RM;
        uint8_t len = strlen (filename);
        datalogger_i2c_write(cmd, 1, (uint8_t *)filename, len);

        /* read 4 bytes for file size */
        datalogger_i2c_read(0, 0, bytes, 4);
        nr_files |= bytes[0]<<24 | bytes [1]<<16 | bytes[2]<<8 | bytes[3] ;
        return nr_files;
    }
}

uint8_t datalogger_get_id(void)
{
    uint8_t cmd = LOGGER_ID;
    uint8_t unique_id;

    datalogger_i2c_read(cmd, 1, &unique_id, 1);
    return unique_id;
}

uint8_t datalogger_status(void)
{
    uint8_t cmd = LOGGER_STATUS;
    uint8_t ret = 0x00;

    //datalogger_i2c_write(cmd, 1, &ret, 1);
    datalogger_i2c_read(cmd, 1, &ret, 1);
    return ret;
}

uint16_t datalogger_fw_version(void)
{
    uint16_t fw = 0x0000;
    uint8_t cmd = LOGGER_FW_MAJ;
    uint8_t data;

    datalogger_i2c_read(cmd, 1, &data, 1);
    fw |= (data&0xFF) << 8 ;
    cmd = LOGGER_FW_MIN;
    datalogger_i2c_read(cmd, 1, &data, 1);
    fw |= (data&0xFF) ;

    return fw;
}

/**
 * @brief Power Up the Datalogger Module
 * 
 */
void datalogger_power_on(void)
{
	am_hal_gpio_output_clear(module.power.pin);

    /* wait for SD card to be initiated */
    uint8_t ret = 0;
    uint8_t delay = 0;
    do {
        am_hal_systick_delay_us(100000);
        ret = datalogger_status();
        delay++;
    } while ( ! (ret&0x01) && delay < 20);
}

/**
 * @brief Power Down the Datalogger Module
 * 
 */
void datalogger_power_off(void)
{
	am_hal_gpio_output_set(module.power.pin);
}

///**
// * @brief Send I2C message
// * 
// * Send a message over I2C.
// * 
// * @param msg Pointer to message buffer
// * @param len Length of message to send
// * @param stop Send stop after transfer
// */
//void datalogger_send(uint8_t *msg, uint16_t len, bool stop)
//{
//	artemis_i2c_t *i2c = &module.i2c;
//	artemis_stream_t txstream = {0};
//	artemis_stream_setbuffer(&txstream, module.txbuffer, LOGGER_BUFFER_SIZE);
//	artemis_stream_reset(&txstream);
//
//	while(len > 0)
//	{
//		if(len > LOGGER_BUFFER_SIZE)
//		{
//			artemis_stream_write(&txstream, msg, LOGGER_BUFFER_SIZE);
//			artemis_i2c_send(i2c, false, &txstream);
//			artemis_stream_reset(&txstream);
//			msg += LOGGER_BUFFER_SIZE;
//            len -= LOGGER_BUFFER_SIZE;
//		}
//		else {
//			artemis_stream_write(&txstream, msg, len);
//			artemis_i2c_send(i2c, stop, &txstream);
//			len =0;
//		}
//	}
//}
//
//void datalogger_read(uint8_t addr, uint8_t *data, uint16_t len)
//{
//    artemis_i2c_t *i2c = &module.i2c;
//
//    artemis_stream_t rxstream = {0};
//    artemis_stream_t txstream = {0};
//    artemis_stream_setbuffer(&rxstream, module.rxbuffer, LOGGER_BUFFER_SIZE);
//    artemis_stream_setbuffer(&txstream, module.txbuffer, LOGGER_BUFFER_SIZE);
//
//    /** Send the command to retreive data length @addr 0xFD */
//    //artemis_stream_put(&txstream, ARTEMIS_PISTON_BUFFER_LENGTH);
//
//    /** Send the command to retreive data @addr 0xFF */
//    artemis_stream_reset(&txstream);
//    artemis_stream_reset(&rxstream);
//
//    //artemis_stream_put(&txstream, ARTEMIS_PISTON_I2C_DATA_LEN_REG);
//
//    artemis_stream_put(&txstream, addr);
//    //artemis_stream_put(&txstream, 0x00);
//    //artemis_i2c_send(i2c, false, &txstream);
//
//    am_hal_systick_delay_us(5000);
//
//    artemis_i2c_receive(i2c, true, &rxstream, len);
//    artemis_stream_read(&rxstream, data, len);
//}

//void send_writeFile_byte() // Send the writeFile byte 0x0C, but don't send a stop signal
//{
//    uint8_t msg[] = {regMap.writeFile};  // The message is the writeFile byte
//    uint16_t len = sizeof(msg) / sizeof(msg[0]);  // The length is the size of the array (1 byte in this case)
//    bool stop = false;  // Don't stop after sending
//
//    datalogger_send(msg, len, stop);
//}
//
//void datalogger_logString(char *str) // Log a String
//{
//	send_writeFile_byte();
//	uint16_t len = strlen(str);  // The length is the size of the string
//    datalogger_send((uint8_t *)str, len, true);  // Cast str to uint8_t. Send stop after the string
//}
//
//void datalogger_create(char *str) // 
//{
//	uint8_t msg[] = {regMap.writeFile};
//}

static void datalogger_write_sync(void)
{
    uint8_t cmd = LOGGER_SYNC_FILE;
    uint8_t data = 0x00;
    datalogger_i2c_write(cmd, 1, &data, 1);
}

static void datalogger_i2c_read(uint8_t offset, uint8_t offsetlen,
                                uint8_t *pBuf, uint16_t size)
{
    am_hal_iom_transfer_t transfer;

    transfer.uPeerInfo.ui32I2CDevAddr = LOGGER_I2C_ADDRESS;
    transfer.ui32InstrLen    = offsetlen;
    transfer.ui32Instr       = offset;
    transfer.eDirection      = AM_HAL_IOM_RX;
    transfer.ui32NumBytes    = size;
    transfer.pui32RxBuffer   = (uint32_t*)pBuf;
    transfer.bContinue       = false;
    transfer.ui8RepeatCount  = 0;
    transfer.ui32PauseCondition = 0;
    transfer.ui32StatusSetClr = 0;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(module.i2c.iom.handle, &transfer));
}

static void datalogger_i2c_write(uint8_t offset, uint8_t offsetlen,
                                 uint8_t *pBuf, uint16_t size)
{
    am_hal_iom_transfer_t transfer;

    transfer.uPeerInfo.ui32I2CDevAddr = LOGGER_I2C_ADDRESS;
    transfer.ui32InstrLen    = offsetlen;
    transfer.ui32Instr       = offset;
    transfer.eDirection      = AM_HAL_IOM_TX;
    transfer.ui32NumBytes    = size;
    transfer.pui32TxBuffer   = (uint32_t*)pBuf;
    transfer.bContinue       = false;
    transfer.ui8RepeatCount  = 0;
    transfer.ui32PauseCondition = 0;
    transfer.ui32StatusSetClr = 0;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(module.i2c.iom.handle, &transfer));
}
