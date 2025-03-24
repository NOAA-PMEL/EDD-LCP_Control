#ifndef DATALOGGER_H
#define DATALOGGER_H

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include "artemis_i2c.h"
#include "artemis_rtc.h"
#include "am_bsp_pins.h"
#include "sensors.h"
#include "stdbool.h"

//*****************************************************************************
//
// Project Files
//
//*****************************************************************************


#define LOGGER_I2C_ADDRESS      0x2A    // I2C address (Qwiic OpenLog Default is 0x2A)
#define LOGGER_BUFFER_SIZE      32      // Bytes

#define LOGGER_ID               0x00
#define LOGGER_STATUS           0x01
#define LOGGER_FW_MAJ           0x02
#define LOGGER_FW_MIN           0x03
#define LOGGER_INT_EN           0x04    // not used in Qwiic datalogger
#define LOGGER_INIT_LOG         0x05
#define LOGGER_CREATE_FILE      0x06
#define LOGGER_MKDIR            0x07
#define LOGGER_CD               0x08
#define LOGGER_READ_FILE        0x09
#define LOGGER_START_POS        0x0A
#define LOGGER_OPEN_FILE        0x0B
#define LOGGER_WRITE_FILE       0x0C
#define LOGGER_FILE_SIZE        0x0D
#define LOGGER_LIST             0x0E
#define LOGGER_RM               0x0F
#define LOGGER_RM_RECUR         0x10
#define LOGGER_SYNC_FILE        0x11

typedef enum e_status_dl
{
    SDINIT_GOOD = 0x00,     //  Bit 0: SD/Init Good
    LAST_CMD_SUCCEEDED,     //  Bit 1: Last Command Succeeded
    LAST_CMD_KNOWN,         //  Bit 2: Last Command Known
    FILE_CURR_OPEN,         //  Bit 3: File Currently Open
    IN_ROOT_DIR             //  Bit 4: In Root Directory

} status_dl;

bool datalogger_init(uint8_t iomNo);    // iomNo. 1 or 4
void datalogger_deinit(uint8_t iomNo);  // iomNo. 1 or 4
bool datalogger_device_info(void);
void datalogger_log_init(void);
void datalogger_power_on(void);
void datalogger_power_off(void);

uint16_t datalogger_fw_version(void);
uint8_t datalogger_status(void);
uint8_t datalogger_get_id(void);

void datalogger_createfile(char *filename);
void datalogger_openfile(char *filename);
void datalogger_writefile(char *contents);
void datalogger_writefile_length(uint8_t *contents, uint16_t length);
void datalogger_readfile(char *filename, char *contents, uint32_t size);

void datalogger_mkdir(char *dirname);
void datalogger_cd(char *dirname);

uint32_t datalogger_rmfile(char *filename, bool recursive);
uint32_t datalogger_rmdir(char *dirname);
int32_t datalogger_filesize(char *filename);

void datalogger_log_debug_init(void);
void datalogger_log_debug(const char *fmt, ...);

/* data logging functions */
void datalogger_predeploy_mode(SensorGps_t *gps, bool check);

char *datalogger_park_create_file(uint16_t park_nr);
void datalogger_park_mode(char *filename, float pressure,
                            float temp, rtc_time *time);

void datalogger_test_sbd_messages_init(void);
void datalogger_test_sbd_messages(char *filename, uint8_t *tData, uint16_t length);

char *datalogger_profile_create_file(uint16_t sps_nr);
void datalogger_profile_mode(char *filename, float pressure,
                            float temp, rtc_time *time);

void datalogger_surface_mode(float pressure, float temp);

/* for reading pressure profile */
void datalogger_pressure(float *pressure);
void datalogger_read_test_profile(bool reset);

#endif
