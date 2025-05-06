#include "piston.h"
#include "artemis_piston.h"
#include "artemis_debug.h"
#include "math.h"
#include "config.h"

// Static variables
static Piston_t piston;
static volatile bool pistonRun = false;

// Static Function Prototypes
static bool module_pis_read_if_full(void);
static bool module_pis_read_if_fullreset(void);
static bool module_pis_read_if_zero(void);
static bool module_pis_trv_eng(void);
static float module_pis_get_volume(void);
static float module_pis_get_length(void);
static void module_pis_information(void);

// Global Functions
bool PIS_initialize(void)
{
    /** Initialize the I2C Port */
    artemis_piston_i2c_initialize(PISTON_I2C_ADDR);
    module_pis_information();
    bool success = true;
    return success;
}

void PIS_uninitialize(void)
{
    /** unInitialize the I2C Port */
    artemis_piston_i2c_uninitialize();
}

static void module_pis_information(void)
{
    /* fetch system_id */
    uint8_t addr = PISTON_I2C_R_SYS_ID;
    uint8_t data[8] = {0};

    ARTEMIS_DEBUG_PRINTF("\nPiston Board Information\n");
    ARTEMIS_DEBUG_PRINTF("**************************************\n");
    ARTEMIS_DEBUG_PRINTF("System Identification\t: ");

    artemis_piston_i2c_read(addr, data, 8);
    for (uint8_t i=0; i<8; i++){
        ARTEMIS_DEBUG_PRINTF("%c", (char)data[i]);
    }
    ARTEMIS_DEBUG_PRINTF("\n");
    /* wait for 50ms */
    am_hal_systick_delay_us(50000);

    /* fetch build year, firmware major, minor and build */
    addr = PISTON_I2C_R_YEAR_BUILD;
    memset(data, 0, 8);
    artemis_piston_i2c_read(addr, data, 8);
    uint16_t year_build = data[1]<<8 | data[0];
    uint8_t maj = data[2];
    uint8_t min = data[3];
    uint32_t build = data[7]<<24 | data[6]<<16 | data[5]<<8 | data[4];

    ARTEMIS_DEBUG_PRINTF("Firmware build year\t: %u\n", year_build);
    ARTEMIS_DEBUG_PRINTF("Firmware version   \t: v%u.%u.%u\n", maj, min, build);
    ARTEMIS_DEBUG_PRINTF("\n");
    am_hal_systick_delay_us(50000);
}

void PIS_task_move_length(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_length,
                                "Piston_Task_move_length", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_volume(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_volume,
                                "Piston_Task_move_volume", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_full(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_full,
                                "Piston_Task_move_full", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xPiston) == pdPASS );
}

void PIS_task_reset_full(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_reset_piston_to_full,
                                "Piston_Task_reset_full", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_zero(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_zero,
                                "Piston_Task_move_zero", 256, NULL,
                                tskIDLE_PRIORITY + 4UL,
                                xPiston) == pdPASS );
}

void PIS_task_delete(TaskHandle_t xHandle)
{
    uint8_t wait = 0;
    bool delete = false;

    /* check the task state 
    while (!delete && wait < 20)
    {
        eTaskState eState = eTaskGetState(xHandle);
        if ( (eState==eReady) || (eState==eBlocked) )
        {
            if (pistonRun)
            {
                pistonRun = false;
            }
        }
        else if (eState==eRunning)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Task is in eRunning state, wait\n");
        }
        else if (eState==eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Task is Suspended\n");
            vTaskDelete(xHandle);
        }
        else if ( (eState==eDeleted)|| (eState==eInvalid) )
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Task is Deleted\n");
            delete = true;
        }
        wait++;
        // wait for 50ms
        vTaskDelay(xDelay50ms);
    }*/

    pistonRun = false;
    vTaskDelay(xDelay1000ms);
}

void task_move_piston_to_zero(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = xDelay5000ms/piston.rtos.rate;
    float length = 0.0;

    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();

        vTaskDelay(xDelay2000ms);

    }

    /** Start Piston Retract to zero length */
    PIS_move_to_zero();
    vTaskDelay(period);

    /** Start the task of reading until we hit the end stop */
    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;

    while(pistonRun)
    {
        /** Read the piston memory to see if we're done */
        if(module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay250ms);
            fullFlag = module_pis_read_if_zero();
            if (fullFlag)
            {
                pistonRun = false;
            }
            count_reset++;
            if (count_reset > 4)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                vTaskDelay(xDelay1000ms);
                PIS_Reset();
                vTaskDelay(period);
                PIS_move_to_zero();
                vTaskDelay(period);
                count_reset = 0;
            }
        }
        else
        {
            vTaskDelay(xDelay250ms);
            length = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length in moving = %0.5f\n", length);
            fullFlag = module_pis_read_if_zero();
        }
        vTaskDelay(xDelay1000ms);
    }

    /* get the length and update it */
    length = module_pis_get_length();
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
    vTaskDelay(xDelay1000ms);
}

void task_move_piston_to_full(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = xDelay5000ms/piston.rtos.rate;
    float length = 0.0;

    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();

        vTaskDelay(xDelay2000ms);

    }

    /** Start Piston Extend to full length */
    PIS_move_to_full();
    vTaskDelay(period);

    /** Start the task of reading until we hit the end stop */
    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;


    while(pistonRun)
    {
        /** Read the piston memory to see if we're done */
        if(module_pis_trv_eng() == false)
        {
            count_reset++;
            vTaskDelay(xDelay250ms);
            fullFlag = module_pis_read_if_full();
            if (fullFlag)
            {
                pistonRun = false;
            }
            if (count_reset > 4)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                vTaskDelay(xDelay1000ms);
                PIS_Reset();
                vTaskDelay(period);
                PIS_move_to_full();
                vTaskDelay(period);
                count_reset = 0;
            }
        }
        else
        {
            vTaskDelay(xDelay250ms);
            length = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length in moving = %0.5f\n", length);
            fullFlag = module_pis_read_if_full();
        }
        vTaskDelay(xDelay1000ms);
    }

    /* get the length and update it */
    length = module_pis_get_length();
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
    vTaskDelay(xDelay1000ms);
}

void task_reset_piston_to_full(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = xDelay5000ms/piston.rtos.rate;
    float length = 0.0;

    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();

        vTaskDelay(xDelay2000ms);

    }

    /** Start Piston Extend to full length and reset encoder counts */
    PIS_reset_to_full();
    vTaskDelay(period);

    /** Start the task of reading until we hit the end stop */
    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;

    while(pistonRun)
    {
        /** Read the piston memory to see if we're done */
        if(module_pis_trv_eng() == false)
        {
            count_reset++;
            vTaskDelay(xDelay250ms);
            fullFlag = module_pis_read_if_fullreset();
            if (fullFlag)
            {
                pistonRun = false;
            }
            if (count_reset > 4)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                vTaskDelay(xDelay1000ms);
                PIS_Reset();
                vTaskDelay(period);
                PIS_reset_to_full();
                vTaskDelay(period);
                count_reset = 0;
            }
        }
        else
        {
            vTaskDelay(xDelay250ms);
            length = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length in moving = %0.5f\n", length);
            fullFlag = module_pis_read_if_fullreset();
        }
        vTaskDelay(xDelay1000ms);
    }

    /* get the length and update it */
    length = module_pis_get_length();
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
    vTaskDelay(xDelay1000ms);
}

void task_move_piston_to_length(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = xDelay5000ms/piston.rtos.rate;
    float length = 0.0;

    /*  before sending any write cmd to piston, read first
        if piston is already moving */
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();

        vTaskDelay(xDelay2000ms);

    }
    vTaskDelay(xDelay50ms);
    /** Start the move */
    PIS_move_to_length(piston.setpoint_l);
    /** wait 500ms after shooting an I2C command */
    vTaskDelay(xDelay250ms);

    /** Start reading until we hit the volume */
    pistonRun = true;
    uint8_t count_reset = 0;

    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;
    float last_length = -1.0;

    while(pistonRun)
    {
        /** Read the piston memory to see if we're done moving and at volume */
        if(module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay250ms);
            length = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length = %0.5f\n", length);
            taskENTER_CRITICAL();
            piston.length = length;
            taskEXIT_CRITICAL();

            if (piston.length >=(piston.setpoint_l - PISTON_LENGTH_DIFF_MAX) &&
                piston.length <=(piston.setpoint_l + PISTON_LENGTH_DIFF_MAX))
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Length reached\n");
                pistonRun = false;
                count_reset = 0;
            }
            else
            {
                count_reset++;
                if (count_reset > 3)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                    vTaskDelay(xDelay500ms);
                    PIS_Reset();
                    vTaskDelay(period);
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Setting length = %0.5f\n", piston.setpoint_l);
                    PIS_move_to_length(piston.setpoint_l);
                    vTaskDelay(period);
                    count_reset = 0;
                }
            }
        }
        else
        {
            vTaskDelay(xDelay250ms);
            length = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length in moving = %0.5f\n", length);
            taskENTER_CRITICAL();
            piston.length = length;
            taskEXIT_CRITICAL();

            if (isnan(last_length) || isinf(last_length) 
                || isnan(length)   || isinf(length) 
                || fabs(length - last_length) < 0.001) { // check for stall condition: Invalid Float for length/last_length or no movement
                stall_count++; // increment stall count
                vTaskDelay(xDelay500ms);
                if (isnan(length) || isinf(length)) {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Invalid length value detected\n");
                }
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count = %d/%d\n", stall_count, stall_count_max);
                if (stall_count > stall_count_max) {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                    PIS_Reset();            // reset the board
                    vTaskDelay(period);     // wait for reset to complete
                    pistonRun = false;      // exit the loop
                    stall_count = 0;        // reset stall count
                }

            } else {
                stall_count = 0;            // reset stall count
            }
            last_length = length;           // update last length
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    vTaskDelay(xDelay100ms);
    length = module_pis_get_length();
    ARTEMIS_DEBUG_PRINTF("PISTON :: Length updated = %0.5f\n", length);
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    /** Check to see if length is valid , and update the last length */
    if( fabs(length - piston.setpoint_l) >= PISTON_LENGTH_DIFF_MAX )
    {
        /** ERROR - Alert the system somehow */
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Length diff = %0.5f\n", (length - piston.setpoint_l));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS, Length = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
                                    length, (length - piston.setpoint_l), PISTON_LENGTH_DIFF_MAX);
    }
    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
    vTaskDelay(xDelay1000ms);
}

bool PIS_Get_Volume(float *volume)
{
    bool retVal = false;
    taskENTER_CRITICAL();
    *volume = piston.volume;
    retVal = true;
    taskEXIT_CRITICAL();
    return retVal;
}

bool PIS_Get_Length(float *length)
{
    bool retVal = false;
    taskENTER_CRITICAL();
    *length = piston.length;
    retVal = true;
    taskEXIT_CRITICAL();
    return retVal;
}

void task_move_piston_to_volume(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = xDelay5000ms/piston.rtos.rate;
    float volume = 0.0;

    /*  before sending any write cmd to piston, read first
        if piston is already moving */
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();

        vTaskDelay(xDelay2000ms);

    }
    vTaskDelay(xDelay50ms);
    PIS_move_to_volume(piston.setpoint_v);
    /** wait 500ms after shooting an I2C command */
    vTaskDelay(xDelay500ms);

    /** Start reading until we hit the volume */
    pistonRun = true;
    uint8_t count_reset = 0;

    while(pistonRun)
    {
        /** Read the piston memory to see if we're done moving and at volume */
        if(module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay250ms);
            volume = module_pis_get_volume();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Volume = %.3fin³\n", volume);

            taskENTER_CRITICAL();
            piston.volume = volume;
            taskEXIT_CRITICAL();

            if (piston.volume >=(piston.setpoint_v - PISTON_VOLUME_DIFF_MAX) &&
                piston.volume <=(piston.setpoint_v + PISTON_VOLUME_DIFF_MAX))
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Volume reached\n");
                pistonRun = false;
                count_reset = 0;
            }
            else
            {
                count_reset++;
                if (count_reset > 3)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                    vTaskDelay(xDelay500ms);
                    PIS_Reset();
                    vTaskDelay(period);
                    PIS_move_to_volume(piston.setpoint_v);
                    vTaskDelay(period);
                    count_reset = 0;
                }

            }
        }
        else
        {
            vTaskDelay(xDelay250ms);
            volume = module_pis_get_volume();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Volume in moving = %.3fin³\n", volume);

            taskENTER_CRITICAL();
            piston.volume = volume;
            taskEXIT_CRITICAL();
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    vTaskDelay(xDelay100ms);
    volume = module_pis_get_volume();
    ARTEMIS_DEBUG_PRINTF("PISTON :: Volume updated = %.3fin³\n", volume);
    taskENTER_CRITICAL();
    piston.volume = volume;
    taskEXIT_CRITICAL();

    /** Check to see if volume is valid , and update the last volume*/
    if( fabs(volume - piston.setpoint_v) >= PISTON_VOLUME_DIFF_MAX)
    {
        /** ERROR - Alert the system somehow */
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Volume diff = %0.3f\n", (volume - piston.setpoint_v));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS. Volume = %0.3f, diff = %0.3f, max_diff = %0.3f\n",
                                    volume, (volume - piston.setpoint_v), PISTON_VOLUME_DIFF_MAX);
    }
    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
    vTaskDelay(xDelay1000ms);
}

void PIS_set_piston_rate(uint8_t rate)
{
    if (rate > 0 && rate <= 10)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Setting rate = %uHz\n", rate);
        piston.rtos.rate = rate;
    }
}

bool PIS_set_volume(float volume)
{
    bool retVal = false;
    if( (volume > 0) && (volume < 1000) )
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Volume, set-point = %0.4f\n", volume);
        piston.setpoint_v = volume;
        retVal = true;
    }
    return retVal;
}

bool PIS_set_length(float length)
{
    bool retVal = false;
    if( (length >= 0) && (length < 12.0) )
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Length, set-point = %0.4f\n", length);
        piston.setpoint_l = length;
        retVal = true;
    }
    return retVal;
}

void PIS_extend(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x01, 0x01, 0x00, 0x01};

    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);
}

void PIS_retract(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_DIR;
    uint8_t cmd[5] = {addr, 0xFF, 0x01, 0x00, 0x01};

    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);
}

void PIS_stop(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x00, 0x01, 0x00, 0x01};

    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);

}

bool PIS_calibration_check(void)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t data = 0;

    artemis_piston_i2c_read(addr, &data, 1);

    if (data == 0x01)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void PIS_calibration(bool cal)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t cmd[2] = {addr, 0x00};

    if (cal == true)
    {
        cmd[1] = (uint8_t) cal;
    }
    else
    {
        return;
    }

    /* check if piston is already calibrated */
    if(PIS_calibration_check() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Already calibrated\n");
        return;
    }
    ARTEMIS_DEBUG_PRINTF("PISTON :: Being calibrated\n");

    am_hal_systick_delay_us(100000);
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

float PIS_get_length(void)
{
    float length = 0;
    length = module_pis_get_length();
    return length;
}

float PIS_get_volume(void)
{
    float volume = 0;
    volume = module_pis_get_volume();
    return volume;
}

bool PIS_move_to_length(float length)
{
    uint8_t addr = PISTON_I2C_W_SET_LENGTH;

    union{
        /* data */
        float fLength;
        uint32_t u32Length;
    }piston;

    piston.fLength = length;

    /* Send command to move to volume */
    uint8_t v[5] = {0};
    v[0] = addr;

    for(uint8_t i=0; i<4; i++)
    {
        v[i+1] = piston.u32Length >> ((i)*8) & 0xFF;
    }

    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(v, 5, true);
    return true;
}

bool PIS_get_position(void)
{
    return true;
}

bool PIS_move_to_volume(float volume)
{
    uint8_t addr = PISTON_I2C_W_SET_VOLUME;

    union{
        /* data */
        float dVolume;
        uint32_t u64Volume;
    }piston;

    piston.dVolume = volume;
    /* Send command to move to volume */
    uint8_t v[5] = {0};
    v[0] = addr;

    for(uint8_t i=0; i<4; i++)
    {
        v[i+1] = piston.u64Volume >> ((i)*8) & 0xFF;
    }

    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(v, 5, true);

    return true;
}


static bool module_pis_read_if_full(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FULL;
    uint8_t data;

    /** Send the message to read TRV_FULL address */
    artemis_piston_i2c_read(addr, &data, 1);
    
    return (bool) data;
}

static bool module_pis_read_if_fullreset(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FRST;
    uint8_t data;

    /** Send the message to read TRV_FRST address */
    artemis_piston_i2c_read(addr, &data, 1);
    
    return (bool) data;
}

static bool module_pis_read_if_zero(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_ZERO;
    uint8_t data;

    /** Send the message to read TRV_FULL address */
    artemis_piston_i2c_read(addr, &data, 1);
    
    return (bool) data;
}

static bool module_pis_trv_eng(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_ENG;
    uint8_t data = 0;
    artemis_piston_i2c_read(addr, &data, 1);
    return (bool) data;
}

static float module_pis_get_length(void)
{
    uint8_t addr = PISTON_I2C_R_LENGTH_TOTAL;
    uint8_t data[4] = {0};
    artemis_piston_i2c_read(addr, data, 4);

    union{
        float fLength;
        uint32_t u32Length;
    }pis;

    pis.u32Length = 0;
    for(uint8_t i=0; i<4; i++)
    {
        pis.u32Length |= data[i] << (8*i);
    }

    return pis.fLength;
}

static float module_pis_get_volume(void)
{
    uint8_t addr = PISTON_I2C_R_VOLUME_TOTAL;
    uint8_t data[4] = {0};

    artemis_piston_i2c_read(addr, data, 4);

    union{
        float fVolume;
        uint32_t u32Volume;
    }pis;

    pis.u32Volume = 0;

    for(uint8_t i=0; i<4; i++)
    {
        pis.u32Volume |= data[i] << (8*i);
    }

    return pis.fVolume;
}

void PIS_Reset(void)
{
    uint8_t addr = PISTON_I2C_W_RESET;
    uint8_t cmd[2] = {addr, PISTON_I2C_W_RESET_KEY};
    
    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

void PIS_move_to_full(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_FULL;
    uint8_t cmd[2] = {addr, 0x01};
    
    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

void PIS_move_to_zero(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_ZERO;
    uint8_t cmd[2] = {addr, 0x01};
    
    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

void PIS_reset_to_full(void)
{
    uint8_t addr = PISTON_I2C_RW_RST_FULL;
    uint8_t cmd[2] = {addr, 0x01};
    
    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}