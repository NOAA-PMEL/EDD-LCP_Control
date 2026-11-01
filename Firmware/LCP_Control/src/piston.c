#include "piston.h"
#include "artemis_piston.h"
#include "artemis_debug.h"
#include "math.h"
#include <string.h>
#include "config.h"

// Static variables
static Piston_t piston;
static volatile bool pistonRun = false;
static volatile bool LiveTask = false;

// Default retry count for transient bus NACKs / slave delays
#define PIS_I2C_DEFAULT_RETRIES     ( 10 )
#define PIS_I2C_STATUS_RETRIES      ( 5 )

// Static Function Prototypes
static bool module_pis_read_if_full(void);
static bool module_pis_read_if_fullreset(void);
static bool module_pis_read_if_zero(void);
static bool module_pis_trv_eng(void);
static float module_pis_get_volume(void);
static float module_pis_get_length(void);
static void module_pis_information(void);

bool PIS_taskStatus(void)
{
    return LiveTask;
}

// Global Functions
bool PIS_initialize(void)
{
    /** Guarantee safe non-zero divisor before any RTOS tasks run */
    taskENTER_CRITICAL();
    piston.rtos.rate = 1;
    taskEXIT_CRITICAL();

    /** Initialize the I2C Port */
    artemis_piston_i2c_initialize(PISTON_I2C_ADDR);
    module_pis_information();
    return true;
}

void PIS_uninitialize(void)
{
    /** Uninitialize the I2C Port */
    artemis_piston_i2c_uninitialize();
}

static void module_pis_information(void)
{
    /* Fetch system_id with retry guard */
    uint8_t addr = PISTON_I2C_R_SYS_ID;
    uint8_t data[8] = {0};

    ARTEMIS_DEBUG_PRINTF("\nPiston Board Information\n");
    ARTEMIS_DEBUG_PRINTF("**************************************\n");
    ARTEMIS_DEBUG_PRINTF("System Identification\t: ");

    if (artemis_piston_i2c_read_retry(addr, data, 8, PIS_I2C_DEFAULT_RETRIES))
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            ARTEMIS_DEBUG_PRINTF("%c", (char)data[i]);
        }
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("UNKNOWN (Slave did not acknowledge)");
    }
    
    ARTEMIS_DEBUG_PRINTF("\n");
    /* Wait for 50ms */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Fetch build year, firmware major, minor and build */
    addr = PISTON_I2C_R_YEAR_BUILD;
    memset(data, 0, 8);
    if (artemis_piston_i2c_read_retry(addr, data, 8, PIS_I2C_DEFAULT_RETRIES))
    {
        uint16_t year_build = (data[1] << 8) | data[0];
        uint8_t maj = data[2];
        uint8_t min = data[3];
        uint32_t build = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];

        ARTEMIS_DEBUG_PRINTF("Firmware build year\t: %u\n", year_build);
        ARTEMIS_DEBUG_PRINTF("Firmware version    \t: v%u.%u.%u\n\n", maj, min, build);
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("Firmware info query timed out\n\n");
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}

void PIS_task_move_length(TaskHandle_t *xPiston)
{
    if (LiveTask)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Warning: Task already active, spawn aborted\n");
        return;
    }

    configASSERT(xTaskCreate((TaskFunction_t)task_move_piston_to_length,
                             "Piston_Task_move_length", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS);
}

void PIS_task_move_volume(TaskHandle_t *xPiston)
{
    if (LiveTask)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Warning: Task already active, spawn aborted\n");
        return;
    }

    configASSERT(xTaskCreate((TaskFunction_t)task_move_piston_to_volume,
                             "Piston_Task_move_volume", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS);
}

void PIS_task_move_full(TaskHandle_t *xPiston)
{
    if (LiveTask)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Warning: Task already active, spawn aborted\n");
        return;
    }

    configASSERT(xTaskCreate((TaskFunction_t)task_move_piston_to_full,
                             "Piston_Task_move_full", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS);
}

void PIS_task_reset_full(TaskHandle_t *xPiston)
{
    if (LiveTask)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Warning: Task already active, spawn aborted\n");
        return;
    }

    configASSERT(xTaskCreate((TaskFunction_t)task_reset_piston_to_full,
                             "Piston_Task_reset_full", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS);
}

void PIS_task_move_zero(TaskHandle_t *xPiston)
{
    if (LiveTask)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Warning: Task already active, spawn aborted\n");
        return;
    }

    configASSERT(xTaskCreate((TaskFunction_t)task_move_piston_to_zero,
                             "Piston_Task_move_zero", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS);
}

bool PIS_task_delete(TaskHandle_t *xPiston)
{
    // 1. Signal task loop to exit and halt physical actuator
    pistonRun = false;
    PIS_stop();

    // 2. Wait up to 500ms for worker task to clean up and self-delete
    uint8_t timeout_count = 0;
    while (LiveTask && (timeout_count < 10))
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout_count++;
    }

    // 3. If worker task did not self-delete, force deletion via handle
    if (xPiston != NULL && *xPiston != NULL)
    {
        if (LiveTask)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning: Task did not exit cleanly, forcing delete\n");
            vTaskDelete(*xPiston);
        }
        *xPiston = NULL;
    }

    LiveTask = false;
    return true;
}

void task_move_piston_to_zero(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = (uint32_t)(xDelay5000ms / rate);
    float length = 0.0f;

    /* Check if already moving; attempt graceful stop before forcing a reset */
    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Moving already, issuing stop\n");
        PIS_stop();
        if (module_pis_trv_eng() == true)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Still moving, resetting board\n");
            PIS_Reset();
        }
    }

    /** Start Piston Retract to zero length */
    PIS_move_to_zero();
    vTaskDelay(period);

    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        if (module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay100ms);
            stall_count = 0;
            fullFlag = module_pis_read_if_zero();
            if (fullFlag)
            {
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else
            {
                count_reset++;
                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                    vTaskDelay(xDelay250ms);
                    PIS_Reset(); // Contains full 2000ms POR recovery window
                    PIS_move_to_zero();
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;
                        
                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                        count_reset = 0;
                        reset = 0;
                    }
                }
            }
        }
        else
        {
            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }  
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count); 
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    /* Wait out the slave's 3.2s shutdown cycle before final readout */
    artemis_piston_blackout_wait();

    /* Get length and update */
    length = module_pis_get_length();
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    vTaskDelay(xDelay100ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_zero self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_move_piston_to_full(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = (uint32_t)(xDelay5000ms / rate);
    float length = 0.0f;

    /* Check if already moving; attempt graceful stop before forcing a reset */
    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Moving already, issuing stop\n");
        PIS_stop();
        if (module_pis_trv_eng() == true)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Still moving, resetting board\n");
            PIS_Reset();
        }
    }

    /** Start Piston Extend to full length */
    PIS_move_to_full();
    vTaskDelay(period);

    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        if (module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay250ms);
            stall_count = 0;
            fullFlag = module_pis_read_if_full();
            if (fullFlag)
            {
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else
            {
                count_reset++;
                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                    vTaskDelay(xDelay250ms);
                    PIS_Reset();
                    PIS_move_to_full();
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;
                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                        count_reset = 0;
                        reset = 0;
                    }
                }
            }
        }
        else
        {
            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    /* Wait out the slave's 3.2s shutdown cycle before final readout */
    artemis_piston_blackout_wait();

    /* Get length and update */
    length = module_pis_get_length();
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    vTaskDelay(xDelay100ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_full self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_reset_piston_to_full(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = (uint32_t)(xDelay5000ms / rate);
    float length = 0.0f;

    /* Check if already moving; attempt graceful stop before forcing a reset */
    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Moving already, issuing stop\n");
        PIS_stop();
        if (module_pis_trv_eng() == true)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Still moving, resetting board\n");
            PIS_Reset();
        }
    }

    /** Start Piston Extend to full length and reset encoder counts */
    PIS_reset_to_full();
    vTaskDelay(period);

    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        if (module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay250ms);
            stall_count = 0;
            fullFlag = module_pis_read_if_fullreset();
            if (fullFlag)
            {
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else
            {
                count_reset++;
                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                    PIS_Reset();
                    PIS_reset_to_full();
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                        count_reset = 0;
                        reset = 0;
                    }
                }
            }
        }
        else
        {
            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);
        }
        
        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    /* Wait out the slave's 3.2s shutdown cycle before final readout */
    artemis_piston_blackout_wait();

    /* Get length and update */
    length = module_pis_get_length();
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    vTaskDelay(xDelay100ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_reset_piston_to_full self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_move_piston_to_length(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = (uint32_t)(xDelay5000ms / rate);
    float length = 0.0f;
    float target_len;

    /* Cache local setpoint atomically to prevent race conditions during execution */
    taskENTER_CRITICAL();
    target_len = piston.setpoint_l;
    taskEXIT_CRITICAL();

    /* Check if already moving; attempt graceful stop before forcing a reset */
    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Moving already, issuing stop\n");
        PIS_stop();
        if (module_pis_trv_eng() == true)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Still moving, resetting board\n");
            PIS_Reset();
        }
    }

    /** Start the move */
    PIS_move_to_length(target_len);
    vTaskDelay(xDelay1000ms);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        if (module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay250ms);
            length = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length = %0.5f\n", length);
            
            taskENTER_CRITICAL();
            piston.length = length;
            taskEXIT_CRITICAL();
            stall_count = 0;

            if (length >= (target_len - PISTON_LENGTH_DIFF_MAX) &&
                length <= (target_len + PISTON_LENGTH_DIFF_MAX))
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Length reached\n");
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else
            {
                count_reset++;
                if (count_reset > 3)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Resetting & re-trying setpoint = %0.5f\n", target_len);
                    PIS_Reset(); // Contains full 2000ms POR recovery window
                    PIS_move_to_length(target_len);
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                        count_reset = 0;
                        reset = 0;
                    }
                }
            }
        }
        else
        {
            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    /* Wait out the slave's 3.2s shutdown cycle before final readout */
    artemis_piston_blackout_wait();

    length = module_pis_get_length();
    ARTEMIS_DEBUG_PRINTF("PISTON :: Length updated = %0.5f\n", length);
    
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    if (fabsf(length - target_len) >= PISTON_LENGTH_DIFF_MAX)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Length diff = %0.5f\n", (length - target_len));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS, Length = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
                             length, (length - target_len), PISTON_LENGTH_DIFF_MAX);
    }
    
    vTaskDelay(xDelay100ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_length self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
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
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = (uint32_t)(xDelay5000ms / rate);
    float volume = 0.0f;
    float target_vol;

    /* Cache local setpoint atomically to prevent race conditions during execution */
    taskENTER_CRITICAL();
    target_vol = piston.setpoint_v;
    taskEXIT_CRITICAL();

    /* Check if already moving; attempt graceful stop before forcing a reset */
    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Moving already, issuing stop\n");
        PIS_stop();
        if (module_pis_trv_eng() == true)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Still moving, resetting board\n");
            PIS_Reset();
        }
    }
    vTaskDelay(xDelay50ms);
    PIS_move_to_volume(target_vol);
    vTaskDelay(xDelay500ms);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        if (module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay100ms);
            volume = module_pis_get_volume();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Volume = %.3fin³\n", volume);

            taskENTER_CRITICAL();
            piston.volume = volume;
            taskEXIT_CRITICAL();
            stall_count = 0;

            if (volume >= (target_vol - PISTON_VOLUME_DIFF_MAX) &&
                volume <= (target_vol + PISTON_VOLUME_DIFF_MAX))
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Volume reached\n");
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else
            {
                count_reset++;
                if (count_reset > 3)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Resetting & re-trying volume = %0.5f\n", target_vol);
                    PIS_Reset(); // Contains full 2000ms POR recovery window
                    PIS_move_to_volume(target_vol);
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                        count_reset = 0;
                        reset = 0;
                    }
                }
            }
        }
        else
        {
            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    /* Wait out the slave's 3.2s shutdown cycle before final readout */
    artemis_piston_blackout_wait();

    volume = module_pis_get_volume();
    ARTEMIS_DEBUG_PRINTF("PISTON :: Volume updated = %.3fin³\n", volume);
    
    taskENTER_CRITICAL();
    piston.volume = volume;
    taskEXIT_CRITICAL();

    if (fabsf(volume - target_vol) >= PISTON_VOLUME_DIFF_MAX)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Volume diff = %0.3f\n", (volume - target_vol));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS. Volume = %0.3f, diff = %0.3f, max_diff = %0.3f\n",
                             volume, (volume - target_vol), PISTON_VOLUME_DIFF_MAX);
    }
    
    vTaskDelay(xDelay50ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_volume self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void PIS_set_piston_rate(uint8_t rate)
{
    if (rate > 0 && rate <= 10)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Setting rate = %uHz\n", rate);
        taskENTER_CRITICAL();
        piston.rtos.rate = rate;
        taskEXIT_CRITICAL();
    }
}

bool PIS_set_volume(float volume)
{
    bool retVal = false;
    if ((volume > 0.0f) && (volume < 1000.0f))
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Volume, set-point = %0.4f\n", volume);
        taskENTER_CRITICAL();
        piston.setpoint_v = volume;
        taskEXIT_CRITICAL();
        retVal = true;
    }
    return retVal;
}

bool PIS_set_length(float length)
{
    bool retVal = false;
    if ((length >= 0.0f) && (length < 12.0f))
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Length, set-point = %0.4f\n", length);
        taskENTER_CRITICAL();
        piston.setpoint_l = length;
        taskEXIT_CRITICAL();
        retVal = true;
    }
    return retVal;
}

void PIS_extend(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x01, 0x01, 0x00, 0x01};

    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
    artemis_piston_i2c_send_msg(cmd, 5, true);
}

void PIS_retract(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_DIR;
    uint8_t cmd[5] = {addr, 0xFF, 0x01, 0x00, 0x01};

    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
    artemis_piston_i2c_send_msg(cmd, 5, true);
}

void PIS_stop(void)
{
    artemis_piston_stop_and_wait();
}

bool PIS_is_at_zero(void)
{
    return module_pis_read_if_zero();
}

bool PIS_is_at_full(void)
{
    return module_pis_read_if_full();
}

bool PIS_calibration_check(void)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t data = 0;

    if (artemis_piston_i2c_read_retry(addr, &data, 1, PIS_I2C_STATUS_RETRIES))
    {
        return (data == 0x01);
    }
    return false;
}

void PIS_calibration(bool cal)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t cmd[2] = {addr, 0x00};

    if (cal == true)
    {
        cmd[1] = (uint8_t)cal;
    }
    else
    {
        return;
    }

    if (PIS_calibration_check() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Already calibrated\n");
        return;
    }
    ARTEMIS_DEBUG_PRINTF("PISTON :: Being calibrated\n");

    vTaskDelay(pdMS_TO_TICKS(100));
    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

float PIS_get_length(void)
{
    return module_pis_get_length();
}

float PIS_get_volume(void)
{
    return module_pis_get_volume();
}

bool PIS_move_to_length(float length)
{
    uint8_t addr = PISTON_I2C_W_SET_LENGTH;

    union {
        float fLength;
        uint32_t u32Length;
    } conv;

    conv.fLength = length;
    uint8_t v[5] = {0};
    v[0] = addr;

    for (uint8_t i = 0; i < 4; i++)
    {
        v[i + 1] = (conv.u32Length >> (i * 8)) & 0xFF;
    }

    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
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

    union {
        float dVolume;
        uint32_t u32Volume;
    } conv;

    conv.dVolume = volume;
    uint8_t v[5] = {0};
    v[0] = addr;

    for (uint8_t i = 0; i < 4; i++)
    {
        v[i + 1] = (conv.u32Volume >> (i * 8)) & 0xFF;
    }

    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
    artemis_piston_i2c_send_msg(v, 5, true);
    return true;
}

static bool module_pis_read_if_full(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FULL;
    uint8_t data = 0;
    if (artemis_piston_i2c_read_retry(addr, &data, 1, PIS_I2C_STATUS_RETRIES))
    {
        return (bool)data;
    }
    return false;
}

static bool module_pis_read_if_fullreset(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FRST;
    uint8_t data = 0;
    if (artemis_piston_i2c_read_retry(addr, &data, 1, PIS_I2C_STATUS_RETRIES))
    {
        return (bool)data;
    }
    return false;
}

static bool module_pis_read_if_zero(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_ZERO;
    uint8_t data = 0;
    if (artemis_piston_i2c_read_retry(addr, &data, 1, PIS_I2C_STATUS_RETRIES))
    {
        return (bool)data;
    }
    return false;
}

static bool module_pis_trv_eng(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_ENG;
    uint8_t data = 0;
    if (artemis_piston_i2c_read_retry(addr, &data, 1, PIS_I2C_STATUS_RETRIES))
    {
        return (bool)data;
    }
    return false;
}

static float module_pis_get_length(void)
{
    uint8_t addr = PISTON_I2C_R_LENGTH_TOTAL;
    uint8_t data[4] = {0};

    if (!artemis_piston_i2c_read_retry(addr, data, 4, PIS_I2C_DEFAULT_RETRIES))
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Warning - Failed to read length from slave\n");
        return 0.0f;
    }

    union {
        float fLength;
        uint32_t u32Length;
    } pis;

    pis.u32Length = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        pis.u32Length |= ((uint32_t)data[i] << (8 * i));
    }

    return pis.fLength;
}

static float module_pis_get_volume(void)
{
    uint8_t addr = PISTON_I2C_R_VOLUME_TOTAL;
    uint8_t data[4] = {0};

    if (!artemis_piston_i2c_read_retry(addr, data, 4, PIS_I2C_DEFAULT_RETRIES))
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Warning - Failed to read volume from slave\n");
        return 0.0f;
    }

    union {
        float fVolume;
        uint32_t u32Volume;
    } pis;

    pis.u32Volume = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        pis.u32Volume |= ((uint32_t)data[i] << (8 * i));
    }

    return pis.fVolume;
}

void PIS_Reset(void)
{
    uint8_t addr = PISTON_I2C_W_RESET;
    uint8_t cmd[2] = {addr, PISTON_I2C_W_RESET_KEY};
    
    /** 1. Put in write mode */
    artemis_piston_set_write_mode(true);
    
    /** 2. CRITICAL: Give slave MCU 25ms to process unlock & exit ISR */
    vTaskDelay(pdMS_TO_TICKS(25));
    
    /** 3. Send the reset command */
    artemis_piston_i2c_send_msg(cmd, 2, true);
    
    /** 4. Allow MSP430 full POR reboot margin (PMM_trigPOR bootloader + init) */
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void PIS_move_to_full(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_FULL;
    uint8_t cmd[2] = {addr, 0x01};
    
    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

void PIS_move_to_zero(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_ZERO;
    uint8_t cmd[2] = {addr, 0x01};
    
    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

void PIS_reset_to_full(void)
{
    uint8_t addr = PISTON_I2C_RW_RST_FULL;
    uint8_t cmd[2] = {addr, 0x01};
    
    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(25));
    artemis_piston_i2c_send_msg(cmd, 2, true);
}