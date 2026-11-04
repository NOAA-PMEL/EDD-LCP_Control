#include "piston.h"
#include "artemis_piston.h"
#include "artemis_debug.h"
#include "math.h"
#include "config.h"
#include <string.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"

// Non-blocking RTOS delay wrapper with sub-tick truncation guard
static inline void pis_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        uint32_t ticks = pdMS_TO_TICKS(ms);
        vTaskDelay(ticks > 0 ? ticks : 1);
    }
    else
    {
        am_util_delay_ms(ms);
    }
}

// Static variables
static Piston_t piston;
static volatile bool pistonRun = false;
static volatile bool LiveTask = false;

// Static Function Prototypes
static int8_t module_pis_status_full(void);
static int8_t module_pis_status_fullreset(void);
static int8_t module_pis_status_zero(void);
static int8_t module_pis_trv_status(void);
static bool module_pis_get_length_safe(float *out_length);
static bool module_pis_get_volume_safe(float *out_volume);
static float module_pis_get_volume(void);
static float module_pis_get_length(void);
static void module_pis_information(void);
static bool pis_unlock_write_mode(void);

/**
 * @brief Helper to unlock write mode on the secondary MSP430 MCU with readback verification.
 * Accommodates slave ISR UART logging and eUSCI peripheral re-enable cycles.
 */
static bool pis_unlock_write_mode(void)
{
    uint8_t check_val = 0;

    for (uint8_t retry = 0; retry < 5; retry++)
    {
        artemis_piston_set_write_mode(true);
        // Allow time for slave USCI_B1_ISR -> CTRL_Check_Write() UART debug logging to clear
        pis_delay_ms(150);

        // Verify the slave accepted 0xA5 into RAM_VAR_WRITE
        check_val = 0;
        if (artemis_piston_i2c_read(PISTON_I2C_W_VAR_WRITE, &check_val, 1) && (check_val == 0xA5))
        {
            return true;
        }
        pis_delay_ms(50);
    }

    ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, write mode unlock failed after retries (read: 0x%02X)\n", check_val);
    return false;
}

bool PIS_taskStatus(void)
{
    return LiveTask;
}

// Global Functions
bool PIS_initialize(void)
{
    /** Initialize the I2C Port */
    artemis_piston_i2c_initialize(PISTON_I2C_ADDR);
    module_pis_information();
    return true;
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

    for (uint8_t i = 0; i < 8; i++)
    {
        ARTEMIS_DEBUG_PRINTF("%c", (char)data[i]);
    }

    ARTEMIS_DEBUG_PRINTF("\n");
    pis_delay_ms(50);

    /* fetch build year, firmware major, minor and build */
    addr = PISTON_I2C_R_YEAR_BUILD;
    memset(data, 0, 8);
    artemis_piston_i2c_read(addr, data, 8);
    uint16_t year_build = (data[1] << 8) | data[0];
    uint8_t maj = data[2];
    uint8_t min = data[3];
    uint32_t build = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) | ((uint32_t)data[5] << 8) | data[4];

    ARTEMIS_DEBUG_PRINTF("Firmware build year\t: %u\n", year_build);
    ARTEMIS_DEBUG_PRINTF("Firmware version   \t: v%u.%u.%u\n\n", maj, min, build);
    pis_delay_ms(50);
}

void PIS_task_move_length(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_length,
                             "Piston_Task_move_length", 512, NULL,
                             tskIDLE_PRIORITY + 3UL,
                             xPiston) == pdPASS );
}

void PIS_task_move_volume(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_volume,
                             "Piston_Task_move_volume", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS );
}

void PIS_task_move_full(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_full,
                             "Piston_Task_move_full", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS );
}

void PIS_task_reset_full(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_reset_piston_to_full,
                             "Piston_Task_reset_full", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS );
}

void PIS_task_move_zero(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_zero,
                             "Piston_Task_move_zero", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
                             xPiston) == pdPASS );
}

bool PIS_task_delete(TaskHandle_t *xPiston)
{
    pistonRun = false;
    pis_delay_ms(100);

    if (xPiston != NULL && *xPiston != NULL)
    {
        if (LiveTask)
        {
            vTaskDelete(*xPiston);
            LiveTask = false;
        }
        *xPiston = NULL;
    }
    return true;
}

void task_move_piston_to_zero(void)
{
    LiveTask = true;
    assert(piston.rtos.rate != 0);
    float length = 0.0f;

    uint8_t wait_settle = 0;
    while (module_pis_trv_status() == 1 && wait_settle < 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        pis_delay_ms(500);
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
    }

    PIS_move_to_zero();
    pis_delay_ms(5000 / piston.rtos.rate);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    const uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0)
        {
            pis_delay_ms(100);
            stall_count = 0;
            int8_t zeroFlag = module_pis_status_zero();

            if (zeroFlag == 1)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Zero limit reached\n");
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else if (zeroFlag == 0)
            {
                count_reset++;
                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                    PIS_Reset();
                    PIS_move_to_zero();
                    pis_delay_ms(5000 / piston.rtos.rate);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                    }
                }
            }
        }
        else
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
            }
        }

        if (pistonRun)
        {
            pis_delay_ms(5000 / piston.rtos.rate);
        }
    }

    if (module_pis_get_length_safe(&length))
    {
        taskENTER_CRITICAL();
        piston.length = length;
        taskEXIT_CRITICAL();
    }

    pis_delay_ms(100);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_zero self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_move_piston_to_full(void)
{
    LiveTask = true;
    assert(piston.rtos.rate != 0);
    float length = 0.0f;

    uint8_t wait_settle = 0;
    while (module_pis_trv_status() == 1 && wait_settle < 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        pis_delay_ms(500);
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
    }

    PIS_move_to_full();
    pis_delay_ms(5000 / piston.rtos.rate);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    const uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0)
        {
            pis_delay_ms(100);
            stall_count = 0;
            int8_t fullFlag = module_pis_status_full();

            if (fullFlag == 1)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Full limit reached\n");
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else if (fullFlag == 0)
            {
                count_reset++;
                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                    PIS_Reset();
                    PIS_move_to_full();
                    pis_delay_ms(5000 / piston.rtos.rate);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                    }
                }
            }
        }
        else
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
            }
        }

        if (pistonRun)
        {
            pis_delay_ms(5000 / piston.rtos.rate);
        }
    }

    if (module_pis_get_length_safe(&length))
    {
        taskENTER_CRITICAL();
        piston.length = length;
        taskEXIT_CRITICAL();
    }

    pis_delay_ms(100);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_full self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_reset_piston_to_full(void)
{
    LiveTask = true;
    assert(piston.rtos.rate != 0);
    float length = 0.0f;

    uint8_t wait_settle = 0;
    while (module_pis_trv_status() == 1 && wait_settle < 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        pis_delay_ms(500);
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
    }

    PIS_reset_to_full();
    pis_delay_ms(5000 / piston.rtos.rate);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    const uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0)
        {
            pis_delay_ms(100);
            stall_count = 0;
            int8_t fullResetFlag = module_pis_status_fullreset();

            if (fullResetFlag == 1)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Full reset complete\n");
                pistonRun = false;
                count_reset = 0;
                reset = 0;
            }
            else if (fullResetFlag == 0)
            {
                count_reset++;
                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting\n");
                    PIS_Reset();
                    PIS_reset_to_full();
                    pis_delay_ms(5000 / piston.rtos.rate);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                    }
                }
            }
        }
        else
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
            }
        }

        if (pistonRun)
        {
            pis_delay_ms(5000 / piston.rtos.rate);
        }
    }

    if (module_pis_get_length_safe(&length))
    {
        taskENTER_CRITICAL();
        piston.length = length;
        taskEXIT_CRITICAL();
    }

    pis_delay_ms(100);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_reset_piston_to_full self-deleting...\n");
    LiveTask = false;  
    vTaskDelete(NULL);
}

void task_move_piston_to_length(void)
{
    LiveTask = true;
    assert(piston.rtos.rate != 0);
    float length = 0.0f;

    uint8_t wait_settle = 0;
    while (module_pis_trv_status() == 1 && wait_settle < 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        pis_delay_ms(500);
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
    }

    PIS_move_to_length(piston.setpoint_l);
    // Allow slave motor driver to ramp PWM and update travel status register
    pis_delay_ms(1000);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    const uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0)
        {
            pis_delay_ms(100);
            if (module_pis_get_length_safe(&length))
            {
                taskENTER_CRITICAL();
                piston.length = length;
                taskEXIT_CRITICAL();
                stall_count = 0;

                if (fabs(piston.length - piston.setpoint_l) <= PISTON_LENGTH_DIFF_MAX)
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
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting (target missed)\n");
                        PIS_Reset();
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Re-setting length = %0.5f\n", piston.setpoint_l);
                        PIS_move_to_length(piston.setpoint_l);
                        pis_delay_ms(5000 / piston.rtos.rate);
                        count_reset = 0;
                        reset++;

                        if (reset > 2)
                        {
                            ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                            pistonRun = false;
                        }
                    }
                }
            }
        }
        else
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);
            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
            }
        }

        if (pistonRun)
        {
            pis_delay_ms(5000 / piston.rtos.rate);
        }
    }

    pis_delay_ms(100);
    if (module_pis_get_length_safe(&length))
    {
        taskENTER_CRITICAL();
        piston.length = length;
        taskEXIT_CRITICAL();
    }
    else
    {
        length = piston.length;
    }

    if (fabs(length - piston.setpoint_l) >= PISTON_LENGTH_DIFF_MAX)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Length diff = %0.5f\n", (length - piston.setpoint_l));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS, Length = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
                             length, (length - piston.setpoint_l), PISTON_LENGTH_DIFF_MAX);
    }

    pis_delay_ms(100);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_length self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_move_piston_to_volume(void)
{
    LiveTask = true;
    assert(piston.rtos.rate != 0);
    float volume = 0.0f;

    uint8_t wait_settle = 0;
    while (module_pis_trv_status() == 1 && wait_settle < 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        pis_delay_ms(500);
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
    }

    PIS_move_to_volume(piston.setpoint_v);
    // Allow slave motor driver to ramp PWM and update travel status register
    pis_delay_ms(1000);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    const uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0)
        {
            pis_delay_ms(100);
            if (module_pis_get_volume_safe(&volume))
            {
                taskENTER_CRITICAL();
                piston.volume = volume;
                taskEXIT_CRITICAL();
                stall_count = 0;

                if (fabs(piston.volume - piston.setpoint_v) <= PISTON_VOLUME_DIFF_MAX)
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
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting (volume missed)\n");
                        PIS_Reset();
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Setting volume = %0.5f\n", piston.setpoint_v);
                        PIS_move_to_volume(piston.setpoint_v);
                        pis_delay_ms(5000 / piston.rtos.rate);
                        count_reset = 0;
                        reset++;

                        if (reset > 2)
                        {
                            ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                            pistonRun = false;
                        }
                    }
                }
            }
        }
        else
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting\n");
                PIS_Reset();
                pistonRun = false;
                stall_count = 0;
            }
        }

        if (pistonRun)
        {
            pis_delay_ms(5000 / piston.rtos.rate);
        }
    }

    pis_delay_ms(50);
    if (module_pis_get_volume_safe(&volume))
    {
        taskENTER_CRITICAL();
        piston.volume = volume;
        taskEXIT_CRITICAL();
    }
    else
    {
        volume = piston.volume;
    }

    if (fabs(volume - piston.setpoint_v) >= PISTON_VOLUME_DIFF_MAX)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Volume diff = %0.3f\n", (volume - piston.setpoint_v));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS. Volume = %0.3f, diff = %0.3f, max_diff = %0.3f\n",
                             volume, (volume - piston.setpoint_v), PISTON_VOLUME_DIFF_MAX);
    }

    pis_delay_ms(50);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_volume self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

bool PIS_Get_Volume(float *volume)
{
    if (volume == NULL) return false;
    taskENTER_CRITICAL();
    *volume = piston.volume;
    taskEXIT_CRITICAL();
    return true;
}

bool PIS_Get_Length(float *length)
{
    if (length == NULL) return false;
    taskENTER_CRITICAL();
    *length = piston.length;
    taskEXIT_CRITICAL();
    return true;
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
    if (volume > 0 && volume < 1000)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Volume, set-point = %0.4f\n", volume);
        piston.setpoint_v = volume;
        return true;
    }
    return false;
}

bool PIS_set_length(float length)
{
    if (length >= 0 && length < 12.0f)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Length, set-point = %0.4f\n", length);
        piston.setpoint_l = length;
        return true;
    }
    return false;
}

void PIS_extend(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x01, 0x01, 0x00, 0x01};

    if (pis_unlock_write_mode())
    {
        artemis_piston_i2c_send_msg(cmd, 5, true);
        pis_delay_ms(150);
        artemis_piston_set_write_mode(false);
        pis_delay_ms(50);
    }
}

void PIS_retract(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_DIR;
    uint8_t cmd[5] = {addr, 0xFF, 0x01, 0x00, 0x01};

    if (pis_unlock_write_mode())
    {
        artemis_piston_i2c_send_msg(cmd, 5, true);
        pis_delay_ms(150);
        artemis_piston_set_write_mode(false);
        pis_delay_ms(50);
    }
}

void PIS_stop(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x00, 0x01, 0x00, 0x01};

    if (pis_unlock_write_mode())
    {
        artemis_piston_i2c_send_msg(cmd, 5, true);
        pis_delay_ms(150);
        artemis_piston_set_write_mode(false);
        pis_delay_ms(50);
    }
}

bool PIS_calibration_check(void)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t data = 0;

    artemis_piston_i2c_read(addr, &data, 1);
    return (data == 0x01);
}

void PIS_calibration(bool cal)
{
    if (!cal) return;

    if (PIS_calibration_check())
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Already calibrated\n");
        return;
    }

    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t cmd[2] = {addr, 0x01};

    ARTEMIS_DEBUG_PRINTF("PISTON :: Being calibrated\n");
    if (pis_unlock_write_mode())
    {
        artemis_piston_i2c_send_msg(cmd, 2, true);
        pis_delay_ms(150);
        artemis_piston_set_write_mode(false);
        pis_delay_ms(50);
    }
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

    // 1. Confirm write mode unlock on secondary MCU with readback
    if (!pis_unlock_write_mode())
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Aborting move_to_length: write mode unlock failed\n");
        return false;
    }

    // 2. Transmit the 5-byte payload with full STOP
    artemis_piston_i2c_send_msg(v, 5, true);
    // Allow slave to finish CTRL_Write() and launch motor routine
    pis_delay_ms(150);

    // 3. Relock write mode
    artemis_piston_set_write_mode(false);
    pis_delay_ms(50);

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

    // 1. Confirm write mode unlock on secondary MCU with readback
    if (!pis_unlock_write_mode())
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Aborting move_to_volume: write mode unlock failed\n");
        return false;
    }

    // 2. Transmit the 5-byte payload with full STOP
    artemis_piston_i2c_send_msg(v, 5, true);
    // Allow slave to finish CTRL_Write() and launch motor routine
    pis_delay_ms(150);

    // 3. Relock write mode
    artemis_piston_set_write_mode(false);
    pis_delay_ms(50);

    return true;
}

static int8_t module_pis_status_zero(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_ZERO;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);
    if (data == 0xFF)
    {
        return -1;
    }
    return (data != 0) ? 1 : 0;
}

static int8_t module_pis_status_full(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FULL;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);
    if (data == 0xFF)
    {
        return -1;
    }
    return (data != 0) ? 1 : 0;
}

static int8_t module_pis_status_fullreset(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FRST;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);
    if (data == 0xFF)
    {
        return -1;
    }
    return (data != 0) ? 1 : 0;
}

static int8_t module_pis_trv_status(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_ENG;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);
    if (data == 0xFF)
    {
        return -1;
    }
    return (data != 0) ? 1 : 0;
}

static bool module_pis_get_length_safe(float *out_length)
{
    uint8_t addr = PISTON_I2C_R_LENGTH_TOTAL;
    uint8_t data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    artemis_piston_i2c_read(addr, data, 4);

    if (data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0xFF)
    {
        return false;
    }

    union {
        float fLength;
        uint32_t u32Length;
    } pis;

    pis.u32Length = ((uint32_t)data[0]) |
                    (((uint32_t)data[1]) << 8) |
                    (((uint32_t)data[2]) << 16) |
                    (((uint32_t)data[3]) << 24);

    *out_length = pis.fLength;
    return true;
}

static float module_pis_get_length(void)
{
    float len = 0.0f;
    if (module_pis_get_length_safe(&len))
    {
        return len;
    }
    return piston.length;
}

static bool module_pis_get_volume_safe(float *out_volume)
{
    uint8_t addr = PISTON_I2C_R_VOLUME_TOTAL;
    uint8_t data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    artemis_piston_i2c_read(addr, data, 4);

    if (data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0xFF)
    {
        return false;
    }

    union {
        float fVolume;
        uint32_t u32Volume;
    } pis;

    pis.u32Volume = ((uint32_t)data[0]) |
                    (((uint32_t)data[1]) << 8) |
                    (((uint32_t)data[2]) << 16) |
                    (((uint32_t)data[3]) << 24);

    *out_volume = pis.fVolume;
    return true;
}

static float module_pis_get_volume(void)
{
    float vol = 0.0f;
    if (module_pis_get_volume_safe(&vol))
    {
        return vol;
    }
    return piston.volume;
}

void PIS_Reset(void)
{
    uint8_t addr = PISTON_I2C_W_RESET;
    uint8_t cmd[2] = {addr, PISTON_I2C_W_RESET_KEY};

    artemis_piston_set_write_mode(true);
    pis_delay_ms(50);

    artemis_piston_i2c_send_msg(cmd, 2, true);

    // 3500ms allows slave MCU to fully boot, initialize DRV8874, and re-enable I2C
    pis_delay_ms(3500);

    // Relock write mode after slave reboot
    artemis_piston_set_write_mode(false);
    pis_delay_ms(50);
}

void PIS_move_to_full(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_FULL;
    uint8_t cmd[2] = {addr, 0x01};

    if (pis_unlock_write_mode())
    {
        artemis_piston_i2c_send_msg(cmd, 2, true);
        pis_delay_ms(150);
        artemis_piston_set_write_mode(false);
        pis_delay_ms(50);
    }
}

void PIS_move_to_zero(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_ZERO;
    uint8_t cmd[2] = {addr, 0x01};

    if (pis_unlock_write_mode())
    {
        artemis_piston_i2c_send_msg(cmd, 2, true);
        pis_delay_ms(150);
        artemis_piston_set_write_mode(false);
        pis_delay_ms(50);
    }
}

void PIS_reset_to_full(void)
{
    uint8_t addr = PISTON_I2C_RW_RST_FULL;
    uint8_t cmd[2] = {addr, 0x01};

    if (pis_unlock_write_mode())
    {
        artemis_piston_i2c_send_msg(cmd, 2, true);
        pis_delay_ms(150);
        artemis_piston_set_write_mode(false);
        pis_delay_ms(50);
    }
}