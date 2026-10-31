#include "piston.h"
#include "artemis_piston.h"
#include "artemis_debug.h"
#include "math.h"
#include "config.h"

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

bool PIS_taskStatus(void)
{
    return LiveTask;
}

// Global Functions
bool PIS_initialize(void)
{
    /** Initialize the I2C Port */
    piston.rtos.rate = 1; // Guarantee safe divisor
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

    for (uint8_t i=0; i<8; i++)
    {
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
                                "Piston_Task_move_length", 512, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_volume(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_volume,
                                "Piston_Task_move_volume", 512, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_full(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_full,
                                "Piston_Task_move_full", 512, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_reset_full(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_reset_piston_to_full,
                                "Piston_Task_reset_full", 512, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_zero(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_zero,
                                "Piston_Task_move_zero", 512, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_delete()
{
    pistonRun = false;
    vTaskDelay(xDelay100ms);
}

void task_move_piston_to_zero(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = xDelay5000ms / rate;
    float length = 0.0f;

    // Wait up to 3s for any prior motion to settle before issuing command
    uint8_t wait_settle = 0;
    while (wait_settle < 6)
    {
        int8_t trv_pre = module_pis_trv_status();
        if (trv_pre == 0)
        {
            break; // Verified stopped
        }
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
        vTaskDelay(pdMS_TO_TICKS(2500));
    }

    /** Start Piston Retract to zero length */
    PIS_move_to_zero();
    vTaskDelay(period);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0) // Motor has stopped
        {
            stall_count = 0;
            vTaskDelay(xDelay250ms);
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
                if (count_reset > 3)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting (zero limit missed)\n");

                    /* Issue robust reset */
                    PIS_Reset();

                    /* Crucial: Slave micro needs 2.5s to finish reboot & load NVM */
                    vTaskDelay(pdMS_TO_TICKS(2500));

                    /* Re-unlock write mode after slave reboot */
                    artemis_piston_set_write_mode(true);
                    vTaskDelay(pdMS_TO_TICKS(50));

                    ARTEMIS_DEBUG_PRINTF("PISTON :: Re-setting move to zero\n");
                    PIS_move_to_zero();
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                    }
                }
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, zero status read dropped\n");
            }
        }
        else // trv == 1 (Still moving)
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting slave\n");
                PIS_Reset();
                vTaskDelay(pdMS_TO_TICKS(2500));
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    if (module_pis_get_length_safe(&length))
    {
        taskENTER_CRITICAL();
        piston.length = length;
        taskEXIT_CRITICAL();
    }

    vTaskDelay(xDelay100ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_zero self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_move_piston_to_full(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = xDelay5000ms / rate;
    float length = 0.0f;

    // Wait up to 3s for any prior motion to settle before issuing command
    uint8_t wait_settle = 0;
    while (wait_settle < 6)
    {
        int8_t trv_pre = module_pis_trv_status();
        if (trv_pre == 0)
        {
            break; // Verified stopped
        }
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
        vTaskDelay(pdMS_TO_TICKS(2500));
    }

    /** Start Piston Extend to full length */
    PIS_move_to_full();
    vTaskDelay(period);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0) // Motor has stopped
        {
            stall_count = 0;
            vTaskDelay(xDelay250ms);
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
                if (count_reset > 3)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting (full limit missed)\n");

                    /* Issue robust reset */
                    PIS_Reset();

                    /* Crucial: Slave micro needs 2.5s to finish reboot & load NVM */
                    vTaskDelay(pdMS_TO_TICKS(2500));

                    /* Re-unlock write mode after slave reboot */
                    artemis_piston_set_write_mode(true);
                    vTaskDelay(pdMS_TO_TICKS(50));

                    ARTEMIS_DEBUG_PRINTF("PISTON :: Re-setting move to full\n");
                    PIS_move_to_full();
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                    }
                }
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, full status read dropped\n");
            }
        }
        else // trv == 1 (Still moving)
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting slave\n");
                PIS_Reset();
                vTaskDelay(pdMS_TO_TICKS(2500));
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    if (module_pis_get_length_safe(&length))
    {
        taskENTER_CRITICAL();
        piston.length = length;
        taskEXIT_CRITICAL();
    }

    vTaskDelay(xDelay100ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_move_piston_to_full self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_reset_piston_to_full(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = xDelay5000ms / rate;
    float length = 0.0f;

    // Wait up to 3s for any prior motion to settle before issuing command
    uint8_t wait_settle = 0;
    while (wait_settle < 6)
    {
        int8_t trv_pre = module_pis_trv_status();
        if (trv_pre == 0)
        {
            break; // Verified stopped
        }
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
        vTaskDelay(pdMS_TO_TICKS(2500));
    }

    /** Start Piston Extend to full length and reset encoder counts */
    PIS_reset_to_full();
    vTaskDelay(period);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0) // Motor has stopped
        {
            stall_count = 0;
            vTaskDelay(xDelay250ms);
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
                if (count_reset > 3)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board Resetting (full reset flag missed)\n");

                    /* Issue robust reset */
                    PIS_Reset();

                    /* Crucial: Slave micro needs 2.5s to finish reboot & load NVM */
                    vTaskDelay(pdMS_TO_TICKS(2500));

                    /* Re-unlock write mode after slave reboot */
                    artemis_piston_set_write_mode(true);
                    vTaskDelay(pdMS_TO_TICKS(50));

                    ARTEMIS_DEBUG_PRINTF("PISTON :: Re-setting reset to full\n");
                    PIS_reset_to_full();
                    vTaskDelay(period);
                    count_reset = 0;
                    reset++;

                    if (reset > 2)
                    {
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout\n");
                        pistonRun = false;
                    }
                }
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, full reset status read dropped\n");
            }
        }
        else // trv == 1 (Still moving)
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting slave\n");
                PIS_Reset();
                vTaskDelay(pdMS_TO_TICKS(2500));
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

    if (module_pis_get_length_safe(&length))
    {
        taskENTER_CRITICAL();
        piston.length = length;
        taskEXIT_CRITICAL();
    }

    vTaskDelay(xDelay100ms);
    ARTEMIS_DEBUG_PRINTF("PISTON :: task_reset_piston_to_full self-deleting...\n");
    LiveTask = false;
    vTaskDelete(NULL);
}

void task_move_piston_to_length(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = xDelay5000ms / rate;
    float length = 0.0f;

    // Wait up to 3s for any prior motion to settle before issuing a reset
    uint8_t wait_settle = 0;
    while (module_pis_trv_status() == 1 && wait_settle < 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
        // Give secondary board 2.5s to boot and reload parameters from non-volatile memory
        vTaskDelay(pdMS_TO_TICKS(2500));
    }

    // Command the move
    PIS_move_to_length(piston.setpoint_l);
    vTaskDelay(pdMS_TO_TICKS(500));

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            // Transient bus dropout: do NOT trigger a reset
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0) // Motor has stopped
        {
            vTaskDelay(xDelay250ms);

            if (module_pis_get_length_safe(&length))
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Length = %0.5f\n", length);
                taskENTER_CRITICAL();
                piston.length = length;
                taskEXIT_CRITICAL();
                stall_count = 0;

                // Check arrival within tolerance
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
                        
                        /* Issue robust reset */
                        PIS_Reset();
                        
                        /* Crucial: Slave micro needs 2.5s to finish reboot & load NVM */
                        vTaskDelay(pdMS_TO_TICKS(2500));
                        
                        /* Re-unlock write mode after slave reboot */
                        artemis_piston_set_write_mode(true);
                        vTaskDelay(pdMS_TO_TICKS(50));

                        ARTEMIS_DEBUG_PRINTF("PISTON :: Re-setting length = %0.5f\n", piston.setpoint_l);
                        PIS_move_to_length(piston.setpoint_l);
                        vTaskDelay(period);
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
        else // trv == 1 (Still moving)
        {
            stall_count++;
            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout, resetting\n");
                PIS_Reset();
                vTaskDelay(pdMS_TO_TICKS(2500));
                pistonRun = false;
            }
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

 // Final read on task exit
if (module_pis_get_length_safe(&length))
{
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();
}
else
{
    // If read dropped, use last known valid position
    length = piston.length;
}

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
    uint32_t period = xDelay5000ms / rate;
    float volume = 0.0f;

    // 1. Wait up to 3s for any prior motion to settle before issuing a reset
    uint8_t wait_settle = 0;
    while (module_pis_trv_status() == 1 && wait_settle < 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Waiting for existing motion to finish...\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_settle++;
    }

    if (wait_settle >= 6)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, motor stuck in travel, resetting slave\n");
        PIS_Reset();
        // Give secondary micro 2.5s to boot and reload parameters from EEPROM
        vTaskDelay(pdMS_TO_TICKS(2500));
    }

    vTaskDelay(xDelay50ms);
    PIS_move_to_volume(piston.setpoint_v);
    vTaskDelay(xDelay500ms);

    pistonRun = true;
    uint8_t count_reset = 0;
    uint8_t reset = 0;
    uint8_t stall_count = 0;
    uint8_t stall_count_max = 38;

    while (pistonRun)
    {
        int8_t trv = module_pis_trv_status();

        if (trv == -1)
        {
            // Transient bus dropout: do NOT trigger a reset
            ARTEMIS_DEBUG_PRINTF("PISTON :: Warning, I2C read dropped, retrying...\n");
        }
        else if (trv == 0) // Motor has stopped
        {
            vTaskDelay(xDelay100ms);

            if (module_pis_get_volume_safe(&volume))
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Volume = %.3fin³\n", volume);
                taskENTER_CRITICAL();
                piston.volume = volume;
                taskEXIT_CRITICAL();
                stall_count = 0;

                // Check arrival within volume tolerance
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
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting (volume missed: cur=%0.3f, target=%0.3f, diff=%0.3f)\n",
                                             piston.volume, piston.setpoint_v, (piston.volume - piston.setpoint_v));

                        /* 1. Issue the robust reset sequence (stops motor, unlocks 0xA5, sends reset key) */
                        PIS_Reset();

                        /* 2. Critical: Wait 2.5s for slave core reboot and EEPROM/NVM reload */
                        vTaskDelay(pdMS_TO_TICKS(2500));

                        /* 3. Re-unlock write mode now that the slave has booted clean */
                        artemis_piston_set_write_mode(true);
                        vTaskDelay(pdMS_TO_TICKS(50));

                        /* 4. Re-issue the target volume */
                        ARTEMIS_DEBUG_PRINTF("PISTON :: Re-setting volume = %0.4f\n", piston.setpoint_v);
                        PIS_move_to_volume(piston.setpoint_v);

                        /* 5. Allow motor start and initial settling before re-entering travel status checks */
                        vTaskDelay(period);

                        count_reset = 0;
                        reset++;

                        if (reset > 2)
                        {
                            ARTEMIS_DEBUG_PRINTF("PISTON :: Reset Timeout - aborting recovery\n");
                            pistonRun = false;
                        }
                    }
                }
                
            }
        }
        else // trv == 1 (Still moving)
        {
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max)
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                vTaskDelay(pdMS_TO_TICKS(2500));
                pistonRun = false;
                stall_count = 0;
                count_reset = 0;
                reset = 0;
            }
        }

        if (pistonRun)
        {
            vTaskDelay(period);
        }
    }

// Final read on exit
vTaskDelay(xDelay50ms);
if (module_pis_get_volume_safe(&volume))
{
    taskENTER_CRITICAL();
    piston.volume = volume;
    taskEXIT_CRITICAL();
}
else
{
    // If read dropped, use last known valid volume
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
    artemis_piston_i2c_send_msg(v, 5, true);
    return true;
}

/* Returns: 1 = at zero, 0 = not at zero, -1 = I2C comms error */
static int8_t module_pis_status_zero(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_ZERO;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);

    if (data == 0xFF)
    {
        return -1; // Bus drop
    }
    return (data != 0) ? 1 : 0;
}

/* Returns: 1 = at full, 0 = not at full, -1 = I2C comms error */
static int8_t module_pis_status_full(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FULL;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);

    if (data == 0xFF)
    {
        return -1; // Bus drop
    }
    return (data != 0) ? 1 : 0;
}

/* Returns: 1 = full reset complete, 0 = not complete, -1 = I2C comms error */
static int8_t module_pis_status_fullreset(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FRST;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);

    if (data == 0xFF)
    {
        return -1; // Bus drop
    }
    return (data != 0) ? 1 : 0;
}

/* Returns: 1 = moving, 0 = stopped, -1 = I2C comms error */
static int8_t module_pis_trv_status(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_ENG;
    uint8_t data = 0xFF;

    artemis_piston_i2c_read(addr, &data, 1);

    if (data == 0xFF)
    {
        return -1; // Bus dropped
    }
    return (data != 0) ? 1 : 0;
}

static bool module_pis_get_length_safe(float *out_length)
{
    uint8_t addr = PISTON_I2C_R_LENGTH_TOTAL;
    uint8_t data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    artemis_piston_i2c_read(addr, data, 4);

    // If I2C failed, data remains 0xFF
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
    // Bus read failed: return last known valid position instead of 0.0
    return piston.length;
}

static bool module_pis_get_volume_safe(float *out_volume)
{
    uint8_t addr = PISTON_I2C_R_VOLUME_TOTAL;
    uint8_t data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    artemis_piston_i2c_read(addr, data, 4);

    // If I2C dropped, data remains filled with 0xFF sentinels
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
    // Return last known valid volume on bus failure, NEVER 0.0
    return piston.volume;
}

void PIS_Reset(void)
{
    uint8_t reset_cmd[2] = {PISTON_I2C_W_RESET, PISTON_I2C_W_RESET_KEY};
    
    /* 1. Force the motor drive off so slave ISR exits any active stall loop */
    PIS_stop();
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 2. Unlock write registers */
    artemis_piston_set_write_mode(true);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Send the hardware reset key */
    artemis_piston_i2c_send_msg(reset_cmd, 2, true);

    /* 4. Hold the master for 10ms to allow slave regulator / micro to enter reset */
    am_hal_systick_delay_us(10000);
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