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
    /** Initialize the I2C Port */
    taskENTER_CRITICAL();
    piston.rtos.rate = 1; // Guarantee safe non-zero divisor
    taskEXIT_CRITICAL();

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
    /* wait for 50ms */
    am_hal_systick_delay_us(50000);

    /* fetch build year, firmware major, minor and build */
    addr = PISTON_I2C_R_YEAR_BUILD;
    memset(data, 0, 8);
    artemis_piston_i2c_read(addr, data, 8);
    uint16_t year_build = (data[1] << 8) | data[0];
    uint8_t maj = data[2];
    uint8_t min = data[3];
    uint32_t build = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];

    ARTEMIS_DEBUG_PRINTF("Firmware build year\t: %u\n", year_build);
    ARTEMIS_DEBUG_PRINTF("Firmware version   \t: v%u.%u.%u\n\n", maj, min, build);
    am_hal_systick_delay_us(50000);
}

void PIS_task_move_length(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_length,
                             "Piston_Task_move_length", 512, NULL,
                             tskIDLE_PRIORITY + 4UL,
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

void PIS_task_delete(void)
{
    pistonRun = false;
    vTaskDelay(xDelay100ms);
}

void task_move_piston_to_zero(void)
{
    LiveTask = true;
    uint32_t rate = (piston.rtos.rate > 0) ? piston.rtos.rate : 1;
    uint32_t period = (uint32_t)(xDelay5000ms / rate);
    float length = 0.0f;

    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();
        vTaskDelay(xDelay1000ms);
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
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);

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
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
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

    /* get the length and update it */
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

    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();
        vTaskDelay(xDelay1000ms);
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
                    vTaskDelay(xDelay1000ms);

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
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
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

    /* get the length and update it */
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

    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();
        vTaskDelay(xDelay1000ms);
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
                    vTaskDelay(xDelay1000ms);

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
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
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

    /* get the length and update it */
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

    /* Before sending write cmd, verify if piston is already moving */
    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();
        vTaskDelay(xDelay1000ms);
    }

    /** Start the move */
    PIS_move_to_length(piston.setpoint_l);
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
            vTaskDelay(xDelay250ms);
            length = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length = %0.5f\n", length);

            taskENTER_CRITICAL();
            piston.length = length;
            taskEXIT_CRITICAL();
            stall_count = 0;

            if (piston.length >= (piston.setpoint_l - PISTON_LENGTH_DIFF_MAX) &&
                piston.length <= (piston.setpoint_l + PISTON_LENGTH_DIFF_MAX))
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
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);

                    ARTEMIS_DEBUG_PRINTF("PISTON :: Setting length = %0.5f\n", piston.setpoint_l);
                    PIS_move_to_length(piston.setpoint_l);
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
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
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

    vTaskDelay(xDelay500ms);
    length = module_pis_get_length();
    ARTEMIS_DEBUG_PRINTF("PISTON :: Length updated = %0.5f\n", length);
    taskENTER_CRITICAL();
    piston.length = length;
    taskEXIT_CRITICAL();

    if (fabsf(length - piston.setpoint_l) >= PISTON_LENGTH_DIFF_MAX)
    {
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
    uint32_t period = (uint32_t)(xDelay5000ms / rate);
    float volume = 0.0f;

    /* Before sending write cmd, verify if piston is already moving */
    if (module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();
        vTaskDelay(xDelay1000ms);
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
        if (module_pis_trv_eng() == false)
        {
            vTaskDelay(xDelay100ms);
            volume = module_pis_get_volume();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Volume = %.3fin³\n", volume);

            taskENTER_CRITICAL();
            piston.volume = volume;
            taskEXIT_CRITICAL();
            stall_count = 0;

            if (piston.volume >= (piston.setpoint_v - PISTON_VOLUME_DIFF_MAX) &&
                piston.volume <= (piston.setpoint_v + PISTON_VOLUME_DIFF_MAX))
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
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                    PIS_Reset();
                    vTaskDelay(xDelay1000ms);

                    ARTEMIS_DEBUG_PRINTF("PISTON :: Setting volume = %0.5f\n", piston.setpoint_v);
                    PIS_move_to_volume(piston.setpoint_v);
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
            stall_count++;
            ARTEMIS_DEBUG_PRINTF("PISTON :: Stall Count = %d\n", stall_count);

            if (stall_count > stall_count_max) 
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                PIS_Reset();
                vTaskDelay(xDelay1000ms);
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

    vTaskDelay(xDelay50ms);
    volume = module_pis_get_volume();
    ARTEMIS_DEBUG_PRINTF("PISTON :: Volume updated = %.3fin³\n", volume);
    taskENTER_CRITICAL();
    piston.volume = volume;
    taskEXIT_CRITICAL();

    if (fabsf(volume - piston.setpoint_v) >= PISTON_VOLUME_DIFF_MAX)
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

    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);
}

void PIS_retract(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_DIR;
    uint8_t cmd[5] = {addr, 0xFF, 0x01, 0x00, 0x01};

    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);
}

void PIS_stop(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x00, 0x01, 0x00, 0x01};

    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);
}

bool PIS_calibration_check(void)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t data = 0;

    artemis_piston_i2c_read(addr, &data, 1);