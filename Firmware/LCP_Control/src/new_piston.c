#include "piston.h"
#include "artemis_piston.h"
#include "artemis_debug.h"
#include "math.h"
#include "config.h"

// Static variables and semaphores
static Piston_t piston;
static SemaphoreHandle_t xPistonSemaphore = NULL;
static SemaphoreHandle_t xI2CSemaphore = NULL;
static volatile bool pistonRun = false;

// Static Function Prototypes
static bool module_pis_read_if_full(void);
static bool module_pis_read_if_fullreset(void);
static bool module_pis_read_if_zero(void);
static bool module_pis_trv_eng(void);
static float module_pis_get_volume(void);
static float module_pis_get_length(void);
static void module_pis_information(void);
static bool module_pis_write_cmd(uint8_t addr, uint8_t value);
static bool module_pis_wait_for_movement_or_timeout(uint32_t period, float desired_value, bool is_volume, uint8_t stall_count_max);

// Holds the I2C bus to prevent other tasks from using it
static bool acquire_i2c_bus(TickType_t wait_time) {
    if (xSemaphoreTake(xI2CSemaphore, wait_time) != pdTRUE) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to acquire I2C bus\n");
        return false;
    }
    return true;
}

// Releases the I2C bus
static void release_i2c_bus(void) {
    xSemaphoreGive(xI2CSemaphore);
}

// I2C read with semaphore protection
static bool pis_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    if (!acquire_i2c_bus(pdMS_TO_TICKS(500))) {
        return false;
    }
    artemis_piston_i2c_read(addr, data, len);
    release_i2c_bus();
    return true;
}

// I2C send with semaphore protection
static bool pis_i2c_send_msg(uint8_t *data, uint16_t len, bool stop) {
    if (!acquire_i2c_bus(pdMS_TO_TICKS(500))) {
        return false;
    }
    artemis_piston_i2c_send_msg(data, len, stop);
    release_i2c_bus();
    return true;
}

// Set the I2C bus to with semaphore protection
static bool pis_i2c_set_write_mode(bool state) {
    if (!acquire_i2c_bus(pdMS_TO_TICKS(500))) {
        return false;
    }
    artemis_piston_set_write_mode(state);
    release_i2c_bus();
    return true;
}

// Global Functions
bool PIS_initialize(void)
{
    // Create semaphore mutex for piston data access
    xPistonSemaphore = xSemaphoreCreateMutex();
    if (xPistonSemaphore == NULL) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, failed to create piston data mutex\n");
        return false;
    }
    
    // Create semaphore binary for I2C bus access
    xI2CSemaphore = xSemaphoreCreateBinary();
    if (xI2CSemaphore == NULL) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, failed to create I2C semaphore\n");
        vSemaphoreDelete(xPistonSemaphore); // Clean up the first semaphore
        return false;
    }
    
    // Initialize I2C semaphore as available
    xSemaphoreGive(xI2CSemaphore);
    
    // Initialize the I2C Port
    if (!acquire_i2c_bus(pdMS_TO_TICKS(1000))) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, couldn't acquire I2C bus for initialization\n");
        vSemaphoreDelete(xPistonSemaphore);
        vSemaphoreDelete(xI2CSemaphore);
        return false;
    }
    
    artemis_piston_i2c_initialize(PISTON_I2C_ADDR);
    // Release the I2C line after initialization
    release_i2c_bus();
    // Print out Piston Board info
    module_pis_information();
    
    // Set default rate
    piston.rtos.rate = 1;
    
    return true;
}

void PIS_uninitialize(void)
{
    // Uninitialize the I2C Port with semaphore protection
    if (acquire_i2c_bus(pdMS_TO_TICKS(1000))) {
        artemis_piston_i2c_uninitialize();
        release_i2c_bus();
    }
    
    // Delete semaphores
    if (xPistonSemaphore != NULL) {
        vSemaphoreDelete(xPistonSemaphore);
    }
    
    if (xI2CSemaphore != NULL) {
        vSemaphoreDelete(xI2CSemaphore);
    }
}

static void module_pis_information(void)
{
    /* fetch system_id */
    uint8_t addr = PISTON_I2C_R_SYS_ID;
    uint8_t data[8] = {0};

    ARTEMIS_DEBUG_PRINTF("\nPiston Board Information\n");
    ARTEMIS_DEBUG_PRINTF("**************************************\n");
    ARTEMIS_DEBUG_PRINTF("System Identification\t: ");

    pis_i2c_read(addr, data, 8);
    for (uint8_t i=0; i<8; i++){
        ARTEMIS_DEBUG_PRINTF("%c", (char)data[i]);
    }
    ARTEMIS_DEBUG_PRINTF("\n");
    /* wait for 50ms */
    am_hal_systick_delay_us(50000);

    /* fetch build year, firmware major, minor and build */
    addr = PISTON_I2C_R_YEAR_BUILD;
    memset(data, 0, 8);
    pis_i2c_read(addr, data, 8);
    uint16_t year_build = data[1]<<8 | data[0];
    uint8_t maj = data[2];
    uint8_t min = data[3];
    uint32_t build = data[7]<<24 | data[6]<<16 | data[5]<<8 | data[4];

    ARTEMIS_DEBUG_PRINTF("Firmware build year\t: %u\n", year_build);
    ARTEMIS_DEBUG_PRINTF("Firmware version   \t: v%u.%u.%u\n", maj, min, build);
    ARTEMIS_DEBUG_PRINTF("\n");
    am_hal_systick_delay_us(50000);
}

// Takes in an address + command, sets I2C to write-mode, checks if successful, and sends the command
static bool module_pis_write_cmd(uint8_t addr, uint8_t value)
{
    uint8_t cmd[2] = {addr, value};
    
    if (!pis_i2c_set_write_mode(true)) {
        return false;
    }
    
    return pis_i2c_send_msg(cmd, 2, true);
}

static bool module_pis_wait_for_movement_or_timeout(uint32_t period, float desired_value, bool is_volume, uint8_t stall_count_max)
{
    bool success = false;
    uint8_t count_reset = 0;
    uint8_t stall_count = 0;
    float current_value = 0.0f;
    float last_value = -1.0f;
    float diff_max = is_volume ? PISTON_VOLUME_DIFF_MAX : PISTON_LENGTH_DIFF_MAX;
    
    pistonRun = true;
    
    while(pistonRun)
    {
        // Read the piston memory to see if we're done moving
        bool is_moving = module_pis_trv_eng();
        
        // Get current value (length or volume)
        if (is_volume) {
            current_value = module_pis_get_volume();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Volume %s = %.3fin³\n", 
                is_moving ? "in moving" : "", current_value);
        } else {
            current_value = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length %s = %0.5f\n", 
                is_moving ? "in moving" : "", current_value);
        }
        
        // Update piston struct with semaphore guards
        if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (is_volume) {
                piston.volume = current_value;
            } else {
                piston.length = current_value;
            }
            xSemaphoreGive(xPistonSemaphore);
        }
        
        // Check for movement completion
        if (!is_moving)
        {
            if ((current_value >= (desired_value - diff_max)) && 
                (current_value <= (desired_value + diff_max)))
            {
                ARTEMIS_DEBUG_PRINTF("PISTON :: %s reached\n", is_volume ? "Volume" : "Length");
                pistonRun = false;
                success = true;
                break;
            }
            else
            {
                count_reset++;
                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                    vTaskDelay(xDelay500ms);
                    PIS_Reset();
                    vTaskDelay(period);
                    
                    if (is_volume) {
                        PIS_move_to_volume(desired_value);
                    } else {
                        PIS_move_to_length(desired_value);
                    }
                    
                    vTaskDelay(period);
                    count_reset = 0;
                }
            }
        }
        else  // Still moving
        {
            // Check for stall condition
            if (isnan(last_value) || isinf(last_value) 
                || isnan(current_value) || isinf(current_value) 
                || fabs(current_value - last_value) < 0.001) 
            {
                stall_count++;
                vTaskDelay(xDelay500ms);
                if (isnan(current_value) || isinf(current_value)) {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Invalid %s value detected\n", 
                        is_volume ? "volume" : "length");
                }
                ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count = %d/%d\n", stall_count, stall_count_max);
                if (stall_count > stall_count_max) {
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Stall count timeout\n");
                    ARTEMIS_DEBUG_PRINTF("PISTON :: Board resetting\n");
                    PIS_Reset();
                    vTaskDelay(period);
                    pistonRun = false;
                    break;
                }
            } else {
                stall_count = 0;
            }
            last_value = current_value;
        }
        
        // Wait before next check 
        if (pistonRun) {
            vTaskDelay(period);
        }
    }
    
    // Final check of position
    if (success) {
        vTaskDelay(xDelay100ms);
        
        if (is_volume) {
            current_value = module_pis_get_volume();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Volume updated = %.3fin³\n", current_value);
            
            if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                piston.volume = current_value;
                xSemaphoreGive(xPistonSemaphore);
            }
            
            if (fabs(current_value - desired_value) >= diff_max) {
                ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Volume diff = %0.3f\n", (current_value - desired_value));
                success = false;
            } else {
                ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS. Volume = %0.3f, diff = %0.3f, max_diff = %0.3f\n",
                                current_value, (current_value - desired_value), diff_max);
            }
        } else {
            current_value = module_pis_get_length();
            ARTEMIS_DEBUG_PRINTF("PISTON :: Length updated = %0.5f\n", current_value);
            
            if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                piston.length = current_value;
                xSemaphoreGive(xPistonSemaphore);
            }
            
            if (fabs(current_value - desired_value) >= diff_max) {
                ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, Length diff = %0.5f\n", (current_value - desired_value));
                success = false;
            } else {
                ARTEMIS_DEBUG_PRINTF("PISTON :: SUCCESS, Length = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
                                current_value, (current_value - desired_value), diff_max);
            }
        }
    }
    
    return success;
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
    bool delete_completed = false;

    // Check the task state
    while (!delete_completed && wait < 20)
    {
        eTaskState eState = eTaskGetState(xHandle);
        if ((eState == eReady) || (eState == eBlocked))
        {
            if (pistonRun)
            {
                pistonRun = false;
            }
        }
        else if (eState == eRunning)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Task is in eRunning state, wait\n");
        }
        else if (eState == eSuspended)
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Task is Suspended\n");
            vTaskDelete(xHandle);
        }
        else if ((eState == eDeleted) || (eState == eInvalid))
        {
            ARTEMIS_DEBUG_PRINTF("PISTON :: Task is Deleted\n");
            delete_completed = true;
        }
        wait++;
        // Wait for 100ms
        vTaskDelay(xDelay100ms);
    }
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

    // Start Piston Retract to zero length
    PIS_move_to_zero();
    vTaskDelay(period);

    // Start the task of reading until we hit the end stop
    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;

    while(pistonRun)
    {
        // Read the piston memory to see if we're done
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

    // Get the length and update it
    length = module_pis_get_length();
    if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        piston.length = length;
        xSemaphoreGive(xPistonSemaphore);
    }

    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
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

    // Start Piston Extend to full length
    PIS_move_to_full();
    vTaskDelay(period);

    // Start the task of reading until we hit the end stop
    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;

    while(pistonRun)
    {
        // Read the piston memory to see if we're done
        if(module_pis_trv_eng() == false)
        {
            count_reset++;
            vTaskDelay(xDelay250ms);
            fullFlag = module_pis_read_if_full();
            if (fullFlag)
            {
                pistonRun = false;
            }
            if (count_reset > 4) // about 1 second
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

    // Get the length and update it
    length = module_pis_get_length();
    if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        piston.length = length;
        xSemaphoreGive(xPistonSemaphore);
    }

    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
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

    // Start Piston Extend to full length and reset encoder counts
    PIS_reset_to_full();
    vTaskDelay(period);

    // Start the task of reading until we hit the end stop
    pistonRun = true;
    bool fullFlag = false;
    uint8_t count_reset = 0;

    while(pistonRun)
    {
        // Read the piston memory to see if we're done
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

    // Get the length and update it
    length = module_pis_get_length();
    if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        piston.length = length;
        xSemaphoreGive(xPistonSemaphore);
    }

    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
}

void task_move_piston_to_length(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = xDelay5000ms/piston.rtos.rate;
    float target_length = 0.0f;
    
    // Get target length from piston structure in a thread-safe way
    if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        target_length = piston.setpoint_l;
        xSemaphoreGive(xPistonSemaphore);
    }

    // Before sending any write cmd to piston, read if piston is already moving
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();
        vTaskDelay(xDelay2000ms);
    }
    vTaskDelay(xDelay50ms);
    
    // Start the move
    PIS_move_to_length(target_length);
    
    // Wait 250ms after shooting an I2C command
    vTaskDelay(xDelay250ms);
    
    // Wait for movement to complete
    module_pis_wait_for_movement_or_timeout(period, target_length, false, 38);
    
    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
}

bool PIS_Get_Volume(float *volume)
{
    bool retVal = false;
    
    if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        *volume = piston.volume;
        retVal = true;
        xSemaphoreGive(xPistonSemaphore);
    }
    
    return retVal;
}

bool PIS_Get_Length(float *length)
{
    bool retVal = false;
    
    if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        *length = piston.length;
        retVal = true;
        xSemaphoreGive(xPistonSemaphore);
    }
    
    return retVal;
}

void task_move_piston_to_volume(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = xDelay5000ms/piston.rtos.rate;
    float target_volume = 0.0f;
    
    // Get target volume from piston structure in a thread-safe way
    if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        target_volume = piston.setpoint_v;
        xSemaphoreGive(xPistonSemaphore);
    }

    // Before sending any write cmd to piston, read if piston is already moving
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: ERROR, moving already\n");
        PIS_Reset();
        vTaskDelay(xDelay2000ms);
    }
    vTaskDelay(xDelay50ms);
    
    // Start the move
    PIS_move_to_volume(target_volume);
    
    // Wait 500ms after shooting an I2C command
    vTaskDelay(xDelay500ms);
    
    // Wait for movement to complete
    module_pis_wait_for_movement_or_timeout(period, target_volume, true, 20);
    
    vTaskDelay(xDelay100ms);
    vTaskDelete(NULL);
}

void PIS_set_piston_rate(uint8_t rate)
{
    if (rate > 0 && rate <= 10)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Setting rate = %uHz\n", rate);
        
        if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            piston.rtos.rate = rate;
            xSemaphoreGive(xPistonSemaphore);
        }
    }
}

bool PIS_set_volume(float volume)
{
    bool retVal = false;
    if( (volume > 0) && (volume < 1000) )
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Volume, set-point = %0.4f\n", volume);
        
        if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            piston.setpoint_v = volume;
            xSemaphoreGive(xPistonSemaphore);
            retVal = true;
        }
    }
    return retVal;
}

bool PIS_set_length(float length)
{
    bool retVal = false;
    if( (length >= 0) && (length < 12.0) )
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Length, set-point = %0.4f\n", length);
        
        if (xSemaphoreTake(xPistonSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            piston.setpoint_l = length;
            xSemaphoreGive(xPistonSemaphore);
            retVal = true;
        }
    }
    return retVal;
}

void PIS_extend(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x01, 0x01, 0x00, 0x01};

    if (!pis_i2c_set_write_mode(true)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to set write mode for extend\n");
        return;
    }
    
    pis_i2c_send_msg(cmd, 5, true);
}

void PIS_retract(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_DIR;
    uint8_t cmd[5] = {addr, 0xFF, 0x01, 0x00, 0x01};

    if (!pis_i2c_set_write_mode(true)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to set write mode for retract\n");
        return;
    }
    
    pis_i2c_send_msg(cmd, 5, true);
}

void PIS_stop(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    uint8_t cmd[5] = {addr, 0x00, 0x01, 0x00, 0x01};

    if (!pis_i2c_set_write_mode(true)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to set write mode for stop\n");
        return;
    }
    
    pis_i2c_send_msg(cmd, 5, true);
}

bool PIS_calibration_check(void)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t data = 0;

    if (!pis_i2c_read(addr, &data, 1)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to read calibration status\n");
        return false;
    }

    return (data == 0x01);
}

void PIS_calibration(bool cal)
{
    if (cal == false) {
        return;
    }

    // Check if piston is already calibrated
    if(PIS_calibration_check() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Already calibrated\n");
        return;
    }
    ARTEMIS_DEBUG_PRINTF("PISTON :: Being calibrated\n");

    am_hal_systick_delay_us(100000);
    module_pis_write_cmd(PISTON_I2C_RW_PST_CAL, 0x01);
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

    union{
        float fLength;
        uint32_t u32Length;
    }piston_data;

    piston_data.fLength = length;

    // Send command to move to length
    uint8_t v[5] = {0};
    v[0] = addr;

    for(uint8_t i=0; i<4; i++)
    {
        v[i+1] = piston_data.u32Length >> ((i)*8) & 0xFF;
    }

    if (!pis_i2c_set_write_mode(true)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to set write mode for move to length\n");
        return false;
    }
    
    return pis_i2c_send_msg(v, 5, true);
}

bool PIS_move_to_volume(float volume)
{
    uint8_t addr = PISTON_I2C_W_SET_VOLUME;

    union{
        float dVolume;
        uint32_t u64Volume;
    }piston_data;

    piston_data.dVolume = volume;
    
    // Send command to move to volume
    uint8_t v[5] = {0};
    v[0] = addr;

    for(uint8_t i=0; i<4; i++)
    {
        v[i+1] = piston_data.u64Volume >> ((i)*8) & 0xFF;
    }

    if (!pis_i2c_set_write_mode(true)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to set write mode for move to volume\n");
        return false;
    }
    
    return pis_i2c_send_msg(v, 5, true);
}

static bool module_pis_read_if_full(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FULL;
    uint8_t data = 0;

    // Send the message to read TRV_FULL address
    if (!pis_i2c_read(addr, &data, 1)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to read TRV_FULL status\n");
        return false;
    }
    
    return (bool) data;
}

static bool module_pis_read_if_fullreset(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FRST;
    uint8_t data = 0;

    // Send the message to read TRV_FRST address
    if (!pis_i2c_read(addr, &data, 1)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to read TRV_FRST status\n");
        return false;
    }
    
    return (bool) data;
}

static bool module_pis_read_if_zero(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_ZERO;
    uint8_t data = 0;

    // Send the message to read TRV_ZERO address
    if (!pis_i2c_read(addr, &data, 1)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to read TRV_ZERO status\n");
        return false;
    }
    
    return (bool) data;
}

static bool module_pis_trv_eng(void)
{
    uint8_t addr = PISTON_I2C_RW_TRV_ENG;
    uint8_t data = 0;
    
    if (!pis_i2c_read(addr, &data, 1)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to read TRV_ENG status\n");
        return false;
    }
    
    return (bool) data;
}

static float module_pis_get_length(void)
{
    uint8_t addr = PISTON_I2C_R_LENGTH_TOTAL;
    uint8_t data[4] = {0};
    
    if (!pis_i2c_read(addr, data, 4)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to read length\n");
        return 0.0f;
    }

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

    if (!pis_i2c_read(addr, data, 4)) {
        ARTEMIS_DEBUG_PRINTF("PISTON :: Failed to read volume\n");
        return 0.0f;
    }

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
    module_pis_write_cmd(PISTON_I2C_W_RESET, PISTON_I2C_W_RESET_KEY);
}

void PIS_move_to_full(void)
{
    module_pis_write_cmd(PISTON_I2C_RW_MOV_FULL, 0x01);
}

void PIS_move_to_zero(void)
{
    module_pis_write_cmd(PISTON_I2C_RW_MOV_ZERO, 0x01);
}

void PIS_reset_to_full(void)
{
    module_pis_write_cmd(PISTON_I2C_RW_RST_FULL, 0x01);
}