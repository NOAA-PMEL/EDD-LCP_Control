/**
 * @file piston.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-14
 * 
 * 
 */
#include "piston.h"
#include "artemis_piston.h"
#include "artemis_debug.h"
#include "math.h"
#include "config.h"

//*****************************************************************************
//
// Static variables
//
//*****************************************************************************
static Piston_t piston;

//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
static bool module_pis_read_if_full(void);
static bool module_pis_read_if_zero(void);
static bool module_pis_trv_eng(void);
static float module_pis_get_volume(void);
static float module_pis_get_length(void);
static void module_pis_information(void);

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
bool PIS_initialize(void)
{
    /** Initialize the I2C Port */
    artemis_piston_i2c_initialize(PISTON_I2C_ADDR);

    /** Power On */
    //artemis_piston_i2c_power_on();

    /** Update state */
    //module_pis_update_state();
    module_pis_information();

    /* create a semaphore */
    piston.rtos.semaphore = xSemaphoreCreateMutex();
    bool success = false;

    if (piston.rtos.semaphore == NULL)
    {
        ARTEMIS_DEBUG_PRINTF("ERROR :: Piston Semaphore is NULL \n\n");
        success = false;
    }
    else
    {
        success = true;
    }

    return success;
}

static void module_pis_information(void)
{
    // fetch system_id
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

    // fetch build year, firmware major, minor and build
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
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_full(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_full,
                                "Piston_Task_move_full", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void PIS_task_move_zero(TaskHandle_t *xPiston)
{
    configASSERT(xTaskCreate((TaskFunction_t) task_move_piston_to_zero,
                                "Piston_Task_move_zero", 256, NULL,
                                tskIDLE_PRIORITY + 3UL,
                                xPiston) == pdPASS );
}

void task_move_piston_to_zero(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = pdMS_TO_TICKS(1000UL)/piston.rtos.rate;

    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON ERROR:: Piston is moving already\n");
        vTaskDelete(NULL);
        return;
    }

    /** Start Piston Retract to zero length */
    PIS_move_to_zero();
    vTaskDelay(period);

    /** Start the task of reading until we hit the end stop */
    bool fullFlag = false;

    // Initialise the xLastWakeTime variable with the current time.
    //TickType_t xLastWakeTime;
    //xLastWakeTime = xTaskGetTickCount();
    uint8_t count_reset = 0;

    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done */
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        {
            if(module_pis_trv_eng() == false)
            {
                count_reset++;
                vTaskDelay(period);
                fullFlag = module_pis_read_if_zero();

                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("Piston Board Resetting ...\n");
                    vTaskDelay(period);
                    PIS_Reset();
                    vTaskDelay(period * 2);
                    PIS_move_to_zero();
                    vTaskDelay(period);
                    count_reset = 0;
                }
            }
            else
            {
                vTaskDelay(period);
                fullFlag = module_pis_read_if_zero();
                //vTaskDelay(100/portTICK_PERIOD_MS);
                //ARTEMIS_DEBUG_PRINTF("Piston not zero yet\n");
            }
            xSemaphoreGive(piston.rtos.semaphore);
        }
        vTaskDelay(period);
        //vTaskDelayUntil(&xLastWakeTime, period);
    }

    ARTEMIS_DEBUG_PRINTF("Piston is at zero length\n");
    vTaskDelete(NULL);
}

void task_move_piston_to_full(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = pdMS_TO_TICKS(1000UL)/piston.rtos.rate;

    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON ERROR:: Piston is moving already\n");
        vTaskDelete(NULL);
        return;
    }

    /** Start Piston Extend to full length */
    PIS_move_to_full();
    vTaskDelay(period);

    /** Start the task of reading until we hit the end stop */
    bool fullFlag = false;

    /* Initialise the xLastWakeTime variable with the current time*/
    //TickType_t xLastWakeTime;
    //xLastWakeTime = xTaskGetTickCount();
    uint8_t count_reset = 0;

    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done */
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        {
            if(module_pis_trv_eng() == false)
            {
                count_reset++;
                vTaskDelay(period);
                fullFlag = module_pis_read_if_full();

                if (count_reset > 4)
                {
                    ARTEMIS_DEBUG_PRINTF("Piston Board Reseting ...\n");
                    vTaskDelay(period);
                    PIS_Reset();
                    vTaskDelay(period * 2);
                    PIS_move_to_full();
                    vTaskDelay(period);
                    count_reset = 0;
                }
            }
            else
            {
                vTaskDelay(period);
                fullFlag = module_pis_read_if_full();
                //vTaskDelay(100/portTICK_PERIOD_MS);
                ARTEMIS_DEBUG_PRINTF("Piston not full yet\n");
            }
            xSemaphoreGive(piston.rtos.semaphore);
        }
        vTaskDelay(period);
        //vTaskDelayUntil(&xLastWakeTime, period);
    }
    ARTEMIS_DEBUG_PRINTF("Piston is at full length\n");
    vTaskDelete(NULL);
}

void task_move_piston_to_length(void)
{
    assert(piston.rtos.rate != 0);

    uint32_t period = pdMS_TO_TICKS(1000UL)/piston.rtos.rate;

    /*  before sending any write cmd to piston, read first
        if piston is already moving */
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON ERROR:: Piston is moving already\n");
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(period);

    /** Start the move */
    PIS_move_to_length(piston.setpoint_l);
    /** wait 500ms to shoot an I2C command */
    vTaskDelay(period);
    //TickType_t xLastWakeTime;
    //xLastWakeTime = xTaskGetTickCount();

    /** Start reading until we hit the volume */
    bool fullFlag = false;
    uint8_t count_reset = 0;
    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done moving and at volume */
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        {
            if(module_pis_trv_eng() == false)
            {
                count_reset++;
                vTaskDelay(period);
                piston.length = module_pis_get_length();
                //ARTEMIS_DEBUG_PRINTF("Piston length %0.5f\n", piston.length);

                if (piston.length >=(piston.setpoint_l-0.09) && piston.length <=(piston.setpoint_l+0.09))
                {
                    ARTEMIS_DEBUG_PRINTF("Piston length approximately reached\n");
                    fullFlag = true;
                    count_reset = 0;
                }
                else
                {
                    if (count_reset > 3)
                    {
                        ARTEMIS_DEBUG_PRINTF("Piston Board Reseting ...\n");
                        vTaskDelay(period);
                        PIS_Reset();
                        vTaskDelay(period);
                        PIS_move_to_length(piston.setpoint_l);
                        vTaskDelay(period);
                        count_reset = 0;
                    }
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(500UL));
                piston.length = module_pis_get_length();
                //ARTEMIS_DEBUG_PRINTF("Piston length is moving...\n");
            }

            xSemaphoreGive(piston.rtos.semaphore);
        }
        vTaskDelay(period);
        //vTaskDelayUntil(&xLastWakeTime, period);
    }

    //vTaskDelay(period);

    ///** Check to see if length is valid , and update the last length */
    //piston.length = module_pis_get_length();
    //if( fabs(piston.length - piston.setpoint_l) >= PISTON_LENGTH_DIFF_MAX )
    //{
    //    /** ERROR - Alert the system somehow */
    //    ARTEMIS_DEBUG_PRINTF("ERROR :: Piston length diff = %0.5f\n", (piston.length - piston.setpoint_l));
    //}
    //else
    //{
    //    ARTEMIS_DEBUG_PRINTF("SUCCESS :: Piston length = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
    //                        piston.length, (piston.length - piston.setpoint_l), PISTON_LENGTH_DIFF_MAX);
    //}
    vTaskDelete(NULL);
}

bool PIS_Get_Volume(float *volume)
{
    bool retVal = false;

    taskENTER_CRITICAL();
    *volume = piston.volume;
    //ARTEMIS_DEBUG_PRINTF("Piston Volume Entered Critical\n");
    retVal = true;
    taskEXIT_CRITICAL();

    return retVal;
}

bool PIS_Get_Length(float *length)
{
    bool retVal = false;

    taskENTER_CRITICAL();
    *length = piston.length;
    //ARTEMIS_DEBUG_PRINTF("Piston Length Entered Critical\n");
    retVal = true;
    taskEXIT_CRITICAL();

    return retVal;
}

void task_move_piston_to_volume(void)
{
    assert(piston.rtos.rate != 0);
    uint32_t period = pdMS_TO_TICKS(1000UL)/piston.rtos.rate;

    /*  before sending any write cmd to piston, read first
        if piston is already moving */
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON ERROR:: Piston is moving already\n");
        vTaskDelete(NULL);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500UL));

    PIS_move_to_volume(piston.setpoint_v);
    /** wait 500ms to shoot an I2C command */
    vTaskDelay(period);

    ARTEMIS_DEBUG_PRINTF("Piston Volume is moving\n");
    //TickType_t xLastWakeTime;
    //xLastWakeTime = xTaskGetTickCount();

    /** Start reading until we hit the volume */
    bool fullFlag = false;
    uint8_t count_reset = 0;

    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done moving and at volume */
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        {

            if(module_pis_trv_eng() == false)
            {
                vTaskDelay(period);
                piston.volume = module_pis_get_volume();
                ARTEMIS_DEBUG_PRINTF("Piston volume %0.5f\n", piston.volume);
                count_reset++;

                if (piston.volume >=(piston.setpoint_v-0.1) && piston.volume <=(piston.setpoint_v+0.1))
                {
                    ARTEMIS_DEBUG_PRINTF("Piston Volume approximately reached\n");
                    fullFlag = true;
                    count_reset = 0;
                }
                else
                {
                    if (count_reset > 3)
                    {
                        ARTEMIS_DEBUG_PRINTF("Piston Board Reseting ...\n");
                        vTaskDelay(period);
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
                //vTaskDelay(period);
                vTaskDelay(pdMS_TO_TICKS(500UL));
                piston.volume = module_pis_get_volume();
            }

            xSemaphoreGive(piston.rtos.semaphore);
        }
        vTaskDelay(period);
        //vTaskDelayUntil(&xLastWakeTime, period);
    }
    vTaskDelay(period);

    ///** Check to see if volume is valid , and update the last volume*/
    //piston.volume = module_pis_get_volume();
    //if( (piston.volume - piston.setpoint_v) >= PISTON_VOLUME_DIFF_MAX )
    //{
    //    /** ERROR - Alert the system somehow */
    //    ARTEMIS_DEBUG_PRINTF("ERROR :: Piston volume diff = %0.5f\n", (piston.volume - piston.setpoint_v));
    //}
    //else
    //{
    //    ARTEMIS_DEBUG_PRINTF("SUCCESS :: Piston volume = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
    //                        piston.volume, (piston.volume - piston.setpoint_v), PISTON_VOLUME_DIFF_MAX);
    //}

    vTaskDelete(NULL);
}

void PIS_set_piston_rate(uint8_t rate)
{
    if (rate > 0 && rate <= 10)
    {
        ARTEMIS_DEBUG_PRINTF("Setting Piston rate = %u\n", rate);
        piston.rtos.rate = rate;
    }
}

bool PIS_set_volume(float volume)
{
    bool retVal = false;
    if( (volume > 0) && (volume < 100000) )
    {
        ARTEMIS_DEBUG_PRINTF("Volume set-point = %0.4f\n", volume);
        piston.setpoint_v = volume;
        retVal = true;
    }
    return retVal;
}

bool PIS_set_length(float length)
{
    bool retVal = false;
    if( (length > 0.0) && (length < 12.0) )
    {
        ARTEMIS_DEBUG_PRINTF("Length set-point = %0.4f\n", length);
        piston.setpoint_l = length;
        retVal = true;
    }
    return retVal;
}

void PIS_extend(void)
{
    //bool retVal = false;
    //uint8_t addr = PISTON_I2C_RW_TRV_DIR;
    uint8_t addr = PISTON_I2C_RW_TRV_USER_OR;
    //uint8_t cmd[4] = {0x01, 0x01, 0x00, 0x01};
    uint8_t cmd[5] = {addr, 0x01, 0x01, 0x00, 0x01};

    //artemis_piston_i2c_send_msg(&addr, 1, false);
    //artemis_piston_i2c_send_msg(cmd, 4, true);

    /** Put in write mode */
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);
    //artemis_piston_set_write_mode(false);
    //return retVal;
}

void PIS_retract(void)
{
    //bool retVal = false;
    // self._write(0x60, [0xFF, 0x01, 0x00, 0x01])

    uint8_t addr = PISTON_I2C_RW_TRV_DIR;
    //uint8_t cmd[4] = {0xFF, 0x01, 0x00, 0x01};
    uint8_t cmd[5] = {addr, 0xFF, 0x01, 0x00, 0x01};

    //artemis_piston_i2c_send_msg(&addr, 1, false);
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 5, true);

    //return retVal;
}

void PIS_stop(void)
{
    //bool retVal = false;
    uint8_t addr = PISTON_I2C_RW_TRV_ENG;
    uint8_t cmd[3] = {addr, 0x00};
    //uint8_t cmd = 0x00;

    artemis_piston_set_write_mode(true);

    //artemis_piston_i2c_send_msg(&addr, 1, false);
    artemis_piston_i2c_send_msg(cmd, 2, true);

    //artemis_piston_i2c_send_msg(&addr, 1, false);
    //artemis_piston_set_write_mode(true);
    //artemis_piston_i2c_send_msg(cmd, 3, true);
    //return retVal;
}

bool PIS_calibration_check(void)
{
    uint8_t addr = PISTON_I2C_RW_PST_CAL;
    uint8_t data = 0;

    artemis_piston_i2c_read(addr, &data, 1);
    //ARTEMIS_DEBUG_PRINTF("\n0x%02X\n", data);

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
        ARTEMIS_DEBUG_PRINTF("Piston is already calibrated\n");
        return;
    }
    ARTEMIS_DEBUG_PRINTF("Piston is being calibrated\n");

    am_hal_systick_delay_us(10000);
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

#ifdef DEBUG
    ARTEMIS_DEBUG_PRINTF("\n");
    ARTEMIS_DEBUG_PRINTF("Length Bytes = ");
    for (uint8_t i=0; i<4; i++){
        ARTEMIS_DEBUG_PRINTF("0x%02X ", v[i+1]);
    }
    ARTEMIS_DEBUG_PRINTF("\n\n");
#endif

    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(v, 5, true);
    //artemis_piston_i2c_send_msg(&addr, 1, false);
    //artemis_piston_i2c_send_msg(v, 8, true);
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
        //v[i+1] = piston.u64Volume >> ((3-i)*8) & 0xFF;
        v[i+1] = piston.u64Volume >> ((i)*8) & 0xFF;
    }
    //v[5] = 0x00;

#ifdef DEBUG
    ARTEMIS_DEBUG_PRINTF("\n");
    ARTEMIS_DEBUG_PRINTF("Volume Bytes = ");
    for (uint8_t i=0; i<4; i++){
        ARTEMIS_DEBUG_PRINTF("0x%02X ", v[i+1]);
    }
    ARTEMIS_DEBUG_PRINTF("\n");
#endif

    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(v, 5, true);
    //artemis_piston_i2c_send_msg(&addr, 1, false);
    //artemis_piston_i2c_send_msg(v, 8, true);

    return true;
}


static bool module_pis_read_if_full(void)
{
    uint8_t addr = PISTON_I2C_R_TRV_FULL;
    uint8_t data;

    
    /** Send the message to read TRV_FULL address */
    artemis_piston_i2c_read(addr, &data, 1);
    
    return data;
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

#ifdef DEBUG
    ARTEMIS_DEBUG_PRINTF("module_pis_trv_eng data = 0x%02X\n", data);
#endif
    return (bool) data;
}

static float module_pis_get_length(void)
{
    uint8_t addr = PISTON_I2C_R_LENGTH_TOTAL;
    uint8_t data[4] = {0};

    artemis_piston_i2c_read(addr, data, 4);

#ifdef DEBUG
    for (uint8_t i=0; i<4; i++){
        ARTEMIS_DEBUG_PRINTF("0x%02X ", data[i]);
    }
    ARTEMIS_DEBUG_PRINTF("\n");
#endif

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

#ifdef DEBUG
    for (uint8_t i=0; i<4; i++){
        ARTEMIS_DEBUG_PRINTF("0x%02X ", data[i]);
    }
    ARTEMIS_DEBUG_PRINTF("\n");
#endif

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
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

void PIS_move_to_full(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_FULL;
    uint8_t cmd[2] = {addr, 0x01};
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

void PIS_move_to_zero(void)
{
    uint8_t addr = PISTON_I2C_RW_MOV_ZERO;
    uint8_t cmd[2] = {addr, 0x01};
    artemis_piston_set_write_mode(true);
    artemis_piston_i2c_send_msg(cmd, 2, true);
}

