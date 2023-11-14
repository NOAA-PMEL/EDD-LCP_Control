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

#define NDEBUG

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
    artemis_piston_i2c_power_on();

    /** Update state */
    //module_pis_update_state();
    module_pis_information();

    return true;
}

static void module_pis_information(void)
{
    // fetch system_id
    uint8_t addr = PISTON_I2C_R_SYS_ID;
    uint8_t data[8] = {0};

    ARTEMIS_DEBUG_PRINTF("\nPiston Board Information\n");
    ARTEMIS_DEBUG_PRINTF("*****************************\n");
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

void task_move_piston_to_zero(void)
{

    TickType_t xLastWakeTime;
    uint16_t period = 1000/piston.rtos.rate;
    period /= portTICK_PERIOD_MS;
    piston.rtos.semaphore = xSemaphoreCreateMutex();

    /** Start Piston Retract to zero length */
    PIS_move_to_zero();
    vTaskDelay(500/ portTICK_PERIOD_MS);

    /** Start the task of reading until we hit the end stop */
    bool fullFlag = false;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done */
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        {
            fullFlag = module_pis_read_if_zero();
            ARTEMIS_DEBUG_PRINTF("Piston not zero yet\n");
            xSemaphoreGive(piston.rtos.semaphore);
        }
        vTaskDelayUntil( &xLastWakeTime, period );
    }

    ARTEMIS_DEBUG_PRINTF("Piston is at zero length\n");
    vTaskDelete(NULL);
}

void task_move_piston_to_full(void)
{
    TickType_t xLastWakeTime;
    uint16_t period = 1000/piston.rtos.rate;
    period /= portTICK_PERIOD_MS;
    piston.rtos.semaphore = xSemaphoreCreateMutex();

    /** Start Piston Extend to full length */
    PIS_move_to_full();
    vTaskDelay(500/ portTICK_PERIOD_MS);

    /** Start the task of reading until we hit the end stop */
    bool fullFlag = false;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done */
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        {
            fullFlag = module_pis_read_if_full();
            //ARTEMIS_DEBUG_PRINTF("Piston not full yet\n");
            xSemaphoreGive(piston.rtos.semaphore);
        }

        vTaskDelayUntil(&xLastWakeTime, period);
        //ARTEMIS_DEBUG_PRINTF("Piston Task is being suspended\n");
        //vTaskSuspend(NULL);
    }
    ARTEMIS_DEBUG_PRINTF("Piston is at full length\n");
    vTaskDelete(NULL);
}

void task_move_piston_to_length(void)
{
    TickType_t xLastWakeTime;
    uint16_t period = 1000/piston.rtos.rate;
    period /= portTICK_PERIOD_MS;
    piston.rtos.semaphore = xSemaphoreCreateMutex();

    /*  before sending any write cmd to piston, read first
        if piston is already moving */
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON ERROR:: Piston is moving already\n");
        vTaskDelete(NULL);
        return;
    }
    vTaskDelay(100/ portTICK_PERIOD_MS);

    /** Start the move */
    PIS_move_to_length(piston.setpoint);
    /** wait 500ms to shoot an I2C command */
    vTaskDelay(500/portTICK_PERIOD_MS);

    xLastWakeTime = xTaskGetTickCount();

    /** Start reading until we hit the volume */
    bool fullFlag = false;
    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done moving and at volume */
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        {
            //float len = PIS_get_length();
            //if(len>=piston.setpoint && len <=piston.setpoint +0.09)
            //{
            //    ARTEMIS_DEBUG_PRINTF("Piston Length has reached to %0.5f\n", len);
            //    fullFlag = true;
            //}
            //else
            //{
            //    ARTEMIS_DEBUG_PRINTF("Piston Length is moving to %0.5f\n", len);
            //}
            if(module_pis_trv_eng() == false)
            {
                ARTEMIS_DEBUG_PRINTF("Piston Length approximately reached\n");
                fullFlag = true;
            }
            else
            {
                ARTEMIS_DEBUG_PRINTF("Piston is moving ...\n");
            }
            xSemaphoreGive(piston.rtos.semaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, period);
    }
    vTaskDelay(100/ portTICK_PERIOD_MS);

    /** Check to see if length is valid */
    float length = module_pis_get_length();
    if( fabs(length - piston.setpoint) >= PISTON_LENGTH_DIFF_MAX )
    {
        /** ERROR - Alert the system somehow */
        ARTEMIS_DEBUG_PRINTF("ERROR :: Piston length diff = %0.5f\n", (length - piston.setpoint));
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("SUCCESS :: Piston length = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
                            length, (length - piston.setpoint), PISTON_LENGTH_DIFF_MAX);
    }
    vTaskDelete(NULL);
}

bool PIS_Get_Volume(double *volume)
{
    bool retVal = false;

    taskENTER_CRITICAL();
    *volume = piston.volume;
    retVal = true;
    taskEXIT_CRITICAL();

    return retVal;
}

void task_move_piston_to_volume(void)
{
    TickType_t xLastWakeTime;
    uint16_t period = 1000/piston.rtos.rate;
    period /= portTICK_PERIOD_MS;

    piston.rtos.semaphore = xSemaphoreCreateMutex();

    /*  before sending any write cmd to piston, read first
        if piston is already moving */
    if(module_pis_trv_eng() == true)
    {
        ARTEMIS_DEBUG_PRINTF("PISTON ERROR:: Piston is moving already\n");
        vTaskDelete(NULL);
        return;
    }
    vTaskDelay(100/portTICK_PERIOD_MS);

    PIS_move_to_volume(piston.setpoint);
    /** wait 500ms to shoot an I2C command */
    vTaskDelay(500/portTICK_PERIOD_MS);

    ARTEMIS_DEBUG_PRINTF("Piston Volume is moving\n");
    xLastWakeTime = xTaskGetTickCount();

    /** Start reading until we hit the volume */
    bool fullFlag = false;
    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done moving and at volume */
        //if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_PERIOD_MS) == pdTRUE)
        if(xSemaphoreTake(piston.rtos.semaphore, period) == pdTRUE)
        {

            //if(vol>=piston.setpoint && vol<=piston.setpoint + 0.09)
            //{
            //    ARTEMIS_DEBUG_PRINTF("Piston volume has reached to %0.5f\n", vol);
            //    fullFlag = true;
            //}
            //else
            //{
            //    ARTEMIS_DEBUG_PRINTF("Piston volume is moving to %0.5f\n", vol);
            //}

            if(module_pis_trv_eng() == false)
            {
                vTaskDelay(200/portTICK_PERIOD_MS);
                piston.volume = module_pis_get_volume();
                ARTEMIS_DEBUG_PRINTF("Piston volume %0.5f\n", piston.volume);

                if (piston.volume >=(piston.setpoint-0.1) && piston.volume <=(piston.setpoint+0.1))
                {
                    ARTEMIS_DEBUG_PRINTF("Piston Volume approximately reached\n");
                    fullFlag = true;
                }
                else
                {
                    vTaskDelay(100/portTICK_PERIOD_MS);
                    PIS_Reset();
                    vTaskDelay(500/portTICK_PERIOD_MS);
                    PIS_move_to_volume(piston.setpoint);
                    vTaskDelay(500/portTICK_PERIOD_MS);
                }
            }
            else
            {
                vTaskDelay(50/portTICK_PERIOD_MS);
                piston.volume = module_pis_get_volume();
                //ARTEMIS_DEBUG_PRINTF("Piston is moving ...\n");
            }

            xSemaphoreGive(piston.rtos.semaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, period);
    }
    vTaskDelay(100/portTICK_PERIOD_MS);

    ///** Check to see if volume is valid */
    //double volume = module_pis_get_volume();
    //if( (volume - piston.setpoint) >= PISTON_VOLUME_DIFF_MAX )
    //{
    //    /** ERROR - Alert the system somehow */
    //    ARTEMIS_DEBUG_PRINTF("ERROR :: Piston volume diff = %0.5f\n", (volume - piston.setpoint));
    //}
    //else
    //{
    //    ARTEMIS_DEBUG_PRINTF("SUCCESS :: Piston volume = %0.5f, diff = %0.5f, max_diff = %0.5f\n",
    //                        volume, (volume - piston.setpoint), PISTON_VOLUME_DIFF_MAX);
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

bool PIS_set_volume(double volume)
{
    bool retVal = false;
    if( (volume > 0) && (volume < 100000) )
    {
        ARTEMIS_DEBUG_PRINTF("Volume set-point = %0.4f\n", volume);
        piston.setpoint = volume;
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

