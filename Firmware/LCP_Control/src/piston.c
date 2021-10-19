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



//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void PIS_initialize(void)
{
    /** Initialize the I2C Port */
    artemis_piston_i2c_initialize(PISTON_I2C_ADDR);

    /** Power On */
    artemis_piston_i2c_power_on();

    /** Update state */
    module_pis_update_state();
}


void task_move_piston_to_zero(void)
{

    TickType_t xLastWakeTime;
    uint16_t period = 1000/rate;
    period /= portTICK_RATE_MS;

    /** Start Piston Retract */
    PIS_retract();

    /** Start the task of reading until we hit the end stop */
    bool fullFlag = false;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done */
        if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_RATE_MS) == pdTRUE)
        {
            fullFlag = module_pis_read_if_zero();
            xSemaphoreGive(piston.rtos.semaphore);
        }

        vTaskDelayUntil( &xLastWakeTime, period );
    }
}

void task_move_piston_to_full(void)
{
    TickType_t xLastWakeTime;
    uint16_t period = 1000/rate;
    period /= portTICK_RATE_MS;

    /** Start Piston Extend */
    PIS_extend();

    /** Start the task of reading until we hit the end stop */
    bool fullFlag = false;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done */
        if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_RATE_MS) == pdTRUE)
        {
            fullFlag = module_pis_read_if_full();
            xSemaphoreGive(piston.rtos.semaphore);
        }

        vTaskDelayUntil( &xLastWakeTime, period );
    }
}


void task_move_piston_to_volume(void)
{
    TickType_t xLastWakeTime;
    uint16_t period = 1000/rate;
    period /= portTICK_RATE_MS;

    /** Start the move */
    PIS_move_to_volume(piston.setpoint);

    /** Start reading until we hit the volume */
    bool fullFlag = false;
    while(!fullFlag)
    {
        /** Read the piston memory to see if we're done moving and at volume */
        if(xSemaphoreTake(piston.rtos.semaphore, piston.rtos.rate/portTICK_RATE_MS) == pdTRUE)
        {
            if(module_pis_trv_eng() == false)
            {
                fullFlag = true;
            }
            xSemaphoreGive(piston.rtos.semaphore);
        }
    }

    /** Check to see if volume is valid */
    double volume = module_pis_get_volume();

    if( (volume - piston.setpoint) >= PISTON_VOLUME_DIFF_MAX )
    {
        /** ERROR - Alert the system somehow */
    }
}

bool PIS_set_volume(double volume)
{
    bool retVal = false;
    if( (volume > 0) && (volume < 100000)
    {
        piston.setpoint = volume;
        retVal = true;
    }

    return retVal;
}

void PIS_extend(void)
{
    bool retVal = false;
    uint8_t addr = PISTON_I2C_MEM_ADDR_EXT_RET;
    uint8_t cmd[4] = [0x01, 0x00, 0x01, 0x01];

    
    artemis_piston_i2c_send_msg(&addr, 1, false);
    artemis_piston_i2c_send_msg(v, 8, true);

    return retVal;
}

void PIS_retract(void)
{
    bool retVal = false;
    // self._write(0x60, [0xFF, 0x01, 0x00, 0x01])
    
    uint8_t addr = PISTON_I2C_MEM_ADDR_EXT_RET;
    uint8_t cmd[4] = [0xFF, 0x00, 0x01, 0x01];
    
    artemis_piston_i2c_send_msg(&addr, 1, false);
    artemis_piston_i2c_send_msg(v, 8, true);

    return retVal;
}

void PIS_stop(void)
{
    bool retVal = false;
    uint8_t addr = PISTON_I2C_MEM_ADDR_TRV_ENG;
    uint8_t cmd = 0x00;

    
    artemis_piston_i2c_send_msg(&addr, 1, false);
    artemis_piston_i2c_send_msg(&cmd, 1, true);

    return retVal;
}


bool PIS_move_to_volume(double volume)
{
    bool retVal = false;

    uint8_t addr = PISTON_I2C_MEM_ADDR_VOLUME;

    union{
        /* data */
        double dVolume;
        uint64_t u64Volume;
    }piston;
    
    piston.dVolume = volume;
    
    /** Send command to move to volume */
    uint8_t v[8] = {0};
    for(uint8_t i=0; i<8; i++)
    {
        v[i] = piston.u64Volume >> ((7-i)*8) & 0x00000000000000FF;
    }

    artemis_piston_i2c_send_msg(&addr, 1, false);
    artemis_piston_i2c_send_msg(v, 8, true);

    return retVal;
}


static bool module_pis_read_if_full(void)
{
    uint8_t addr = PISTON_I2C_MEM_ADDR_AT_FULL;
    uint8_t data;

    
    /** Send the message to read TRV_FULL address */
    artemis_piston_i2c_read(&addr, &data, 1);
    
    return data;
}

static bool module_pis_read_if_zero(void)
{
    uint8_t addr = PISTON_I2C_MEM_ADDR_AT_ZERO;
    uint8_t data;

    
    /** Send the message to read TRV_FULL address */
    artemis_piston_i2c_read(&addr, &data, 1);
    
    return (bool) data;
}

static bool module_pis_trv_eng(void)
{
    uint8_t addr = PISTON_I2C_MEM_ADDR_TRV_ENG;
    uint8_t data = 0;

    artemis_piston_i2c_read(&addr, &data, 1);

    return (bool) data;
}

static double module_pis_get_volume(void)
{
    uint8_t addr = PISTON_I2C_MEM_ADDR_VOLUME;
    uint8_t data[8] = {0};

    artemis_piston_i2c_read(&addr, data, 8);

    union{
        float fVolume;
        uint64_t u64Volume;
    }pis;
    
    uint64_t v = 0;

    for(uint8_t i=0; i<8; i++)
    {
        v |= data[i] << 8*i;
    }

    pis.u64Volume = v;

    return pis.fVolume;
}