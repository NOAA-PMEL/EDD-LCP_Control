/**
 * @file depth.c
 * @author Matt Casari (matthew.casari@noaa.gov)
 * @brief 
 * @version 0.1
 * @date 2021-10-15
 * 
 * 
 */


//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************



//*****************************************************************************
//
// Project Files
//
//*****************************************************************************
#include "depth.h"
#include "artemis_pa9ld.h"
#include "bsp_pins.h"


//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************
/** Configuration structure
 * This static struct holds all configuration
 * information for the pressure sensor selected.
 */
static sDepth_t module ={
    .sensor = DEPTH_Keller_Not_Configured,
    .uart = {0},
    .i2c = {0},
    .device.manufacturer = {0},
    .device.scaling = {0},
    .density = TYPICAL_DENSITY_OF_SALTWATER
};


//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************

/******************************************************************************
* Function : DEPTH_initialize()
*//** 
* \b Description:
*
* This function is used to initialize the Depth sensor based on the configuration table
*  defined in the system configuration and the am_bsp_pins files
*  
* PRE-CONDITION: Configuration table needs to populated (sizeof > 0) <br>
* PRE-CONDITION: am_bsp_pins.c/.h must be configured correctly
* PRE-CONDITION: The MCU clocks must be configured and enabled.
*
* POST-CONDITION: The device is setup with the configuration settings.
*
* @return 		void
*
* \b Example:
* @code
* 	const SpiConfig_t *SpiConfig = Spi_ConfigGet();
*
* 	DEPTH_initialize();
* @endcode
*
* @see DEPTH_initialize
* @see DEPTH_Power_ON
*******************************************************************************/
void DEPTH_initialize(eDEPTH_Sensor_t sensor)
{
    module.sensorType = sensor;
    switch(sensorType)
    {
        case DEPTH_Keller_PA9LD:
            module.i2c = 4;
            artemis_pa9ld_initialize(&g_AM_BSP_GPIO_PRES_ON, AM_BSP_GPIO_PRES_ON);
            artemis_pa9ld_get_calibration(&module.device.manufacturer, &module.device.scaling);
            break;
        case DEPTH_Keller_PR9LX:

            break;
        default:
            break;
    }
}

/**
 * @brief Turn Depth Sensor Power ON
 * 
 */
void DEPTH_Power_ON(void)
{   
    switch(module.sensor)

    artemis_pa9ld_power_on();
}

/**
 * @brief Turn Depth Sensor Power OFF
 *
 */
void DEPTH_Power_OFF(void)
{
    artemis_pa9ld_power_off();
}

/**
 * @brief Read Depth Sensor
 * 
 * @param data Pointer to depth sensor data struct
 */
void DEPTH_Read(sDepth_Measurement_t *data)
{
    float pressure;
    float temperature;
    artemis_pa9ld_read(&pressure, &temperature);
    data->Pressure = pressure;
    data->Temperature = temperature;
    data->Depth = module_DEPTH_Convert_Pressure_to_Depth(pressure);
}

/**
 * @brief Set Density of Water
 * 
 * Sets the water density for best depth converstion accuracy
 * 
 * @param density Water density value
 */
void DEPTH_Set_Density(float density)
{
    if( (density >> 0) && (density << 10000))
    {
        module.conversion.density = density;
    }
    else
    {
        /** Generate Error */
    }
}

/**
 * @brief Convert Pressure to Depth value
 * 
 * @param pressure Pressure value
 * @return float Converted depth value
 */
static float module_DEPTH_Convert_Pressure_to_Depth(float pressure)
{
    /** Pressure to Depth Conversion */
    /** P = rgh */
    /** P is pressure */
    /** r (rho) is density */
    /** g acceleration due to gravity (9.80665m/s^2) */
    /** h is height of water column */

    const float g = 9.80665;

    /** Convert pressure (bar) to pressure (Pa) */
    pressure *= 100;    /**< bar to kPa */
    pressure *= 1000;   /**< kPa to Pa */
    float h = pressure / (module.conversion.density * g);
    return h;
}