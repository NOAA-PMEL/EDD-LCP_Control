#include "control.h"
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
#include "sensors.h"
#include "config.h"
#include "i9603n.h"
#include "artemis_debug.h"


#define DEPTH_BOUND      ( 1.0f )

static void module_turn_off_gps_and_iridium(void);
static void task_move_to_depth(void);
static void task_maintain_depth(void);
static void task_profile(void);

typedef struct sProfilerSettings_t{

    float weight;                 /**< Weight (kg) */
    const float cross_section;    /**< Surface area of profiler for terminal velocity */
    const float drag_coefficient;   /**< Coefficient of drag for a cylinder */
    float density;                /**< Water density */
    struct {
        const float min;          /**< Minimum volume of system (mL) */
        const float max;          /**< Maximum volume of system (without reserve) (mL) */
        const float reserve;      /**< Used to surface (mL) */  
    }volume;
    struct {
        float neutral;            /**< System neutral buoyancy (N) */
        
    }buoyancy;
    
    struct {
        float depth;            /**< Park depth (m) */
        float error;            /**< +/- error allowed (m) */
        uint32_t duration;        /**< Duration of park (seconds) */
        float rate;             /**< Depth Sensor Rate */
    }park;
    struct {
        float depth;            /**< Start depth of profile (m) */
        float error;            /**< +/- error allowed (m) */
        float rate;             /**< All Ocean Sensor rate */
        float off;              /**< Depth to stop profile (m) */
        bool extra_oomph;       /**< Help break through lens */
    }profile;
    struct {
        struct {
            const float max;    /** Maximum rise rate (m/s) */
            const float min;    /** Minimum rise rate (m/s) */
            float setpoint;     /**< Rise rate setpoint (m/s) */
        }rise;
        struct {
            const float max;    /** Maximum fall rate (m/s) */
            const float min;    /** Minimum fall rate (m/s) */
            float setpoint;     /**< Fall rate setpoint (m/s) */
        }fall;
    }rate;
    const float crush_limit;
}ProfilerSettings_t;

typedef struct sProfiler_t{
    struct {
        float previous;     /**< Previous volume setting (mL) */
        float current;      /**< Current volume of system (mL) */
    }volume;

    struct{
        float previous;     /**< Previous volume setting (N) */
        float current;      /**< Current system buoyancy (N) */
    }buoyancy;
    struct {
        struct {
            float records[3];   /**< Previous 3 rate data record */
            float average;      /**< Average of previous 3 rates */
        }previous;

        float current;          /**< Current rate */
    }rate;
}Profiler_t;




static ProfilerSettings_t settings = {
    .weight = SYSTEM_WEIGHT_EST,
    .cross_section = SYSTEM_CROSSSECTION_AREA,
    .drag_coefficient = CYLINDER_DRAG_COEFF,
    .density = SYSTEM_DENSITY_SEAWATER,
    .volume = {
        .max = SYSTEM_VOLUME_MAX,
        .min = SYSTEM_VOLUME_MIN,
        .reserve = SYSTEM_VOLUME_RESERVE
    },
    .buoyancy.neutral = SYSTEM_NEUTRAL_BUOYANCY,
    .park = {
        .depth = SYSTEM_PROFILER_PARK_DEPTH,
        .error = SYSTEM_PROFILER_PARK_DEPTH_ERR,
        .duration = SYSTEM_PROFILER_PARK_DURATION_SEC,
        .rate = SYSTEM_PROFILER_PARK_RATE
    },
    .profile = {
        .depth = SYSTEM_PROFILER_PROFILE_DEPTH,
        .error = SYSTEM_PROFILER_PROFILE_DEPTH_ERR,
        .rate = SYSTEM_PROFILER_PROFILE_RATE,
        .off = SYSTEM_PROFILER_PROFILE_OFF_DEPTH,
        .extra_oomph = SYSTEM_PROFILER_PROFILE_EXTRA_OOMPH
    },
    .rate = {
        .rise = {
            .max = SYSTEM_RISE_RATE_MAX,
            .max = SYSTEM_RISE_RATE_MIN,
            .setpoint = SYSTEM_RISE_RATE_SETPOINT
        },
        .fall = {
            .max = SYSTEM_FALL_RATE_MAX,
            .min = SYSTEM_FALL_RATE_MAX,
            .setpoint = SYSTEM_FALL_RATE_SETPOINT
        }
    },
    .crush_limit = SYSTEM_CRUSH_LIMIT_DEPTH
};

//*****************************************************************************
//
// Static Variables
//
//*****************************************************************************
static Profiler_t profiler;


//*****************************************************************************
//
// Static Function Prototypes
//
//*****************************************************************************
static float module_calculate_buoyancy_from_ascent_rate(float rate);
static float module_calculate_volume_from_ascent_rate(float rate);
static float module_calculate_buoyancy_from_descent_rate(float rate);
static float module_calculate_volume_from_descent_rate(float rate);
static float module_calculate_volume_from_buoyancy(float buoyancy);

//*****************************************************************************
//
// Global Functions
//
//*****************************************************************************
void task_move_to_park(void)
{

    /** Set the depth */
    //PST_set_depth(profiler.park.depth);
    PST_set_depth(settings.park.depth);

    /** Monitor the movement */
    bool moveFlag;
    do
    {
        /* code */
        moveFlag = true;

    } while (moveFlag);

    /** Kill the task */
    vTaskDelete(NULL);
}
void task_move_to_depth(void)
{   
    /** Send Command to Move to Depth */
    //PST_set_depth(settings.profile.depth);
      
    PIS_move_to_volume();

    /** Loop forever */
    while(1);

}

void task_maintain_depth(void)
{
    /** @todo is there anything to this? Probably not */
}


void task_profile(void)
{
    
    /** Send Command to surface (well, float depth) */
    /** @todo Add function call */

    /** Loop forever */
    while(1);
}

void task_move_to_surface(void)
{
    /** Send command to move to surface, but not tx position */
//    PST_set_depth();
}



int32_t CTRL_MoveToPark(float depth)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t measurement = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;
    

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();


    /** Set the Depth we're moving to */
    //CTRL_set_park_depth(depth);
    SENS_set_depth_rate(2);


    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, &xDepthHandle);
    //xTaskCreate((TaskFunction_t) task_move_to_park, "move_to_park", 128, NULL, 1, &xPistonHandle);

    //if( (xDepthHandle != NULL) && (xPistonHandle != NULL) )
    if( (xDepthHandle != NULL) )
    {
        ARTEMIS_DEBUG_PRINTF("DEBUG  :: Schedular is going to start\n");
        /** Start the tasks */
        vTaskStartScheduler();

        ARTEMIS_DEBUG_PRINTF("DEBUG  :: Schedular is started\n");
        bool atDepth = false;

        float depth_max = depth + DEPTH_BOUND;
        float depth_min = depth - DEPTH_BOUND;

        while(!atDepth)
        {
            DEPTH_Read(&measurement);

            if(measurement.Depth >= settings.crush_limit)
            {
                /** Emergency blow !!! */

            } else if ( (measurement.Depth >= depth_min) && (measurement.Depth <= depth_max) )
            {   
                /** Wait for X # of seconds to validate at we're not drifting */

            }
        }

        vTaskDelete(xDepthHandle);
        vTaskDelete(xPistonHandle);
    } else {


    }
    return retVal;
}

int32_t CTRL_MoveToStartDepth(float depth)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t measurement = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;
    

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();


    /** Set the Depth we're moving to */
    CTRL_set_park_depth(depth);


    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, &xDepthHandle);
    xTaskCreate((TaskFunction_t) task_move_to_depth, "move_to_depth", 128, NULL, 1, &xPistonHandle);

    if( (xDepthHandle != NULL) && (xPistonHandle != NULL) )
    {
        /** Start the tasks */
        vTaskStartScheduler();

        bool atDepth = false;

        float depth_max = depth + DEPTH_BOUND;
        float depth_min = depth - DEPTH_BOUND;

        while(!atDepth)
        {
            DEPTH_Read(&measurement);

            if(measurement.Depth >= settings.crush_limit)
            {
                /** Emergency blow !!! */

            } else if ( (measurement.Depth >= depth_min) && (measurement.Depth <= depth_max) )
            {   
                /** Wait for X # of seconds to validate at we're not drifting */

            }
        }

        vTaskDelete(xDepthHandle);
        vTaskDelete(xPistonHandle);
    } else {


    }
    return retVal;
}


int32_t CTRL_MaintainDepth(float depth, uint32_t time_s)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
//    sDepth_Measurement_t measurement = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();

    /** Set a 1 Hz rate to the pressure sensor */
    SENS_set_depth_rate(1);

    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, &xDepthHandle);
    xTaskCreate((TaskFunction_t) task_maintain_depth, "move_to_depth", 128, NULL, 1, &xPistonHandle);


    uint32_t time = time_s;

    while(time--)
    {

    }

    vTaskDelete(xDepthHandle);
    vTaskDelete(xPistonHandle);
    
    return retVal;
    
}


int32_t CTRL_Profile(float top_depth, float rise_rate, bool break_thru_lens)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
//    sDepth_Measurement_t depth = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xTemperatureHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();

    /** Set a 1 Hz rate to the pressure sensor */
    SENS_set_depth_rate(4);

    /** Set the temperature setting */
    SENS_set_temperature_rate(4);

    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, &xDepthHandle);
    xTaskCreate((TaskFunction_t) task_temperature, "temperature", 128, NULL, 1, &xTemperatureHandle);
    xTaskCreate((TaskFunction_t) task_profile, "move_to_depth", 128, NULL, 1, &xPistonHandle);


    if( (xDepthHandle != NULL) && (xPistonHandle != NULL))
    {

        /** */
        bool completeFlag = false;
        while(!completeFlag)
        {



        }

    }

    
    return retVal;
}

int32_t CTRL_MoveToSurface(uint32_t timeout)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t depth = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;
    

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();


    /** Set the Depth we're moving to */
    CTRL_set_park_depth(0.0f);


    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, &xDepthHandle);
    xTaskCreate((TaskFunction_t) task_move_to_depth, "move_to_depth", 128, NULL, 1, &xPistonHandle);

    if( (xDepthHandle != NULL) && (xPistonHandle != NULL) )
    {
        /** Start the tasks */
        vTaskStartScheduler();

        bool atDepth = false;

        float depth_max = 0.0f + DEPTH_BOUND;
        float depth_min = 0.0f - DEPTH_BOUND;

        while(!atDepth)
        {
            DEPTH_Read(&depth);

            if(depth.Depth >= settings.crush_limit)
            {
                /** Emergency blow !!! */

            } else if ( (depth.Depth >= depth_min) && (depth.Depth <= depth_max) )
            {   
                /** Wait for X # of seconds to validate at we're not drifting */

            }
        }

        vTaskDelete(xDepthHandle);
        vTaskDelete(xPistonHandle);
    } else {


    }
    
    return retVal;
}


//*****************************************************************************
//
// Static Functions
//
//*****************************************************************************
static void module_turn_off_gps_and_iridium(void)
{
    // turn off the iridium modem
    i9603n_off();
    // turn off ublox gps
    GPS_off();
}

static float module_calculate_buoyancy_from_descent_rate(float rate)
{
    float f_gravity = 0;
    float f_drag = 0;
    float f_buoyant = 0;

    /** Calculate Force due to gravity , F_g = m*g */
    f_gravity = settings.weight * G_CONST;

    /** Calculate the drag force @ that velocity */
    /** F_d = 1/2 (rho) * v^2 * Cd * A */
    f_drag = 0.5f;
    f_drag *= settings.density;
    f_drag *= (rate * rate);
    f_drag *= CYLINDER_DRAG_COEFF;
    f_drag *= settings.cross_section;

    /** Calculate the required buoyant force */
    f_buoyant = f_gravity - f_drag;

    return f_buoyant;
}

static float module_calculate_buoyancy_from_ascent_rate(float rate)
{
    float f_gravity = 0;
    float f_drag = 0;
    float f_buoyant = 0;

    /** Calculate Force due to gravity , F_g = m*g */
    f_gravity = settings.weight * G_CONST;

    /** Calculate the drag force @ that velocity */
    /** F_d = 1/2 (rho) * v^2 * Cd * A */
    f_drag = 0.5;
    f_drag *= settings.density;
    f_drag *= (rate * rate);
    f_drag *= CYLINDER_DRAG_COEFF;
    f_drag *= settings.cross_section;

    /** Calculate the required buoyant force */
    f_buoyant = f_gravity + f_drag;

    return f_buoyant;
}

static float module_calculate_volume_from_descent_rate(float rate)
{
    float f_buoyant = module_calculate_buoyancy_from_descent_rate(rate);

    /** Calculate volume from buoyant force */
    float volume = module_calculate_volume_from_buoyancy(f_buoyant);
    return volume;
}

static float module_calculate_volume_from_ascent_rate(float rate)
{
    float f_buoyant = module_calculate_buoyancy_from_ascent_rate(rate);

    /** Calculate volume from buoyant force */
    float volume = module_calculate_volume_from_buoyancy(f_buoyant);
    return volume;
}

static float module_calculate_volume_from_buoyancy(float buoyancy)
{
    /** Calculate volume from buoyant force */
    /** F_buoyant = V * (rho) * g */
    /** so, V = F_buoyant / (rho) * g */
    float volume = buoyancy / (settings.density * G_CONST) ;
    return volume;
}

float module_ctrl_set_buoyancy_from_rate(float rate, bool falling)
{
    float buoyancy = 0.0f;
    float volume = 0.0f;
    if(falling)
    {
        buoyancy = module_calculate_buoyancy_from_descent_rate(rate);
        volume = module_calculate_volume_from_descent_rate(rate);
        ARTEMIS_DEBUG_PRINTF("falling, buoyanc = %0.3f, volume=%0.3f\n", buoyancy, volume);
    } else {
        buoyancy = module_calculate_buoyancy_from_ascent_rate(rate);
        volume = module_calculate_volume_from_ascent_rate(rate);
        ARTEMIS_DEBUG_PRINTF("rising, buoyanc = %0.3f, volume=%0.3f\n", buoyancy, volume);
    }

    //if(PIS_set_volume(volume))
    //{
    //    task_move_piston_to_volume();
    //    profiler.volume.previous = profiler.volume.current;
    //    profiler.volume.current = volume;
    //    profiler.buoyancy.previous = profiler.buoyancy.current;
    //    profiler.buoyancy.current = buoyancy;
    //}
    return volume;
}
