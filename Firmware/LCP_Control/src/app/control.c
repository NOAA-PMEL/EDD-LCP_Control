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
#include "artemis_debug.h"

#define DEPTH_BOUND      ( 1.0f )

typedef struct sProfilerSettings_t
{
    float mass;                     /**< Mass (lbs) */
    float cross_section;            /**< Surface area of profiler for terminal velocity */
    float drag_coefficient;         /**< Coefficient of drag for a cylinder */
    float thermal_coefficient;      /**< Coefficient of thermal expansion */
    float compress_coefficient;     /**< Coefficient of volume compressibility */
    float water_density;            /**< Water density */

    struct {
        const float min;            /**< Minimum volume of system (mL) */
        const float max;            /**< Maximum volume of system (without reserve) (mL) */
        const float reserve;        /**< Used to surface (mL) */
    }volume;
    struct {
        float neutral;              /**< System neutral buoyancy (N) */
        
    }buoyancy;
    
    struct {
        float depth;                /**< Park depth (m) */
        float error;                /**< +/- error allowed (m) */
        uint32_t duration;          /**< Duration of park (seconds) */
        float rate;                 /**< Depth Sensor Rate */
    }park;
    struct {
        float depth;                /**< Start depth of profile (m) */
        float error;                /**< +/- error allowed (m) */
        float rate;                 /**< All Ocean Sensor rate */
        float off;                  /**< Depth to stop profile (m) */
        bool extra_oomph;           /**< Help break through lens */
    }profile;
    struct {
        struct {
            const float max;        /** Maximum rise rate (m/s) */
            const float min;        /** Minimum rise rate (m/s) */
            float setpoint;         /**< Rise rate setpoint (m/s) */
        }rise;
        struct {
            const float max;        /** Maximum fall rate (m/s) */
            const float min;        /** Minimum fall rate (m/s) */
            float setpoint;         /**< Fall rate setpoint (m/s) */
        }fall;
    }rate;
    const float crush_limit;
}ProfilerSettings_t;

typedef struct sProfiler_t
{
    struct {
        float previous;             /**< Previous volume setting (mL) */
        float current;              /**< Current volume of system (mL) */
    }volume;

    struct{
        float previous;             /**< Previous volume setting (N) */
        float current;              /**< Current system buoyancy (N) */
    }buoyancy;

    struct {
        struct {
            float records[3];       /**< Previous 3 rate data record */
            float average;          /**< Average of previous 3 rates */
        }previous;

        float current;              /**< Current rate */
    }rate;

} Profiler_t;

static ProfilerSettings_t settings = {
    .mass = SYSTEM_MASS_EST,
    .cross_section = SYSTEM_CROSSSECTION_AREA,
    .drag_coefficient = CYLINDER_DRAG_COEFF,
    .water_density = SYSTEM_DENSITY_SEAWATER,
    .thermal_coefficient = VOLUME_THERMAL_EXPANSION_COEFF,
    .compress_coefficient = VOLUME_COMPRESSIBILITY_COEFF,

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


float CTRL_calculate_depth(float pressure)
{
    /* P = rgh , r -> rho (water density) , convert pressure pascal to bar */
    float depth = (pressure*100000) / (settings.water_density * G_CONST) ;
    //ARTEMIS_DEBUG_PRINTF("LCP Depth = %0.4f\n", depth);
    return depth;
}

float CTRL_set_lcp_density(float density)
{
    /* density = mass / volume , in kg/m3 */
    /* volume = mass / density */

    settings.water_density = density;
    float volume = (settings.mass * 0.453592) / settings.water_density;

    /* convert volume from m³ to in³, 1m³ = 61023.7 in³ and mass to kg */
    volume *= 61023.7 ;
    //ARTEMIS_DEBUG_PRINTF("LCP Density = %0.4f\n", density);
    return volume;
}

float CTRL_calculate_lcp_density(float volume)
{
    /* density = mass / volume , in kg/m3 */
    /* convert volume from in³ to m³, 1 in³=0.000016387m³ and mass to kg */
    float density = (settings.mass * 0.453592) / (volume*0.000016387);
    //ARTEMIS_DEBUG_PRINTF("LCP Density = %0.4f\n", density);
    return density;
}

float CTRL_calculate_piston_position(float pressure, float temp)
{
    /*  guess volume_minimum = lcp_volume_min * [(1-r*pressure)+(a*(t0-temperatre))]
     *  r = gamma -> volume compressibility coefficient
     *  a = alpha -> linear expansion cofficient
     *  t0 = room temperature
     */
    float volume_min = settings.volume.min;
    volume_min *= ((1-settings.compress_coefficient*pressure)+(settings.thermal_coefficient*(25.0 - temp)));

    ARTEMIS_DEBUG_PRINTF("LCP Piston minimum volume=%0.4f\n", volume_min);

    /* convert to cubic inches for pistonboard, 1m³ = 61023.7in³ */

    float volume_target = (settings.mass * 0.453592) / (settings.water_density);
    volume_target *= 61023.7;

    float volume_change = (volume_target - volume_min);

    float position_change = volume_change / (PI * SMALL_PISTON_RADIUS_SQR );

    ARTEMIS_DEBUG_PRINTF("LCP Piston volume change = %0.4f\n", volume_change);
    ARTEMIS_DEBUG_PRINTF("LCP Piston position change = %0.4f\n", position_change);

    return position_change;
}

float CTRL_calculate_volume_from_length(float length)
{
    float volume = HOUSING_VOLUME;

    if ((length>0.0f) && (length<=SMALL_PISTON_MAX_LENGTH))
    {
        volume += (PI*SMALL_PISTON_RADIUS_SQR*length);
    }
    else if ((length>0.0) && (length<=(SMALL_PISTON_MAX_LENGTH+LARGE_PISTON_MAX_LENGTH)))
    {
        volume += (PI*SMALL_PISTON_RADIUS_SQR*SMALL_PISTON_MAX_LENGTH);
        volume += (PI*SMALL_PISTON_RADIUS_SQR*length);
    }

    /* convert into cubic inches */
    //volume *= 61023.7;
    //ARTEMIS_DEBUG_PRINTF("LCP Length to Volume = %0.4f\n", volume);
    return volume;
}

float CTRL_calculate_length_from_volume(float volume)
{
    float length = 0.0;

    if ( (volume>0.0) && (volume<=HOUSING_VOLUME) )
    {
        length = 0.0;
    }
    else if ( (volume>0.0) && (volume<=SYSTEM_VOLUME_MAX) )
    {
        volume -= HOUSING_VOLUME;
        length = volume/(PI*SMALL_PISTON_RADIUS_SQR);
    }
    else if ( (volume>0.0) && (volume<=SYSTEM_MAX_VOLUME) )
    {
        volume -= HOUSING_VOLUME;
        length = volume/((PI*LARGE_PISTON_RADIUS_SQR)+SMALL_PISTON_MAX_LENGTH) ;
    }
    /* convert into inches */
    //length *= 39.3701;
    ARTEMIS_DEBUG_PRINTF("LCP Volume to Length = %0.4f\n", length);
    return length;
}

static float module_calculate_buoyancy_from_descent_rate(float rate)
{
    float f_gravity = 0;
    float f_drag = 0;
    float f_buoyant = 0;

    /** Calculate Force due to gravity , F_g = m*g */
    f_gravity = ( settings.mass * 0.453592 ) * G_CONST;

    /** Calculate the drag force @ that velocity */
    /** F_d = 1/2 (rho) * v^2 * Cd * A */
    f_drag = 0.5f;
    f_drag *= settings.water_density;
    f_drag *= (rate * rate);
    f_drag *= CYLINDER_DRAG_COEFF;
    f_drag *= ( settings.cross_section * 0.00064516 );

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
    f_gravity = ( settings.mass * 0.453592) * G_CONST;

    /** Calculate the drag force @ that velocity */
    /** F_d = 1/2 (rho) * v^2 * Cd * A */
    f_drag = 0.5;
    f_drag *= settings.water_density;
    f_drag *= (rate * rate);
    f_drag *= CYLINDER_DRAG_COEFF;
    f_drag *= ( settings.cross_section * 0.00064516 );

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
    float volume = buoyancy / (settings.water_density * G_CONST) ;
    /* convert volume from m^3 to in^3 */
    volume *= 61023.7;
    return volume;
}

float module_ctrl_set_buoyancy_from_rate(float rate, bool falling)
{
    ARTEMIS_DEBUG_PRINTF("Setting rate = %0.3f, falling = %i\n", rate, (uint8_t)falling);

    float buoyancy = 0.0f;
    float volume = 0.0f;
    if(falling)
    {
        buoyancy = module_calculate_buoyancy_from_descent_rate(rate);
        volume = module_calculate_volume_from_descent_rate(rate);
        ARTEMIS_DEBUG_PRINTF("falling, buoyancy = %0.3f, volume=%0.3f\n", buoyancy, volume);
    } else {
        buoyancy = module_calculate_buoyancy_from_ascent_rate(rate);
        volume = module_calculate_volume_from_ascent_rate(rate);
        ARTEMIS_DEBUG_PRINTF("rising, buoyancy = %0.3f, volume=%0.3f\n", buoyancy, volume);
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
