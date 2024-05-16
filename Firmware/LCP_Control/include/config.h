#ifndef CONFIG_H
#define CONFIG_H

#include "artemis_debug.h"

#define PI                                  ( 3.14159265359 )
#define G_CONST                             ( 9.80665 )

#define SYS_SMALL_PISTON_DIAMETER           ( 2.25f * 1.0f ) //0.0254f )     /* in inches */
#define SYS_SMALL_PISTON_MAX_LENGTH         ( 6.0f  * 1.0f ) //0.0254f )
#define SYS_LARGE_PISTON_DIAMETER           ( 4.50f * 1.0f ) //0.0254f )
#define SYS_LARGE_PISTON_MAX_LENGTH         ( 6.0f  * 1.0f ) //0.0254f )
#define SYS_HOUSING_DIAMETER                ( 4.88f * 1.0f ) //0.0254f )
#define SYS_HOUSING_LENGTH                  ( 35.0f * 1.0f ) //0.0254f )

#define SMALL_PISTON_DIAMETER               ( SYS_SMALL_PISTON_DIAMETER )
#define SMALL_PISTON_RADIUS                 ( SYS_SMALL_PISTON_DIAMETER / 2.0f )
#define SMALL_PISTON_RADIUS_SQR             ( SMALL_PISTON_RADIUS * SMALL_PISTON_RADIUS )
#define SMALL_PISTON_MAX_LENGTH             ( SYS_SMALL_PISTON_MAX_LENGTH )
#define SMALL_PISTON_MAX_VOLUME             ( PI * SMALL_PISTON_RADIUS_SQR * SMALL_PISTON_MAX_LENGTH )

#define LARGE_PISTON_DIAMETER               ( SYS_LARGE_PISTON_DIAMETER )
#define LARGE_PISTON_RADIUS                 ( SYS_LARGE_PISTON_DIAMETER / 2.0f )
#define LARGE_PISTON_RADIUS_SQR             ( LARGE_PISTON_RADIUS * LARGE_PISTON_RADIUS )
#define LARGE_PISTON_MAX_LENGTH             ( SYS_LARGE_PISTON_MAX_LENGTH )
#define LARGE_PISTON_MAX_VOLUME             ( PI * LARGE_PISTON_RADIUS_SQR * LARGE_PISTON_MAX_LENGTH )

#define HOUSING_DIAMETER                    ( SYS_HOUSING_DIAMETER )
#define HOUSING_RADIUS                      ( SYS_HOUSING_DIAMETER / 2.0f )
#define HOUSING_RADIUS_SRQ                  ( HOUSING_RADIUS * HOUSING_RADIUS )
#define HOUSING_LENGTH                      ( SYS_HOUSING_LENGTH )
#define HOUSING_VOLUME                      ( PI * HOUSING_RADIUS_SRQ * HOUSING_LENGTH )

#define SYSTEM_MAX_VOLUME                   ( HOUSING_VOLUME + SMALL_PISTON_MAX_VOLUME + LARGE_PISTON_MAX_VOLUME + 0.01f )
#define SYSTEM_MIN_VOLUME                   ( HOUSING_VOLUME - 0.01f )
#define SYSTEM_MIN_LENGTH                   ( 0.0f )
#define SYSTEM_MAX_LENGTH                   ( SMALL_PISTON_MAX_LENGTH + LARGE_PISTON_MAX_LENGTH )

#define DRAG_PLATE_DIAMETER                 ( 8.0f * 1.0 ) //0.0254f )
#define DRAG_PLATE_RADIUS                   ( DRAG_PLATE_DIAMETER / 2.0f )
#define DRAG_PLATE_RADIUS_SRQ               ( DRAG_PLATE_RADIUS * DRAG_PLATE_RADIUS )
#define DRAG_PLATE_AREA                     ( PI * DRAG_PLATE_RADIUS_SRQ )

//#define SYSTEM_MASS_EST                     ( 24.17f * 1.0 ) //0.453592 )   /* in kg */
#define SYSTEM_MASS_EST                     ( 24.604f  * 1.0 ) //0.453592 )   /* in lbs */
#define SYSTEM_CROSSSECTION_AREA            ( DRAG_PLATE_AREA )

#define CYLINDER_DRAG_COEFF                 ( 0.81f )

#define SYSTEM_VOLUME_MIN                   ( HOUSING_VOLUME )
#define SYSTEM_VOLUME_MAX                   ( HOUSING_VOLUME + SMALL_PISTON_MAX_VOLUME )
#define SYSTEM_VOLUME_RESERVE               ( LARGE_PISTON_MAX_VOLUME )

#define SYSTEM_NEUTRAL_BUOYANCY             ( 1025.0f )
#define SYSTEM_DENSITY_SEAWATER             ( 1000.0f ) //1033.0f )
#define SYSTEM_RISE_RATE_SETPOINT           ( 0.100f )
#define SYSTEM_RISE_RATE_MAX                ( 0.15f )
#define SYSTEM_RISE_RATE_MIN                ( 0.05f )  //0.125f )
#define SYSTEM_FALL_RATE_SETPOINT           ( 0.100f )
#define SYSTEM_FALL_RATE_MAX                ( 0.125f )
#define SYSTEM_FALL_RATE_MIN                ( 0.075f )

#define SYSTEM_PROFILER_PARK_DEPTH          ( 180.0f )      /* in meters */
#define SYSTEM_PROFILER_PARK_DEPTH_ERR      ( 2.0f )
#define SYSTEM_PROFILER_PARK_DURATION_MIN   ( 5.0f )        /* in minutes */
#define SYSTEM_PROFILER_PARK_DURATION_SEC   ( SYSTEM_PROFILER_PARK_DURATION_MIN * 60.0f )
#define SYSTEM_PROFILER_PARK_RATE           ( 1.0f / 1.0f ) //60.0f )

#define VOLUME_COMPRESSIBILITY_COEFF        ( 0.0f )        /* gamma, coefficient of volume compressiblity, related to pressure */
#define VOLUME_THERMAL_EXPANSION_COEFF      ( 0.0f )        /* alpha, coeffciient of linear expansion of material, related to temperature */

#define SYSTEM_PROFILER_PROFILE_DEPTH       ( 200.0f )      /* in meters */
#define SYSTEM_PROFILER_PROFILE_DEPTH_ERR   ( 1.0f )
#define SYSTEM_PROFILER_PROFILE_RATE        ( 1.0f )
#define SYSTEM_PROFILER_PROFILE_OFF_DEPTH   ( 1.0f )
#define SYSTEM_PROFILER_PROFILE_EXTRA_OOMPH ( true )

#define SYSTEM_CRUSH_LIMIT_DEPTH            ( 220.0f )      /* crash limit depth in meters */
#define SYSTEM_PROFILE_NUMBER               ( 100 )         /* for testing */
#define PISTON_POSITION_INCREMENT           ( 0.015f )      /* Piston position increment-decrement in inches */
#define PISTON_POSITION_INCREMENT2          ( 0.008f )      /* Piston position increment-decrement in inches, minimum length is 0.008in */
#define PISTON_POSITION_MAXIMUM             ( 2.0f ) //11.0f )       /* Piston position maximum in inches */
#define CRUSH_DEPTH_PISTON_POSITION         ( 5.25f )       /* Piston position maximum in inches */
#define CRITICAL_PISTON_POSITON_DEPTH       ( 50.0f )       /* Critical depth where piston length must not exceed 5.25 inches */
#define PISTON_MOVE_TO_SURFACE              ( 2.0f ) //11.0f )       /* Piston position move to surface in inches */
#define PROFILE_DEPTH_RATE_COUNTER          ( 5.0f )       /* Change piston position after this number of measurements */
#define PARK_DEPTH_RATE_COUNTER             ( 2.0f )       /* Change piston position after this number of measurements */


/*************************************
** Settable Variables
*************************************/
#define GPS_TIMER                           ( 1.0f )        /* XX mins xGPS timer */
#define PARK_TRANSMIT_TRIES                 ( 01 )          /* Park mode measurements transmission nr. of tries */
#define PROF_TRANSMIT_TRIES                 ( 01 )          /* Profile mode measurements transmission nr. of tries */
#define SATELLITE_VISIBILITY_TRIES          ( 01 )          /* Look for satellite visitbility (nr. of times ) */
#define IRIDIUM_TRIES                       ( 01 )          /* Try XX number of times to transmit */
#define SATELLITE_TIMER                     ( 05 )          /* Try XX seconds for checking satellite visibility */

/** Test Profiles */
#if defined(__TEST_OCEAN__)
/** Ocean , set the values accordingly */
#define BALLAST_DEPTH                       ( 2.0f )
#define BALLAST_DEPTH_SAMPLE_RATE           ( 1.0f )
#define BALLAST_DEPTH_PROFILE               ( 2.0f )
#define PARK_DEPTH                          ( SYSTEM_PROFILER_PARK_DEPTH )
#define PARK_DEPTH_MAX                      ( SYSTEM_PROFILER_PARK_DEPTH + 10.f)
#define PARK_DEPTH_ERR                      ( SYSTEM_PROFILER_PARK_DEPTH_ERR )
#define PARK_RATE                           ( SYSTEM_PROFILER_PARK_RATE )
#define PARK_TIME                           ( SYSTEM_PROFILER_PARK_DURATION_SEC )
#define PARK_DENSITY                        ( SYSTEM_DENSITY_SEAWATER )
#define PROFILE_DEPTH                       ( SYSTEM_PROFILER_PROFILE_DEPTH )
#define PROFILE_DEPTH_ERR                   ( SYSTEM_PROFILER_PROFILE_DEPTH_ERR )
#define PROFILE_RATE                        ( SYSTEM_PROFILER_PROFILE_RATE )
#define TO_PROFILE_DENSITY                  ( SYSTEM_DENSITY_SEAWATER )
#define PROFILE_DENSITY                     ( SYSTEM_NEUTRAL_BUOYANCY )
#define CRUSH_DEPTH                         ( SYSTEM_CRUSH_LIMIT_DEPTH )
#define PARK_POSITION_INCREMENT             ( PISTON_POSITION_INCREMENT )
#define PARK_POSITION_INCREMENT2            ( PISTON_POSITION_INCREMENT2 )

#elif defined(__TEST_PROFILE_1__)
/** TEST profile1 */
#define BALLAST_DEPTH                       ( 1.0f )    // pressure reading ( 0.0101f )
#define BALLAST_DEPTH_SAMPLE_RATE           ( 1.0f )
#define BALLAST_DEPTH_PROFILE               ( 1.0f )
#define MOVE_TO_PARK_SAMPLE_RATE            ( 2.0f )
#define PARK_DEPTH                          ( 180.0f )
#define PARK_DEPTH_ERR                      ( 10.0f )
#define PARK_DEPTH_MAX                      ( 190.0f )
#define PARK_RATE                           ( SYSTEM_PROFILER_PARK_RATE )
#define PARK_TIME                           ( 3.0f * 60.0f ) /* 3 mins */
#define PARK_DENSITY                        ( 1033.0f )
#define MOVE_TO_PROFILE_SAMPLE_RATE         ( 2.0f )
#define PROFILE_DEPTH                       ( 200.0f )
#define PROFILE_DEPTH_ERR                   ( 10.0f )
#define PROFILE_RATE                        ( SYSTEM_PROFILER_PROFILE_RATE )
#define TO_PROFILE_DENSITY                  ( 1035.0f )
#define PROFILE_DENSITY                     ( 1025.0f )
#define CRUSH_DEPTH                         ( 220.0f )
#define PARK_POSITION_INCREMENT             ( PISTON_POSITION_INCREMENT )
#define PARK_POSITION_INCREMENT2            ( PISTON_POSITION_INCREMENT2 )

#elif defined(__TEST_PROFILE_2__)
/** TEST profile2 */
#define BALLAST_DEPTH                       ( 1.0f )    // pressure reading ( 0.0101f )
#define BALLAST_DEPTH_SAMPLE_RATE           ( 1.0f )
#define BALLAST_DEPTH_PROFILE               ( 1.0f )
#define MOVE_TO_PARK_SAMPLE_RATE            ( 2.0f )
#define PARK_DEPTH                          ( 20.0f )
#define PARK_DEPTH_ERR                      ( 2.0f )
#define PARK_DEPTH_MAX                      ( 23.0f )
#define PARK_RATE                           ( SYSTEM_PROFILER_PARK_RATE )
#define PARK_TIME                           ( 3.0f * 60.0f ) /* 3 mins */
#define PARK_DENSITY                        ( 1033.0f )
#define MOVE_TO_PROFILE_SAMPLE_RATE         ( 2.0f )
#define PROFILE_DEPTH                       ( 24.0f )
#define PROFILE_DEPTH_ERR                   ( 1.0f )
#define PROFILE_RATE                        ( SYSTEM_PROFILER_PROFILE_RATE )
#define TO_PROFILE_DENSITY                  ( 1035.0f )
#define PROFILE_DENSITY                     ( 1025.0f )
#define CRUSH_DEPTH                         ( 30.0f )
#define PARK_POSITION_INCREMENT             ( PISTON_POSITION_INCREMENT )
#define PARK_POSITION_INCREMENT2            ( PISTON_POSITION_INCREMENT2 )

#elif defined(__TEST_TANK__)
/** TANK Testing */
#define BALLAST_DEPTH                       ( 1.0f )
#define BALLAST_DEPTH_SAMPLE_RATE           ( 1.0f )
#define BALLAST_DEPTH_PROFILE               ( 1.0f )
#define MOVE_TO_PARK_SAMPLE_RATE            ( 2.0f )
#define PARK_DEPTH                          ( 4.0f )
#define PARK_DEPTH_ERR                      ( 0.25f )
#define PARK_DEPTH_MAX                      ( 6.0f )
#define PARK_RATE                           ( SYSTEM_PROFILER_PARK_RATE )
#define PARK_TIME                           ( 3.0 * 60.0f ) /* 3 mins */
#define PARK_DENSITY                        ( 1010.2f )
#define MOVE_TO_PROFILE_SAMPLE_RATE         ( 2.0f )
#define PROFILE_DEPTH                       ( 4.5f )
#define PROFILE_DEPTH_ERR                   ( 0.1f )
#define PROFILE_RATE                        ( SYSTEM_PROFILER_PROFILE_RATE )
#define TO_PROFILE_DENSITY                  ( 1012.5f )
#define PROFILE_DENSITY                     ( 1009.1f )
#define CRUSH_DEPTH                         ( 10.0f )
#define PARK_POSITION_INCREMENT             ( PISTON_POSITION_INCREMENT )
#define PARK_POSITION_INCREMENT2            ( PISTON_POSITION_INCREMENT2 )

#elif defined(__TEST_LAKE__)
/** LAKE Testing */
#define BALLAST_DEPTH                       ( )
#define BALLAST_DEPTH_SAMPLE_RATE           ( )
#define BALLAST_DEPTH_PROFILE               ( )
#define MOVE_TO_PARK_SAMPLE_RATE            ( )
#define PARK_DEPTH                          ( )
#define PARK_DEPTH_ERR                      ( )
#define PARK_DEPTH_MAX                      ( )
#define PARK_RATE                           ( )
#define PARK_TIME                           ( )
#define PARK_DENSITY                        ( )
#define MOVE_TO_PROFILE_SAMPLE_RATE         ( )
#define PROFILE_DEPTH                       ( )
#define PROFILE_DEPTH_ERR                   ( )
#define PROFILE_RATE                        ( )
#define TO_PROFILE_DENSITY                  ( )
#define PROFILE_DENSITY                     ( )
#define CRUSH_DEPTH                         ( )
#define PARK_POSITION_INCREMENT             ( PISTON_POSITION_INCREMENT )
#define PARK_POSITION_INCREMENT2            ( PISTON_POSITION_INCREMENT2 )

#else
    #error "<<< ERROR:: Please select at least one TEST Profile in the artemis_debug.h !!! >>>"
#endif

#endif // CONFIG_H

