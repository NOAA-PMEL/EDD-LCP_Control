#ifndef CONFIG_H
#define CONFIG_H

#define PI                                  ( 3.14159265359 )

#define SYS_SMALL_PISTON_DIAMETER           ( 2.25f )
#define SYS_SMALL_PISTON_MAX_LENGTH         ( 6.00f )
#define SYS_LARGE_PISTON_DIAMETER           ( 4.50f )
#define SYS_LARGE_PISTON_MAX_LENGTH         ( 6.0f )
#define SYS_HOUSING_DIAMETER                ( 4.88f )
#define SYS_HOUSING_LENGTH                  ( 35.0f )

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

#define DRAG_PLATE_DIAMETER                 ( 8.0f )
#define DRAG_PLATE_RADIUS                   ( DRAG_PLATE_DIAMETER / 2.0f )
#define DRAG_PLATE_RADIUS_SRQ               ( DRAG_PLATE_RADIUS * DRAG_PLATE_RADIUS )
#define DRAG_PLATE_AREA                     ( PI * DRAG_PLATE_RADIUS_SRQ )

#define SYSTEM_WEIGHT_EST                   ( 10.963 ) /* in Kg */   //( 25.0f ) /* in lbs*/
#define SYSTEM_CROSSSECTION_AREA            ( DRAG_PLATE_AREA )

#define CYLINDER_DRAG_COEFF                 ( 0.81f )

#define SYSTEM_DENSITY_SEAWATER             ( 1033.0f )
#define SYSTEM_VOLUME_MIN                   ( HOUSING_VOLUME )
#define SYSTEM_VOLUME_MAX                   ( HOUSING_VOLUME + SMALL_PISTON_MAX_VOLUME )
#define SYSTEM_VOLUME_RESERVE               ( LARGE_PISTON_MAX_VOLUME )
#define SYSTEM_NEUTRAL_BUOYANCY             ( 1025.0f )
#define SYSTEM_RISE_RATE_MAX                ( 0.075f )
#define SYSTEM_RISE_RATE_SETPOINT           ( 0.100f )
#define SYSTEM_RISE_RATE_MIN                ( 0.125f )
#define SYSTEM_FALL_RATE_MAX                ( 0.125f )
#define SYSTEM_FALL_RATE_MIN                ( 0.075f )
#define SYSTEM_FALL_RATE_SETPOINT           ( 0.100f )

#define SYSTEM_PROFILER_PARK_DEPTH          ( 30.0f )
#define SYSTEM_PROFILER_PARK_DEPTH_ERR      ( 2.0f )
#define SYSTEM_PROFILER_PARK_DURATION_MIN   ( 10.0f )
#define SYSTEM_PROFILER_PARK_DURATION_SEC   ( SYSTEM_PROFILER_PARK_DURATION_MIN * 60.0f )
#define SYSTEM_PROFILER_PARK_RATE           ( 1.0f / 60.0f )

#define SYSTEM_PROFILER_PROFILE_DEPTH       ( 40.0f )
#define SYSTEM_PROFILER_PROFILE_DEPTH_ERR   ( 1.0f )
#define SYSTEM_PROFILER_PROFILE_RATE        ( 1.0f )
#define SYSTEM_PROFILER_PROFILE_OFF_DEPTH   ( 1.0f )
#define SYSTEM_PROFILER_PROFILE_EXTRA_OOMPH ( true )

#define SYSTEM_CRUSH_LIMIT_DEPTH            ( 220.0f )

#endif // CONFIG_H

