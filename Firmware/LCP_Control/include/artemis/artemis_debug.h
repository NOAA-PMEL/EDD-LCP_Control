
///
/// @file artemis_debug.h
///

#ifndef ARTEMIS_DEBUG_H
#define ARTEMIS_DEBUG_H

#include <stdbool.h>
#include <stdint.h>
#include "am_util_stdio.h"
#include "am_hal_status.h"
#include "datalogger.h"

#ifdef __cplusplus
extern "C" {
#endif

/** put false OR true to log every DEBUG_PRINTF into the SD-Card */
#define DATALOG_DEBUG true

/** comment to disable the DEBUG_PRINTF */
#define ARTEMIS_DEBUG

/** defined test profiles, tank, lake etc, uncomment one at a time */
//#define __TEST_PROFILE_1__
#define __TEST_PROFILE_2__
//#define __TEST_TANK__
//#define __TEST_LAKE__
//#define __TEST_OCEAN__
//#define __TEST_PS__

#ifdef ARTEMIS_DEBUG
    #define ARTEMIS_DEBUG_ASSERT(expr) (!!(expr) || (artemis_debug_assert(#expr, __FUNCTION__, __FILE__, __LINE__), 0))
    #define ARTEMIS_DEBUG_PRINTF(...)           \
    do {                                        \
        am_util_stdio_printf(__VA_ARGS__);      \
        if (DATALOG_DEBUG == true)              \
        {                                       \
            datalogger_log_debug(__VA_ARGS__);  \
        }                                       \
    } while(0)

    #define ARTEMIS_DEBUG_HALSTATUS(hfunc) \
    do { \
        uint32_t artemis_debug_halstatus = (hfunc); \
        if (artemis_debug_halstatus != AM_HAL_STATUS_SUCCESS) { \
            artemis_debug_halerror(#hfunc, artemis_debug_halstatus, __FUNCTION__, __FILE__, __LINE__); \
        } \
    } while(0)
#else
    #define ARTEMIS_DEBUG_ASSERT(expr) ((void)0)
    #define ARTEMIS_DEBUG_PRINTF(...)           \
    do {                                        \
        ((void)0);                              \
        if (DATALOG_DEBUG == true)              \
        {                                       \
            datalogger_log_debug(__VA_ARGS__);  \
        }                                       \
    } while(0)

    #define ARTEMIS_DEBUG_HALSTATUS(hfunc) (hfunc)
#endif

void artemis_debug_initialize(void);
void artemis_debug_assert(const char *expr, const char *func, const char *file, uint32_t line);
void artemis_debug_halerror(const char *hfunc, uint32_t error, const char *func, const char *file, uint32_t line);

#ifdef __cplusplus
}
#endif

#endif
