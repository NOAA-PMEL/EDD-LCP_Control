/// @file artemis_debug.c

#include "artemis_debug.h"
#include "am_bsp.h"
#include "datalogger.h"

void artemis_debug_initialize(void)
{

#ifdef ARTEMIS_DEBUG
    // am_bsp_itm_printf_enable();
    am_bsp_uart_printf_enable();
    am_util_stdio_printf("UART ENABLED\n");
#else
    //am_bsp_debug_printf_disable();
#endif

#if defined (DATALOG_DEBUG) && DATALOG_DEBUG
    am_util_stdio_printf("DATALOG_DEBUG is enabled\n");
#else
    am_util_stdio_printf("\n<<< !!! DATALOG_DEBUG is disbled !!! >>>\n");
    am_util_stdio_printf("<<< Please Note : it's not going to log to LCP_LOG_XXXX.txt files >>>\n");
#endif

    bool success = false;
    success = datalogger_init(4);
    if (success)
    {
        datalogger_power_on();
        am_util_delay_ms(500);
#if defined (DATALOG_DEBUG) && DATALOG_DEBUG
        datalogger_log_debug_init();
#endif

#if defined(__TEST_PROFILE_1__)
        /* reading from the file first time and fill up the buffer */
        ARTEMIS_DEBUG_PRINTF("\n<<< TEST_PROFILE_1 Profile selected >>>\n\n");
        datalogger_read_test_profile(false);
#elif defined(__TEST_PROFILE_2__)
        /* reading from the file first time and fill up the buffer */
        ARTEMIS_DEBUG_PRINTF("\n<<< TEST_PROFILE_2 Profile selected >>>\n\n");
        datalogger_read_test_profile(false);
#elif defined(__TEST_TANK__)
        ARTEMIS_DEBUG_PRINTF("\n<<< TEST_TANK Profile selected >>>\n\n");
#elif defined(__TEST_LAKE__)
        ARTEMIS_DEBUG_PRINTF("\n<<< TEST_LAKE Profile selected >>>\n\n");
#elif defined(__TEST_OCEAN__)
        ARTEMIS_DEBUG_PRINTF("\n<<< TEST_OCEAN Profile selected >>>\n\n");
#elif defined(__TEST_PS__)
        ARTEMIS_DEBUG_PRINTF("\n<<< TEST_PS Profile selected >>>\n\n");
#else
        ARTEMIS_DEBUG_PRINTF("\n<<< ERROR:: No Profile was selected >>>\n\n");
        #error "ERROR:: No test profile was selected"
#endif

    }
    else
    {
        am_util_stdio_printf("\nDATALOGGER :: ERROR : << SD Card is missing >>");
        am_util_stdio_printf("\nDATALOGGER :: ERROR : << LCP Log File was not created >>\n\n");
    }
}

void artemis_debug_assert(const char *expr, const char *func, const char *file, uint32_t line)
{
	ARTEMIS_DEBUG_PRINTF("ASSERT FAILED: {\n");
	ARTEMIS_DEBUG_PRINTF("\texpr:\t%s\n", expr);
	ARTEMIS_DEBUG_PRINTF("\tfunc:\t%s\n", func);
	ARTEMIS_DEBUG_PRINTF("\tfile:\t%s\n", file);
	ARTEMIS_DEBUG_PRINTF("\tline:\t%u\n", line);
	ARTEMIS_DEBUG_PRINTF("}\n");

	while(1);
}

void artemis_debug_halerror(const char *hfunc, uint32_t error, const char *func, const char *file, uint32_t line)
{
	ARTEMIS_DEBUG_PRINTF("AM HAL ERROR: {\n");
	ARTEMIS_DEBUG_PRINTF("\thfunc:\t%s\n", hfunc);
	ARTEMIS_DEBUG_PRINTF("\terror:\t%u\n", error);
	ARTEMIS_DEBUG_PRINTF("\tfunc:\t%s\n", func);
	ARTEMIS_DEBUG_PRINTF("\tfile:\t%s\n", file);
	ARTEMIS_DEBUG_PRINTF("\tline:\t%u\n", line);
	ARTEMIS_DEBUG_PRINTF("}\n");

	while(1);
}
