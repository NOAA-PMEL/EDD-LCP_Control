/// @file artemis_debug.c

#include "artemis_debug.h"
#include "am_bsp.h"
#include "datalogger.h"

void artemis_debug_initialize(void)
{
    // enable Single Wire Output (SWO) using Instrumentation Trace Macrocell (ITM)
    // am_bsp_itm_printf_enable();
    am_bsp_uart_printf_enable();

    bool success = false;
    success = datalogger_init(4);

    if (success)
    {
        datalogger_power_on();
        am_util_delay_ms(500);

        /* reading from the file first time and fill up the buffer */
        datalogger_read_test_profile(false);
        ARTEMIS_DEBUG_PRINTF("UART ENABLED\n");
    }
    else
    {
        ARTEMIS_DEBUG_PRINTF("\n\nERROR :: SD Card is missing\n\n");
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
