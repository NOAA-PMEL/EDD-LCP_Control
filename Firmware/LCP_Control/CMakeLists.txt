# CMake Minimum Version
cmake_minimum_required(VERSION 3.15)

# Project Definition
project(LCP_Control LANGUAGES C)

# --- Configuration ---
# Toolchain file is expected to set CMAKE_C_COMPILER, CMAKE_CXX_COMPILER, etc.
# and necessary target properties (CPU, FPU, ABI).
# Example toolchain file content will be provided below.

# Application specific variables (adjust paths as necessary)
# Option 1: Use environment variable (like the Makefile)
# Requires AmbiqSuiteSDK environment variable to be set before running CMake.
# get_environment_variable(AMBIQ_SDK_PATH AmbiqSuiteSDK)
# if(NOT AMBIQ_SDK_PATH)
#   message(FATAL_ERROR "Environment variable AmbiqSuiteSDK is not set.")
# endif()

# Option 2: Set as CMake variable (can be set via -D command line argument)
set(AMBIQ_SDK_PATH "$ENV{AmbiqSuiteSDK}" CACHE PATH "Path to the Ambiq Suite SDK")
if(NOT AMBIQ_SDK_PATH)
    message(FATAL_ERROR "AMBIQ_SDK_PATH is not set. Set it using -DAMBIQ_SDK_PATH=/path/to/sdk or environment variable AmbiqSuiteSDK")
endif()
message(STATUS "Using Ambiq Suite SDK Path: ${AMBIQ_SDK_PATH}")

set(TARGET_NAME ${PROJECT_NAME})
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/lcpcontrol_lowpower.ld)
set(LINKER_SCRIPT_ASB ${CMAKE_CURRENT_SOURCE_DIR}/linker/lcpcontrol_lowpower_asb.ld)
set(LINKER_SCRIPT_SVL ${CMAKE_CURRENT_SOURCE_DIR}/linker/lcpcontrol_lowpower_svl.ld)

set(PART apollo3) # Corresponds to PART in Makefile
set(CPU cortex-m4) # Corresponds to CPU in Makefile
set(FPU fpv4-sp-d16) # Corresponds to FPU in Makefile
set(FABI hard) # Corresponds to FABI in Makefile

# --- Executable Target ---
add_executable(${TARGET_NAME} "") # Sources added later

# --- Source Files ---
# Get source files from the Makefile analysis and file structure
set(SOURCES
    src/main.c
    # SDK Sources (relative to SDK path - VPATH/SRC from Makefile)
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/portable/MemMang/heap_4.c
    ${AMBIQ_SDK_PATH}/devices/am_devices_button.c
    ${AMBIQ_SDK_PATH}/devices/am_devices_led.c
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/event_groups.c
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/list.c
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/queue.c
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/tasks.c
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/timers.c
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/portable/GCC/AMapollo2/port.c
    ${AMBIQ_SDK_PATH}/utils/am_util_debug.c
    ${AMBIQ_SDK_PATH}/utils/am_util_delay.c
    ${AMBIQ_SDK_PATH}/utils/am_util_faultisr.c
    ${AMBIQ_SDK_PATH}/utils/am_util_stdio.c
    ${AMBIQ_SDK_PATH}/utils/am_util_string.c
    ${AMBIQ_SDK_PATH}/utils/am_util_time.c
    ${AMBIQ_SDK_PATH}/utils/am_util_id.c
    # Project Sources (relative to project root - VPATH/SRC from Makefile)
    src/startup_gcc.c # Note: Startup file might need adjustment based on CMake/Compiler
    src/buffer_c.c
    src/buffer_u32.c
    src/depth.c
    src/GPS.c
    src/i9603n.c
    src/MAX14830.c
    src/piston.c
    src/app/control.c
    src/app/data.c
    src/app/sensors.c
    src/app/StateMachine.c
    src/artemis/artemis_debug.c
    src/artemis/artemis_i2c.c
    src/artemis/artemis_i9603n.c
    src/artemis/artemis_iom.c
    src/artemis/artemis_mcu.c
    # src/artemis/artemis_pa9ld.c # Assuming K9lx is used based on depth.c
    src/artemis/artemis_piston.c
    src/artemis/artemis_spi.c
    src/artemis/artemis_stream.c
    src/artemis/artemis_supercap.c
    src/artemis/artemis_time.c
    src/artemis/artemis_uart.c
    src/artemis/artemis_ublox_i2c.c
    src/RTOS/rtos.c
    src/sys/sysinfo.c
    src/temperature.c
    src/ublox.c
    src/ublox_ubx.c
    src/S9_temperature.c
    src/K9lx_pressure.c
    src/artemis/artemis_rtc.c
    src/artemis/artemis_accel.c
    src/datalogger.c
    src/app/sensors_test.c
    src/app/memory.c
    # src/LED.c # Commented out in Makefile
    # src/bsp/bsp_uart.c # Commented out in Makefile
    # src/artemis/artemis_max14830.c # Commented out in Makefile
    # src/isbd_at_cmd.c # Commented out in Makefile
    # src/dataframe.c # Commented out in Makefile
)

target_sources(${TARGET_NAME} PRIVATE ${SOURCES})

# --- Include Directories ---
target_include_directories(${TARGET_NAME} PRIVATE
    # SDK Includes
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/portable/GCC/AMapollo2
    ${AMBIQ_SDK_PATH}/CMSIS/AmbiqMicro/Include
    ${AMBIQ_SDK_PATH}/devices
    ${AMBIQ_SDK_PATH}
    ${AMBIQ_SDK_PATH}/mcu/apollo3
    ${AMBIQ_SDK_PATH}/third_party/FreeRTOSv10.1.1/Source/include
    ${AMBIQ_SDK_PATH}/utils
    ${AMBIQ_SDK_PATH}/CMSIS/ARM/Include
    ${AMBIQ_SDK_PATH}/mcu/apollo3/hal
    ${AMBIQ_SDK_PATH}/third_party/exactle/wsf/sources/port/freertos
    # Project Includes
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/app
    ${CMAKE_CURRENT_SOURCE_DIR}/include/artemis
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sys
    )

# --- Compile Definitions ---
target_compile_definitions(${TARGET_NAME} PRIVATE
    "PART_${PART}"
    AM_PART_APOLLO3
    AM_UTIL_FAULTISR_PRINT
    AM_FREERTOS
    AM_PACKAGE_BGA
    AM_DEBUG_PRINTF
    gcc
    # AM_BSP_GPIO_ITM_SWO # Optional: Define if you want SWO output
    )

# --- Compile Options ---
# Corresponds to CFLAGS in Makefile
# The toolchain file should set -mthumb, -mcpu, -mfpu, -mfloat-abi
target_compile_options(${TARGET_NAME} PRIVATE
    -ffunction-sections
    -fdata-sections
    -MMD -MP # CMake handles dependency generation
    -std=gnu99
    -Wall
    -Wno-unused-function # Added this explicitly
    -g # Debug symbols
    -O3 # Optimization level
    # Add any EXTRA_CFLAGS here if needed
    )

# --- Linker Options ---
# Corresponds to LFLAGS in Makefile
# Toolchain file sets -mthumb, -mcpu, -mfpu, -mfloat-abi
set(LINKER_FLAGS
    -nostartfiles
    -static
    -Wl,--gc-sections
    -Wl,--entry=Reset_Handler # Ensure Reset_Handler is the entry point
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.map
    --specs=rdimon.specs # For semihosting printf/debug
    -lgcc
    -lc
    -lm
    -lrdimon
    -u _printf_float # Link float support for printf
    )

# Add linker script T flag separately for target_link_options
set(LINKER_SCRIPT_FLAG "-Wl,-T,${LINKER_SCRIPT}")

target_link_options(${TARGET_NAME} PRIVATE ${LINKER_SCRIPT_FLAG} ${LINKER_FLAGS})

# --- Libraries ---
# Corresponds to LIBS in Makefile
# Assuming these are pre-built or handled elsewhere (e.g., SDK build process)
# Option 1: Find pre-built libraries
# find_library(AM_BSP_LIB am_bsp PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src/bsp/bin)
# find_library(AM_HAL_LIB am_hal PATHS ${AMBIQ_SDK_PATH}/mcu/apollo3/hal/gcc/bin)
# if (AM_BSP_LIB AND AM_HAL_LIB)
#   target_link_libraries(${TARGET_NAME} PRIVATE ${AM_BSP_LIB} ${AM_HAL_LIB})
# else()
#   message(WARNING "Could not find pre-built BSP or HAL libraries. Linking might fail.")
# endif()

# Option 2: Add as INTERFACE libraries if headers are sufficient for linking stage (less likely for .a)
# Option 3: Build them as part of this CMake project using add_subdirectory (Requires CMakeLists for those libs)
# For now, just listing them as general libraries. Ensure they are found by the linker.
target_link_libraries(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bsp/bin/libam_bsp.a
    ${AMBIQ_SDK_PATH}/mcu/apollo3/hal/gcc/bin/libam_hal.a
    )

# --- Post-Build Steps (objcopy) ---
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin
    COMMAND ${CMAKE_OBJDUMP} -S $<TARGET_FILE:${TARGET_NAME}> > ${CMAKE_BINARY_DIR}/${TARGET_NAME}.lst
    COMMENT "Creating ${TARGET_NAME}.bin and ${TARGET_NAME}.lst"
    VERBATIM)

# --- Custom Targets for SVL/ASB and Bootloading ---
# Note: This assumes PYTHON3 is findable on the PATH or specified via CMAKE_COMMAND

find_program(PYTHON_EXECUTABLE NAMES python3.8 python3 python HINTS ENV PATH)
if(NOT PYTHON_EXECUTABLE)
    message(WARNING "Python 3.8 (or compatible) not found. Bootload targets may fail.")
    set(PYTHON_EXECUTABLE "python3.8") # Use default from Makefile as fallback
endif()

set(ASB_UPLOADER_SCRIPT ${AMBIQ_SDK_PATH}/boards_sfe/common/tools_sfe/asb/asb.py)
set(SVL_UPLOADER_SCRIPT ${AMBIQ_SDK_PATH}/boards_sfe/common/tools_sfe/svl/svl.py)

# SVL Target (depends on the main executable)
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${CMAKE_BINARY_DIR}/${TARGET_NAME}.axf ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin # Simplified, uses default linker script output for svl bin
    DEPENDS ${TARGET_NAME}
    COMMENT "Creating ${TARGET_NAME}_svl.bin"
    VERBATIM)
add_custom_target(svl ALL DEPENDS ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin) # 'ALL' makes it build by default

# ASB Target (depends on the main executable)
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${CMAKE_BINARY_DIR}/${TARGET_NAME}.axf ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin # Simplified, uses default linker script output for asb bin
    DEPENDS ${TARGET_NAME}
    COMMENT "Creating ${TARGET_NAME}_asb.bin"
    VERBATIM)
add_custom_target(asb DEPENDS ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin) # Not built by default unless specified

# Bootload SVL Target
# Note: COM_PORT needs to be defined, e.g., via -DCOM_PORT=COM3 or environment variable
set(COM_PORT "$ENV{COM_PORT}" CACHE STRING "Serial port for bootloading")
if(NOT COM_PORT)
    set(COM_PORT "/dev/ttyACM0") # Default for Linux, adjust as needed
    message(STATUS "COM_PORT not set, defaulting to ${COM_PORT}")
endif()
set(SVL_UPLOAD_BAUD 921600)

add_custom_target(bootload_svl DEPENDS svl
    COMMAND ${PYTHON_EXECUTABLE} ${SVL_UPLOADER_SCRIPT} ${COM_PORT} -f ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin -b ${SVL_UPLOAD_BAUD} -v
    COMMENT "Bootloading SVL via ${COM_PORT}"
    VERBATIM)

# Bootload ASB Target
set(ASB_UPLOAD_BAUD 115200)
add_custom_target(bootload_asb DEPENDS asb
    COMMAND ${PYTHON_EXECUTABLE} ${ASB_UPLOADER_SCRIPT} --bin ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin --load-address-blob 0x20000 --magic-num 0xCB -o ${CMAKE_BINARY_DIR}/${TARGET_NAME} --version 0x0 --load-address-wired 0xC000 -i 6 --options 0x1 -b ${ASB_UPLOAD_BAUD} -port ${COM_PORT} -r 2 -v
    COMMENT "Bootloading ASB via ${COM_PORT}"
    VERBATIM)

# Default bootload target
add_custom_target(bootload DEPENDS bootload_asb)
# add_custom_target(bootload DEPENDS bootload_svl) # Uncomment to default to SVL

# Optional: Add a target to build the BSP/HAL libraries if their Makefiles are present
# add_custom_target(build_libs
#     COMMAND ${CMAKE_COMMAND} -E env "AmbiqSuiteSDK=${AMBIQ_SDK_PATH}" ${CMAKE_MAKE_PROGRAM} -C ${CMAKE_CURRENT_SOURCE_DIR}/src/bsp
#     COMMAND ${CMAKE_COMMAND} -E env "AmbiqSuiteSDK=${AMBIQ_SDK_PATH}" ${CMAKE_MAKE_PROGRAM} -C ${AMBIQ_SDK_PATH}/mcu/apollo3/hal
#     COMMENT "Building dependent libraries"
#     VERBATIM)
# add_dependencies(${TARGET_NAME} build_libs) # Make target depend on libs being built

# --- Output ---
message(STATUS "Target: ${TARGET_NAME}")
message(STATUS "SDK Path: ${AMBIQ_SDK_PATH}")
message(STATUS "Linker Script: ${LINKER_SCRIPT}")
message(STATUS "Output directory: ${CMAKE_BINARY_DIR}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}") # Informative