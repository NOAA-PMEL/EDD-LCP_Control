# CMakeLists.txt for the BSP library (src/bsp)

add_library(am_bsp STATIC
    am_bsp.c
    am_bsp_pins.c
)

# Specify include directories needed ONLY to compile the BSP library sources
target_include_directories(am_bsp PRIVATE
    # Path to the directory containing am_bsp.h and am_bsp_pins.h
    ${CMAKE_SOURCE_DIR}/include/bsp
    # SDK Includes needed for BSP compilation
    ${AMBIQ_SDK_PATH}/CMSIS/AmbiqMicro/Include
    ${AMBIQ_SDK_PATH}/devices
    ${AMBIQ_SDK_PATH}/mcu/apollo3
    ${AMBIQ_SDK_PATH}/CMSIS/ARM/Include
    ${AMBIQ_SDK_PATH}/utils
)

# Explicitly set compile definitions for am_bsp target ONLY
target_compile_definitions(am_bsp PRIVATE
    AM_PART_APOLLO3
    AM_PACKAGE_BGA
    gcc
)

# Explicitly set compile options for am_bsp target that DIFFER
# from the main project or toolchain defaults.
target_compile_options(am_bsp PRIVATE
    # Match the BSP Makefile's C standard, overriding the main project's -std=gnu99
    -std=c99
    # Core flags (-mthumb, -mcpu, -O3, -Wall etc.) should be inherited.
)

message(STATUS "Configuring BSP library target: am_bsp with minimal overrides and corrected include")