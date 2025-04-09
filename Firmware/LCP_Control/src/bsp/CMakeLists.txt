# CMakeLists.txt for the BSP library (src/bsp)

add_library(am_bsp STATIC
    am_bsp.c
    am_bsp_pins.c
)

# Specify include directories needed ONLY to compile the BSP library sources
target_include_directories(am_bsp PRIVATE
    ${CMAKE_SOURCE_DIR}/include      # For am_bsp.h
    ${CMAKE_SOURCE_DIR}/include/bsp # For am_bsp_pins.h
    # SDK Includes needed for BSP compilation
    ${AMBIQ_SDK_PATH}/CMSIS/AmbiqMicro/Include
    ${AMBIQ_SDK_PATH}/devices
    ${AMBIQ_SDK_PATH}/mcu/apollo3
    ${AMBIQ_SDK_PATH}/CMSIS/ARM/Include # <--- Ensure this line is present
    ${AMBIQ_SDK_PATH}/utils
    # Add HAL include path needed by am_hal_gpio used in am_bsp_pins.c
    ${AMBIQ_SDK_PATH}/mcu/apollo3/hal
)

# Explicitly set compile definitions for am_bsp target ONLY
target_compile_definitions(am_bsp PRIVATE
    AM_PART_APOLLO3
    AM_PACKAGE_BGA
    gcc
)

# Explicitly set compile options for am_bsp target that DIFFER
target_compile_options(am_bsp PRIVATE
    -std=c99
)

message(STATUS "Configuring BSP library target: am_bsp with explicit includes/flags/defs")