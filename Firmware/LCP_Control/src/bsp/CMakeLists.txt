# CMakeLists.txt for the BSP library (src/bsp)

# Define the library target
add_library(am_bsp STATIC
    am_bsp.c
    am_bsp_pins.c
    # Add other .c files from src/bsp if any are needed for the library
)

# Specify include directories needed to compile the library sources
target_include_directories(am_bsp PUBLIC
    # Allow code linking against am_bsp to see headers in include/bsp
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # Adjust if installation is planned
PRIVATE
    # Internal includes needed by am_bsp.c/am_bsp_pins.c
    ${CMAKE_SOURCE_DIR}/include/bsp # Contains am_bsp.h, am_bsp_pins.h
    # Include necessary SDK directories
    ${AMBIQ_SDK_PATH}/CMSIS/AmbiqMicro/Include
    ${AMBIQ_SDK_PATH}/devices
    ${AMBIQ_SDK_PATH}/mcu/apollo3
    ${AMBIQ_SDK_PATH}/mcu/apollo3/hal
    ${AMBIQ_SDK_PATH}/utils
    # Add other SDK includes if required by am_bsp sources
)

# Add compile definitions if specific ones are needed for the BSP library
# target_compile_definitions(am_bsp PRIVATE
#     SOME_BSP_SPECIFIC_DEFINE
# )

# Set the output directory for the library (optional, defaults to build tree)
# set_target_properties(am_bsp PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

message(STATUS "Configuring BSP library target: am_bsp")