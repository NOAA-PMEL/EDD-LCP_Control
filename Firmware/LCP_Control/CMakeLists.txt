# CMake Minimum Version
cmake_minimum_required(VERSION 3.15)

# Project Definition
project(LCP_Control LANGUAGES C)

# --- Configuration ---
get_filename_component(AMBIQ_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../AmbiqSuiteSDK ABSOLUTE)

if(NOT EXISTS "${AMBIQ_SDK_PATH}")
    message(FATAL_ERROR "Ambiq Suite SDK path not found: ${AMBIQ_SDK_PATH}.")
endif()
message(STATUS "Using Ambiq Suite SDK Path: ${AMBIQ_SDK_PATH}")

set(TARGET_NAME ${PROJECT_NAME})
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/lcpcontrol_lowpower.ld)

set(PART apollo3)
set(CPU cortex-m4)
set(FPU fpv4-sp-d16)
set(FABI hard)

# --- Executable Target ---
add_executable(${TARGET_NAME} "") # Sources added later
# Set output suffix to .elf so J-Link recognizes the format
set_target_properties(${TARGET_NAME} PROPERTIES SUFFIX ".elf")

# --- Add BSP Subdirectory ---
# This will define the 'am_bsp' library target using src/bsp/CMakeLists.txt
add_subdirectory(src/bsp)

# --- Source Files (Main Application) ---
set(SOURCES
    src/main.c
    # SDK Sources
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
    # Project Sources
    src/startup_gcc.c
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
    # src/artemis/artemis_pa9ld.c
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
    # TraceRecorder Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcAssert.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcCounter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcDependency.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcDiagnostics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcEntryTable.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcError.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcEvent.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcEventBuffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcExtension.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcHardwarePort.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcHeap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcInternalEventBuffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcInterval.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcISR.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcKernelPort.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcMultiCoreEventBuffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcObject.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcPrint.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcRunnable.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcSnapshotRecorder.c # Include even if using streaming for potential common code
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcStackMonitor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcStateMachine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcStaticBuffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcStreamingRecorder.c # Include this as default mode is streaming [cite: 2]
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcString.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcTask.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/trcTimestamp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/streamports/RingBuffer/trcStreamPort.c
)

target_sources(${TARGET_NAME} PRIVATE ${SOURCES})

# --- Include Directories (Main Target) ---
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
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bsp # Include project's BSP headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sys
    # TraceRecorder Includes
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/include
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/config
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/streamports/RingBuffer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceRecorder/streamports/RingBuffer/config
)

# --- Compile Definitions (Main Target) ---
target_compile_definitions(${TARGET_NAME} PRIVATE
    "PART_${PART}"
    AM_PART_APOLLO3
    AM_UTIL_FAULTISR_PRINT
    AM_FREERTOS
    AM_PACKAGE_BGA
    AM_DEBUG_PRINTF
    gcc
    # AM_BSP_GPIO_ITM_SWO # Optional
)

# --- Compile Options (Main Target) ---
target_compile_options(${TARGET_NAME} PRIVATE
    # Inherits flags from toolchain CMAKE_C_FLAGS
    # -Wno-unused-variable # Example if you want to suppress specific warnings
)

# --- Linker Options (Main Target) ---
# Uses nosys.specs from toolchain file
set(LINKER_FLAGS
    -nostartfiles
    -static
    -Wl,--gc-sections
    -Wl,--entry=Reset_Handler
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.map
    -lgcc
    -lc
    -lm   # Standard libs (libm added explicitly later too)
    -u _printf_float
)
set(LINKER_SCRIPT_FLAG "-Wl,-T,${LINKER_SCRIPT}")
target_link_options(${TARGET_NAME} PRIVATE ${LINKER_SCRIPT_FLAG} ${LINKER_FLAGS})

# --- Libraries (Main Target - Link BSP, HAL, and Math lib) ---
target_link_libraries(${TARGET_NAME} PRIVATE
    am_bsp # Link the library target defined in src/bsp/CMakeLists.txt
    ${AMBIQ_SDK_PATH}/mcu/apollo3/hal/gcc/bin/libam_hal.a
    m # Explicitly link libm
)

# --- Post-Build Steps (objcopy/objdump) ---
# $<TARGET_FILE:${TARGET_NAME}> resolves to LCP_Control.elf
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin
    COMMAND ${CMAKE_OBJDUMP} -S $<TARGET_FILE:${TARGET_NAME}> > ${CMAKE_BINARY_DIR}/${TARGET_NAME}.lst
    COMMENT "Creating ${TARGET_NAME}.bin and ${TARGET_NAME}.lst"
    VERBATIM)

# --- Custom Targets for SVL/ASB binary generation ---
set(ASB_UPLOADER_SCRIPT ${AMBIQ_SDK_PATH}/boards_sfe/common/tools_sfe/asb/asb.py)
set(SVL_UPLOADER_SCRIPT ${AMBIQ_SDK_PATH}/boards_sfe/common/tools_sfe/svl/svl.py)

# SVL Target
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin
    DEPENDS ${TARGET_NAME}
    COMMENT "Creating ${TARGET_NAME}_svl.bin"
    VERBATIM)
# Removed ALL, svl is not built by default
add_custom_target(svl DEPENDS ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin)

# ASB Target
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin
    DEPENDS ${TARGET_NAME}
    COMMENT "Creating ${TARGET_NAME}_asb.bin"
    VERBATIM)
add_custom_target(asb DEPENDS ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin)

# --- Custom Target for Flashing with SEGGER J-Link ---

find_program(JLINK_EXECUTABLE NAMES JLink.exe HINTS ENV "ProgramFiles(x86)" ENV ProgramFiles PATH_SUFFIXES SEGGER/JLink)

if(NOT JLINK_EXECUTABLE)
    message(WARNING "JLink.exe not found. Please ensure SEGGER J-Link software is installed and JLink.exe is in the system PATH or provide path via CMAKE_PREFIX_PATH. 'flash_jlink' target may fail.")
    set(JLINK_EXECUTABLE "JLink.exe") # Fallback to assuming it's in PATH
endif()

set(JLINK_SCRIPT_PATH ${CMAKE_BINARY_DIR}/flash_script_elf.jlink)
set(ELF_FILE_PATH "${CMAKE_BINARY_DIR}/${TARGET_NAME}.elf")
file(TO_CMAKE_PATH "${ELF_FILE_PATH}" ELF_FILE_PATH_CMAKE)

# Content of the J-Link script file (remains the same from previous step)
set(JLINK_SCRIPT_CONTENT
# "erase\n" # Keep erase commented out
"loadfile \"${ELF_FILE_PATH_CMAKE}\"\n"
"r\n"
"g\n"
"exit\n"
)
file(WRITE ${JLINK_SCRIPT_PATH} "${JLINK_SCRIPT_CONTENT}")

# Define the custom target - Pass arguments individually to COMMAND
add_custom_target(flash_jlink DEPENDS ${TARGET_NAME}
    COMMAND ${JLINK_EXECUTABLE} # Program
            # Arguments, each as a separate item
            -device AMA3B1KK-KBR
            -if SWD
            -speed 4000
            -autoconnect 1
            -ExitOnError 1
            -CommanderScript ${JLINK_SCRIPT_PATH}
    COMMENT "Flashing target with SEGGER J-Link using ELF file ${ELF_FILE_PATH}"
    VERBATIM
    USES_TERMINAL
)

# --- Bootload Targets (Using specific .bin files) ---
set(COM_PORT "$ENV{COM_PORT}" CACHE STRING "Serial port for bootloading")
if(NOT COM_PORT)
    set(COM_PORT "/dev/ttyACM0")
    message(STATUS "COM_PORT not set, defaulting to ${COM_PORT}")
endif()
set(SVL_UPLOAD_BAUD 921600)
add_custom_target(bootload_svl DEPENDS svl # Depends on the svl target which creates _svl.bin
    COMMAND ${PYTHON_EXECUTABLE} ${SVL_UPLOADER_SCRIPT} ${COM_PORT} -f ${CMAKE_BINARY_DIR}/${TARGET_NAME}_svl.bin -b ${SVL_UPLOAD_BAUD} -v
    COMMENT "Bootloading SVL via ${COM_PORT}"
    VERBATIM)
set(ASB_UPLOAD_BAUD 115200)
add_custom_target(bootload_asb DEPENDS asb # Depends on the asb target which creates _asb.bin
    COMMAND ${PYTHON_EXECUTABLE} ${ASB_UPLOADER_SCRIPT} --bin ${CMAKE_BINARY_DIR}/${TARGET_NAME}_asb.bin --load-address-blob 0x20000 --magic-num 0xCB -o ${CMAKE_BINARY_DIR}/${TARGET_NAME} --version 0x0 --load-address-wired 0xC000 -i 6 --options 0x1 -b ${ASB_UPLOAD_BAUD} -port ${COM_PORT} -r 2 -v
    COMMENT "Bootloading ASB via ${COM_PORT}"
    VERBATIM)
add_custom_target(bootload DEPENDS bootload_asb) # Default bootload uses ASB


# --- Output ---
message(STATUS "Target: ${TARGET_NAME}.elf") # Reflect suffix change
message(STATUS "SDK Path: ${AMBIQ_SDK_PATH}")
message(STATUS "Linker Script: ${LINKER_SCRIPT}")
message(STATUS "Output directory: ${CMAKE_BINARY_DIR}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")