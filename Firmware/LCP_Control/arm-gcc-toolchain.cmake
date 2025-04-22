# CMake Toolchain file for arm-none-eabi-gcc based projects

# Target system specification
set(CMAKE_SYSTEM_NAME Generic) # Usually sufficient for embedded targets
set(CMAKE_SYSTEM_PROCESSOR cortex-m4) # Matches Makefile CPU setting

# Specify the cross-compiler executables
# CMake will search for these in your PATH. Ensure the toolchain's bin/ is added to PATH.
find_program(CMAKE_C_COMPILER NAMES arm-none-eabi-gcc REQUIRED)
find_program(CMAKE_CXX_COMPILER NAMES arm-none-eabi-g++ REQUIRED)
find_program(CMAKE_ASM_COMPILER NAMES arm-none-eabi-as REQUIRED)
find_program(CMAKE_AR NAMES arm-none-eabi-ar REQUIRED)
find_program(CMAKE_OBJCOPY NAMES arm-none-eabi-objcopy REQUIRED)
find_program(CMAKE_OBJDUMP NAMES arm-none-eabi-objdump REQUIRED)
find_program(CMAKE_SIZE NAMES arm-none-eabi-size REQUIRED)
find_program(CMAKE_RANLIB NAMES arm-none-eabi-ranlib) # Often needed with CMAKE_AR
find_program(CMAKE_STRIP NAMES arm-none-eabi-strip) # Optional, but good practice

# Check if compilers were found
if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
  message(FATAL_ERROR "ARM GCC cross-compilers (arm-none-eabi-gcc/g++) not found. Please ensure the toolchain is installed and its 'bin' directory is in your system's PATH.")
endif()

# --- Target Architecture Flags ---
# Set flags based on Makefile variables (CPU, FPU, FABI)
set(CPU_FLAGS "-mcpu=cortex-m4")
set(FPU_FLAGS "-mfpu=fpv4-sp-d16 -mfloat-abi=hard") # -mfloat-abi=hard corresponds to FABI=hard
set(THUMB_FLAGS "-mthumb")

# --- Default Compile Flags ---
# Combine architecture flags with common flags from Makefile's CFLAGS
# (-ffunction-sections, -fdata-sections, -g, -O3, -Wall, -std=gnu99 etc.)
# Note: -MMD -MP are handled by CMake automatically.
# We set these as CACHE variables so they can be potentially overridden if needed,
# but FORCE ensures they are set initially.
set(COMMON_COMPILE_FLAGS "-ffunction-sections -fdata-sections -g -O3 -Wall -Wno-unused-function")

set(CMAKE_C_FLAGS "${THUMB_FLAGS} ${CPU_FLAGS} ${FPU_FLAGS} ${COMMON_COMPILE_FLAGS} -std=gnu99" CACHE STRING "Default C Flags" FORCE)
set(CMAKE_CXX_FLAGS "${THUMB_FLAGS} ${CPU_FLAGS} ${FPU_FLAGS} ${COMMON_COMPILE_FLAGS} -std=gnu++11" CACHE STRING "Default CXX Flags" FORCE) # Assuming C++11 if C++ used
set(CMAKE_ASM_FLAGS "${THUMB_FLAGS} ${CPU_FLAGS} ${FPU_FLAGS} -g" CACHE STRING "Default ASM Flags" FORCE)

# --- Default Linker Flags ---
# Include CPU/FPU flags, and potentially linker script handling if not done in CMakeLists.txt
# The --specs=rdimon.specs is handled in the main CMakeLists.txt LFLAGS/target_link_options
# Add essential architecture flags for linking.
#set(CMAKE_EXE_LINKER_FLAGS "${THUMB_FLAGS} ${CPU_FLAGS} ${FPU_FLAGS} --specs=nosys.specs" CACHE STRING "Default Linker Flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "${THUMB_FLAGS} ${CPU_FLAGS} ${FPU_FLAGS} --specs=rdimon.specs" CACHE STRING "Default Linker Flags" FORCE)
# Using nosys.specs here as a common baseline; rdimon.specs is added specifically in the main CMakeLists

# --- Search Behavior for Cross-Compilation ---
# Configure CMake find commands to search only in the target system paths, not the host system.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER) # Don't find host programs
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY) # Find libraries only in target paths
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY) # Find includes only in target paths
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY) # Find packages only in target paths

# (Optional) Set the CMAKE_FIND_ROOT_PATH to the toolchain's sysroot if needed
# find_path(TOOLCHAIN_SYSROOT NAMES include/stdio.h PATHS ${CMAKE_C_COMPILER}/../arm-none-eabi NO_DEFAULT_PATH)
# if(TOOLCHAIN_SYSROOT)
#   set(CMAKE_SYSROOT ${TOOLCHAIN_SYSROOT})
#   message(STATUS "Using toolchain sysroot: ${CMAKE_SYSROOT}")
# endif()

message(STATUS "Using ARM GCC Toolchain: ${CMAKE_C_COMPILER}")
message(STATUS "Toolchain C Flags: ${CMAKE_C_FLAGS}")