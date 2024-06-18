#******************************************************************************
#
# Makefile - Rules for building the libraries, examples and docs.
#
# Copyright (c) 2019, Ambiq Micro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# Third party software included in this distribution is subject to the
# additional license terms as defined in the /docs/licenses directory.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# This is part of revision 2.1.0 of the AmbiqSuite Development Package.
#
#******************************************************************************

#******************************************************************************
#
# This is an example makefile for SparkFun Apollo3 boards as used in the
#   AmbiqSuite SDK.
#
# Recommended usage
#   make
#   make bootload_svl COM_PORT=/dev/ttyUSBX (uses the SparkFun Variable Loader to upload code)
#   make bootload_asb COM_PORT=/dev/ttyUSBX (uses the Ambiq Secure Bootlaoder to upload code)
#   make clean
#
#******************************************************************************

#******************************************************************************
# Modified by   : Basharat, Basharat
# Email         : Basharat.martin@noaa.gov
# Date          : 11/29/2022
#******************************************************************************

#******************************************************************************
#
# User Options
#
#******************************************************************************

### Project Settings
TARGET := LCP_Control
COMPILERNAME := gcc
PROJECT := $(TARGET)

SHELL:=/bin/bash

TOOLCHAIN ?= arm-none-eabi
PART = apollo3
CPU = cortex-m4
FPU = fpv4-sp-d16
# Default to FPU hardware calling convention.  However, some customers and/or
# applications may need the software calling convention.
#FABI = softfp
FABI = hard
PYTHON3 = python3

LINKER_FILE := ./linker/lcpcontrol_lowpower.ld
STARTUP_FILE := ./src/startup_$(COMPILERNAME).c

#### Required Executables ####
CC   = $(TOOLCHAIN)-gcc
GCC  = $(TOOLCHAIN)-gcc
CPP  = $(TOOLCHAIN)-cpp
CXX  = $(TOOLCHAIN)-g++
LD   = $(TOOLCHAIN)-ld
CP   = $(TOOLCHAIN)-objcopy
OD   = $(TOOLCHAIN)-objdump
RD   = $(TOOLCHAIN)-readelf
AR   = $(TOOLCHAIN)-ar
SIZE = $(TOOLCHAIN)-size
RM   = $(shell which rm 2>/dev/null)

EXECUTABLES = CC LD CP OD AR RD SIZE GCC CXX
K := $(foreach exec,$(EXECUTABLES),\
        $(if $(shell which $($(exec)) 2>/dev/null),,\
        $(info $(exec) not found on PATH ($($(exec))).)$(exec)))
$(if $(strip $(value K)),$(info Required Program(s) $(strip $(value K)) not found))

ifneq ($(strip $(value K)),)
all clean:
	$(info Tools $(TOOLCHAIN)-$(COMPILERNAME) not installed.)
	$(RM) -rf bin
else

SDKPATH     = $(AmbiqSuiteSDK)
COMMONPATH  = $(SDKPATH)/boards_sfe/common
PROJECTPATH = .
BOARD       = lcpcontrol
BIN         = $(PROJECTPATH)/bin
ASB_UPLOADER=$(PYTHON3) $(COMMONPATH)/tools_sfe/asb/asb.py
SVL_UPLOADER=$(PYTHON3) $(COMMONPATH)/tools_sfe/svl/svl.py

CONFIG := $(PROJECTPATH)/gcc/$(BOARD)/obj

# Global Defines
DEFINES=  -DPART_$(PART)
DEFINES+= -DAM_PART_APOLLO3
DEFINES+= -DAM_UTIL_FAULTISR_PRINT
DEFINES+= -DAM_FREERTOS
DEFINES+= -DAM_PACKAGE_BGA
DEFINES+= -DAM_DEBUG_PRINTF
DEFINES+= -Dgcc
##DEFINES+= -DAM_BSP_GPIO_ITM_SWO

# Includes (Add paths to where example header files are located)
INCLUDES = -I$(SDKPATH)/third_party/FreeRTOSv10.1.1/Source/portable/GCC/AMapollo2
INCLUDES+= -I$(SDKPATH)/CMSIS/AmbiqMicro/Include
INCLUDES+= -I$(SDKPATH)/devices
INCLUDES+= -I$(SDKPATH)
INCLUDES+= -I$(SDKPATH)/mcu/apollo3
INCLUDES+= -I$(SDKPATH)/third_party/FreeRTOSv10.1.1/Source/include
INCLUDES+= -I$(SDKPATH)/utils
INCLUDES+= -I$(SDKPATH)/CMSIS/ARM/Include
INCLUDES+= -I$(SDKPATH)/mcu/apollo3/hal
INCLUDES+= -I$(SDKPATH)/third_party/exactle/wsf/sources/port/freertos
### Custom Inclusion
INCLUDES+= -I$(PROJECTPATH)/include
INCLUDES+= -I$(PROJECTPATH)/include/app
INCLUDES+= -I$(PROJECTPATH)/include/artemis
INCLUDES+= -I$(PROJECTPATH)/include/bsp
INCLUDES+= -I$(PROJECTPATH)/include/RTOS
INCLUDES+= -I$(PROJECTPATH)/include/sys

# VPATH (Add paths to where your source files are located)
VPATH = $(SDKPATH)/third_party/FreeRTOSv10.1.1/Source/portable/MemMang
VPATH+=:$(SDKPATH)/devices
VPATH+=:$(SDKPATH)/third_party/FreeRTOSv10.1.1/Source/portable/GCC/AMapollo2
VPATH+=:$(SDKPATH)/third_party/FreeRTOSv10.1.1/Source
VPATH+=:$(SDKPATH)/utils
VPATH+=:$(PROJECTPATH)
VPATH+=:$(PROJECTPATH)/src
VPATH+=:$(PROJECTPATH)/src/app
VPATH+=:$(PROJECTPATH)/src/artemis
VPATH+=:$(PROJECTPATH)/src/bsp
VPATH+=:$(PROJECTPATH)/src/RTOS
VPATH+=:$(PROJECTPATH)/src/sys

# Compilation Units (Add all the .c files you need to compile)
SRC  = main.c
SRC += port.c
SRC += heap_4.c
SRC += am_devices_button.c
SRC += am_devices_led.c
SRC += event_groups.c
SRC += list.c
SRC += queue.c
SRC += tasks.c
SRC += timers.c
SRC += am_util_debug.c
SRC += am_util_delay.c
SRC += am_util_faultisr.c
SRC += am_util_stdio.c
SRC += am_util_string.c
SRC += am_util_time.c
SRC += am_util_id.c
SRC += startup_gcc.c
### custom inclusion
SRC += buffer_c.c
SRC += buffer_u32.c
SRC += depth.c
SRC += GPS.c
SRC += i9603n.c
SRC += MAX14830.c
SRC += piston.c
SRC += control.c
SRC += data.c
SRC += sensors.c
SRC += StateMachine.c
SRC += artemis_debug.c
SRC += artemis_i2c.c
SRC += artemis_i9603n.c
SRC += artemis_iom.c
SRC += artemis_mcu.c
SRC += artemis_pa9ld.c
SRC += artemis_piston.c
SRC += artemis_spi.c
SRC += artemis_stream.c
SRC += artemis_supercap.c
SRC += artemis_time.c
SRC += artemis_uart.c
SRC += artemis_ublox_i2c.c
SRC += rtos.c
SRC += sysinfo.c
SRC += temperature.c
SRC += ublox.c
SRC += ublox_ubx.c
SRC += S9_temperature.c
SRC += K9lx_pressure.c
SRC += artemis_rtc.c
SRC += artemis_accel.c
SRC += datalogger.c
SRC += sensors_test.c
#files removed from the compilation process
##SRC += LED.c
##SRC += bsp_uart.c
##SRC += artemis_max14830.c
##SRC += isbd_at_cmd.c
##SRC += dataframe.c

CSRC = $(filter %.c,$(SRC))
OBJS = $(CSRC:%.c=$(CONFIG)/%.o)

DEPS = $(CSRC:%.c=$(CONFIG)/%.d)

# LIBS (Precompiled libraries to include in the linker step)
LIBS = $(PROJECTPATH)/src/bsp/bin/libam_bsp.a
LIBS+= $(SDKPATH)/mcu/apollo3/hal/gcc/bin/libam_hal.a

#******************************************************************************
#
# Machinery
#
#******************************************************************************

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections
CFLAGS+= -MMD -MP -std=gnu99 -Wall -g
CFLAGS+= -Wall -Wno-unused-function
CFLAGS+= -O3
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)
CFLAGS+=

LFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
LFLAGS+= $(LIBS)
LFLAGS+= -nostartfiles -static
LFLAGS+= -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(BIN)/$(TARGET).map
LFLAGS+= --specs=rdimon.specs -lgcc -lc -lm -lrdimon
LFLAGS+= -u _printf_float
LFLAGS+=

# Additional user specified CFLAGS
CFLAGS+=$(EXTRA_CFLAGS)

CPFLAGS = -Obinary
ODFLAGS = -S

#******************************************************************************
#
# Targets / Rules
#
#******************************************************************************
all: svl
asb: directories $(BIN)/$(TARGET)_asb.bin
svl: directories $(BIN)/$(TARGET)_svl.bin
sim: directories $(BIN)/$(TARGET).bin

### Bootloader Tools
ASB_UPLOADER=$(PYTHON3) $(COMMONPATH)/tools_sfe/asb/asb.py
SVL_UPLOADER=$(PYTHON3) $(COMMONPATH)/tools_sfe/svl/svl.py
ASB_UPLOAD_BAUD ?=115200
SVL_UPLOAD_BAUD ?=921600

directories:
	@mkdir -p $(CONFIG)
	@mkdir -p $(BIN)

$(CONFIG)/%.o: %.c $(CONFIG)/%.d
	@echo " Compiling $(CC) $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@

$(BIN)/$(TARGET).axf: $(OBJS) $(LIBS)
	@echo " $(OBJS) $(LIBS) "
	@echo " Linking $(COMPILERNAME) $@" ;\
	$(CC) $(OBJS) $(LFLAGS) -Wl,-T,$(LINKER_FILE) -o $@

$(BIN)/$(TARGET).bin: $(BIN)/$(TARGET).axf
	@echo " Copying $(COMPILERNAME) $@..." ;\
	$(CP) $(CPFLAGS) $< $@ ;\
	$(OD) $(ODFLAGS) $< > $(BIN)/$(TARGET).lst

$(BIN)/$(TARGET)_asb.axf: LINKER_FILE_ASB = ./linker/lcpcontrol_lowpower_asb.ld
$(BIN)/$(TARGET)_asb.axf: $(OBJS) $(LIBS)
	@echo " Linking $(COMPILERNAME) $@ with script $(LINKER_FILE_ASB)";\
	$(CC) $(OBJS) $(LFLAGS) -Wl,-T,$(LINKER_FILE_ASB) -o $@

$(BIN)/$(TARGET)_svl.axf: LINKER_FILE_SVL = ./linker/lcpcontrol_lowpower_svl.ld
$(BIN)/$(TARGET)_svl.axf: $(OBJS) $(LIBS)
	@echo " Linking $(COMPILERNAME) $@ with script $(LINKER_FILE_SVL)";\
	$(CC) -Wl,-T,$(LINKER_FILE_SVL) -o $@ $(OBJS) $(LFLAGS)

$(BIN)/$(TARGET)_%.bin: $(BIN)/$(TARGET)_%.axf
	@echo " Copying $(COMPILERNAME) $@..." ;\
	$(CP) $(CPFLAGS) $< $@ ;\
	$(OD) $(ODFLAGS) $< > $(BIN)/$(TARGET).lst

bootload_asb: directories $(BIN)/$(TARGET)_asb.bin
	$(ASB_UPLOADER) --bin $(BIN)/$(TARGET)_asb.bin --load-address-blob 0x20000 --magic-num 0xCB -o $(BIN)/$(TARGET) --version 0x0 --load-address-wired 0xC000 -i 6 --options 0x1 -b $(ASB_UPLOAD_BAUD) -port $(COM_PORT) -r 2 -v

bootload_svl: directories $(BIN)/$(TARGET)_svl.bin
	$(SVL_UPLOADER) $(COM_PORT) -f $(BIN)/$(TARGET)_svl.bin -b $(SVL_UPLOAD_BAUD) -v

bootload: bootload_asb
##bootload: bootload_svl

clean:
	@echo "Cleaning..." ;\
	$(RM) -f $(OBJS) $(DEPS) \
		$(BIN)/$(TARGET).bin $(BIN)/$(TARGET).axf \
		$(BIN)/$(TARGET).lst $(BIN)/$(TARGET).map \
		$(BIN)/$(TARGET)_svl.bin $(BIN)/$(TARGET)_svl.axf \
		$(BIN)/$(TARGET)_svl.lst $(BIN)/$(TARGET)_svl.map \
		$(BIN)/$(TARGET)_asb.bin $(BIN)/$(TARGET)_asb.axf \
		$(BIN)/$(TARGET)_asb.lst $(BIN)/$(TARGET)_asb.map

$(CONFIG)/%.d: ;

$(PROJECTPATH)/src/bsp/bin/libam_bsp.a:
	$(MAKE) -C ./src/bsp

$(SDKPATH)/mcu/apollo3/hal/gcc/bin/libam_hal.a:
	$(MAKE) -C $(SDKPATH)/mcu/apollo3/hal

# Automatically include any generated dependencies
-include $(DEPS)
endif
.PHONY: all clean directories
