TARGET := libam_bsp
COMPILERNAME := gcc
PROJECT := libam_bsp_gcc
CONFIG := bin
SDKPATH := $(AmbiqSuiteSDK)
PROJECTPATH := ../../

SHELL:=/bin/sh
#### Setup ####

TOOLCHAIN ?= arm-none-eabi
PART = apollo3
CPU = cortex-m4
FPU = fpv4-sp-d16
# Default to FPU hardware calling convention.  However, some customers and/or
# applications may need the software calling convention.
#FABI = softfp
FABI = hard

#### Required Executables ####
CC = $(TOOLCHAIN)-gcc
GCC = $(TOOLCHAIN)-gcc
CPP = $(TOOLCHAIN)-cpp
LD = $(TOOLCHAIN)-ld
CP = $(TOOLCHAIN)-objcopy
OD = $(TOOLCHAIN)-objdump
RD = $(TOOLCHAIN)-readelf
AR = $(TOOLCHAIN)-ar
SIZE = $(TOOLCHAIN)-size
RM = $(shell which rm 2>/dev/null)

EXECUTABLES = CC LD CP OD AR RD SIZE GCC
K := $(foreach exec,$(EXECUTABLES),\
        $(if $(shell which $($(exec)) 2>/dev/null),,\
        $(info $(exec) not found on PATH ($($(exec))).)$(exec)))
$(if $(strip $(value K)),$(info Required Program(s) $(strip $(value K)) not found))

ifneq ($(strip $(value K)),)
all clean:
	$(info Tools $(TOOLCHAIN)-$(COMPILERNAME) not installed.)
	$(RM) -rf bin
else

DEFINES = -DAM_PART_APOLLO3
DEFINES+= -DAM_PACKAGE_BGA
DEFINES+= -Dgcc

INCLUDES = -I$(SDKPATH)/CMSIS/AmbiqMicro/Include
INCLUDES+= -I$(SDKPATH)/devices
INCLUDES+= -I$(SDKPATH)/mcu/apollo3
INCLUDES+= -I$(SDKPATH)/CMSIS/ARM/Include
INCLUDES+= -I$(SDKPATH)/utils
INCLUDES+= -I$(PROJECTPATH)/include/bsp

VPATH = ..

SRC = am_bsp.c
SRC += am_bsp_pins.c

CSRC = $(filter %.c,$(SRC))
ASRC = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=$(CONFIG)/%.o)
OBJS+= $(ASRC:%.s=$(CONFIG)/%.o)

DEPS = $(CSRC:%.c=$(CONFIG)/%.d)
DEPS+= $(ASRC:%.s=$(CONFIG)/%.d)

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections
CFLAGS+= -MMD -MP -std=c99 -Wall
# Libraries O3 for production, examples O0 for debug.
CFLAGS+= -O3
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)
CFLAGS+= 

# Additional user specified CFLAGS
CFLAGS+=$(EXTRA_CFLAGS)

ODFLAGS = -S

#### Rules ####
all: directories $(CONFIG)/$(TARGET).a

directories: $(CONFIG)

$(CONFIG):
	@mkdir -p $@

$(CONFIG)/%.o: %.c $(CONFIG)/%.d $(INCS)
	@echo " Compiling $(COMPILERNAME) $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@

$(CONFIG)/%.o: %.s $(CONFIG)/%.d $(INCS)
	@echo " Assembling $(COMPILERNAME) $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@

$(CONFIG)/$(TARGET).a: $(OBJS)
	@echo " Library $(COMPILERNAME) $@" ;\
	$(AR) rsvc $@ $(OBJS)

clean:
	@echo "Cleaning..." ;\
	$(RM) -f $(OBJS) $(DEPS) \
	    $(CONFIG)/$(TARGET).a

$(CONFIG)/%.d: ;

# Automatically include any generated dependencies
-include $(DEPS)
endif
.PHONY: all clean directories