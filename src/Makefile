# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support 
# ----------------------------------------------------------------------------
# Copyright (c) 2008, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

# 	Makefile for compiling the USB CDC serial project

#-------------------------------------------------------------------------------
#		User-modifiable options
#-------------------------------------------------------------------------------

# Chip & board used for compilation
# (can be overriden by adding CHIP=chip and BOARD=board to the command-line)
CHIP  = at91sam3u4
BOARD = 4pi

# Trace level used for compilation
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0
TRACE_LEVEL = 3

# Optimization level, put in comment for debugging
OPTIMIZATION = -Os

# AT91 library directory
AT91LIB = ../at91lib

# External library
EXT_LIBS= ../external_libs

# Output file basename
OUTPUT = Sprinter-$(BOARD)-$(CHIP)

# Compile with chip specific features
include $(AT91LIB)/boards/$(BOARD)/$(CHIP)/chip.mak

# Compile for all memories available on the board (this sets $(MEMORIES))
include $(AT91LIB)/boards/$(BOARD)/board.mak

# Output directories
BIN = bin
OBJ = obj
DEPDIR = .deps

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Compilation tools
CC = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy

# Flags
INCLUDES += -I$(AT91LIB)/boards/$(BOARD)
INCLUDES += -I$(AT91LIB)/peripherals
INCLUDES += -I$(AT91LIB)/components
INCLUDES += -I$(AT91LIB)/usb/device
INCLUDES += -I$(AT91LIB)/memories
INCLUDES += -I$(AT91LIB)/drivers
INCLUDES += -I$(AT91LIB)
INCLUDES += -I$(EXT_LIBS)
INCLUDES += -I$(EXT_LIBS)/fat

ifeq ($(CHIP_CORE), cortexm3)
TARGET_OPTS = -mcpu=cortex-m3 -mthumb
else
TARGET_OPTS = 
endif

CFLAGS += $(TARGET_OPTS)
CFLAGS += -Wall -mlong-calls -ffunction-sections
CFLAGS += -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -DTRACE_LEVEL=$(TRACE_LEVEL)
ASFLAGS = $(TARGET_OPTS) -Wall -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -D__ASSEMBLY__
LDFLAGS = -g $(OPTIMIZATION) -nostartfiles $(TARGET_OPTS) -Wl,--gc-sections

#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found
USB = $(AT91LIB)/usb
UTILITY = $(AT91LIB)/utility
PERIPH = $(AT91LIB)/peripherals
BOARDS = $(AT91LIB)/boards
MEMORY = $(AT91LIB)/memories
DRV = $(AT91LIB)/drivers
FATFS = $(EXT_LIBS)/fat/fatfs

VPATH += $(USB)/device/cdc-serial
VPATH += $(USB)/device/core
VPATH += $(USB)/common/core
VPATH += $(USB)/common/cdc
VPATH += $(UTILITY)
VPATH += $(PERIPH)/dbgu
VPATH += $(PERIPH)/irq
VPATH += $(PERIPH)/usart
VPATH += $(PERIPH)/adc
VPATH += $(PERIPH)/tc
#VPATH += $(PERIPH)/spi
VPATH += $(PERIPH)/systick
VPATH += $(PERIPH)/pio
VPATH += $(PERIPH)/pmc
VPATH += $(PERIPH)/cp15
VPATH += $(PERIPH)/dma
VPATH += $(PERIPH)/mci
VPATH += $(BOARDS)/$(BOARD)
VPATH += $(BOARDS)/$(BOARD)/$(CHIP)
VPATH += $(EXT_LIBS)/cmsis
VPATH += $(PERIPH)/systick
VPATH += $(PERIPH)/eefc
VPATH += $(MEMORY)
VPATH += $(MEMORY)/flash
VPATH += $(MEMORY)/sdmmc/
VPATH += $(DRV)/dmad
VPATH += $(FATFS)/src
VPATH += $(FATFS)/src/option



# C source files
C_SOURCES += main.c
C_SOURCES += syscalls.c
C_SOURCES += CDCDSerialDriver.c
C_SOURCES += CDCDSerialDriverDescriptors.c
C_SOURCES += CDCSetControlLineStateRequest.c
C_SOURCES += CDCLineCoding.c
C_SOURCES += USBD_OTGHS.c
C_SOURCES += USBD_UDP.c
C_SOURCES += USBD_UDPHS.c
C_SOURCES += USBDDriver.c
C_SOURCES += USBDCallbacks_Initialized.c
C_SOURCES += USBDCallbacks_Reset.c
#C_SOURCES += USBDCallbacks_Resumed.c
#C_SOURCES += USBDCallbacks_Suspended.c
C_SOURCES += USBDDriverCb_CfgChanged.c
C_SOURCES += USBDDriverCb_IfSettingChanged.c
C_SOURCES += USBSetAddressRequest.c
C_SOURCES += USBGenericDescriptor.c
C_SOURCES += USBInterfaceRequest.c
C_SOURCES += USBGenericRequest.c
C_SOURCES += USBGetDescriptorRequest.c
C_SOURCES += USBSetConfigurationRequest.c
C_SOURCES += util.c
C_SOURCES += stdio.c
C_SOURCES += systick.c
C_SOURCES += serial.c
C_SOURCES += motoropts.c
C_SOURCES += tc.c
C_SOURCES += USBFeatureRequest.c
C_SOURCES += USBEndpointDescriptor.c
C_SOURCES += USBConfigurationDescriptor.c
C_SOURCES += led.c
C_SOURCES += adc12.c
C_SOURCES += samadc.c
#C_SOURCES += spi.c
C_SOURCES += string.c
C_SOURCES += dbgu.c
C_SOURCES += math.c
C_SOURCES += usart.c
C_SOURCES += pio.c
C_SOURCES += pio_it.c
C_SOURCES += pmc.c
C_SOURCES += trace.c
C_SOURCES += board_memories.c
C_SOURCES += board_lowlevel.c
C_SOURCES += heaters.c
C_SOURCES += arc_func.c
C_SOURCES += planner.c
C_SOURCES += stepper_control.c
C_SOURCES += parameters.c
C_SOURCES += eefc.c
C_SOURCES += flashd_eefc.c
C_SOURCES += sdcard.c
C_SOURCES += gcode_parser.c
C_SOURCES += globals.c

#media
C_SOURCES += Media.c
C_SOURCES += MEDSdcard.c
C_SOURCES += sdmmc_mci.c

#fatfs
C_SOURCES += diskio.c
C_SOURCES += ff.c
C_SOURCES += unicode.c

# C sources for different chips
ifeq ($(CHIP_CORE), cortexm3)
C_SOURCES += nvic.c
C_SOURCES += exceptions.c
C_SOURCES += board_cstartup_gnu.c
C_SOURCES += core_cm3.c
else
C_SOURCES += aic.c
C_SOURCES += cp15.c
endif

ifeq ($(CHIP_IP_MCI), MCI_DMA)
C_SOURCES += dmad.c
C_SOURCES += dma.c
C_SOURCES += mci_hs.c
else
C_SOURCES += mci.c
endif


# Assembly source files
ifneq ($(CHIP_CORE), cortexm3)
ASM_SOURCES += board_cstartup.S
ASM_SOURCES += cp15_asm.S
endif

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ) $(DEPDIR):
	mkdir $@

define RULES
C_OBJECTS_$(1) = $(C_SOURCES:%.c=$(OBJ)/$(1)_%.o)
ASM_OBJECTS_$(1) = $(ASM_SOURCES:%.S=$(OBJ)/$(1)_%.o)

$(1): $(OUTPUT)-$(1).bin

$(OUTPUT)-$(1).elf: $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1)) | $(BIN)
	$(CC) $(LDFLAGS) -T"$(AT91LIB)/boards/$(BOARD)/$(CHIP)/$(1).lds" -o $$@ $$^ -lm
	$(SIZE) $$^ $$@

$$(C_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile | $(OBJ) $(DEPDIR)
	$(CC) $(CFLAGS) -D$(1) -MD -MP -MF .deps/$$(notdir $$(@:.o=.d)) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.S Makefile | $(OBJ)
	$(CC) $(ASFLAGS) -D$(1) -c -o $$@ $$<

debug_$(1): $(1)
	perl ../resources/gdb/debug.pl $(OUTPUT)-$(1).elf

-include $(C_SOURCES:%.c=$(DEPDIR)/$(1)_%.d)
endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -f $(OBJ)/*.o $(BIN)/*.bin $(BIN)/*.elf $(DEPDIR)/*.d

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

.PHONY: all $(MEMORIES)
