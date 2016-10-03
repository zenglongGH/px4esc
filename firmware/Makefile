#
# Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

PROJECT = io.px4.px4esc

HW_VERSION = 1
FW_VERSION_MAJOR = 1
FW_VERSION_MINOR = 0

#
# Application
#

CPPSRC = $(shell find src -name '*.cpp')
CSRC = $(shell find src -name '*.c')

UINCDIR = src                \
          src/os_config

UDEFS = -DFW_VERSION_MAJOR=$(FW_VERSION_MAJOR)           \
        -DFW_VERSION_MINOR=$(FW_VERSION_MINOR)           \
        -DHW_VERSION=$(HW_VERSION)                       \
        -DPRODUCT_ID_STRING=\"$(PROJECT)\"               \
        -DPRODUCT_NAME_STRING=\"PX4ESC\"

# MAVLink v1 compliance
UDEFS += -DCONFIG_PARAM_MAX_NAME_LENGTH=16

# Eigen library
USE_OPT += -Wno-deprecated-declarations
UINCDIR += eigen

# CMSIS DSP library
UDEFS += -DARM_MATH_CM4=1
UINCDIR += src/math/cmsis_dsp_lib/

#
# UAVCAN library
#

UDEFS += -DUAVCAN_STM32_TIMER_NUMBER=7                   \
         -DUAVCAN_STM32_NUM_IFACES=2                     \
         -DUAVCAN_CPP_VERSION=UAVCAN_CPP11               \
         -DUAVCAN_STM32_CHIBIOS=1

include libuavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

include libuavcan/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# ChibiOS
#

BUILD_CONFIG = 1

# 32K bootloader + 16K configuration storage
APPLICATION_OFFSET = 49152

DDEFS += -DCORTEX_VTOR_INIT=$(APPLICATION_OFFSET)       \
         -DCRT1_AREAS_NUMBER=0

# IRQ levels are assigned as follows:
#   0 - Motor Hardware Driver IRQ (ADC measurements and related processing)
#   1 - Low-priority FOC IRQ
#   2 - RTOS SVC IRQ
#   3 - CAN driver (normal RTOS IRQ)
#   Rest - Other RTOS IRQ
DDEFS += -DCORTEX_PRIORITY_SVCALL=2

USE_LTO := no

SERIAL_CLI_PORT_NUMBER = 3

# TODO: More warnings need to be enabled here. In order to avoid conflicts with third-party code, including the
# operating system itself, it needs to be compiled as separate static libraries with warnings disabled, and then
# linked with the application. This approach will require some modifications of the build system.
# See this thread: http://www.chibios.com/forum/viewtopic.php?t=1860
USE_OPT += -Wdouble-promotion -Wswitch-enum -Wfloat-equal -fno-strict-aliasing -fno-strict-overflow
USE_CPPOPT += -Wconversion -Wno-error=conversion -Wsign-promo -Wframe-larger-than=1024

RELEASE_OPT = -O2 -fomit-frame-pointer
DEBUG_OPT = -O2 -g3 -DDISABLE_WATCHDOG=1

LDSCRIPT = ld.ld

HW_VERSION_MAJOR_MINOR := $(HW_VERSION).0
FW_VERSION_MAJOR_MINOR_VCS_HASH := $(FW_VERSION_MAJOR).$(FW_VERSION_MINOR).$(GIT_HASH)
COMPOUND_IMAGE_FILE := $(PROJECT)-$(HW_VERSION_MAJOR_MINOR)-$(FW_VERSION_MAJOR_MINOR_VCS_HASH).compound.bin
BOOTLOADER_IMAGE := $(abspath bootloader.bin)

# This rule hook is defined in the ChibiOS build system
POST_MAKE_ALL_RULE_HOOK: build/$(PROJECT).bin build/$(PROJECT).elf
	rm -rf build/*.application.bin build/*.compound.bin
	cd build && dd if=/dev/zero bs=$(APPLICATION_OFFSET) count=1 | tr "\000" "\377" >padded_bootloader.tmp.bin
	cd build && dd if=$(BOOTLOADER_IMAGE) of=padded_bootloader.tmp.bin conv=notrunc
	cd build && cat padded_bootloader.tmp.bin $(PROJECT).bin >$(COMPOUND_IMAGE_FILE)
	cd build && ../zubax_chibios/tools/make_boot_descriptor.py $(PROJECT).bin $(PROJECT) $(HW_VERSION_MAJOR_MINOR) \
	                                                           --also-patch-descriptor-in=$(PROJECT).elf           \
	                                                           --also-patch-descriptor-in=$(COMPOUND_IMAGE_FILE)
	cd build && $(TOOLCHAIN_PREFIX)-objcopy --add-section bootloader=$(BOOTLOADER_IMAGE)   \
	                                        --set-section-flags bootloader=load,alloc      \
	                                        --change-section-address bootloader=0x08000000 \
	                                        $(PROJECT).elf compound.elf
	cd build && rm -f $(PROJECT).bin $(PROJECT).elf *.hex *.tmp.bin

upload: build/compound.elf
	./zubax_chibios/tools/blackmagic_flash.sh

include zubax_chibios/rules_stm32f446.mk
