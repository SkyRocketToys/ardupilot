######################################
# target
######################################
TARGET = ChibiOS

######################################
# building variables
######################################
# debug build?
RELEASE = 0

#######################################
# pathes
#######################################
# Build path
PROJECT = ch
SERIAL_CLI_PORT_NUMBER = 1
HWDEF = $(AP_HAL)/hwdef

# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
# HAL-OSAL files
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
include $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_F412ZG/board.mk


USE_FPU = hard
MCU = cortex-m4


ifndef SERIAL_CLI_PORT_NUMBER
    $(error SERIAL_CLI_PORT_NUMBER must be assigned an integer value greater than zero)
endif

UDEFS += -DSTDOUT_SD=SD$(SERIAL_CLI_PORT_NUMBER) -DSTDIN_SD=STDOUT_SD -DSERIAL_CLI_PORT_NUMBER=$(SERIAL_CLI_PORT_NUMBER)

UDEFS += -DCORTEX_ENABLE_WFI_IDLE=1 -DGF_CHIBIOS -DCHPRINTF_USE_FLOAT=1

USE_LINK_GC = yes
USE_THUMB ?= yes
USE_VERBOSE_COMPILE ?= yes

include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk


VARIOUSSRC = $(CHIBIOS)/os/various/syscalls.c            \
             $(CHIBIOS)/os/hal/lib/streams/chprintf.c    \
             $(CHIBIOS)/os/hal/lib/streams/memstreams.c

CSRC +=$(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC)

CPPSRC += $(CHCPPSRC)

ASMSRC =
ASMXSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR += $(CHIBIOS)/os/license \
         $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(TESTINC) \
         $(CHIBIOS)/os/various $(HWDEF) 

#
# OS optional components
#

BUILD_CHIBIOS_SHELL ?= 0
ifneq ($(BUILD_CHIBIOS_SHELL),0)
    VARIOUSSRC += $(CHIBIOS)/os/various/shell.c
endif

#
# Build configuration
#

NO_BUILTIN += -fno-builtin-printf -fno-builtin-fprintf  -fno-builtin-vprintf -fno-builtin-vfprintf -fno-builtin-puts

USE_OPT += -falign-functions=16 -U__STRICT_ANSI__ -fno-exceptions -fno-unwind-tables -fno-stack-protector \
           $(NO_BUILTIN) -fconserve-stack

# Explicit usage flags are needed for LTO:
USE_OPT += -u_port_lock -u_port_unlock -u_exit -u_kill -u_getpid -uchThdExit -u_printf_float

# Fixing float constants - otherwise the C++ standard library may fail to compile:
UDEFS += -fno-single-precision-constant

USE_COPT += -std=c99
USE_CPPOPT += -std=c++14 -fno-rtti -fno-exceptions -fno-threadsafe-statics

USE_OPT += -nodefaultlibs -lc -lgcc -lm -lnosys -lrdimon

RELEASE ?= 0
RELEASE_OPT ?= -Os -fomit-frame-pointer
DEBUG_OPT ?= -O1 -g3
ifneq ($(RELEASE),0)
    DDEFS += -DRELEASE_BUILD=1 -DNDEBUG=1
    DADEFS += -DRELEASE_BUILD=1 -DNDEBUG=1
    USE_OPT += $(RELEASE_OPT)
else
    DDEFS += -DDEBUG_BUILD=1
    DADEFS += -DDEBUG_BUILD=1
    USE_OPT += $(DEBUG_OPT)
endif

#
# Compiler options
#

TOOLCHAIN_PREFIX ?= arm-none-eabi
CC   = $(TOOLCHAIN_PREFIX)-gcc
CPPC = $(TOOLCHAIN_PREFIX)-g++
LD   = $(TOOLCHAIN_PREFIX)-g++
CP   = $(TOOLCHAIN_PREFIX)-objcopy
AS   = $(TOOLCHAIN_PREFIX)-g++ -x assembler-with-cpp
OD   = $(TOOLCHAIN_PREFIX)-objdump
SZ   = $(TOOLCHAIN_PREFIX)-size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT +=

# THUMB-specific options here
TOPT ?= -mthumb -DTHUMB=1

CWARN += -Wall -Wextra -Wstrict-prototypes
CPPWARN += -Wundef -Wall -Wextra -Werror

# asm statement fix
DDEFS += -Dasm=__asm

#
# End of user defines
##############################################################################
include $(HWDEF)/chibios_common.mk