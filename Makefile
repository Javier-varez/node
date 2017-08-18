TARGET = test_node

#######################
# Build configuration #
#######################

DEBUG = 1
OPT = -Og
STM32F4Cube_DIR = /home/javier/STM32Cube/Repository/STM32Cube_FW_F4_V1.16.0

####################
# Path declaration #
####################
# Sources

#SOURCES_DIR = \
#	src \
#	startup \
#	Node \
#	Node/LinkedList \
#	Node/Sensor \
#	Node/Sensor/Libs \
#	Node/HAL \
#	Node/Configuration \
#	Node/Comms_module \
#	Node/Comms_module/Libs \
#	Middlewares/Third_Party/FreeRTOS/Source \
#	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
#	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
#	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/ \

SOURCES_DIR = \
        src \
	startup \
        Node \
        Middlewares/Third_Party/FreeRTOS/Source \

# External sources
SOURCES_DIR += \
	$(STM32F4Cube_DIR)/Drivers/STM32F4xx_HAL_Driver/Src

# Build directory
BUILD_DIR = build

##########
# Source #
##########

C_SOURCES := $(shell find $(SOURCES_DIR) -name '*.c')
ASM_SOURCES = $(shell find $(SOURCES_DIR) -name '*.s')

############
# binaries #
############
BINPATH = /home/javier/gcc-arm-none-eabi-6-2017-q2-update/bin/
PREFIX = arm-none-eabi-
CC = $(BINPATH)$(PREFIX)gcc
AS = $(BINPATH)$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)$(PREFIX)objcopy
AR = $(BINPATH)$(PREFIX)ar
SZ = $(BINPATH)$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

##########
# CFLAGS #
##########

# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for GCC
# AS defines
AS_DEFS =

# C defines
C_DEFS = \
	-DUSE_HAL_DRIVER \
	-DSTM32F411VETx \
	-DSTM32F4 \
	-DSTM32 \
	-DSTM32F411E_DISCO \
	-DUSE_RTOS_SYSTICK \
	-DSTM32F411xE

# AS includes
AS_INCLUDES =

# C Includes
C_INCLUDES = \
	-Iinc \
	-INode \
	-INode/Comms_module \
	-INode/Comms_module/Libs \
	-INode/Configuration \
	-INode/HAL \
	-INode/LinkedList \
	-INode/Sensor \
	-INode/Sensor/Libs \
	-IMiddlewares/Third_Party/FreeRTOS/Source/include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
	-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \

C_INCLUDES += \
	-I$(STM32F4Cube_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc \
	-I$(STM32F4Cube_DIR)/Drivers/CMSIS/Include \
	-I$(STM32F4Cube_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = LinkerScript.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@


$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


#####################
# Debuging commands #
#####################


DEBUG_INTF := -f "interface/stlink-v2.cfg"
TARGET_MCU := -f "target/stm32f4x.cfg"
RESET_CFG := -c "reset_config srst_only srst_nogate"
OPENOCD := openocd $(DEBUG_INTF) $(TARGET_MCU) $(RESET_CFG)

PROGRAM_CMD := -c "program $(BUILD_DIR)/$(TARGET).elf exit"

flash: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) $(PROGRAM_CMD)
