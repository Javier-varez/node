################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f411xe.s 

OBJS += \
./startup/startup_stm32f411xe.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"/home/javier/workspace/stm32f411e-disco_hal_lib" -I"/home/javier/workspace/test_node/Node" -I"/home/javier/workspace/test_node/inc" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/HAL_Driver/Inc" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/HAL_Driver/Inc/Legacy" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Fonts" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/STM32F411E-Discovery" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/Common" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/lis302dl" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/otm8009a" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/ov2640" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/exc7200" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/n25q256a" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/l3gd20" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/lis3dsh" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/s5k5cag" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/stmpe1600" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/n25q512a" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/cs43l22" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/st7735" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/ft6x06" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/stmpe811" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/wm8994" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/ili9325" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/n25q128a" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/ili9341" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/ampire640480" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/ts3510" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/ampire480272" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/mfxstm32l152" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/s25fl512s" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/Utilities/Components/lsm303dlhc" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/CMSIS/device" -I"/home/javier/workspace/stm32f411e-disco_hal_lib/CMSIS/core" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


