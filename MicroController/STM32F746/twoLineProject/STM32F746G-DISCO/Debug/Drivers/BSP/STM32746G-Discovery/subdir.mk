################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.c 

OBJS += \
./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.o 

C_DEPS += \
./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32746G-Discovery/%.o Drivers/BSP/STM32746G-Discovery/%.su: ../Drivers/BSP/STM32746G-Discovery/%.c Drivers/BSP/STM32746G-Discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../X-CUBE-AI/App -I../X-CUBE-AI -I../X-CUBE-AI/Target -I../Core/Inc -I../Middlewares/ST/AI/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/BSP/STM32746G-Discovery -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32746G-2d-Discovery

clean-Drivers-2f-BSP-2f-STM32746G-2d-Discovery:
	-$(RM) ./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.d ./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.o ./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32746G-2d-Discovery

