################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../X-CUBE-AI/App/network.c \
../X-CUBE-AI/App/network_data.c \
../X-CUBE-AI/App/network_data_params.c \
../X-CUBE-AI/App/twoline.c \
../X-CUBE-AI/App/twoline_data.c \
../X-CUBE-AI/App/twoline_data_params.c 

OBJS += \
./X-CUBE-AI/App/network.o \
./X-CUBE-AI/App/network_data.o \
./X-CUBE-AI/App/network_data_params.o \
./X-CUBE-AI/App/twoline.o \
./X-CUBE-AI/App/twoline_data.o \
./X-CUBE-AI/App/twoline_data_params.o 

C_DEPS += \
./X-CUBE-AI/App/network.d \
./X-CUBE-AI/App/network_data.d \
./X-CUBE-AI/App/network_data_params.d \
./X-CUBE-AI/App/twoline.d \
./X-CUBE-AI/App/twoline_data.d \
./X-CUBE-AI/App/twoline_data_params.d 


# Each subdirectory must supply rules for building sources it contributes
X-CUBE-AI/App/%.o X-CUBE-AI/App/%.su X-CUBE-AI/App/%.cyclo: ../X-CUBE-AI/App/%.c X-CUBE-AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -I../Middlewares/ST/AI/Inc -I../X-CUBE-AI/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-X-2d-CUBE-2d-AI-2f-App

clean-X-2d-CUBE-2d-AI-2f-App:
	-$(RM) ./X-CUBE-AI/App/network.cyclo ./X-CUBE-AI/App/network.d ./X-CUBE-AI/App/network.o ./X-CUBE-AI/App/network.su ./X-CUBE-AI/App/network_data.cyclo ./X-CUBE-AI/App/network_data.d ./X-CUBE-AI/App/network_data.o ./X-CUBE-AI/App/network_data.su ./X-CUBE-AI/App/network_data_params.cyclo ./X-CUBE-AI/App/network_data_params.d ./X-CUBE-AI/App/network_data_params.o ./X-CUBE-AI/App/network_data_params.su ./X-CUBE-AI/App/twoline.cyclo ./X-CUBE-AI/App/twoline.d ./X-CUBE-AI/App/twoline.o ./X-CUBE-AI/App/twoline.su ./X-CUBE-AI/App/twoline_data.cyclo ./X-CUBE-AI/App/twoline_data.d ./X-CUBE-AI/App/twoline_data.o ./X-CUBE-AI/App/twoline_data.su ./X-CUBE-AI/App/twoline_data_params.cyclo ./X-CUBE-AI/App/twoline_data_params.d ./X-CUBE-AI/App/twoline_data_params.o ./X-CUBE-AI/App/twoline_data_params.su

.PHONY: clean-X-2d-CUBE-2d-AI-2f-App

