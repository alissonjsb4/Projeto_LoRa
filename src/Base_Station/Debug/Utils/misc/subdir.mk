################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utils/misc/stm32_mem.c 

OBJS += \
./Utils/misc/stm32_mem.o 

C_DEPS += \
./Utils/misc/stm32_mem.d 


# Each subdirectory must supply rules for building sources it contributes
Utils/misc/%.o Utils/misc/%.su Utils/misc/%.cyclo: ../Utils/misc/%.c Utils/misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I"/home/danilo-alencar/Documents/GitHub/Projeto_LoRa/Firmware/LoRa_Base/Drivers/BSP/STM32WLxx_Nucleo" -I"/home/danilo-alencar/Documents/GitHub/Projeto_LoRa/Firmware/LoRa_Base/Radio" -I"/home/danilo-alencar/Documents/GitHub/Projeto_LoRa/Firmware/LoRa_Base/Utils" -I"/home/danilo-alencar/Documents/GitHub/Projeto_LoRa/Firmware/LoRa_Base/Utils/misc" -I"/home/danilo-alencar/Documents/GitHub/Projeto_LoRa/Firmware/LoRa_Base/Utils/conf" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Utils-2f-misc

clean-Utils-2f-misc:
	-$(RM) ./Utils/misc/stm32_mem.cyclo ./Utils/misc/stm32_mem.d ./Utils/misc/stm32_mem.o ./Utils/misc/stm32_mem.su

.PHONY: clean-Utils-2f-misc

