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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I"C:/Users/hardw/STM32CubeIDE/workspace_1.19.0/LoRa_P2P_LowPower/Radio" -I"C:/Users/hardw/STM32CubeIDE/workspace_1.19.0/LoRa_P2P_LowPower/Drivers" -I"C:/Users/hardw/OneDrive/Documentos/LORA/STM32Cube_FW_WL_V1.3.0/Drivers/BSP/STM32WLxx_Nucleo" -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Drivers/STM32WLxx_HAL_Driver" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Utils/misc" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Utils/conf" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Drivers/BSP/STM32WLxx_Nucleo" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Drivers/BSP" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Drivers" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Radio" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Utils" -I"/home/alisson/Documentos/GitHub/UHF_LoRa/Debug" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Utils-2f-misc

clean-Utils-2f-misc:
	-$(RM) ./Utils/misc/stm32_mem.cyclo ./Utils/misc/stm32_mem.d ./Utils/misc/stm32_mem.o ./Utils/misc/stm32_mem.su

.PHONY: clean-Utils-2f-misc

