################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Radio/radio_board_if.c \
../Radio/radio_driver.c 

OBJS += \
./Radio/radio_board_if.o \
./Radio/radio_driver.o 

C_DEPS += \
./Radio/radio_board_if.d \
./Radio/radio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Radio/%.o Radio/%.su Radio/%.cyclo: ../Radio/%.c Radio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I"C:/Users/hardw/STM32CubeIDE/workspace_1.19.0/LoRa_P2P_LowPower/Radio" -I"C:/Users/hardw/STM32CubeIDE/workspace_1.19.0/LoRa_P2P_LowPower/Drivers" -I"C:/Users/hardw/OneDrive/Documentos/LORA/STM32Cube_FW_WL_V1.3.0/Drivers/BSP/STM32WLxx_Nucleo" -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Drivers/STM32WLxx_HAL_Driver" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Utils/misc" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Utils/conf" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Drivers/BSP/STM32WLxx_Nucleo" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Drivers/BSP" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Drivers" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Radio" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Utils" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/Placa2_Rx_Estacao/Debug" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Radio

clean-Radio:
	-$(RM) ./Radio/radio_board_if.cyclo ./Radio/radio_board_if.d ./Radio/radio_board_if.o ./Radio/radio_board_if.su ./Radio/radio_driver.cyclo ./Radio/radio_driver.d ./Radio/radio_driver.o ./Radio/radio_driver.su

.PHONY: clean-Radio

