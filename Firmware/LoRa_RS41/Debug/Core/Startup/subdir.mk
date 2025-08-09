################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32wl55jcix.s 

OBJS += \
./Core/Startup/startup_stm32wl55jcix.o 

S_DEPS += \
./Core/Startup/startup_stm32wl55jcix.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Radio" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Utils" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Utils/misc" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Utils/conf" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Drivers/BSP/STM32WLxx_Nucleo" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Drivers/CMSIS" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Drivers/STM32WLxx_HAL_Driver" -I"/home/alisson/STM32CubeIDE/novo_workspace_limpo/LoRa_RS41/Core/Inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32wl55jcix.d ./Core/Startup/startup_stm32wl55jcix.o

.PHONY: clean-Core-2f-Startup

