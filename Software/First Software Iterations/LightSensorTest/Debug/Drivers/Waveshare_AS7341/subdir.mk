################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Waveshare_AS7341/Waveshare_AS7341.c 

OBJS += \
./Drivers/Waveshare_AS7341/Waveshare_AS7341.o 

C_DEPS += \
./Drivers/Waveshare_AS7341/Waveshare_AS7341.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Waveshare_AS7341/%.o Drivers/Waveshare_AS7341/%.su: ../Drivers/Waveshare_AS7341/%.c Drivers/Waveshare_AS7341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Waveshare_AS7341

clean-Drivers-2f-Waveshare_AS7341:
	-$(RM) ./Drivers/Waveshare_AS7341/Waveshare_AS7341.d ./Drivers/Waveshare_AS7341/Waveshare_AS7341.o ./Drivers/Waveshare_AS7341/Waveshare_AS7341.su

.PHONY: clean-Drivers-2f-Waveshare_AS7341

