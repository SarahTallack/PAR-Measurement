################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DEV_Config/DEV_Config.c 

OBJS += \
./Drivers/DEV_Config/DEV_Config.o 

C_DEPS += \
./Drivers/DEV_Config/DEV_Config.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DEV_Config/%.o Drivers/DEV_Config/%.su: ../Drivers/DEV_Config/%.c Drivers/DEV_Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-DEV_Config

clean-Drivers-2f-DEV_Config:
	-$(RM) ./Drivers/DEV_Config/DEV_Config.d ./Drivers/DEV_Config/DEV_Config.o ./Drivers/DEV_Config/DEV_Config.su

.PHONY: clean-Drivers-2f-DEV_Config

