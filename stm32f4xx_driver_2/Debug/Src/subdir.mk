################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001_LED_Toggle.c 

OBJS += \
./Src/001_LED_Toggle.o 

C_DEPS += \
./Src/001_LED_Toggle.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F411VEHx -DSTM32F4 -c -I../Inc -I"D:/Manh/stm32_advance/Udemy/stm32f4xx_driver_2/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/001_LED_Toggle.d ./Src/001_LED_Toggle.o ./Src/001_LED_Toggle.su

.PHONY: clean-Src

