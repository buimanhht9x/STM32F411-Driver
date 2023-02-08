################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002_Button.c 

OBJS += \
./Src/002_Button.o 

C_DEPS += \
./Src/002_Button.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F411VEHx -DSTM32F4 -c -I../Inc -I"F:/C n STM/STM_Udemy/STM32F411-Driver/stm32f4xx_driver_2/drivers/Inc" -I"F:/C n STM/STM_Udemy/STM32F411-Driver/stm32f4xx_driver_2/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/002_Button.d ./Src/002_Button.o ./Src/002_Button.su

.PHONY: clean-Src

