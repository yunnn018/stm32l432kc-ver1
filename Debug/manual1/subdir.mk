################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../manual1/control1.c 

OBJS += \
./manual1/control1.o 

C_DEPS += \
./manual1/control1.d 


# Each subdirectory must supply rules for building sources it contributes
manual1/%.o manual1/%.su manual1/%.cyclo: ../manual1/%.c manual1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-manual1

clean-manual1:
	-$(RM) ./manual1/control1.cyclo ./manual1/control1.d ./manual1/control1.o ./manual1/control1.su

.PHONY: clean-manual1

