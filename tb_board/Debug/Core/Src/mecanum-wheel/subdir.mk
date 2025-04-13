################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/mecanum-wheel/encoder.c \
../Core/Src/mecanum-wheel/movement.c \
../Core/Src/mecanum-wheel/pid-mecanum.c 

OBJS += \
./Core/Src/mecanum-wheel/encoder.o \
./Core/Src/mecanum-wheel/movement.o \
./Core/Src/mecanum-wheel/pid-mecanum.o 

C_DEPS += \
./Core/Src/mecanum-wheel/encoder.d \
./Core/Src/mecanum-wheel/movement.d \
./Core/Src/mecanum-wheel/pid-mecanum.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/mecanum-wheel/%.o Core/Src/mecanum-wheel/%.su Core/Src/mecanum-wheel/%.cyclo: ../Core/Src/mecanum-wheel/%.c Core/Src/mecanum-wheel/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-mecanum-2d-wheel

clean-Core-2f-Src-2f-mecanum-2d-wheel:
	-$(RM) ./Core/Src/mecanum-wheel/encoder.cyclo ./Core/Src/mecanum-wheel/encoder.d ./Core/Src/mecanum-wheel/encoder.o ./Core/Src/mecanum-wheel/encoder.su ./Core/Src/mecanum-wheel/movement.cyclo ./Core/Src/mecanum-wheel/movement.d ./Core/Src/mecanum-wheel/movement.o ./Core/Src/mecanum-wheel/movement.su ./Core/Src/mecanum-wheel/pid-mecanum.cyclo ./Core/Src/mecanum-wheel/pid-mecanum.d ./Core/Src/mecanum-wheel/pid-mecanum.o ./Core/Src/mecanum-wheel/pid-mecanum.su

.PHONY: clean-Core-2f-Src-2f-mecanum-2d-wheel

