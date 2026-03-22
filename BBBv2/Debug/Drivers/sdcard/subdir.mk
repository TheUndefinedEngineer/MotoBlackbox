################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/sdcard/sdcard.c 

OBJS += \
./Drivers/sdcard/sdcard.o 

C_DEPS += \
./Drivers/sdcard/sdcard.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/sdcard/%.o Drivers/sdcard/%.su Drivers/sdcard/%.cyclo: ../Drivers/sdcard/%.c Drivers/sdcard/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/ssd1306" -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/sdcard" -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/neo6m" -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/mpu6050" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-sdcard

clean-Drivers-2f-sdcard:
	-$(RM) ./Drivers/sdcard/sdcard.cyclo ./Drivers/sdcard/sdcard.d ./Drivers/sdcard/sdcard.o ./Drivers/sdcard/sdcard.su

.PHONY: clean-Drivers-2f-sdcard

