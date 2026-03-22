################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/neo6m/gps.c 

OBJS += \
./Drivers/neo6m/gps.o 

C_DEPS += \
./Drivers/neo6m/gps.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/neo6m/%.o Drivers/neo6m/%.su Drivers/neo6m/%.cyclo: ../Drivers/neo6m/%.c Drivers/neo6m/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/ssd1306" -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/sdcard" -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/neo6m" -I"/home/theundefinedengineer/Workspace/BBBv2/BBBv2/Drivers/mpu6050" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-neo6m

clean-Drivers-2f-neo6m:
	-$(RM) ./Drivers/neo6m/gps.cyclo ./Drivers/neo6m/gps.d ./Drivers/neo6m/gps.o ./Drivers/neo6m/gps.su

.PHONY: clean-Drivers-2f-neo6m

