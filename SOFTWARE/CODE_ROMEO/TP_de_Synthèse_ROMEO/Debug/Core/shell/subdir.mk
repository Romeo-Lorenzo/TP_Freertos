################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/shell/drv_uart2.c \
../Core/shell/shell.c \
../Core/shell/shell_driver.c 

OBJS += \
./Core/shell/drv_uart2.o \
./Core/shell/shell.o \
./Core/shell/shell_driver.o 

C_DEPS += \
./Core/shell/drv_uart2.d \
./Core/shell/shell.d \
./Core/shell/shell_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/shell/%.o Core/shell/%.su Core/shell/%.cyclo: ../Core/shell/%.c Core/shell/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/lorenzo/Documents/GitHub/TP_Freertos/SOFTWARE/CODE_ROMEO/TP_de_Synthèse_ROMEO/Core/shell" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-shell

clean-Core-2f-shell:
	-$(RM) ./Core/shell/drv_uart2.cyclo ./Core/shell/drv_uart2.d ./Core/shell/drv_uart2.o ./Core/shell/drv_uart2.su ./Core/shell/shell.cyclo ./Core/shell/shell.d ./Core/shell/shell.o ./Core/shell/shell.su ./Core/shell/shell_driver.cyclo ./Core/shell/shell_driver.d ./Core/shell/shell_driver.o ./Core/shell/shell_driver.su

.PHONY: clean-Core-2f-shell

