################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/freertos.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/freertos.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/freertos.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Inc" -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Avell 1511/Desktop/Nova pasta/ProjetoFinal/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


