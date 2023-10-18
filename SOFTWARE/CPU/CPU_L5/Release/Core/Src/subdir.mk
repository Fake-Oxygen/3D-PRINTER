################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GCodes.c \
../Core/Src/MoveControl.c \
../Core/Src/Temperature.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/reader.c \
../Core/Src/stm32l5xx_hal_msp.c \
../Core/Src/stm32l5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l5xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

CPP_SRCS += \
../Core/Src/MiniPID.cpp \
../Core/Src/TMC2208.cpp \
../Core/Src/main.cpp 

C_DEPS += \
./Core/Src/GCodes.d \
./Core/Src/MoveControl.d \
./Core/Src/Temperature.d \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/reader.d \
./Core/Src/stm32l5xx_hal_msp.d \
./Core/Src/stm32l5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l5xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 

OBJS += \
./Core/Src/GCodes.o \
./Core/Src/MiniPID.o \
./Core/Src/MoveControl.o \
./Core/Src/TMC2208.o \
./Core/Src/Temperature.o \
./Core/Src/adc.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/reader.o \
./Core/Src/stm32l5xx_hal_msp.o \
./Core/Src/stm32l5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l5xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

CPP_DEPS += \
./Core/Src/MiniPID.d \
./Core/Src/TMC2208.d \
./Core/Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L562xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"E:/3D-PRINTER/SOFTWARE/CPU/CPU_L5/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32L562xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/GCodes.cyclo ./Core/Src/GCodes.d ./Core/Src/GCodes.o ./Core/Src/GCodes.su ./Core/Src/MiniPID.cyclo ./Core/Src/MiniPID.d ./Core/Src/MiniPID.o ./Core/Src/MiniPID.su ./Core/Src/MoveControl.cyclo ./Core/Src/MoveControl.d ./Core/Src/MoveControl.o ./Core/Src/MoveControl.su ./Core/Src/TMC2208.cyclo ./Core/Src/TMC2208.d ./Core/Src/TMC2208.o ./Core/Src/TMC2208.su ./Core/Src/Temperature.cyclo ./Core/Src/Temperature.d ./Core/Src/Temperature.o ./Core/Src/Temperature.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/reader.cyclo ./Core/Src/reader.d ./Core/Src/reader.o ./Core/Src/reader.su ./Core/Src/stm32l5xx_hal_msp.cyclo ./Core/Src/stm32l5xx_hal_msp.d ./Core/Src/stm32l5xx_hal_msp.o ./Core/Src/stm32l5xx_hal_msp.su ./Core/Src/stm32l5xx_it.cyclo ./Core/Src/stm32l5xx_it.d ./Core/Src/stm32l5xx_it.o ./Core/Src/stm32l5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l5xx.cyclo ./Core/Src/system_stm32l5xx.d ./Core/Src/system_stm32l5xx.o ./Core/Src/system_stm32l5xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src
