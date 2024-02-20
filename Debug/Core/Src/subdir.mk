################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LPS22HB.c \
../Core/Src/PCap04.c \
../Core/Src/Pc_Comm_App.c \
../Core/Src/hal_usb.c \
../Core/Src/hw_drivers.c \
../Core/Src/main.c \
../Core/Src/selector.c \
../Core/Src/sht3x.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/usartdll.c \
../Core/Src/utils.c 

OBJS += \
./Core/Src/LPS22HB.o \
./Core/Src/PCap04.o \
./Core/Src/Pc_Comm_App.o \
./Core/Src/hal_usb.o \
./Core/Src/hw_drivers.o \
./Core/Src/main.o \
./Core/Src/selector.o \
./Core/Src/sht3x.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/usartdll.o \
./Core/Src/utils.o 

C_DEPS += \
./Core/Src/LPS22HB.d \
./Core/Src/PCap04.d \
./Core/Src/Pc_Comm_App.d \
./Core/Src/hal_usb.d \
./Core/Src/hw_drivers.d \
./Core/Src/main.d \
./Core/Src/selector.d \
./Core/Src/sht3x.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/usartdll.d \
./Core/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/LPS22HB.cyclo ./Core/Src/LPS22HB.d ./Core/Src/LPS22HB.o ./Core/Src/LPS22HB.su ./Core/Src/PCap04.cyclo ./Core/Src/PCap04.d ./Core/Src/PCap04.o ./Core/Src/PCap04.su ./Core/Src/Pc_Comm_App.cyclo ./Core/Src/Pc_Comm_App.d ./Core/Src/Pc_Comm_App.o ./Core/Src/Pc_Comm_App.su ./Core/Src/hal_usb.cyclo ./Core/Src/hal_usb.d ./Core/Src/hal_usb.o ./Core/Src/hal_usb.su ./Core/Src/hw_drivers.cyclo ./Core/Src/hw_drivers.d ./Core/Src/hw_drivers.o ./Core/Src/hw_drivers.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/selector.cyclo ./Core/Src/selector.d ./Core/Src/selector.o ./Core/Src/selector.su ./Core/Src/sht3x.cyclo ./Core/Src/sht3x.d ./Core/Src/sht3x.o ./Core/Src/sht3x.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/usartdll.cyclo ./Core/Src/usartdll.d ./Core/Src/usartdll.o ./Core/Src/usartdll.su ./Core/Src/utils.cyclo ./Core/Src/utils.d ./Core/Src/utils.o ./Core/Src/utils.su

.PHONY: clean-Core-2f-Src

