################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/alg_pid.c \
../Core/Src/can.c \
../Core/Src/cfg_pid_params.c \
../Core/Src/dma.c \
../Core/Src/drv_can.c \
../Core/Src/drv_comm.c \
../Core/Src/drv_uart.c \
../Core/Src/dvc_motor.c \
../Core/Src/dvc_motor_config.c \
../Core/Src/dvc_serialplot.c \
../Core/Src/dvc_timebase.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/alg_pid.o \
./Core/Src/can.o \
./Core/Src/cfg_pid_params.o \
./Core/Src/dma.o \
./Core/Src/drv_can.o \
./Core/Src/drv_comm.o \
./Core/Src/drv_uart.o \
./Core/Src/dvc_motor.o \
./Core/Src/dvc_motor_config.o \
./Core/Src/dvc_serialplot.o \
./Core/Src/dvc_timebase.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/alg_pid.d \
./Core/Src/can.d \
./Core/Src/cfg_pid_params.d \
./Core/Src/dma.d \
./Core/Src/drv_can.d \
./Core/Src/drv_comm.d \
./Core/Src/drv_uart.d \
./Core/Src/dvc_motor.d \
./Core/Src/dvc_motor_config.d \
./Core/Src/dvc_serialplot.d \
./Core/Src/dvc_timebase.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/alg_pid.cyclo ./Core/Src/alg_pid.d ./Core/Src/alg_pid.o ./Core/Src/alg_pid.su ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/cfg_pid_params.cyclo ./Core/Src/cfg_pid_params.d ./Core/Src/cfg_pid_params.o ./Core/Src/cfg_pid_params.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/drv_can.cyclo ./Core/Src/drv_can.d ./Core/Src/drv_can.o ./Core/Src/drv_can.su ./Core/Src/drv_comm.cyclo ./Core/Src/drv_comm.d ./Core/Src/drv_comm.o ./Core/Src/drv_comm.su ./Core/Src/drv_uart.cyclo ./Core/Src/drv_uart.d ./Core/Src/drv_uart.o ./Core/Src/drv_uart.su ./Core/Src/dvc_motor.cyclo ./Core/Src/dvc_motor.d ./Core/Src/dvc_motor.o ./Core/Src/dvc_motor.su ./Core/Src/dvc_motor_config.cyclo ./Core/Src/dvc_motor_config.d ./Core/Src/dvc_motor_config.o ./Core/Src/dvc_motor_config.su ./Core/Src/dvc_serialplot.cyclo ./Core/Src/dvc_serialplot.d ./Core/Src/dvc_serialplot.o ./Core/Src/dvc_serialplot.su ./Core/Src/dvc_timebase.cyclo ./Core/Src/dvc_timebase.d ./Core/Src/dvc_timebase.o ./Core/Src/dvc_timebase.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

