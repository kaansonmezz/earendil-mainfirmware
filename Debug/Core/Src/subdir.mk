################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ack_manager.c \
../Core/Src/activity_light.c \
../Core/Src/app_main.c \
../Core/Src/command_handler.c \
../Core/Src/control_mode.c \
../Core/Src/i2c_scanner.c \
../Core/Src/imu_mpu9250.c \
../Core/Src/logger.c \
../Core/Src/mag_qmc5883p.c \
../Core/Src/main.c \
../Core/Src/motion_controller.c \
../Core/Src/motor_dispatcher.c \
../Core/Src/motor_link.c \
../Core/Src/motor_protocol.c \
../Core/Src/motor_tx_dma.c \
../Core/Src/motor_uart_dma.c \
../Core/Src/operating_mode.c \
../Core/Src/safety_manager.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/terminal_if.c \
../Core/Src/terminal_parser.c 

OBJS += \
./Core/Src/ack_manager.o \
./Core/Src/activity_light.o \
./Core/Src/app_main.o \
./Core/Src/command_handler.o \
./Core/Src/control_mode.o \
./Core/Src/i2c_scanner.o \
./Core/Src/imu_mpu9250.o \
./Core/Src/logger.o \
./Core/Src/mag_qmc5883p.o \
./Core/Src/main.o \
./Core/Src/motion_controller.o \
./Core/Src/motor_dispatcher.o \
./Core/Src/motor_link.o \
./Core/Src/motor_protocol.o \
./Core/Src/motor_tx_dma.o \
./Core/Src/motor_uart_dma.o \
./Core/Src/operating_mode.o \
./Core/Src/safety_manager.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/terminal_if.o \
./Core/Src/terminal_parser.o 

C_DEPS += \
./Core/Src/ack_manager.d \
./Core/Src/activity_light.d \
./Core/Src/app_main.d \
./Core/Src/command_handler.d \
./Core/Src/control_mode.d \
./Core/Src/i2c_scanner.d \
./Core/Src/imu_mpu9250.d \
./Core/Src/logger.d \
./Core/Src/mag_qmc5883p.d \
./Core/Src/main.d \
./Core/Src/motion_controller.d \
./Core/Src/motor_dispatcher.d \
./Core/Src/motor_link.d \
./Core/Src/motor_protocol.d \
./Core/Src/motor_tx_dma.d \
./Core/Src/motor_uart_dma.d \
./Core/Src/operating_mode.d \
./Core/Src/safety_manager.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/terminal_if.d \
./Core/Src/terminal_parser.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ack_manager.cyclo ./Core/Src/ack_manager.d ./Core/Src/ack_manager.o ./Core/Src/ack_manager.su ./Core/Src/activity_light.cyclo ./Core/Src/activity_light.d ./Core/Src/activity_light.o ./Core/Src/activity_light.su ./Core/Src/app_main.cyclo ./Core/Src/app_main.d ./Core/Src/app_main.o ./Core/Src/app_main.su ./Core/Src/command_handler.cyclo ./Core/Src/command_handler.d ./Core/Src/command_handler.o ./Core/Src/command_handler.su ./Core/Src/control_mode.cyclo ./Core/Src/control_mode.d ./Core/Src/control_mode.o ./Core/Src/control_mode.su ./Core/Src/i2c_scanner.cyclo ./Core/Src/i2c_scanner.d ./Core/Src/i2c_scanner.o ./Core/Src/i2c_scanner.su ./Core/Src/imu_mpu9250.cyclo ./Core/Src/imu_mpu9250.d ./Core/Src/imu_mpu9250.o ./Core/Src/imu_mpu9250.su ./Core/Src/logger.cyclo ./Core/Src/logger.d ./Core/Src/logger.o ./Core/Src/logger.su ./Core/Src/mag_qmc5883p.cyclo ./Core/Src/mag_qmc5883p.d ./Core/Src/mag_qmc5883p.o ./Core/Src/mag_qmc5883p.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion_controller.cyclo ./Core/Src/motion_controller.d ./Core/Src/motion_controller.o ./Core/Src/motion_controller.su ./Core/Src/motor_dispatcher.cyclo ./Core/Src/motor_dispatcher.d ./Core/Src/motor_dispatcher.o ./Core/Src/motor_dispatcher.su ./Core/Src/motor_link.cyclo ./Core/Src/motor_link.d ./Core/Src/motor_link.o ./Core/Src/motor_link.su ./Core/Src/motor_protocol.cyclo ./Core/Src/motor_protocol.d ./Core/Src/motor_protocol.o ./Core/Src/motor_protocol.su ./Core/Src/motor_tx_dma.cyclo ./Core/Src/motor_tx_dma.d ./Core/Src/motor_tx_dma.o ./Core/Src/motor_tx_dma.su ./Core/Src/motor_uart_dma.cyclo ./Core/Src/motor_uart_dma.d ./Core/Src/motor_uart_dma.o ./Core/Src/motor_uart_dma.su ./Core/Src/operating_mode.cyclo ./Core/Src/operating_mode.d ./Core/Src/operating_mode.o ./Core/Src/operating_mode.su ./Core/Src/safety_manager.cyclo ./Core/Src/safety_manager.d ./Core/Src/safety_manager.o ./Core/Src/safety_manager.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/terminal_if.cyclo ./Core/Src/terminal_if.d ./Core/Src/terminal_if.o ./Core/Src/terminal_if.su ./Core/Src/terminal_parser.cyclo ./Core/Src/terminal_parser.d ./Core/Src/terminal_parser.o ./Core/Src/terminal_parser.su

.PHONY: clean-Core-2f-Src

