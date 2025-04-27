################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/actuator.c \
../Core/Src/adc.c \
../Core/Src/ai_comm.c \
../Core/Src/buzzer_control.c \
../Core/Src/can_ibis.c \
../Core/Src/control_omni_angle.c \
../Core/Src/control_speed.c \
../Core/Src/control_theta.c \
../Core/Src/dma.c \
../Core/Src/error.c \
../Core/Src/fdcan.c \
../Core/Src/gpio.c \
../Core/Src/icm20602_spi.c \
../Core/Src/keeper.c \
../Core/Src/main.c \
../Core/Src/odom.c \
../Core/Src/omni_wheel.c \
../Core/Src/ring_buffer.c \
../Core/Src/robot_control.c \
../Core/Src/spi.c \
../Core/Src/state_func.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/stop_state_control.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/util.c 

OBJS += \
./Core/Src/actuator.o \
./Core/Src/adc.o \
./Core/Src/ai_comm.o \
./Core/Src/buzzer_control.o \
./Core/Src/can_ibis.o \
./Core/Src/control_omni_angle.o \
./Core/Src/control_speed.o \
./Core/Src/control_theta.o \
./Core/Src/dma.o \
./Core/Src/error.o \
./Core/Src/fdcan.o \
./Core/Src/gpio.o \
./Core/Src/icm20602_spi.o \
./Core/Src/keeper.o \
./Core/Src/main.o \
./Core/Src/odom.o \
./Core/Src/omni_wheel.o \
./Core/Src/ring_buffer.o \
./Core/Src/robot_control.o \
./Core/Src/spi.o \
./Core/Src/state_func.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/stop_state_control.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/util.o 

C_DEPS += \
./Core/Src/actuator.d \
./Core/Src/adc.d \
./Core/Src/ai_comm.d \
./Core/Src/buzzer_control.d \
./Core/Src/can_ibis.d \
./Core/Src/control_omni_angle.d \
./Core/Src/control_speed.d \
./Core/Src/control_theta.d \
./Core/Src/dma.d \
./Core/Src/error.d \
./Core/Src/fdcan.d \
./Core/Src/gpio.d \
./Core/Src/icm20602_spi.d \
./Core/Src/keeper.d \
./Core/Src/main.d \
./Core/Src/odom.d \
./Core/Src/omni_wheel.d \
./Core/Src/ring_buffer.d \
./Core/Src/robot_control.d \
./Core/Src/spi.d \
./Core/Src/state_func.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/stop_state_control.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/util.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu17 -g3 -DUSE_HAL_DRIVER -DSTM32G474xx -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/actuator.cyclo ./Core/Src/actuator.d ./Core/Src/actuator.o ./Core/Src/actuator.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/ai_comm.cyclo ./Core/Src/ai_comm.d ./Core/Src/ai_comm.o ./Core/Src/ai_comm.su ./Core/Src/buzzer_control.cyclo ./Core/Src/buzzer_control.d ./Core/Src/buzzer_control.o ./Core/Src/buzzer_control.su ./Core/Src/can_ibis.cyclo ./Core/Src/can_ibis.d ./Core/Src/can_ibis.o ./Core/Src/can_ibis.su ./Core/Src/control_omni_angle.cyclo ./Core/Src/control_omni_angle.d ./Core/Src/control_omni_angle.o ./Core/Src/control_omni_angle.su ./Core/Src/control_speed.cyclo ./Core/Src/control_speed.d ./Core/Src/control_speed.o ./Core/Src/control_speed.su ./Core/Src/control_theta.cyclo ./Core/Src/control_theta.d ./Core/Src/control_theta.o ./Core/Src/control_theta.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/error.cyclo ./Core/Src/error.d ./Core/Src/error.o ./Core/Src/error.su ./Core/Src/fdcan.cyclo ./Core/Src/fdcan.d ./Core/Src/fdcan.o ./Core/Src/fdcan.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/icm20602_spi.cyclo ./Core/Src/icm20602_spi.d ./Core/Src/icm20602_spi.o ./Core/Src/icm20602_spi.su ./Core/Src/keeper.cyclo ./Core/Src/keeper.d ./Core/Src/keeper.o ./Core/Src/keeper.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/odom.cyclo ./Core/Src/odom.d ./Core/Src/odom.o ./Core/Src/odom.su ./Core/Src/omni_wheel.cyclo ./Core/Src/omni_wheel.d ./Core/Src/omni_wheel.o ./Core/Src/omni_wheel.su ./Core/Src/ring_buffer.cyclo ./Core/Src/ring_buffer.d ./Core/Src/ring_buffer.o ./Core/Src/ring_buffer.su ./Core/Src/robot_control.cyclo ./Core/Src/robot_control.d ./Core/Src/robot_control.o ./Core/Src/robot_control.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/state_func.cyclo ./Core/Src/state_func.d ./Core/Src/state_func.o ./Core/Src/state_func.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/stop_state_control.cyclo ./Core/Src/stop_state_control.d ./Core/Src/stop_state_control.o ./Core/Src/stop_state_control.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/util.cyclo ./Core/Src/util.d ./Core/Src/util.o ./Core/Src/util.su

.PHONY: clean-Core-2f-Src

