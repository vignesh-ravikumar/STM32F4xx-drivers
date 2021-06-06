################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/014uartcase.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/014uartcase.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/014uartcase.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/014uartcase.o: ../Src/014uartcase.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/91883/Documents/STM32/STM32-wokspace/cortex-mx/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/014uartcase.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/91883/Documents/STM32/STM32-wokspace/cortex-mx/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/91883/Documents/STM32/STM32-wokspace/cortex-mx/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

