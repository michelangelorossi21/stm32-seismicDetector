################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/hts221/hts221.c 

OBJS += \
./Drivers/BSP/Components/hts221/hts221.o 

C_DEPS += \
./Drivers/BSP/Components/hts221/hts221.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/hts221/%.o Drivers/BSP/Components/hts221/%.su Drivers/BSP/Components/hts221/%.cyclo: ../Drivers/BSP/Components/hts221/%.c Drivers/BSP/Components/hts221/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"D:/febom/Documents/Ingegneria Informatica UNIMORE - MN/2 Anno/Fondamenti di Elettronica/Lab 2/Progetto_esame/Drivers/BSP/B-L475E-IOT01" -I"D:/febom/Documents/Ingegneria Informatica UNIMORE - MN/2 Anno/Fondamenti di Elettronica/Lab 2/Progetto_esame/Drivers/BSP/Components/Common" -I"D:/febom/Documents/Ingegneria Informatica UNIMORE - MN/2 Anno/Fondamenti di Elettronica/Lab 2/Progetto_esame/Drivers/BSP/Components/hts221" -I"D:/febom/Documents/Ingegneria Informatica UNIMORE - MN/2 Anno/Fondamenti di Elettronica/Lab 2/Progetto_esame/Drivers/BSP/Components/lps22hb" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-hts221

clean-Drivers-2f-BSP-2f-Components-2f-hts221:
	-$(RM) ./Drivers/BSP/Components/hts221/hts221.cyclo ./Drivers/BSP/Components/hts221/hts221.d ./Drivers/BSP/Components/hts221/hts221.o ./Drivers/BSP/Components/hts221/hts221.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-hts221

