################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/DigitalIoPin.cpp \
../src/Parser.cpp \
../src/Pen.cpp \
../src/cr_cpp_config.cpp \
../src/cr_startup_lpc15xx.cpp \
../src/main.cpp 

C_SRCS += \
../src/crp.c \
../src/sysinit.c 

OBJS += \
./src/DigitalIoPin.o \
./src/Parser.o \
./src/Pen.o \
./src/cr_cpp_config.o \
./src/cr_startup_lpc15xx.o \
./src/crp.o \
./src/main.o \
./src/sysinit.o 

CPP_DEPS += \
./src/DigitalIoPin.d \
./src/Parser.d \
./src/Pen.d \
./src/cr_cpp_config.d \
./src/cr_startup_lpc15xx.d \
./src/main.d 

C_DEPS += \
./src/crp.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -DDEBUG -D__CODE_RED -D__NEWLIB__ -DCORE_M3 -D__USE_LPCOPEN -DCPP_USE_HEAP -D__LPC15XX__ -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\lpc_board_nxp_lpcxpresso_1549\inc" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\lpc_chip_15xx\inc" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\FreeRTOS\inc" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\FreeRTOS\src\include" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\FreeRTOS\src\portable\GCC\ARM_CM3" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -mcpu=cortex-m3 -mthumb -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=c11 -DDEBUG -D__CODE_RED -D__NEWLIB__ -DCORE_M3 -D__USE_LPCOPEN -DCPP_USE_HEAP -D__LPC15XX__ -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\lpc_board_nxp_lpcxpresso_1549\inc" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\lpc_chip_15xx\inc" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\FreeRTOS\inc" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\FreeRTOS\src\include" -I"C:\Users\samul\Documents\MCUXpressoIDE_11.0.1_2563\workspace\FreeRTOS\src\portable\GCC\ARM_CM3" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


