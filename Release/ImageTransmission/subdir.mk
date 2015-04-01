################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ImageTransmission/RX.cpp \
../ImageTransmission/TX.cpp 

OBJS += \
./ImageTransmission/RX.o \
./ImageTransmission/TX.o 

CPP_DEPS += \
./ImageTransmission/RX.d \
./ImageTransmission/TX.d 


# Each subdirectory must supply rules for building sources it contributes
ImageTransmission/%.o: ../ImageTransmission/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -DTIMMING -I"/home/diego/Git/ROVRadioTransmission/SerialTransmissionChannel/includes" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


