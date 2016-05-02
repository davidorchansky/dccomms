################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Arduino.cpp \
../src/BlockRadioTransmitter.cpp \
../src/Checksum.cpp \
../src/DataLinkFrame.cpp \
../src/DataLinkStream.cpp \
../src/Radio.cpp \
../src/RadioException.cpp \
../src/SerialPortInterface.cpp \
../src/SerialTransmissionChannel.cpp \
../src/Stream.cpp \
../src/Utils.cpp 

OBJS += \
./src/Arduino.o \
./src/BlockRadioTransmitter.o \
./src/Checksum.o \
./src/DataLinkFrame.o \
./src/DataLinkStream.o \
./src/Radio.o \
./src/RadioException.o \
./src/SerialPortInterface.o \
./src/SerialTransmissionChannel.o \
./src/Stream.o \
./src/Utils.o 

CPP_DEPS += \
./src/Arduino.d \
./src/BlockRadioTransmitter.d \
./src/Checksum.d \
./src/DataLinkFrame.d \
./src/DataLinkStream.d \
./src/Radio.d \
./src/RadioException.d \
./src/SerialPortInterface.d \
./src/SerialTransmissionChannel.d \
./src/Stream.d \
./src/Utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -fPIC -DTIMMING -I"/home/diego/Git/ROVRadioTransmission/git/ROVRadioTransmission/includes" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


