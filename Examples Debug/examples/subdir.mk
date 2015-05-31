################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../examples/DoorOpener.cc \
../examples/FaultySensorDetection.cc \
../examples/ObservableWeatherBayes.cc \
../examples/Weather.cc \
../examples/WeatherMarkovChain.cc 

CC_DEPS += \
./examples/DoorOpener.d \
./examples/FaultySensorDetection.d \
./examples/ObservableWeatherBayes.d \
./examples/Weather.d \
./examples/WeatherMarkovChain.d 

OBJS += \
./examples/DoorOpener.o \
./examples/FaultySensorDetection.o \
./examples/ObservableWeatherBayes.o \
./examples/Weather.o \
./examples/WeatherMarkovChain.o 


# Each subdirectory must supply rules for building sources it contributes
examples/%.o: ../examples/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


