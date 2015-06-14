################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../cognitoware/math/probability/continuous/GaussianCanonical_test.cpp \
../cognitoware/math/probability/continuous/GaussianMoment_test.cpp 

OBJS += \
./cognitoware/math/probability/continuous/GaussianCanonical_test.o \
./cognitoware/math/probability/continuous/GaussianMoment_test.o 

CPP_DEPS += \
./cognitoware/math/probability/continuous/GaussianCanonical_test.d \
./cognitoware/math/probability/continuous/GaussianMoment_test.d 


# Each subdirectory must supply rules for building sources it contributes
cognitoware/math/probability/continuous/%.o: ../cognitoware/math/probability/continuous/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


