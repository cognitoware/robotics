################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../cognitoware/math/data/Vector.cpp 

OBJS += \
./cognitoware/math/data/Vector.o 

CPP_DEPS += \
./cognitoware/math/data/Vector.d 


# Each subdirectory must supply rules for building sources it contributes
cognitoware/math/data/%.o: ../cognitoware/math/data/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


