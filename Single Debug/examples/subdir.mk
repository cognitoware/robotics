################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../examples/RobotPoseEKF.cc 

CC_DEPS += \
./examples/RobotPoseEKF.d 

OBJS += \
./examples/RobotPoseEKF.o 


# Each subdirectory must supply rules for building sources it contributes
examples/%.o: ../examples/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


