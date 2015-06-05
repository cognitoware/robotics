################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../cognitoware/robotics/state_estimation/ActionModel_test.cpp \
../cognitoware/robotics/state_estimation/GaussianActionModel_test.cpp \
../cognitoware/robotics/state_estimation/GaussianSensorModel_test.cpp \
../cognitoware/robotics/state_estimation/KalmanActionModel_test.cpp \
../cognitoware/robotics/state_estimation/KalmanFilter_test.cpp \
../cognitoware/robotics/state_estimation/KalmanSensorModel_test.cpp \
../cognitoware/robotics/state_estimation/SensorModel_test.cpp 

OBJS += \
./cognitoware/robotics/state_estimation/ActionModel_test.o \
./cognitoware/robotics/state_estimation/GaussianActionModel_test.o \
./cognitoware/robotics/state_estimation/GaussianSensorModel_test.o \
./cognitoware/robotics/state_estimation/KalmanActionModel_test.o \
./cognitoware/robotics/state_estimation/KalmanFilter_test.o \
./cognitoware/robotics/state_estimation/KalmanSensorModel_test.o \
./cognitoware/robotics/state_estimation/SensorModel_test.o 

CPP_DEPS += \
./cognitoware/robotics/state_estimation/ActionModel_test.d \
./cognitoware/robotics/state_estimation/GaussianActionModel_test.d \
./cognitoware/robotics/state_estimation/GaussianSensorModel_test.d \
./cognitoware/robotics/state_estimation/KalmanActionModel_test.d \
./cognitoware/robotics/state_estimation/KalmanFilter_test.d \
./cognitoware/robotics/state_estimation/KalmanSensorModel_test.d \
./cognitoware/robotics/state_estimation/SensorModel_test.d 


# Each subdirectory must supply rules for building sources it contributes
cognitoware/robotics/state_estimation/%.o: ../cognitoware/robotics/state_estimation/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


