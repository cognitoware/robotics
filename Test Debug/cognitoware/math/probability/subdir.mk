################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../cognitoware/math/probability/BayesDistribution_test.cpp \
../cognitoware/math/probability/ConditionalMap_test.cpp \
../cognitoware/math/probability/IndependentPairDistribution_test.cpp \
../cognitoware/math/probability/RandomConditional_test.cpp \
../cognitoware/math/probability/RandomDistribution_test.cpp \
../cognitoware/math/probability/RangedUniform_test.cpp 

OBJS += \
./cognitoware/math/probability/BayesDistribution_test.o \
./cognitoware/math/probability/ConditionalMap_test.o \
./cognitoware/math/probability/IndependentPairDistribution_test.o \
./cognitoware/math/probability/RandomConditional_test.o \
./cognitoware/math/probability/RandomDistribution_test.o \
./cognitoware/math/probability/RangedUniform_test.o 

CPP_DEPS += \
./cognitoware/math/probability/BayesDistribution_test.d \
./cognitoware/math/probability/ConditionalMap_test.d \
./cognitoware/math/probability/IndependentPairDistribution_test.d \
./cognitoware/math/probability/RandomConditional_test.d \
./cognitoware/math/probability/RandomDistribution_test.d \
./cognitoware/math/probability/RangedUniform_test.d 


# Each subdirectory must supply rules for building sources it contributes
cognitoware/math/probability/%.o: ../cognitoware/math/probability/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


