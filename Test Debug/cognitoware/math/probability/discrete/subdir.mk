################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../cognitoware/math/probability/discrete/ConditionalValueMap_test.cpp \
../cognitoware/math/probability/discrete/DiscreteDistribution_test.cpp \
../cognitoware/math/probability/discrete/DistributionCounter_test.cpp \
../cognitoware/math/probability/discrete/DistributionValueMap_test.cpp \
../cognitoware/math/probability/discrete/MarkovChain_test.cpp 

OBJS += \
./cognitoware/math/probability/discrete/ConditionalValueMap_test.o \
./cognitoware/math/probability/discrete/DiscreteDistribution_test.o \
./cognitoware/math/probability/discrete/DistributionCounter_test.o \
./cognitoware/math/probability/discrete/DistributionValueMap_test.o \
./cognitoware/math/probability/discrete/MarkovChain_test.o 

CPP_DEPS += \
./cognitoware/math/probability/discrete/ConditionalValueMap_test.d \
./cognitoware/math/probability/discrete/DiscreteDistribution_test.d \
./cognitoware/math/probability/discrete/DistributionCounter_test.d \
./cognitoware/math/probability/discrete/DistributionValueMap_test.d \
./cognitoware/math/probability/discrete/MarkovChain_test.d 


# Each subdirectory must supply rules for building sources it contributes
cognitoware/math/probability/discrete/%.o: ../cognitoware/math/probability/discrete/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


