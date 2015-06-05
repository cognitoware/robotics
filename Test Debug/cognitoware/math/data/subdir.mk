################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../cognitoware/math/data/Matrix.cc 

CPP_SRCS += \
../cognitoware/math/data/MatrixVectorOperators.cpp \
../cognitoware/math/data/Matrix_test.cpp \
../cognitoware/math/data/Vector.cpp \
../cognitoware/math/data/VectorRange_test.cpp \
../cognitoware/math/data/Vector_test.cpp 

CC_DEPS += \
./cognitoware/math/data/Matrix.d 

OBJS += \
./cognitoware/math/data/Matrix.o \
./cognitoware/math/data/MatrixVectorOperators.o \
./cognitoware/math/data/Matrix_test.o \
./cognitoware/math/data/Vector.o \
./cognitoware/math/data/VectorRange_test.o \
./cognitoware/math/data/Vector_test.o 

CPP_DEPS += \
./cognitoware/math/data/MatrixVectorOperators.d \
./cognitoware/math/data/Matrix_test.d \
./cognitoware/math/data/Vector.d \
./cognitoware/math/data/VectorRange_test.d \
./cognitoware/math/data/Vector_test.d 


# Each subdirectory must supply rules for building sources it contributes
cognitoware/math/data/%.o: ../cognitoware/math/data/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

cognitoware/math/data/%.o: ../cognitoware/math/data/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -std=gnu++0x -D__cplusplus=201103L -I"D:\gtest-1.7.0\include" -I"D:\workspace\Cognitoware" -O0 -g3 -pedantic -Wall -Wextra -Wconversion -c -fmessage-length=0 -v -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


