################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/DefineFG.cpp \
../src/arguments.cpp \
../src/jsoncpp.cpp \
../src/parameters.cpp \
../src/problem.cpp \
../src/problemG7.cpp \
../src/problemS10.cpp \
../src/snoptProblem.cpp \
../src/tol.cpp 

OBJS += \
./src/DefineFG.o \
./src/arguments.o \
./src/jsoncpp.o \
./src/parameters.o \
./src/problem.o \
./src/problemG7.o \
./src/problemS10.o \
./src/snoptProblem.o \
./src/tol.o 

CPP_DEPS += \
./src/DefineFG.d \
./src/arguments.d \
./src/jsoncpp.d \
./src/parameters.d \
./src/problem.d \
./src/problemG7.d \
./src/problemS10.d \
./src/snoptProblem.d \
./src/tol.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -I/usr/include/python2.7 -I/usr/local/MATLAB/R2015a/extern/include -I"/home/silva/workspace/tol/include" -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


