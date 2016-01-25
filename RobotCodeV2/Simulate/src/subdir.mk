################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Drivetrain.cpp \
../src/MyRobot.cpp 

OBJS += \
./src/Drivetrain.o \
./src/MyRobot.o 

CPP_DEPS += \
./src/Drivetrain.d \
./src/MyRobot.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"C:\Users\987\Documents\2016RobotCode\RobotCodev1/src" -IC:\Users\987/wpilib/cpp/current/sim/include -I/usr/include -I/usr/include/gazebo-3.2 -I/usr/include/sdformat-2.2 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


