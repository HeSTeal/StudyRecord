cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Sentry-26_CMAKE)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# CubeMX 侧的库(USB/FreeRTOS/HAL)会经 main.h 间接包含 BSP 头文件，需要补路径
target_include_directories(stm32cubemx INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    BSP/BSP_sys/bsp_tools.c
    BSP/bsp_can.c
    BSP/bsp_dwt.c
    BSP/bsp_gpio.c
    BSP/bsp_pwm.c
    BSP/bsp_RS485.c
    BSP/bsp_spi.c
    BSP/bsp_usart.c
    BSP/bsp_usb.c
    MODULES/RGB.c
    MODULES/algorithm/controller.c
    MODULES/algorithm/crc16.c
    MODULES/algorithm/crc8.c
    MODULES/algorithm/kalman_filter.c
    MODULES/algorithm/QuaternionEKF.c
    MODULES/algorithm/user_lib.c
    MODULES/algorithm/SMC.c
    MODULES/BMI088/bmi088.c
    MODULES/can_comm/can_comm.c
    MODULES/daemon/daemon.c
    MODULES/imu/ins_task.c
    MODULES/imu/BMI088driver.c
    MODULES/imu/BMI088Middleware.c
    MODULES/master_machine/master_process.c
    MODULES/message_center/message_center.c
    MODULES/motor/motor_task.c
    MODULES/motor/power_control.c
    MODULES/motor/DJIMotor/DJImotor.c
    MODULES/motor/DMMotor/DMmotor.c
    MODULES/referee/crc_ref.c
    MODULES/referee/referee_task.c
    MODULES/referee/rm_referee.c
    MODULES/remote/remote_control.c
    APP/robot.c
    APP/chassis/chassis.c
    APP/cmd/robot_cmd.c
    APP/gimbal/gimbal.c
    APP/shoot/shoot.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    BSP/
    BSP/BSP_sys
    MODULES/
    MODULES/algorithm
    MODULES/BMI088
    MODULES/can_comm
    MODULES/daemon
    MODULES/imu
    MODULES/master_machine
    MODULES/message_center
    MODULES/motor
    MODULES/motor/DJIMotor
    MODULES/motor/DMMotor
    MODULES/referee
    MODULES/remote
    APP/
    APP/chassis
    APP/cmd
    APP/gimbal
    APP/shoot

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 让 newlib-nano 支持 printf/sprintf 输出浮点数
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
