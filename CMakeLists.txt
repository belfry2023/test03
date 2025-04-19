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
set(CMAKE_PROJECT_NAME test03)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
Bsp/can/bsp_can.c
Bsp/dwt/bsp_dwt.c
Bsp/usart/bsp_usart.c
Bsp/log/bsp_log.c
Bsp/bsp_tools.c
Modules/algorithm/controller.c
Modules/algorithm/crc8.c
Modules/algorithm/crc16.c
Modules/algorithm/kalman_filter.c
Modules/algorithm/QuaternionEKF.c
Modules/algorithm/user_lib.c
Modules/DJImotor/dji_motor.c
Modules/DJImotor/motor_task.c
Modules/imu/BMI088_driver.c
Modules/imu/BMI088.c
Modules/imu/ins_task.c
Modules/message_center/message_center.c
Modules/remote/remote_control.c
Modules/daemon/daemon.c
Modules/vision/master_process.c
Modules/vision/seasky_protocol.c
Application/car/car.c
Application/cmd/cmd.c
Application/startup.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
Middlewares/Third_Party/SEGGER/Config
Middlewares/Third_Party/SEGGER/RTT
Bsp
Bsp/can
Bsp/dwt
Bsp/usart
Bsp/log
Modules
Modules/algorithm
Modules/DJImotor
Modules/imu
Modules/message_center
Modules/remote
Modules/daemon
Modules/vision
Application
Application/car
Application/cmd
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# Add linker script
SET(ELFFILE ${CMAKE_PROJECT_NAME}.elf)
SET(HEXFILE ${CMAKE_PROJECT_NAME}.hex)
SET(OBJCOPY arm-none-eabi-objcopy.exe)
SET(JFLASH jflash.exe)
# Add custom commands to build the project
ADD_CUSTOM_COMMAND(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Building ${CMAKE_PROJECT_NAME}"
    COMMAND ${CMAKE_COMMAND} -E echo "Generating ELF file: ${ELFFILE}"
    COMMAND ${CMAKE_COMMAND} -E echo "Generating HEX file: ${HEXFILE}"
    COMMAND ${OBJCOPY} -O ihex ${ELFFILE} ${HEXFILE}
    COMMAND ${CMAKE_COMMAND} -E echo "Build complete!"
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing ${HEXFILE} to the device"
    COMMAND ${JFLASH} -openprj ${CMAKE_SOURCE_DIR}/stm32.jflash -open ${HEXFILE}, -8000000 -auto -startapp -exit
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing complete!"
)