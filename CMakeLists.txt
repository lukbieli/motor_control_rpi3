cmake_minimum_required(VERSION 3.10)

project(motor_control_rpi3)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the path to the directory containing FindPigpio.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Add your source files
set(SOURCES
    remote_control.cpp
    adc.cpp
    motor_ctrl.cpp
    robot_move.cpp
    xbox_controller.cpp
    rotary_encoder.cpp
    pid.cpp
)

# Add your header files (if needed for IDEs or autocompletion)
set(HEADERS
    motor_ctrl.hpp
    adc.hpp
    robot_move.hpp
    xbox_controller.hpp
    rotary_encoder.hpp
    pid.hpp
)

# Find required packages (if any)
find_package(Libevdev REQUIRED)
find_package(pigpio REQUIRED)

# Specify the path to the Libevdev include directory and library
# set(Libevdev_INCLUDE_DIR /usr/include/libevdev-1.0)  # Replace with the actual path
# set(Libevdev_LIBRARY /usr/lib/x86_64-linux-gnu/libevdev.so)  # Replace with the actual path

# Add the executable
add_executable(remote_control ${SOURCES})

# Link libraries
target_link_libraries(remote_control
    PRIVATE
    ${LIBEVDEV_LIBRARIES}
    pigpio
    pthread
)

# Specify include directories
target_include_directories(remote_control PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${LIBEVDEV_INCLUDE_DIRS}
    ${PIGPIO_INCLUDE_DIRS}
)

# Specify any additional compiler flags
target_compile_options(remote_control PRIVATE -Wall)

# Output binary to 'build' directory
set_target_properties(remote_control
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build
)
