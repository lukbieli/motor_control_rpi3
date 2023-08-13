cmake_minimum_required(VERSION 3.10)

project(motor_control_rpi3)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the path to the directory containing FindPigpio.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Set the build type to Debug by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Add your common source files
set(COMMON_SOURCES
    adc.cpp
    motor_ctrl.cpp
    robot_move.cpp
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
set(Libevdev_INCLUDE_DIR /usr/include/libevdev-1.0)  # Replace with the actual path
# Specify the path to the Libevdev library
if(NOT DEFINED GITHUB_RUN)
    # set(Libevdev_LIBRARY /usr/lib/x86_64-linux-gnu/libevdev.so)  # Default path
    set(Libevdev_LIBRARY /usr/lib/arm-linux-gnueabihf/libevdev.so)  # Replace with the actual path
endif()
# set(Libevdev_LIBRARY /usr/lib/arm-linux-gnueabihf/libevdev.so)  # Replace with the actual path

# Add the remote_control executable
add_executable(remote_control
    remote_control.cpp  # Add only the remote_control-specific source
    xbox_controller.cpp
    ${COMMON_SOURCES}   # Include the common sources
)

# Add the single_control executable
add_executable(single_control
    single_control.cpp  # Add only the single_control-specific source
    ${COMMON_SOURCES}   # Include the common sources
)

# Link libraries for both targets
target_link_libraries(remote_control
    PRIVATE
    ${Libevdev_LIBRARY}
    pigpio
    pthread
)

target_link_libraries(single_control
    PRIVATE
    pigpio
    pthread
)

# Specify include directories for both targets
target_include_directories(remote_control PRIVATE
    ${CMAKE_SOURCE_DIR}
    # ${LIBEVDEV_INCLUDE_DIRS}
    ${Libevdev_INCLUDE_DIR}
    ${PIGPIO_INCLUDE_DIRS}
)

target_include_directories(single_control PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${PIGPIO_INCLUDE_DIRS}
)

# Specify any additional compiler flags for both targets
target_compile_options(remote_control PRIVATE -Wall)
target_compile_options(single_control PRIVATE -Wall)

# Set debug flags for the Debug build type
if(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_options(remote_control PRIVATE -g)
    target_compile_options(single_control PRIVATE -g)
endif()

# Output binaries to 'build' directory for both targets
set_target_properties(remote_control single_control
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build
)
