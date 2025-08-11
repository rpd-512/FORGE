cmake_minimum_required(VERSION 3.10)
project(FORGE VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable debug symbols if needed
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Include Eigen headers
include_directories(/usr/include/eigen3)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Add the main executable
add_executable(FORGE_linux main.cpp)

# Link libraries
target_link_libraries(FORGE_linux PRIVATE yaml-cpp)

# Optional: add custom target for GDB
add_custom_target(gdb
    COMMAND gdb ${CMAKE_CURRENT_BINARY_DIR}/FORGE_linux
    DEPENDS FORGE_linux
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

