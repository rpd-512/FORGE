cmake_minimum_required(VERSION 3.10)
project(FORGE)

set(CMAKE_CXX_STANDARD 17)

# Look for yaml-cpp
find_package(yaml-cpp REQUIRED)

# Eigen is header-only; just include the path
include_directories(/usr/include/eigen3)

add_executable(FORGE_linux main.cpp)

target_link_libraries(FORGE_linux yaml-cpp)
