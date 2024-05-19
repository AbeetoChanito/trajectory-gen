cmake_minimum_required(VERSION 3.10)
project(trajectory_gen)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS On)

add_executable(
    trajectory_gen
    src/main.cpp
    src/trapezoidal.cpp
    src/point.cpp
    src/path.cpp
    src/differentialKinematics.cpp
    src/generator.cpp
)

target_include_directories(trajectory_gen PUBLIC include)