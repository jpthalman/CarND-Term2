cmake_minimum_required(VERSION 3.5)
project(ExtendedKalmanFilter)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++0x)

set(SOURCE_FILES
        src/main.cpp
        src/kalman_filter.cpp
        src/tools.cpp
        src/FusionEKF.cpp)
add_executable(ExtendedKalmanFilter ${SOURCE_FILES})