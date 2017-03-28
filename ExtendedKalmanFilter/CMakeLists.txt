cmake_minimum_required(VERSION 3.6)
project(ExtendedKalmanFilter)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/main.cpp)
add_executable(ExtendedKalmanFilter ${SOURCE_FILES})