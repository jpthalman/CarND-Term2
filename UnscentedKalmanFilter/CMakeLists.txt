cmake_minimum_required(VERSION 3.6)
project(UnscentedKalmanFilter)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/main.cpp
        src/unscented_kalman_filter.cpp
        src/sensor_data_packet.cpp)
add_executable(UnscentedKalmanFilter ${SOURCE_FILES})