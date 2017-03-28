cmake_minimum_required(VERSION 3.6)
project(ExtendedKalmanFilter)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/main.cpp src/sensor_data_packet.h src/kalman_filter.cpp src/kalman_filter.h src/tools.h src/tools.cpp src/FusionEKF.cpp src/FusionEKF.h)
add_executable(ExtendedKalmanFilter ${SOURCE_FILES})