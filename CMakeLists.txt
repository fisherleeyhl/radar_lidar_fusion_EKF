cmake_minimum_required(VERSION 3.8.2)
project(radar_lidar_fusion_EKF)

# Include the headers into the project.
include_directories(include)

# for advanced features of c++
add_definitions(-std=c++11)

# Manually add the sources using the set command.
#set(SOURCES src/main.cpp src/student.cpp)

file(GLOB SOURCES "src/*.cpp")

add_executable(radar_lidar_fusion_EKF ${SOURCES})
