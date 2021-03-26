cmake_minimum_required(VERSION 2.8)

project(BoneCuttingRobot)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 11)

find_package(RobotLibs REQUIRED)

message(STATUS "RobotLibs: ${RobotLibs_LIBRARIES}")

add_subdirectory(RobotControlLib)
add_subdirectory(RobotMain)