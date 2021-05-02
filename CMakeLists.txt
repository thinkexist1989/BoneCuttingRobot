cmake_minimum_required(VERSION 3.10)

project(BoneCuttingRobot)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 11)

find_package(RobotLibs REQUIRED)

message(STATUS "RobotLibs: ${RobotLibs_LIBRARIES}")

add_subdirectory(RobotControlLib)
add_subdirectory(RobotMain)

configure_file(LibCopy.sh.in ${PROJECT_BINARY_DIR}/LibCopy.sh @ONLY)