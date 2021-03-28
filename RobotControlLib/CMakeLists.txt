cmake_minimum_required(VERSION 2.8)

project(RobotControlLib)

find_package(RobotLibs REQUIRED)
find_package(Threads REQUIRED)

file(GLOB ROBOT_CONTROL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*")

add_library(RobotControlLib SHARED ${ROBOT_CONTROL_SRC})
add_library(RobotControlLib::RobotControlLib ALIAS RobotControlLib)
target_include_directories(RobotControlLib PUBLIC ${RobotLibs_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(RobotControlLib ${RobotLibs_LIBRARIES})