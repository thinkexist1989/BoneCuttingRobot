cmake_minimum_required(VERSION 3.10)

project(RobotMain)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(RobotLibs REQUIRED)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread timer)

find_package(Qt5 REQUIRED COMPONENTS Core)


file(GLOB ROBOT_MAIN_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*")

link_directories(${RobotLibs_LIBRARY_DIRS})
add_executable(RobotMain ${ROBOT_MAIN_SRC})
target_include_directories(RobotMain PUBLIC ${RobotLibs_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
#target_link_directories(RobotMain PUBLIC ${RobotLibs_LIBRARY_DIRS})
target_link_libraries(RobotMain ${RobotLibs_LIBRARIES} RobotControlLib::RobotControlLib Boost::system Boost::thread Boost::boost Boost::timer Qt5::Core)