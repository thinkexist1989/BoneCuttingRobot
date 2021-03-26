#.rst:
# FindRobotLib
# --------------
#
# Find RobotLib include dirs, library dirs, libraries and post-build commands
#
# Use this module by invoking find_package with the form::
#
#    find_package( RobotLib [REQUIRED] )
#
# Results for users are reported in following variables::
#
#    RobotLib_FOUND                - Return "TRUE" when RobotLib found. Otherwise, Return "FALSE".
#    RobotLib_INCLUDE_DIRS         - RobotLib include directories.
#    RobotLib_LIBRARY_DIRS         - RobotLib library directories.
#    RobotLib_LIBRARIES            - RobotLib library files.
#
# CMake entries::
#
#    RobotLib_DIR                  - RobotLib root directory. (Default ${CMAKE_SOURCE_DIR}/RobotLib)
#
# Example to find Kinect SDK v2::
#
#    cmake_minimum_required( VERSION 2.8 )
#
#    project( project )
#    add_executable( project main.cpp )
#
#    # Find package using this module.
#    find_package( RobotLib REQUIRED )
#
#    if(RobotLib_FOUND)
#      # [C/C++]>[General]>[Additional Include Directories]
#      include_directories( ${RobotLib_INCLUDE_DIRS} )
#
#      # [Linker]>[General]>[Additional Library Directories]
#      link_directories( ${RobotLib_LIBRARY_DIRS} )
#
#      # [Linker]>[Input]>[Additional Dependencies]
#      target_link_libraries( project ${RobotLib_LIBRARIES} )

#    endif()
#
# =============================================================================
#
# Copyright (c) 2021 Yang Luo
# email:luoyang@sia.cn
# Distributed under the MIT License.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# =============================================================================

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
    if (NOT EXISTS "${${_DIR}}")
        message(WARNING "Directory \"${${_DIR}}\" not found.")
        set(RobotLibs_FOUND FALSE)
        unset(_DIR)
    endif ()
endmacro()

# Check Files Macro
macro(CHECK_FILES _FILES _DIR)
    set(_MISSING_FILES)
    foreach (_FILE ${${_FILES}})
        if (NOT EXISTS "${_FILE}")
            get_filename_component(_FILE ${_FILE} NAME)
            set(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
        endif ()
    endforeach ()
    if (_MISSING_FILES)
        message(WARNING "In directory \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
        set(RobotLibs_FOUND FALSE)
        unset(_FILES)
    endif ()
endmacro()

##### Find RobotLib #####

# Found
set(RobotLibs_FOUND TRUE)

#Root Directory
if (RobotLibs_FOUND)
    set(RobotLibs_DIR ${CMAKE_SOURCE_DIR}/RobotLib)
    check_dir(RobotLibs_DIR)
endif ()

#Include Directory
if (RobotLibs_FOUND)
    set(RobotLibs_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/RobotLib/include)
    set(RobotLibs_INCLUDE_DIR ${RobotLibs_INCLUDE_DIRS})
    check_dir(RobotLibs_INCLUDE_DIRS)
endif ()

#Library Directories
if (RobotLibs_FOUND)
    set(RobotLibs_LIBRARY_DIRS ${CMAKE_SOURCE_DIR}/RobotLib/lib)
    check_dir(RobotLibs_LIBRARY_DIRS)
endif ()

# Dependencies
if (RobotLibs_FOUND)
#    file(GLOB LIBS "${CMAKE_SOURCE_DIR}/RobotLib/lib/lib*.so")
    set(RobotLibs_LIBRARIES RobotInterface
                            RobotExtenLibs
                            RobotLib
                            RobotControlerAPI
                            RobotControlLib
                            BaseLib
                            DynamicsLib
                            GripLib
                            EtherCATLib
                            InterpolateLib
                            InterpolateLibCustom
                            AdditionAxisLib
                            KinematicCalibrationLib
                            KinematicLib
                            KinematicLibCustom
                            TorqueSensorLib
                            TorqueSensor
                            ExtraLib
                            ExtraLib2
                            RobotIQLib
                            nlopt
                            modbus
                            m
                            pthread
                            rt
    )
#    check_files(RobotLibs_LIBRARIES RobotLibs_LIBRARY_DIRS)
endif ()

message(STATUS "RobotLibs_FOUND: ${RobotLibs_FOUND}")