#/**
#  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
#  * This software is proprietary to Analog Devices, Inc. and its licensors.
#  *
#  * Use of this file is governed by the license agreement
#  * included in this repository.
#  *
#  * @file		    CMakeLists.txt
#  * @author		Sundar Palani (sundar.palani@analog.com)
#  * @brief 		Main CMake module.
# **/

cmake_minimum_required(VERSION 3.0.2)

set(PROJ_VERSION_MAJOR 1)
set(PROJ_VERSION_MINOR 1)
set(PROJ_VERSION_PATCH 0)
set(PROJ_VERSION_TWEAK 0)
set(PROJ_VERSION ${PROJ_VERSION_MAJOR}.${PROJ_VERSION_MINOR}.${PROJ_VERSION_PATCH}.${PROJ_VERSION_TWEAK} )

set(ADIMU_VERSION_MAJOR ${PROJ_VERSION_MAJOR})
set(ADIMU_VERSION_MINOR ${PROJ_VERSION_MINOR})
set(ADIMU_VERSION_PATCH ${PROJ_VERSION_PATCH})
set(ADIMU_VERSION_TWEAK ${PROJ_VERSION_TWEAK})

project(adi_imu_driver VERSION ${PROJ_VERSION} LANGUAGES C CXX)

# default build type is Release
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RELEASE")
    set(CMAKE_BUILD_TYPE "RELEASE")
endif()

if(BAREMETAL)
    message(WARNING "BUILDING FOR BAREMETAL.\nPLEASE PROVIDE APPROPRIATE SPI LIBRARY FOR YOUR PLATFORM.\nSEE README.")
    add_definitions("-DBAREMETAL")
endif()

if(!BUFFER)
    message(STATUS "Building WITHOUT support for the SPI buffer board")
else()
    message(STATUS "Building WITH support for the SPI buffer board")
    add_definitions(-DBUFFER)
endif()

# set flags
set(CMAKE_C_FLAGS "-Wall -ffunction-sections -fdata-sections -Wno-unused-function -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "-Wall -ffunction-sections -fdata-sections -Wno-unused-function -Wno-unused-variable")

add_subdirectory(lib/imu)
add_subdirectory(lib/imu_buf)

# For non-baremetal platforms
if(UNIX AND NOT APPLE)
    add_subdirectory(linux)
    add_subdirectory(asyncio)
elseif(APPLE)
    message(WARNING "BUILDING FOR OSX IS ONLY FOR TESTING PURPOSE.\nENSURE SPI LIBRARY IS PROPERLY IMPLEMENTED.\nSEE README.")
    add_subdirectory(osx)
endif()

add_executable(imu_test examples/imu_test.c)
target_link_libraries(imu_test adimu adimubuf)

add_executable(spi_test examples/spi_test.c)
target_link_libraries(spi_test spi_driver m)

add_executable(gpio_test examples/gpio_test.c)
target_link_libraries(gpio_test spi_driver m gpio_driver)

if(NOT BAREMETAL)
enable_testing()
add_subdirectory(tests)
endif()
