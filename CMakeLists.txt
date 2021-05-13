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

# read and parse version from file VERSION
file(READ "VERSION" VERSION_STRING)
string(REPLACE "." ";" VERSION_LIST ${VERSION_STRING})
list(GET VERSION_LIST 0 PROJ_VERSION_MAJOR)
list(GET VERSION_LIST 1 PROJ_VERSION_MINOR)
list(GET VERSION_LIST 2 PROJ_VERSION_PATCH)
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

# set runtime search paths for linux and OSX systems
if(UNIX AND NOT APPLE)
    set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib" "$ORIGIN")
endif() 
if(APPLE)
    set(CMAKE_INSTALL_RPATH "@loader_path/../lib" "@loader_path")
endif()

#######################
#  GENERATE BUILD INFO
#######################

string(TIMESTAMP BUILD_DATETIME "%Y%m%d%H%M%S" UTC)
#message("-DGIT_HASH=\"${GIT_HASH}-${GIT_DIRTY}\" -DGIT_BRANCH=\"${GIT_BRANCH}\" -DDATETIME=\"${BUILD_DATETIME} UTC\"")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/include/adi_imu_common.h.in" 
                "${CMAKE_CURRENT_SOURCE_DIR}/include/adi_imu_common.h" 
                @ONLY
            )

# set flags
set(CMAKE_C_FLAGS "-Wall -ffunction-sections -fdata-sections -Wno-unused-function -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "-Wall -ffunction-sections -fdata-sections -Wno-unused-function -Wno-unused-variable")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

include_directories(include/)

add_subdirectory(lib/imu)
add_subdirectory(lib/imu_buf)

# For non-baremetal platforms
if(UNIX AND NOT APPLE)
    add_subdirectory(linux)
elseif(APPLE)
    message(WARNING "BUILDING FOR OSX IS ONLY FOR TESTING PURPOSE.\nENSURE SPI LIBRARY IS PROPERLY IMPLEMENTED.\nSEE README.")
    add_subdirectory(osx)
endif()

install(DIRECTORY include/ DESTINATION include PATTERN "include/*.in" EXCLUDE)

add_executable(imu_test examples/imu_test.c)
target_link_libraries(imu_test adimu_s adimubuf_s)
install(TARGETS imu_test DESTINATION bin)

add_executable(spi_test examples/spi_test.c)
target_link_libraries(spi_test spi_driver m)

add_executable(uart_test examples/uart_test.c)
target_link_libraries(uart_test uart_driver m)

add_executable(gpio_test examples/gpio_test.c)
target_link_libraries(gpio_test spi_driver m gpio_driver)

if(NOT BAREMETAL)
enable_testing()
add_subdirectory(tests)
endif()
