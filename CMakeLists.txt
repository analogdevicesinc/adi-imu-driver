cmake_minimum_required(VERSION 3.0.2)

set(PROJ_VERSION_MAJOR 0)
set(PROJ_VERSION_MINOR 0)
set(PROJ_VERSION_PATCH 0)
set(PROJ_VERSION_TWEAK 1)
set(PROJ_VERSION ${PROJ_VERSION_MAJOR}.${PROJ_VERSION_MINOR}.${PROJ_VERSION_PATCH}.${PROJ_VERSION_TWEAK} )

project(adi_imu_driver VERSION ${PROJ_VERSION} LANGUAGES C)

# default build type is DEBUG build
if ( (NOT DEFINED BUILDTYPE) OR ( NOT ${BUILDTYPE} STREQUAL DEBUG AND NOT ${BUILDTYPE} STREQUAL RELEASE ) )
    set(BUILDTYPE DEBUG)
endif() 
message("BUILDTYPE ${BUILDTYPE}")

# define log level
if (DEFINED LOG_LEVEL)
    message("LOG LEVEL ${LOG_LEVEL}")
    add_definitions(-DLOG_LEVEL=${LOG_LEVEL})
else()
    message("LOG LEVEL 2 (default)")
    add_definitions(-DLOG_LEVEL=2)
endif()

# set flags
set(CMAKE_C_FLAGS "-Wall -ffunction-sections -fdata-sections -O3 -Wno-unused-function -Wno-unused-variable")

add_subdirectory(src/adi_imu_driver)
add_subdirectory(src/spi_linux)
add_subdirectory(src/util)

add_executable(main src/main.c)
target_link_libraries(main adi_imu_driver spi_linux util)
