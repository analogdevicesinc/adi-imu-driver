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

if(BAREMETAL)
message(WARNING "BUILDING FOR BAREMETAL.\nPLEASE PROVIDE APPROPRIATE SPI LIBRARY FOR YOUR PLATFORM.\nSEE README.")
endif()

# set flags
set(CMAKE_C_FLAGS "-Wall -ffunction-sections -fdata-sections -O3 -Wno-unused-function -Wno-unused-variable")

add_subdirectory(lib)

# For non-baremetal platforms
if(NOT BAREMETAL)
add_subdirectory(linux)
endif()

add_executable(imu_main main.c)
target_link_libraries(imu_main adimu spi_linux)

if(NOT BAREMETAL)
enable_testing()
add_subdirectory(tests)
endif()
