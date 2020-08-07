cmake_minimum_required(VERSION 3.0.2)

set(PROJ_VERSION_MAJOR 0)
set(PROJ_VERSION_MINOR 0)
set(PROJ_VERSION_PATCH 0)
set(PROJ_VERSION_TWEAK 1)
set(PROJ_VERSION ${PROJ_VERSION_MAJOR}.${PROJ_VERSION_MINOR}.${PROJ_VERSION_PATCH}.${PROJ_VERSION_TWEAK} )

set(ADIMU_VERSION_MAJOR ${PROJ_VERSION_MAJOR})
set(ADIMU_VERSION_MINOR ${PROJ_VERSION_MINOR})
set(ADIMU_VERSION_PATCH ${PROJ_VERSION_PATCH})
set(ADIMU_VERSION_TWEAK ${PROJ_VERSION_TWEAK})

project(adi_imu_driver VERSION ${PROJ_VERSION} LANGUAGES C CXX)

# default build type is Release
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

if(BAREMETAL)
message(WARNING "BUILDING FOR BAREMETAL.\nPLEASE PROVIDE APPROPRIATE SPI LIBRARY FOR YOUR PLATFORM.\nSEE README.")
endif()

# set flags
set(CMAKE_C_FLAGS "-Wall -ffunction-sections -fdata-sections -ffast-math -Ofast -Wno-unused-function -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "-Wall -ffunction-sections -fdata-sections -ffast-math -Ofast -Wno-unused-function -Wno-unused-variable")

add_subdirectory(lib/imu)
add_subdirectory(lib/imu_buf)

# For non-baremetal platforms
if(NOT BAREMETAL)
add_subdirectory(linux)
endif()

add_executable(imu_main examples/imu_main.c)
target_link_libraries(imu_main adimu spi_linux m)

add_executable(imu_buf_main examples/imu_buf_main.c)
target_link_libraries(imu_buf_main adimu adimubuf spi_linux m)

add_executable(spi_main examples/spi_main.c)
target_link_libraries(spi_main spi_linux m)

add_executable(imu_fast_main examples/imu_fast_main.cpp)
target_link_libraries(imu_fast_main adimu spi_linux m pthread)

add_executable(imu_buf_fast_main examples/imu_buf_fast_main.cpp)
target_link_libraries(imu_buf_fast_main adimu adimubuf spi_linux m pthread)

add_executable(imu_buf_burst examples/imu_buf_burst.c)
target_link_libraries(imu_buf_burst adimu adimubuf spi_linux m)

if(NOT BAREMETAL)
enable_testing()
add_subdirectory(tests)
endif()
