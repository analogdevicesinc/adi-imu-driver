cmake_minimum_required(VERSION 3.0.2)

set(ADIMU_VERSION ${PROJ_VERSION_MAJOR}.${PROJ_VERSION_MINOR}.${PROJ_VERSION_PATCH} )
add_library(adimu SHARED adi_imu_driver.c)
target_include_directories(adimu PUBLIC .)
set_target_properties(adimu PROPERTIES VERSION ${ADIMU_VERSION} SOVERSION ${PROJ_VERSION_MAJOR})
