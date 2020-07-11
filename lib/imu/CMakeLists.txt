cmake_minimum_required(VERSION 3.0.2)

add_library(adimu SHARED adi_imu_driver.c)
target_include_directories(adimu PUBLIC .)
set_target_properties(adimu PROPERTIES VERSION "${ADIMU_VERSION_MAJOR}.${ADIMU_VERSION_MINOR}.${ADIMU_VERSION_PATCH}" SOVERSION ${ADIMU_VERSION_MAJOR})
