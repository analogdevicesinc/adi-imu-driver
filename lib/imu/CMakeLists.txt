cmake_minimum_required(VERSION 3.0.2)

add_library(adimu SHARED adi_imu_driver.c)
target_include_directories(adimu PUBLIC .)
