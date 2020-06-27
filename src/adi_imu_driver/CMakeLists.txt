cmake_minimum_required(VERSION 3.0.2)

add_library(adi_imu_driver adi_imu_driver.c)
target_include_directories(adi_imu_driver PUBLIC .)
