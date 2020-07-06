cmake_minimum_required(VERSION 3.0.2)

add_library(adimubuf SHARED imu_spi_buffer.c)
target_include_directories(adimubuf PUBLIC . ../lib)
