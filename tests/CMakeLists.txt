#/**
#  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
#  * This software is proprietary to Analog Devices, Inc. and its licensors.
#  *
#  * Use of this file is governed by the license agreement
#  * included in this repository.
#  *
#  * @file		    CMakeLists.txt
#  * @author		Sundar Palani (sundar.palani@analog.com)
#  * @brief 		CMake module for cmake test executable.
# **/

cmake_minimum_required(VERSION 3.0.2)

add_executable(test1 test_imu_buffer_data_seq.c)
target_link_libraries(test1 adimu adimubuf spi_driver m)
add_test(test1 test1)

add_executable(test2 test_imu_buffer_data_seq_burst_mode.c)
target_link_libraries(test2 adimu adimubuf spi_driver m)
add_test(test2 test2)