#/**
#  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
#  * This software is proprietary to Analog Devices, Inc. and its licensors.
#  *
#  * Use of this file is governed by the license agreement
#  * included in this repository.
#  *
#  * @file		    CMakeLists.txt
#  * @author		Sundar Palani (sundar.palani@analog.com)
#  * @brief 		CMake module for adimu library.
# **/

cmake_minimum_required(VERSION 3.0.2)

add_library(adimu_s STATIC adi_imu_driver.c)
target_include_directories(adimu_s PUBLIC .)
target_link_libraries(adimu_s spi_driver)

add_library(adimu SHARED adi_imu_driver.c)
target_include_directories(adimu PUBLIC .)
target_link_libraries(adimu spi_driver)
set_target_properties(adimu PROPERTIES VERSION "${ADIMU_VERSION_MAJOR}.${ADIMU_VERSION_MINOR}.${ADIMU_VERSION_PATCH}" SOVERSION ${ADIMU_VERSION_MAJOR})
install(TARGETS adimu DESTINATION lib)
install(TARGETS adimu_s DESTINATION lib)
install(FILES ./adi_imu_driver.h DESTINATION include)
install(FILES ./adi_imu_regmap.h DESTINATION include)