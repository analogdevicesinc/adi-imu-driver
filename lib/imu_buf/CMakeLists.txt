#/**
#  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
#  * This software is proprietary to Analog Devices, Inc. and its licensors.
#  *
#  * Use of this file is governed by the license agreement
#  * included in this repository.
#  *
#  * @file		    CMakeLists.txt
#  * @author		Sundar Palani (sundar.palani@analog.com)
#  * @brief 		CMake module for adimubuf library.
# **/

cmake_minimum_required(VERSION 3.0.2)

add_library(adimubuf_s STATIC imu_spi_buffer.c)
target_include_directories(adimubuf_s PUBLIC .)
target_link_libraries(adimubuf_s adimu)

add_library(adimubuf SHARED imu_spi_buffer.c)
target_include_directories(adimubuf PUBLIC .)
target_link_libraries(adimubuf adimu)
set_target_properties(adimubuf PROPERTIES VERSION "${ADIMU_VERSION_MAJOR}.${ADIMU_VERSION_MINOR}.${ADIMU_VERSION_PATCH}" SOVERSION ${ADIMU_VERSION_MAJOR})
install(TARGETS adimubuf DESTINATION lib)
install(TARGETS adimubuf_s DESTINATION lib)
install(FILES ./imu_spi_buffer.h DESTINATION include)
install(FILES ./imu_spi_buffer_regmap.h DESTINATION include)
