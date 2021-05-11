#/**
#  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
#  * This software is proprietary to Analog Devices, Inc. and its licensors.
#  *
#  * Use of this file is governed by the license agreement
#  * included in this repository.
#  *
#  * @file		    CMakeLists.txt
#  * @author		Sundar Palani (sundar.palani@analog.com)
#  * @brief 		CMake module for spi_driver and gpio_driver library.
# **/

cmake_minimum_required(VERSION 3.0.2)

add_library(spi_driver STATIC spi_driver.c)
if(DEBUG_SPI)
target_compile_definitions(spi_driver PRIVATE -DDEBUG_SPI)
message("DEBUG_SPI ENABLED")
endif()
target_compile_options(spi_driver PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(spi_driver PUBLIC .)

add_library(gpio_driver STATIC gpio.c)
target_compile_options(gpio_driver PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(gpio_driver PUBLIC .)
target_link_libraries(gpio_driver spi_driver utils)

add_library(uart_driver STATIC uart_driver.c)
if(DEBUG_UART)
target_compile_definitions(uart_driver PRIVATE -DDEBUG_UART)
message("DEBUG_UART ENABLED")
endif()
target_compile_options(uart_driver PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(uart_driver PUBLIC .)
target_link_libraries(uart_driver utils)

add_library(utils STATIC utils.c)
target_compile_options(utils PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(utils PUBLIC .)

add_library(hwcomm_s STATIC hw_main.c)
target_compile_options(hwcomm_s PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(hwcomm_s PUBLIC .)
target_link_libraries(hwcomm_s spi_driver uart_driver utils)

add_library(hwcomm SHARED hw_main.c)
target_compile_options(hwcomm PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(hwcomm PUBLIC .)
target_link_libraries(hwcomm spi_driver uart_driver utils)

set_target_properties(hwcomm PROPERTIES VERSION "${ADIMU_VERSION_MAJOR}.${ADIMU_VERSION_MINOR}.${ADIMU_VERSION_PATCH}" SOVERSION ${ADIMU_VERSION_MAJOR})
install(TARGETS hwcomm DESTINATION lib)
install(TARGETS hwcomm_s DESTINATION lib)