#/**
#  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
#  * This software is proprietary to Analog Devices, Inc. and its licensors.
#  *
#  * Use of this file is governed by the license agreement
#  * included in this repository.
#  *
#  * @file		    CMakeLists.txt
#  * @author		Sundar Palani (sundar.palani@analog.com)
#  * @brief 		CMake module for asyncio library.
# **/

cmake_minimum_required(VERSION 3.0.2)

add_library(asyncio STATIC asyncio.cpp)
target_compile_options(asyncio PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(asyncio PUBLIC .)
