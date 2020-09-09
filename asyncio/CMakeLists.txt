cmake_minimum_required(VERSION 3.0.2)

add_library(asyncio asyncio.cpp)
target_compile_options(asyncio PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(asyncio PUBLIC .)
