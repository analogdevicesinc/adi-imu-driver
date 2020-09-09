cmake_minimum_required(VERSION 3.0.2)

add_library(fileio fileio.cpp)
target_compile_options(fileio PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(fileio PUBLIC .)
