cmake_minimum_required(VERSION 3.0.2)

add_library(util util.c)
target_include_directories(util PUBLIC .)
