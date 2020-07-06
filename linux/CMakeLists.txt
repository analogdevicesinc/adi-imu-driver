cmake_minimum_required(VERSION 3.0.2)

add_library(spi_linux spi_driver.c)
target_compile_options(spi_linux PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(spi_linux PUBLIC . ../lib)
