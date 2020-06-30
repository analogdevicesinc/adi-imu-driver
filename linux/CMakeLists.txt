cmake_minimum_required(VERSION 3.0.2)

add_library(spi_linux spi_driver.c)
target_include_directories(spi_linux PUBLIC .)
target_include_directories(spi_linux PUBLIC ../lib)
