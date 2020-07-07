cmake_minimum_required(VERSION 3.0.2)

add_library(spi_linux spi_driver.c)

if(DEBUG_SPI)
target_compile_definitions(spi_linux PRIVATE -DDEBUG_SPI)
message("DEBUG_SPI ENABLED")
endif()

target_compile_options(spi_linux PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(spi_linux PUBLIC . ../lib)
