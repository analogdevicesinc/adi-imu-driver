cmake_minimum_required(VERSION 3.0.2)

add_library(spi_driver spi_driver.c)

if(DEBUG_SPI)
target_compile_definitions(spi_driver PRIVATE -DDEBUG_SPI)
message("DEBUG_SPI ENABLED")
endif()

target_compile_options(spi_driver PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(spi_driver PUBLIC . ../lib/imu)

add_library(gpio_driver gpio.c)
target_compile_options(gpio_driver PUBLIC -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)
target_include_directories(gpio_driver PUBLIC . ../lib/imu)
target_link_libraries(gpio_driver spi_driver)