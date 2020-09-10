cmake_minimum_required(VERSION 3.0.2)

add_library(adimubuf SHARED imu_spi_buffer.c)
target_include_directories(adimubuf PUBLIC .)
target_link_libraries(adimubuf adimu)
set_target_properties(adimubuf PROPERTIES VERSION "${ADIMU_VERSION_MAJOR}.${ADIMU_VERSION_MINOR}.${ADIMU_VERSION_PATCH}" SOVERSION ${ADIMU_VERSION_MAJOR})
install(TARGETS adimubuf DESTINATION /usr/local/lib)
install(FILES ./imu_spi_buffer.h DESTINATION /usr/local/include)
install(FILES ./imu_spi_buffer_regmap.h DESTINATION /usr/local/include)
