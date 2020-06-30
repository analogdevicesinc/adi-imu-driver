cmake_minimum_required(VERSION 3.0.2)

add_executable(test_imu_16495 test_imu_16495.c)
target_link_libraries(test_imu_16495 adimu spi_linux)
add_test(test_imu_16495 test_imu_16495)
