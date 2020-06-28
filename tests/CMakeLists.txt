cmake_minimum_required(VERSION 3.0.2)

add_executable(test_imu_16495 test_imu_16495.c)
target_link_libraries(test_imu_16495 adi_imu_driver spi_linux util)
add_test(test_imu_16495 test_imu_16495)
