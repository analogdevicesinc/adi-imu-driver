cmake_minimum_required(VERSION 3.0.2)

add_executable(test1 test_imu_16495.c)
target_link_libraries(test1 adimu spi_linux)
add_test(test1 test1)

add_executable(test2 test_imu_buffer_data_seq.c)
target_link_libraries(test2 adimu adimubuf spi_linux m)
add_test(test2 test2)

add_executable(test3 test_imu_buffer_data_seq_burst_mode.c)
target_link_libraries(test3 adimu adimubuf spi_linux m)
add_test(test3 test3)