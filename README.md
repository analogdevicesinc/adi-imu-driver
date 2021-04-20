# ADI IMU driver

[![CI](https://github.com/spalani7/adi_imu_driver/workflows/CI/badge.svg?branch=master)](https://github.com/spalani7/adi_imu_driver/actions)

This library contains driver API to interface to IMUs from Analog Devices.
One can also use libiio to interface to IMUs which is the recommended one. But this library can provide more control from user space and also easily portable to many systems. This is a pure C library. You can port to any systems by just implementing your own spi library. See [below](#porting) for details.

Currently supported IMUs: 
* [ADIS16495](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16495.pdf)

## iSensor buffer board

At high speed data rates (greater than 1KHz), non-baremetal systems might have non-deterministic SPI interface which results in sample drops. [iSensor-SPI-buffer](https://github.com/ajn96/iSensor-SPI-Buffer) board, developed by Juan and Alex, can act as a buffer between host and IMU providing necessary capabability to acquire data at very high speed. This repo implements `imu_buf` library (`lib/imu_buf`) for interfacing with `iSensor-SPI-buffer` board.

Both the libraries are tested on Jetson Nano and can be easily run on any linux based platform with spi-dev support.


## Contents
* `lib/imu` - contains imu driver code.
* `lib/imu_buf` - contains driver for [iSensor-SPI-buffer](https://github.com/ajn96/iSensor-SPI-Buffer) board. 
* `linux/` - contains linux specific spi driver and utils .
* `tests/` - contains unit tests (TO BE IMPLEMENTED).
* `CMakeLists.txt` - main cmake build file.
* `examples/imu_test.c` - simple example on how to use imu/imu buf library.
* `examples/spi_test.c` - simple example on how to use spi library.
* `examples/gpio_test.c` - simple example on how to use gpio library.


## Build
```bash
$ cd adi_imu_driver
$ mkdir build
$ cd build
$ cmake [OPTIONS] ..
$ make -j2
```

`OPTIONS`:  

`-DCMAKE_BUILD_TYPE=<DEBUG|RELEASE>`: set build type (default: RELEASE) 

`-DBAREMETAL=(y|n)`: to compile the library for baremetal platforms. If enabled, you should implement below methods for your platform. 

`-DBUILD_SHARED_LIBS=<ON|OFF>`: set build type (default: OFF) 


## Porting
Below functions should be provided when ported to different platform:

```c
int spi_Init(adi_imu_Device_t *pDevice);
int spi_ReadWrite(adi_imu_Device_t *pDevice, const uint8_t *txBuf, uint8_t *rxBuf, uint32_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx);
void delay_MicroSeconds (uint32_t microseconds);
```

`Note`: Need to add `lib/imu/adi_imu_driver.h` and/or  `lib/imu/imu_spi_buffer.h` header file in your implementation. 

`For Linux`:

This library already comes with Linux SPI interface (`linux/spi_driver.c`) that can be used on Linux + spidev platforms.


## Acknowledgements

[Juan Chong](https://github.com/juchong)  
[Alex Nolan](https://github.com/ajn96)


## License
MIT License
