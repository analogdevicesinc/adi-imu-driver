# ADI IMU driver

[![CI](https://github.com/spalani7/adi_imu_driver/workflows/CI/badge.svg?branch=master)](https://github.com/spalani7/adi_imu_driver/actions)

This library contains driver API to interface to most of the ADI IMUs.

Currently supported IMUs: 
* [ADIS16495](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16495.pdf)

Also, this repo contains library(`lib/imu_buf`) for driving [iSensor-SPI-buffer](https://github.com/ajn96/iSensor-SPI-Buffer) board which helps buffer data at very high rate for non-real time application that requires real-time data. This library depends on main imu driver(`lib/imu`). See link for more details. 

Both the libraries are tested on Jetson Nano and can be easily run on any linux based platform with spi-dev support.
For non-spidev and non-linux platforms, see [below](#porting) for details.


## Contents
* `lib/imu` - contains imu driver code.
* `lib/imu_buf` - contains driver for [iSensor-SPI-buffer](https://github.com/ajn96/iSensor-SPI-Buffer) board. 
* `linux/` - contains linux specific spi driver and utils .
* `tests/` - contains unit tests (TO BE IMPLEMENTED).
* `CMakeLists.txt` - main cmake build file.
* `examples/imu_main.c` - simple example on how to drive any adi imu.
* `examples/imu_buf_main.c` - simple example on how to drive iSensor spi buffer board.


## Build
```bash
$ cd adi_imu_driver
$ mkdir build
$ cd build
$ cmake [OPTIONS] ..
$ make -j2
```

`OPTIONS`:  
`-DBAREMETAL=(y|n)`: to compile the library for baremetal platforms. If enabled, you should implement below methods for your platform.  
`-DBUILDTYPE=<DEBUG|RELEASE>`: set build type (default: DEBUG).  
`-DDEBUG_SPI=(y|n)`: set DEBUG mode for spi transactions (default: n).  

## Test
```bash
$ cd build
$ ctest -V
```

## Porting
Below functions should be provided when ported to different platform:

```c
extern int adi_imu_SpiInit (adi_imu_Device_t *pDevice);
extern int adi_imu_SpiReadWrite (adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);
extern void adi_imu_DelayMicroSeconds (uint32_t microseconds);
```

`Note`: Need to add `lib/imu/adi_imu_driver.h` header file in your implementation. 

`For linux`:

This library already comes with linux spi interface (`linux/spi_driver.c`) that can be used on linux + spidev platforms.

## License
MIT License
