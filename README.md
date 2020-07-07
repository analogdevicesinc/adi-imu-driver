# ADI IMU driver

[![CI](https://github.com/spalani7/adi_imu_driver/workflows/CI/badge.svg?branch=master)](https://github.com/spalani7/adi_imu_driver/actions)

This library contains driver API to interface to most of the ADI IMUs.

Currently supported IMUs: ADIS16495


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
`adi_imu_Device_t` is defined in `lib/adi_imu_driver/adi_imu_driver.h`.

This library already comes with spi interface (`linux/spi_driver.c`) supported for platforms based on linux + spidev.

## License
MIT License
