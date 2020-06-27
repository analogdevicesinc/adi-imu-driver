# ADI IMU driver

![repo status](https://img.shields.io/badge/repo--status-WIP-yellow.svg)

This library contains driver API to interface to most of the ADI IMUs.

Currently supported IMUs: ADIS16495


## Build
```bash
$ cd adi_imu_driver
$ mkdir build
$ cd build
$ cmake [OPTIONS] ..
$ make -j2
$ ./main # to test with IMU
```

`OPTIONS`:  
`-DBAREMETAL`: to compile the library for baremetal platforms.  
`-DBUILDTYPE=<DEBUG|RELEASE>`: set build type (default: DEBUG).  

## Porting
Below functions should be provided when ported to different platform:

```c
extern int adi_imu_SpiInit (adi_imu_Device_t *pDevice);
extern int adi_imu_SpiReadWrite (adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);
```
`adi_imu_Device_t` is defined in `src/adi_imu_driver/adi_imu_driver.h`.

This library already comes with spi interface (`src/spi_linux`) supported for platforms based on linux.

## License
MIT License
