/*******************************************************************************
 *   @file   spi_driver.h
 *   @brief  Header file of Spidev Driver.
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#ifndef __SPI_DRIVER_H_
#define __SPI_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "adi_imu_driver.h"
	
int adi_imu_SpiInit(adi_imu_Device_t *pDevice);
int adi_imu_SpiReadWrite(adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);
void adi_imu_DelayMicroSeconds (uint32_t microseconds);

#ifdef __cplusplus
}
#endif
#endif
