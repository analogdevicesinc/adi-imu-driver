/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		spi_driver.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for linux SPI driver.
 **/


#ifndef __SPI_DRIVER_H_
#define __SPI_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "adi_imu_driver.h"
	
int spi_Init(adi_imu_Device_t *pDevice);
int spi_ReadWrite(adi_imu_Device_t *pDevice, const uint8_t *txBuf, uint8_t *rxBuf, uint32_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx);
void delay_MicroSeconds (uint32_t microseconds);

#ifdef __cplusplus
}
#endif
#endif
