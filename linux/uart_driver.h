/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		uart_driver.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for linux Serial devices.
 **/


#ifndef __UART_DRIVER_H_
#define __UART_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_imu_common.h"

int uart_Init(adi_imu_UartDevice_t* device);
int uart_Read(adi_imu_UartDevice_t* device, uint8_t *buf, size_t bufLen);
int uart_ReadLine(adi_imu_UartDevice_t* device, uint8_t *buf, size_t bufLen);
int uart_FlushInput(adi_imu_UartDevice_t* device);
int uart_FlushOutput(adi_imu_UartDevice_t* device);
int uart_Write(adi_imu_UartDevice_t* device, const uint8_t *buf, size_t bufLen);
int uart_WriteRead(adi_imu_UartDevice_t* device, const uint8_t *txBuf, size_t txBufLen, uint8_t *rxBuf, size_t rxBufLen);
int uart_RxParse16bit(uint8_t* in, uint16_t* out, size_t len);

#ifdef __cplusplus
}
#endif
#endif
