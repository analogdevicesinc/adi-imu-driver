/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		gpio.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for linux gpio driver.
 **/


#ifndef __GPIO_DRIVER_H_
#define __GPIO_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

int gpio_export(const char* gpio);
int gpio_unexport(const char* gpio);
int gpio_set_direction(const char* gpio, const char* direction);
int gpio_read(const char* gpio, char* val);
int gpio_write(const char* gpio, const char* val);

#ifdef __cplusplus
}
#endif
#endif
