/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		gpio_test.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Example usage of gpio driver.
 **/

#include <stdio.h>
#include "gpio.h"

int main()
{
    char value[5];
    if (gpio_export("38")<0) return -1;
    
    if (gpio_set_direction("38", "out")<0) return -1;
    value[0] = '0';
    if (gpio_write("38", value)<0) return -1;
    if (gpio_set_direction("38", "in")<0) return -1;
    if (gpio_read("38", value)<0) return -1;
    printf("%s\n", value);

    if (gpio_set_direction("38", "out")<0) return -1;
    value[0] = '1';
    if (gpio_write("38", value)<0) return -1;
    if (gpio_set_direction("38", "in")<0) return -1;
    if (gpio_read("38", value)<0) return -1;
    printf("%s\n", value);

    if (gpio_set_direction("38", "out")<0) return -1;
    value[0] = '0';
    if (gpio_write("38", value)<0) return -1;
    if (gpio_set_direction("38", "in")<0) return -1;
    if (gpio_read("38", value)<0) return -1;
    printf("%s\n", value);

    if (gpio_set_direction("38", "out")<0) return -1;
    value[0] = '1';
    if (gpio_write("38", value)<0) return -1;
    if (gpio_set_direction("38", "in")<0) return -1;
    if (gpio_read("38", value)<0) return -1;
    printf("%s\n", value);

    if (gpio_unexport("38")<0) return -1;

}