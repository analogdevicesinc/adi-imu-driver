/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		uart_test.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Example usage of UART driver.
 **/

#include <string.h>
#include <stdlib.h>
#include "uart_driver.h"

void test_prod_id(adi_imu_UartDevice_t* imu)
{
    unsigned char txbuf[100] = {0};
    unsigned char rxbuf[100] = {0};
    uint16_t rxbuf16[10] = {0};

    int ret = 0;

    snprintf((char*)txbuf, 100, "read 0\r\n");
    printf("TX: %s\n", (char*)txbuf);
    if ((ret = uart_WriteRead(imu, txbuf, strlen((char*)txbuf), rxbuf, 100)) < 0) return;
    printf("RX: %s\n", (char*)rxbuf);

    uart_FlushInput(imu);
    uart_FlushOutput(imu);

    snprintf((char*)txbuf, 100, "echo 0\r\n");
    printf("TX: %s\n", (char*)txbuf);
    if ((ret = uart_Write(imu, txbuf, strlen((char*)txbuf))) < 0) return;
    printf("RX: %s\n", (char*)rxbuf);

    uart_FlushInput(imu);
    uart_FlushOutput(imu);

    snprintf((char*)txbuf, 100, "read 0 4\r\n");
    printf("TX: %s\n", (char*)txbuf);
    if ((ret = uart_WriteRead(imu, txbuf, strlen((char*)txbuf), rxbuf, 100)) < 0) return;
    printf("RX: %s\n", (char*)rxbuf);
    uart_RxParse16bit(rxbuf, rxbuf16, 10);
    printf("RXParsed: %x %x %x\n", rxbuf16[0], rxbuf16[1], rxbuf16[2]);
    if (rxbuf16[0]!=0)
    {
        snprintf((char*)txbuf, 100, "write 0 0\r\n");
        printf("TX: %s\n", txbuf);
        if ((ret = uart_Write(imu, txbuf, strlen((char*)txbuf))) < 0) return;
    }
    snprintf((char*)txbuf, 100, "read 7E\r\n");
    printf("TX: %s\n", (char*)txbuf);
    if ((ret = uart_WriteRead(imu, txbuf, strlen((char*)txbuf), rxbuf, 100)) < 0) return;
    printf("RX: %s\n", (char*)rxbuf);
    uart_RxParse16bit(rxbuf, rxbuf16, 10);
    printf("Product ID: %d\n", rxbuf16[0]);
}

int main()
{
   adi_imu_Device_t imu;
   imu.prodId = 16495;
   imu.g = 1.0;
   imu.devType = IMU_HW_UART;
   imu.uartDev.dev = "/dev/ttyACM0";
   imu.uartDev.baud = 1000000; // 115200, 921600

   int ret = uart_Init(&imu.uartDev);
   if (ret < 0) return ret;

   test_prod_id(&imu.uartDev);

   return 0;
}
