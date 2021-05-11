/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		spi_test.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Example usage of SPI driver.
 **/

#include "spi_driver.h"

void test_prod_id(adi_imu_Device_t* imu)
{
   uint8_t buf[6] = { 0x80, 0x00, 0x7E, 0x00, 0x00, 0x00};

   printf("\n\n[SPI TX]: ");
   for (int i=0; i<6; i++) printf("0x%02X ", buf[i]);
   printf("\n");

   /* send read request */
   if (spi_ReadWrite(&imu->spiDev, buf, buf, 2, 3, 1, 0) < 0) return;

   printf("[SPI RX]: ");
   for (int i=0; i<6; i++) printf("0x%02X ", buf[i]);
   printf("\n");

   printf("\nProduct ID: %d\n",  buf[5] | ((uint16_t)buf[4] << 8));
}

void test_spi_cmd(adi_imu_Device_t* imu)
{
   unsigned const xferlen = 4;
   uint8_t tx_buf[4] = {0x80, 0xFD, 0x00, 0x00};
   uint8_t rx_buf[4] = {0x00, 0x00};

   printf("\n\n[SPI TX]: ");
   for (int i=0; i<xferlen; i++) printf("0x%02X ", tx_buf[i]);
   printf("\n");

   /* send read request */
   if (spi_ReadWrite(&imu->spiDev, tx_buf, rx_buf, 2, 2, 1, 0) < 0) return;

   printf("[SPI RX]: ");
   for (int i=0; i<xferlen; i++) printf("0x%02X ", rx_buf[i]);
   printf("\n");

}

int main()
{
   adi_imu_Device_t imu;
   imu.prodId = 16495;
   imu.g = 1.0;
   imu.spiDev.dev = "/dev/spidev1.0";
   imu.spiDev.speed = 5000000;
   imu.spiDev.mode = 3;
   imu.spiDev.bitsPerWord = 8;
   imu.spiDev.delay = 0;

   /* Initialize spi */
   int ret = spi_Init(&imu.spiDev);
   if (ret < 0) return ret;

   //test_prod_id(&imu);

   test_spi_cmd(&imu);

   return 0;
}
