#include "spi_driver.h"

int main()
{
   adi_imu_Device_t imu;
   imu.prodId = 16495;
   imu.g = 1.0;
   imu.spiDev = "/dev/spidev1.0";
   imu.spiSpeed = 2000000;
   imu.spiMode = 3;
   imu.spiBitsPerWord = 8;
   imu.spiDelay = 0;

   /* Initialize spi */
   int ret = spi_Init(&imu);
   if (ret < 0) return ret;

   uint8_t buf[6] = { 0x80, 0x00, 0x7E, 0x00, 0x00, 0x00};

   printf("\n\n[SPI TX]: ");
   for (int i=0; i<6; i++) printf("0x%02X ", buf[i]);
   printf("\n");

   /* send read request */
   if (spi_ReadWrite(&imu, buf, buf, 2, 3, 1, 0) < 0) return adi_spi_RwFailed_e;

   printf("[SPI RX]: ");
   for (int i=0; i<6; i++) printf("0x%02X ", buf[i]);
   printf("\n");

   printf("\nProduct ID: %d\n",  buf[5] | ((uint16_t)buf[4] << 8));
   return 0;
}