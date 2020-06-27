#include <stdio.h>
#include "adi_imu_driver.h"

int main()
{
	adi_imu_Device_t imu;
	imu.prodId = 497;
	imu.spiDev = "/dev/spidev0.0";
	imu.spiSpeed = 2000000;
	imu.spiMode = 3;
	imu.spiBitsPerWord = 8;
	imu.spiDelay = 0;

	int ret = adi_imu_Init(&imu);
	if (ret != adi_imu_Success_e) return -1;

	return 0;
}