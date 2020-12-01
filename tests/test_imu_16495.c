#include <stdio.h>
#include <stdlib.h>
#include "adi_imu_driver.h"
#include "spi_driver.h"

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16495;
    imu.g = 9.81;
    imu.spiDev = "/dev/spidev0.1";
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 0;
    if (BUFF_EN) {
        imu.spiSpeed = 12000000;
    }
    else {
        imu.spiSpeed = 2000000;
    }

    /* Initialize spi */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* Initialize IMU */
    ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return -1;

    /* Set output data rate */
    if (BUFF_EN) {
        if ((ret = adi_imu_SetOutputDataRate(&imu, 2000)) < 0) return ret;
    }
    else {
        if ((ret = adi_imu_SetOutputDataRate(&imu, 1000)) < 0) return ret;
    }    
    
    /* Read and print IMU info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;

    /* Perform self test and display results*/
    if ((ret = adi_imu_PerformSelfTest(&imu)) < 0) return ret;

    /* Burst read 1000 samples if buffer board not enabled */
    if (!BUFF_EN) {
        printf("\nPerforming burst read..\n");
        adi_imu_BurstOutput_t out;
        uint8_t rawOut[MAX_BRF_LEN_BYTES];
        char imu_out[200];
        uint16_t curDataCnt = 0;
        for (int i=0; i<1000; i++){
            if ((ret = adi_imu_ReadBurst(&imu, rawOut, 1, &out)) < 0) return ret;
            printf("%u,", out.dataCntOrTimeStamp);
            printf("%u,", out.sysEFlag);
            printf("%f,", out.tempOut);
            printf("%f,", out.gyro.x);
            printf("%f,", out.gyro.y);
            printf("%f,", out.gyro.z);
            printf("%f,", out.accl.x);
            printf("%f,", out.accl.y);
            printf("%f,", out.accl.z);
            printf("%u\n", out.crc);
        }
    }
    else {
        printf("\nBuffer board enabled, skipping burst read test.");
    }
    return 0;
}