#include <stdio.h>
#include <stdlib.h>
#include "adi_imu_driver.h"
#include "spi_driver.h"

void cleanup(adi_imu_Device_t *imu)
{
    adi_imu_PerformSelfTest(imu);
    exit(0);
}

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.g = 9.81;
    imu.spiDev = "/dev/spidev0.1";
    imu.spiSpeed = 8000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 0;

    /* Initialize spi */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* Initialize IMU */
    ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return -1;

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(&imu, 2000)) < 0) return ret;
    
    /* Read and print IMU info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return -1;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return -1;

    /* Burst read 10 samples if buffer board not enabled */
    if (!BUFF_EN) {
        printf("\nPerforming burst read..\n");
        adi_imu_BurstOutput_t out;
        uint8_t rawOut[MAX_BRF_LEN_BYTES];
        char imu_out[200];
        uint16_t curDataCnt = 0;
        for (int i=0; i<20000; i++){
            if ((ret = adi_imu_ReadBurst(&imu, rawOut, &out, 1)) < 0) return -1;
            uint16_t dc = out.dataCntOrTimeStamp;
            // if (i == 1444) dc = 0x9; // insert fail condition
            if (i>1 && dc != 0 && dc != curDataCnt) {
                if (dc % 1000 == 0) printf("%d\n", dc);
                if (curDataCnt != 0 && dc != (curDataCnt+1)){
                    printf("%d\n%d ##", curDataCnt, dc);
                    printf("\nTEST FAILED\n");
                    cleanup(&imu);
                }
                curDataCnt = dc;
            }
            sprintf(imu_out, "\ndatacnt_Or_ts=%d, sys_status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf\n", out.dataCntOrTimeStamp, out.sysEFlag, out.tempOut, out.accl.x, out.accl.y, out.accl.z, out.gyro.x, out.gyro.y, out.gyro.z);
            // printf("%s\n", imu_out);
        }
    }
    else {
        printf("\nBuffer board enabled, skipping burst read test.");
    }

    /* Perform self test and display results*/
    if ((ret = adi_imu_PerformSelfTest(&imu)) < 0) return -1;

    return 0;
}