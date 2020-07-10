#include <stdio.h>
#include "adi_imu_driver.h"
#include "spi_driver.h"

#define PI 3.141592653589793238

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.spiDev = "/dev/spidev0.0";
    imu.spiSpeed = 2000000;
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
    uint16_t output_rate = 2; // Hz
    uint16_t dec_rate = (uint16_t)(4250 / output_rate) - 1;
    if ((ret = adi_imu_SetDecimationRate(&imu, dec_rate)) < 0) return ret;
    
    /* Read and print IMU info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return -1;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return -1;

    /* Burst read 10 samples */
    printf("\nPerforming burst read..\n");
    adi_imu_BurstOutput_t out;
    char imu_out[200];
    for (int i=0; i<10; i++){
        if ((ret = adi_imu_ReadBurst(&imu, 9.81, &out)) < 0) return -1;
        sprintf(imu_out, "\ndatacnt_Or_ts=%d, sys_status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf\n", out.dataCntOrTimeStamp, out.sysEFlag, out.tempOut, out.accl.x, out.accl.y, out.accl.z, out.gyro.x, out.gyro.y, out.gyro.z);
        printf("%s\n", imu_out);
        delay_MicroSeconds(10000);
    }

    /* Perform self test and display results*/
    if ((ret = adi_imu_PerformSelfTest(&imu)) < 0) return -1;

    return 0;
}