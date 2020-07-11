#include <stdio.h>
#include <math.h>
#include "adi_imu_driver.h"
#include "spi_driver.h"

void printbuf(const char* header, uint16_t* buf, int buflen)
{
    printf("%s", header);
    for (int i=0; i<buflen; i++)
        printf("0x%04X ", buf[i]);
    printf("\n");
}

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.g = 1.0;
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
    if (ret != adi_imu_Success_e) return ret;

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(&imu, 100)) < 0) return ret;
    
    if ((ret = adi_imu_ConfigSyncClkMode(&imu, SYNC, DISABLE, FALLING_EDGE, DIO1)) < 0) return ret;

    /* Read and print IMU info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;

    /* Burst read 10 samples */
    printf("\nPerforming burst read..\n");
    adi_imu_BurstOutput_t out;
    uint8_t burstBuf[MAX_BRF_LEN_BYTES] = {0};
    unsigned pPayloadOffset = 0;
    
    // Using adi_imu_ReadBurstRaw
    for (int i=0; i<10; i++){
        if ((ret = adi_imu_ReadBurstRaw(&imu, burstBuf, &pPayloadOffset)) < 0) return ret;
        printbuf("\nBuffer: ", (uint16_t*)burstBuf, MAX_BRF_LEN_BYTES/2);
        adi_imu_ScaleBurstOut_1(&imu, &burstBuf[pPayloadOffset], &out);
        printf("datacnt_Or_ts=%d, sys_status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf\n", out.dataCntOrTimeStamp, out.sysEFlag, out.tempOut, out.accl.x, out.accl.y, out.accl.z, out.gyro.x, out.gyro.y, out.gyro.z);
        printf("Pitch = %f deg \n", 180 * atan2(out.accl.x, sqrt(out.accl.y*out.accl.y + out.accl.z*out.accl.z))/M_PI);
        printf("Roll = %f deg\n", 180 * atan2(out.accl.y, sqrt(out.accl.x*out.accl.x + out.accl.z*out.accl.z))/M_PI);
        delay_MicroSeconds(10000);
    }

    // Using adi_imu_ReadBurst
    for (int i=0; i<10; i++){
        if ((ret = adi_imu_ReadBurst(&imu, burstBuf, &out)) < 0) return ret;
        printf("\ndatacnt_Or_ts=%d, sys_status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf\n", out.dataCntOrTimeStamp, out.sysEFlag, out.tempOut, out.accl.x, out.accl.y, out.accl.z, out.gyro.x, out.gyro.y, out.gyro.z);
        printf("Pitch = %f deg \n", 180 * atan2(out.accl.x, sqrt(out.accl.y*out.accl.y + out.accl.z*out.accl.z))/M_PI);
        printf("Roll = %f deg\n", 180 * atan2(out.accl.y, sqrt(out.accl.x*out.accl.x + out.accl.z*out.accl.z))/M_PI);
        delay_MicroSeconds(1000);
    }
    printf("\n");
    
    /* Perform self test and display results*/
    if ((ret = adi_imu_PerformSelfTest(&imu)) < 0) return ret;

    return 0;
}