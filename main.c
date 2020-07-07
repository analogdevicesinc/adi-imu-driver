#include <stdio.h>
#include "adi_imu_driver.h"
#include "spi_driver.h"

#include <math.h>
#define PI 3.141592653589793238

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16495;
    imu.spiDev = "/dev/spidev0.0";
    imu.spiSpeed = 2000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 0;

    /* 1. Initialize IMU */
    int ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return -1;

    /* 2. Set output data rate */
    uint16_t output_rate = 100; // Hz
    uint16_t dec_rate = (uint16_t)(4250 / output_rate) - 1;
    if ((ret = adi_imu_SetDecimationRate(&imu, dec_rate)) < 0) return ret;
    
    /* 3. Read and print IMU info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return -1;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return -1;

    /* 4. Burst read 10 samples */

    float acclLSB  = 0.25 * 9.81 / 65536000; /* 0.25mg/2^16 */
    float gyroLSB  = (4 * 10000 * 0.00625 / 655360000) * ( PI / 180); /* 0.00625 deg / 2^16 */
    float tempLSB = (1.0/80);

    printf("\nPerforming burst read..\n");
    adi_imu_BurstOutput_t out;
    for (int i=0; i<10; i++){
        if ((ret = adi_imu_ReadBurst(&imu, &out)) < 0) return -1;
        printf("\nsys_status=%x, temp=%f\u2103, accX=%f, accY=%f, accZ=%f, gyroX=%f, gyroY=%f, gyroZ=%f datacnt_Or_ts=%d crc=0x%x\n", out.sysEFlag, 25 + out.tempOut * tempLSB, out.accl.x * acclLSB, out.accl.y * acclLSB, out.accl.z * acclLSB, out.gyro.x * gyroLSB, out.gyro.y * gyroLSB, out.gyro.z * gyroLSB, out.dataCntOrTimeStamp, out.crc);
        double accelX = out.accl.x * acclLSB;
        double accelY = out.accl.y * acclLSB;
        double accelZ = out.accl.z * acclLSB;
        printf("Pitch = %f deg \n", 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI);
        printf("Roll = %f deg\n", 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI);
        adi_imu_DelayMicroSeconds(10000);
    }
    printf("\n");
    
    /* 5. Perform self test and display results*/
    if ((ret = adi_imu_PerformSelfTest(&imu)) < 0) return -1;

    return 0;
}