#include <stdio.h>
#include "adi_imu_driver.h"
#include "util.h"

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

    int ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return -1;

    // set ~100Hz data rate
    if ((ret = adi_imu_SetDecimationRate(&imu, 43)) < 0) return ret; /* Output data rate = 4250 / (DEC_RATE + 1) */

    adi_imu_DevInfo_t imuInfo;
    adi_imu_GetDevInfo(&imu, &imuInfo);
    adi_imu_PrintDevInfo(&imu, &imuInfo);

    float acclLSB  = 0.25 * 9.81 / 65536000; /* 0.25mg/2^16 */
    float gyroLSB  = (4 * 10000 * 0.00625 / 655360000) * ( PI / 180); /* 0.00625 deg / 2^16 */
    float tempLSB = (1.0/80);

    adi_imu_BurstOutput_t out;
    char imu_out[200];
    for (int i=0; i<10; i++){
        if ((ret = adi_imu_ReadBurst(&imu, &out)) < 0) return -1;
        sprintf(imu_out, "sys_status=%x, temp=%fC, accX=%f, accY=%f, accZ=%f, gyroX=%f, gyroY=%f, gyroZ=%f\n", out.sysEFlag, 25 + out.tempOut * tempLSB, out.accl.x * acclLSB, out.accl.y * acclLSB, out.accl.z * acclLSB, out.gyro.x * gyroLSB, out.gyro.y * gyroLSB, out.gyro.z * gyroLSB);
        printf("%s\n", imu_out);
        sleep_us(10000);
    }
    return 0;
}