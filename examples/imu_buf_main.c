#include <stdio.h>
#include <math.h>
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include "imu_spi_buffer.h"

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
    imu.spiSpeed = 16000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 0; // stall time (us); to be safe

    /* initialize spi device */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* Initialize IMU BUF first to stop any activity*/
    ret = imubuf_init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    /* Read and print iSensor SPI Buffer info and config*/
    imubuf_DevInfo_t imuBufInfo;
    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* Initialize IMU */
    ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    /* Set DATA ready pin */
    if ((ret = adi_imu_ConfigDataReady(&imu, DIO1, RISING_EDGE)) < 0) return ret;
    if ((ret = adi_imu_SetDataReady(&imu, ENABLE)) < 0) return ret;

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(&imu, 100)) < 0) return ret;
    
    /* Read and print IMU device info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;

    /* set DIO pin config (both input and output) for iSensor SPI buffer */
    imubuf_ImuDioConfig_t dioConfig;
    dioConfig.dataReadyPin = IMUBUF_DIO1;
    dioConfig.dataReadyPolarity = RISING_EDGE;
    dioConfig.ppsPin = 0x00;
    dioConfig.ppsPolarity = 0x0;
    dioConfig.passThruPin = 0x00;
    dioConfig.watermarkIrqPin = IMUBUF_DIO2;
    dioConfig.overflowIrqPin = 0x00;
    dioConfig.errorIrqPin = 0x00;
    if ((ret = imubuf_ConfigDio(&imu, dioConfig)) < 0) return ret;

    #define MAX_BUF_LENGTH 50 // should greater than (imu_output_rate / fetch_rate). Ex: (4000Hz / 200Hz) = 20
    typedef struct {
        adi_imu_BurstOutputRaw_t burstRawOut;
        uint32_t utcTime;
    } BufOutputRaw_t;

    BufOutputRaw_t bufRawOut[MAX_BUF_LENGTH] = {0};
    adi_imu_BurstOutput_t burstOut = {0};

    /* set register pattern to read/write IMU registers after every data ready interrupt */
    uint16_t bufPattern[] = {REG_SYS_E_FLAG, REG_TEMP_OUT, \
                            REG_X_GYRO_LOW, REG_X_GYRO_OUT, REG_Y_GYRO_LOW, REG_Y_GYRO_OUT, REG_Z_GYRO_LOW, REG_Z_GYRO_OUT, \
                            REG_X_ACCL_LOW, REG_X_ACCL_OUT, REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, REG_Z_ACCL_LOW, REG_Z_ACCL_OUT, \
                            REG_DATA_CNT, REG_CRC_LWR, REG_CRC_UPR, REG_ISENSOR_BUF_TIMESTAMP_LWR, REG_ISENSOR_BUF_TIMESTAMP_UPR};
    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
    if ((ret = imubuf_SetPatternAuto(&imu, bufPatternLen, bufPattern)) < 0) return ret;
    
    /* read back pattern for sanity check */
    uint16_t patternChk[(const int)bufPatternLen];
    uint16_t buflen = 0;
    if ((ret = imubuf_GetPattern(&imu, &buflen, patternChk)) < 0) return ret;
    printbuf("Pattern: ", patternChk, buflen);

    /* start capture */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StartCapture(&imu, IMUBUF_FALSE, &curBufCnt)) < 0) return ret;

    uint16_t buf_len = 0;
    int32_t readBufCnt = 0;
    printf("\nReading max 5 buffers at a time\n\n");
    for(int j=0; j<5; j++){
        delay_MicroSeconds(1000000);
        if ((ret = imubuf_ReadBufferAutoMax(&imu, 5, &readBufCnt, (uint16_t *)bufRawOut, &buf_len)) < 0) return ret;
        for (int n=0; n<readBufCnt; n++) {
            uint16_t* buf = (uint16_t *)bufRawOut;
            printbuf("\nBuffer: ", &buf[n * buf_len], buf_len);
            
            burstOut.sysEFlag = buf[0 + n * buf_len];
            adi_imu_ScaleTempOut(&imu, buf[1 + n * buf_len], &burstOut.tempOut);

            adi_imu_GyroOutputRaw32_t* gRaw = (adi_imu_GyroOutputRaw32_t*) (buf + 2 + n * buf_len);
            adi_imu_ScaleGyro32Out(&imu, gRaw, &burstOut.gyro);

            adi_imu_AcclOutputRaw32_t* aRaw = (adi_imu_AcclOutputRaw32_t*) (buf + 8 + n * buf_len);
            adi_imu_ScaleAccl32Out(&imu, aRaw, &burstOut.accl);

            burstOut.dataCntOrTimeStamp = buf[14 + n * buf_len];
            burstOut.crc = buf[15 + n * buf_len] | ((uint32_t)buf[16 + n * buf_len]) << 16;
            uint32_t utc = buf[17 + n * buf_len] | ((uint32_t)buf[18 + n * buf_len]) << 16;

            printf("datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc=%d ts=%d\n", burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc, utc);
        }
    }

    // printf("\nReading all available data at a time\n\n");
    // for(int j=0; j<5; j++)
    // {
    //     // delay_MicroSeconds(10000);
    //     if ((ret = imubuf_ReadBufferAutoMax(&imu, MAX_BUF_LENGTH, &readBufCnt, (uint16_t *)bufRawOut, &buf_len)) <0) return ret;
    //     for (int n=0; n<readBufCnt; n++)
    //     {
    //         // printbuf("\nBuffer: ", (uint16_t*)&bufRawOut[n], buf_len);
	// 		adi_imu_ScaleBurstOut_2(&imu, &(bufRawOut[n].burstRawOut), &burstOut);
    //         // uint16_t* buf = (uint16_t *)bufRawOut;
    //         // uint32_t utc = buf[17 + n * buf_len] | ((uint32_t)buf[18 + n * buf_len]) << 16;
    //         printf("datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc =%d\n", burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc);
    //     }
    // }
    // printf("\n\n");

    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, &curBufCnt)) < 0) return ret;

    return 0;
}