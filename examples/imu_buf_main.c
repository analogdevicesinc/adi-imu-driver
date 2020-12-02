/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		imu_buf_main.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Example usage of IMU + iSensor Buffer driver.
 **/


#include <stdio.h>
#include <stdlib.h>
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

void cleanup(adi_imu_Device_t *imu)
{
    uint16_t curBufCnt = 0;
    imubuf_StopCapture(imu, &curBufCnt);
    exit(0);
}

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.g = 1.0;
    imu.spiDev = "/dev/spidev1.0";
    imu.spiSpeed = 4000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 0; // stall time (us); to be safe

    /* initialize spi device */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* detect is buffer board is present */
    ret = imubuf_Detect(&imu);
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
    if ((ret = adi_imu_ConfigDataReady(&imu, DIO1, POSITIVE)) < 0) return ret;
    if ((ret = adi_imu_SetDataReady(&imu, ENABLE)) < 0) return ret;

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(&imu, 2000)) < 0) return ret;
    
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

    /* enable burst mode */
    imubuf_BufConfig_t config;
    config.overflowAction = 0;
    config.imuBurstEn = 0;
    config.bufBurstEn = 0;
    if ((ret = imubuf_ConfigBuf(&imu, config)) < 0) return ret;

    #define MAX_BUF_LENGTH 1000 // should greater than (imu_output_rate / fetch_rate). Ex: (4000Hz / 200Hz) = 20
    typedef struct {
        int16_t dummy1; // dummy response from previous last transaction
        int16_t dummy2; // dummy response for buf_retrieve command
        uint16_t sysEFlag;
        int16_t tempOut;
        int32_t gyroX;
        int32_t gyroY;
        int32_t gyroZ;
        int32_t acclX;
        int32_t acclY;
        int32_t acclZ;
        uint16_t dataCntOrTimeStamp;
        uint32_t crc;
        // uint32_t ts;
    } __attribute__ ((packed)) BufOutputRaw_t;

    BufOutputRaw_t bufRawOut[MAX_BUF_LENGTH] = {0};
    adi_imu_BurstOutput_t burstOut = {0};

    /* set register pattern to read/write IMU registers after every data ready interrupt */
    uint16_t bufPattern[] = {REG_SYS_E_FLAG, REG_TEMP_OUT,\
                            REG_X_GYRO_LOW, REG_X_GYRO_OUT, REG_Y_GYRO_LOW, REG_Y_GYRO_OUT, REG_Z_GYRO_LOW, REG_Z_GYRO_OUT,\
                            REG_X_ACCL_LOW, REG_X_ACCL_OUT, REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, REG_Z_ACCL_LOW, REG_Z_ACCL_OUT, \
                            REG_DATA_CNT, REG_CRC_LWR, REG_CRC_UPR}; //, REG_ISENSOR_BUF_TIMESTAMP_LWR, REG_ISENSOR_BUF_TIMESTAMP_UPR};
    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
    if ((ret = imubuf_SetPatternAuto(&imu, bufPatternLen, bufPattern)) < 0) return ret;

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* start capture */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StartCapture(&imu, IMUBUF_TRUE, &curBufCnt)) < 0) return ret;
    imu.spiDelay = 20; // kernel latency is large enough for stall time

    uint16_t buf_len = 0;
    int32_t readBufCnt = 0;
    for(int j=0; j<10; j++)
    {
        if ((ret = imubuf_ReadBufferAutoMax(&imu, 10, &readBufCnt, (uint16_t *)bufRawOut, &buf_len)) <0) return ret;
        for (int n=0; n<readBufCnt; n++)
        {
            // delay_MicroSeconds(1000);
            uint8_t* buf = (uint8_t*)(bufRawOut + n) + 4;
            // uint8_t* buf = (uint8_t*)((uint16_t *)bufRawOut + buf_len * n + 2);
			adi_imu_ScaleBurstOut_1(&imu, buf, FALSE, &burstOut);
            printf("datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc =%x\n", burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc);
        }
    }
    printf("\n\n");
    imu.spiDelay = 50; // stall time (us); to be safe

    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, &curBufCnt)) < 0) return ret;
    printf("\n## WARNING: Some samples may be missing if too high data rate (> 1800KHz) is used.\n\n");
    return 0;
}