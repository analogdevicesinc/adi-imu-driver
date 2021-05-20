/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		test_imu_buffer_data_seq_burst_mode.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		A simple test for ADIS16495 IMU with iSensor-Spi-Buffer in non-burst mode.
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
    int ret = 0;
    imubuf_StopCapture(imu, &curBufCnt);

    // read any error flags
    imubuf_DevInfo_t imuBufInfo;
    imubuf_GetInfo(imu, &imuBufInfo);
    imubuf_PrintInfo(imu, &imuBufInfo);
    exit(0);
}

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.g = 1.0;
    imu.spiDev.dev = "/dev/spidev1.0";
    imu.spiDev.speed = 9000000;
    imu.spiDev.mode = 3;
    imu.spiDev.bitsPerWord = 8;
    imu.spiDev.delay = 0; // stall time (us); to be safe
    imu.enable_buffer = IMU_TRUE;

    /* initialize spi device */
    int ret = hw_Init(&imu);
    if (ret < 0) return ret;

    /* Initialize IMU BUF first to stop any activity*/
    ret = imubuf_init(&imu);
    if (ret != Err_imu_Success_e) return ret;

    /* Read and print iSensor SPI Buffer info and config*/
    imubuf_DevInfo_t imuBufInfo;
    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* software reset */
    // if ((ret = imubuf_SoftwareReset(&imu)) < 0) return ret;
    
    /* Initialize IMU */
    ret = adi_imu_Init(&imu);
    if (ret != Err_imu_Success_e) return ret;

    /* Set DATA ready pin */
    if ((ret = adi_imu_ConfigDataReady(&imu, IMU_DIO1, IMU_POS_POLARITY)) < 0) return ret;
    if ((ret = adi_imu_SetDataReady(&imu, IMU_ENABLE)) < 0) return ret;

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(&imu, 1000)) < 0) return ret;
    
    /* Read and print IMU device info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;

    /* set DIO pin config (both input and output) for iSensor SPI buffer */
    imubuf_ImuDioConfig_t dioConfig;
    dioConfig.dataReadyPin = IMUBUF_DIO1;
    dioConfig.dataReadyPolarity = IMU_RISING_EDGE;
    dioConfig.ppsPin = 0x00;
    dioConfig.ppsPolarity = 0x0;
    dioConfig.ppsFreq = IMUBUF_PPS_FREQ_1HZ;
    dioConfig.passThruPin = 0x00;
    dioConfig.watermarkIrqPin = IMUBUF_DIO2;
    dioConfig.overflowIrqPin = 0x00;
    dioConfig.errorIrqPin = 0x00;
    if ((ret = imubuf_SetDioConfig(&imu, &dioConfig)) < 0) return ret;

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
    uint16_t bufPattern[] = {   
        IMUBUF_PATTERN_WRITE_REG(REG_PAGE_ID, 0x00), 
        IMUBUF_PATTERN_READ_REG(REG_SYS_E_FLAG), \
        IMUBUF_PATTERN_READ_REG(REG_TEMP_OUT), \
        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_LOW), \
        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_OUT), \
        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_LOW), \
        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_OUT), \
        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_LOW), \
        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_OUT), \
        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_LOW), \
        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_OUT), \
        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_LOW), \
        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_OUT), \
        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_LOW), \
        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_OUT), \
        IMUBUF_PATTERN_READ_REG(REG_DATA_CNT), \
        IMUBUF_PATTERN_READ_REG(REG_CRC_LWR), \
        IMUBUF_PATTERN_READ_REG(REG_CRC_UPR), 0x0000, \
    };
    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
    if ((ret = imubuf_SetPatternRaw(&imu, bufPatternLen, bufPattern)) < 0) return ret;

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* start capture */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StartCapture(&imu, IMU_TRUE, &curBufCnt)) < 0) return ret;
    imu.spiDev.delay = 0;  // kernel latency is large enough for stall time

    uint16_t buf_len = 0;
    int32_t readBufCnt = 1;
    uint16_t curDataCnt = 0;
    for(int j=0; j<10000; j++){
        if ((ret = imubuf_ReadBufferN(&imu, readBufCnt, (uint16_t *)bufRawOut, &buf_len)) < 0) return ret;
        // printf("readBufCnt %d buf_len %d\n", readBufCnt, buf_len);
        for (int n=0; n<readBufCnt; n++) {
            uint16_t* buf = (uint16_t*)bufRawOut + n*buf_len;
            // printf("[BURST_RAW]: ");
            // for (int i=0; i<buf_len; i++)
            //     printf("0x%x ", buf[i]);
            // printf("\n");
			adi_imu_ScaleBurstOut_1(&imu, (uint8_t*)buf, IMU_TRUE, IMU_FALSE, &burstOut);
            uint16_t dc = burstOut.dataCntOrTimeStamp;
            // printf("%d\n", dc);
            // if (j == 24444 && n == 0) dc = 0x9; // insert fail condition
            if (dc != 0 && dc != curDataCnt) {
                if (dc % 1000 == 0) {
                    printf("%d\n", dc);
                    fflush(stdout);
                }
                if (curDataCnt != 0 && dc != (curDataCnt+1)){
                    printf("%d\n%d ##", curDataCnt, dc);
                    printf("\nTEST FAILED\n");
                    cleanup(&imu);
                }
                curDataCnt = dc;
            }
        }
    }
    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, &curBufCnt)) < 0) return ret;
    printf("\n\nTEST PASSED\n");

    return 0;
}