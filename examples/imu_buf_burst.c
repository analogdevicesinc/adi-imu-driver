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
    imu.spiDelay = 100; // stall time (us); to be safe

    /* initialize spi device */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* Initialize IMU BUF first to stop any activity*/
    ret = imubuf_init(&imu);
    if (ret != adi_imu_Success_e) return ret;
    // if ((ret = imubuf_ClearFault(&imu)) < 0) return ret;

    // if ((ret = imubuf_FactoryReset(&imu)) < 0) return ret;
    // if ((ret = imubuf_FlashUpdate(&imu)) < 0) return ret;

    /* Read and print iSensor SPI Buffer info and config*/
    imubuf_DevInfo_t imuBufInfo;
    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* software reset */
    // if ((ret = imubuf_SoftwareReset(&imu)) < 0) return ret;
    
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

    #define MAX_BUF_LENGTH ((38+12) * 100) // should greater than (imu_output_rate / fetch_rate). Ex: (4000Hz / 200Hz) = 20

    /* set register pattern to read/write IMU registers after every data ready interrupt */
    uint16_t bufPattern[] = {REG_SYS_E_FLAG, REG_TEMP_OUT,\
                            REG_X_GYRO_LOW, REG_X_GYRO_OUT, REG_Y_GYRO_LOW, REG_Y_GYRO_OUT, REG_Z_GYRO_LOW, REG_Z_GYRO_OUT,\
                            REG_X_ACCL_LOW, REG_X_ACCL_OUT, REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, REG_Z_ACCL_LOW, REG_Z_ACCL_OUT, \
                            REG_DATA_CNT, REG_CRC_LWR, REG_CRC_UPR}; //, REG_ISENSOR_BUF_TIMESTAMP_LWR, REG_ISENSOR_BUF_TIMESTAMP_UPR};
    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
    if ((ret = imubuf_SetPatternAuto(&imu, bufPatternLen, bufPattern)) < 0) return ret;
    
    // set burst length
    uint16_t bufLen = 0;
    if ((ret = imubuf_GetBufLength(&imu, &bufLen)) < 0) return ret;
    // const int burstLen = bufLen + 12;
    uint16_t burstRaw[MAX_BUF_LENGTH] = {0};
    adi_imu_BurstOutput_t burstOut = {0};

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* enable burst mode */
    imubuf_BufConfig_t config;
    config.overflowAction = 0;
    config.imuBurstEn = 0;
    config.bufBurstEn = 1;
    if ((ret = imubuf_ConfigBuf(&imu, config)) < 0) return ret;

    /* start capture */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StartCapture(&imu, IMUBUF_FALSE, &curBufCnt)) < 0) return ret;
    imu.spiDelay = 10; // kernel latency is large enough for stall time

    uint16_t buf_len = 0;
    int32_t readBufCnt = 0;
    
    uint8_t g_BurstTxBuf[MAX_BUF_LEN_BYTES+2+12] = {0xFF & REG_ISENSOR_BUF_RETRIEVE, 0x00,\
                                                    /*0xFF & REG_ISENSOR_BUF_DATA_0, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_1, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_2, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_3, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_4, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_5, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_6, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_7, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_8, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_9, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_10, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_11, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_12, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_13, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_14, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_15, 0x00,\
                                                    0xFF & REG_ISENSOR_BUF_DATA_16, 0x00,*/
                                                };
    for (int i=0; i<10; i++) {
        delay_MicroSeconds(100000);
        if (spi_ReadWrite(&imu, g_BurstTxBuf, (uint8_t*)burstRaw, 38+20, 1, 1, TRUE) < 0) printf("Spi returned error");
        // printbuf("==", burstRaw, 25);
    }

    // for(int j=0; j<1; j++)
    // {
    //     if ((ret = imubuf_ReadBurstN(&imu, 10, &readBufCnt, (uint16_t *)burstRaw, &buf_len)) <0) return ret;
    //     for (int n=0; n<10; n++)
    //     {
    //         delay_MicroSeconds(100);
    //         // printf("%d\n", buf_len);
    //         printbuf("==", burstRaw + (buf_len * n), buf_len);
    //         // printf("%d, %d: %x %x \n", readBufCnt, n, *((uint8_t*)burstRaw + (buf_len*2 * n) + 44),  *((uint8_t*)burstRaw + (buf_len*2 * n) + 43));
	// 		// adi_imu_ScaleBurstOut_1(&imu, buf, FALSE, &burstOut);
    //         // printf("datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc =%x\n", burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc);
    //     }
    // }
    printf("\n\n");
    imu.spiDelay = 100; // stall time (us); to be safe

    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, &curBufCnt)) < 0) return ret;
    delay_MicroSeconds(100000);

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    printf("\n## WARNING: Some samples may be missing if too high data rate (> 1800KHz) is used.\n\n");
    return 0;
}