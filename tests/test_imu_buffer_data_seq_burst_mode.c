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
    // if ((ret = imubuf_SoftwareReset(&imu)) < 0) return ret;

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
    config.imuBurstEn = 1;
    config.bufBurstEn = 1;
    if ((ret = imubuf_ConfigBuf(&imu, config)) < 0) return ret;

    if (config.imuBurstEn)
    {
        if ((ret = imubuf_SetPatternImuBurst(&imu)) < 0) return ret;
    }
    else
    {
        /* set register pattern to read/write IMU registers after every data ready interrupt */
        uint16_t bufPattern[] = {REG_SYS_E_FLAG, REG_TEMP_OUT,\
                                REG_X_GYRO_LOW, REG_X_GYRO_OUT, REG_Y_GYRO_LOW, REG_Y_GYRO_OUT, REG_Z_GYRO_LOW, REG_Z_GYRO_OUT,\
                                REG_X_ACCL_LOW, REG_X_ACCL_OUT, REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, REG_Z_ACCL_LOW, REG_Z_ACCL_OUT,\
                                REG_DATA_CNT, REG_CRC_LWR, REG_CRC_UPR};
        uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
        if ((ret = imubuf_SetPatternAuto(&imu, bufPatternLen, bufPattern)) < 0) return ret;
    }
    #define MAX_BUF_LENGTH ((38+12) * 100) // should greater than (imu_output_rate / fetch_rate). Ex: (4000Hz / 200Hz) = 20
    uint16_t burstRaw[MAX_BUF_LENGTH/2] = {0};
    adi_imu_BurstOutput_t burstOut = {0};

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* set IMU to page 0 before starting capture */
    if ((ret = adi_imu_Write(&imu, 0x0000, 0x0000)) < 0) return ret;

    /* start capture */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StartCapture(&imu, IMUBUF_TRUE, &curBufCnt)) < 0) return ret;
    imu.spiDelay = 20; // kernel latency is large enough for stall time

    uint16_t buf_len = 0;
    uint16_t curDataCnt = 0;
    uint64_t totalDataCnt = 0;

    // initial burst read should be discard as it doesn't contain any good data (lets discard initial 5 to be safe)
    if ((ret = imubuf_ReadBurstN(&imu, 5, (uint16_t *)burstRaw, &buf_len)) <0) return ret;

    for(int j=0; j<20000; j++)
    {
        if ((ret = imubuf_ReadBurstN(&imu, 5, (uint16_t *)burstRaw, &buf_len)) <0) return ret;
        for (int n=0; n<5; n++)
        {
			adi_imu_ScaleBurstOut_1(&imu, (uint8_t*)(burstRaw + (buf_len * n) + 9), FALSE, &burstOut);
            // if (burstOut.crc != 0)
                // printf("datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc =%x\n", burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc);
            
            int32_t BURST_OK = (*(burstRaw + (buf_len * n) + 8) == 0xA5A5) ? 1 : 0;
            uint16_t dc = burstOut.dataCntOrTimeStamp;
            if (BURST_OK && dc != 0 && dc != curDataCnt) {
                if (dc % 1000 == 0) printf("%d \n", dc);
                if ((curDataCnt==65535 && dc != 1) || (curDataCnt != 0 && curDataCnt != 65535 && dc != (curDataCnt+1))){
                    printf("%d\n%d ##", curDataCnt, dc);
                    printf("\nTEST FAILED\n");
                    // printbuf(":: ", burstRaw + buf_len * n, buf_len);
                    cleanup(&imu);
                }
                curDataCnt = dc;
                totalDataCnt += 1;
            }
        }
    }
    
    imu.spiDelay = 100;
    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, &curBufCnt)) < 0) return ret;
    printf("\n\nTEST PASSED\n");

    return 0;
}