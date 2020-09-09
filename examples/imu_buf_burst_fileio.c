#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h> // time()
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include "imu_spi_buffer.h"
#include "asyncio.h"

adi_imu_Device_t imu;
FILE* fp;
uint64_t g_total_data_cnt = 0;
uint64_t g_rollover_cnt = 0;
uint16_t g_prev_data_cnt = 0;

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

void post_proc(AsyncIOBufElement_e element)
{
    adi_imu_BurstOutput_t burstOut = {0};
    uint8_t* buf_start = (uint8_t*)(element.buf + 18);
    adi_imu_ScaleBurstOut_1(&imu, buf_start, FALSE, &burstOut);
    if (burstOut.crc != 0){
        if (g_total_data_cnt == 0) {
            g_prev_data_cnt =burstOut.dataCntOrTimeStamp;
            g_total_data_cnt = burstOut.dataCntOrTimeStamp;
        }
        else g_total_data_cnt++;

        if ( (g_prev_data_cnt > 0) && (burstOut.dataCntOrTimeStamp == 0)) g_rollover_cnt++;
        g_prev_data_cnt =burstOut.dataCntOrTimeStamp;
        uint64_t imu_cnt = burstOut.dataCntOrTimeStamp + g_rollover_cnt * 65536;

        uint16_t buf_cnt = IMU_GET_16BITS( element.buf, 0);
        uint32_t utc_time = IMU_GET_32BITS( element.buf, 2);
        uint32_t utc_time_us = IMU_GET_32BITS( element.buf, 6);
        if ((burstOut.dataCntOrTimeStamp%1000) == 0) printf("imu data cnt= %ld driver data cnt = %ld\n", imu_cnt, g_total_data_cnt);
        if (burstOut.dataCntOrTimeStamp + g_rollover_cnt * 65536 != g_total_data_cnt) printf("Samples lost\n");
        burstOut.gyro.x = burstOut.gyro.x * M_PI/180;
        burstOut.gyro.y = burstOut.gyro.y * M_PI/180;
        burstOut.gyro.z = burstOut.gyro.z * M_PI/180;
        fwrite((uint8_t*)&imu_cnt,sizeof(imu_cnt),1,fp);
        fwrite((uint8_t*)&g_total_data_cnt,sizeof(g_total_data_cnt),1,fp);
        fwrite((uint8_t*)&burstOut.dataCntOrTimeStamp,sizeof(burstOut.dataCntOrTimeStamp),1,fp);
        fwrite((uint8_t*)&burstOut.sysEFlag,sizeof(burstOut.sysEFlag),1,fp);
        fwrite((uint8_t*)&burstOut.tempOut,sizeof(burstOut.tempOut),1,fp);
        fwrite((uint8_t*)&burstOut.accl.x,sizeof(burstOut.accl.x),1,fp);
        fwrite((uint8_t*)&burstOut.accl.y,sizeof(burstOut.accl.y),1,fp);
        fwrite((uint8_t*)&burstOut.accl.z,sizeof(burstOut.accl.z),1,fp);
        fwrite((uint8_t*)&burstOut.gyro.x,sizeof(burstOut.gyro.x),1,fp);
        fwrite((uint8_t*)&burstOut.gyro.y,sizeof(burstOut.gyro.y),1,fp);
        fwrite((uint8_t*)&burstOut.gyro.z,sizeof(burstOut.gyro.z),1,fp);
        // printf("[UTC: %d.%d] datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc =%x\n", utc_time, utc_time_us, burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc);
    }
    free(element.buf);
}

int main()
{
    imu.prodId = 16545;
    imu.g = 9.8;
    imu.spiDev = "/dev/spidev1.0";
    imu.spiSpeed = 4000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 100; // stall time (us); to be safe

    unsigned write_to_file = 1;

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
    dioConfig.ppsPin = IMUBUF_DIO2;
    dioConfig.ppsPolarity = RISING_EDGE;
    dioConfig.passThruPin = 0x00;
    dioConfig.watermarkIrqPin = 0x00;
    dioConfig.overflowIrqPin = 0x00;
    dioConfig.errorIrqPin = 0x00;
    if ((ret = imubuf_ConfigDio(&imu, dioConfig)) < 0) return ret;
    if ((ret = imubuf_EnablePPSSync(&imu)) < 0) return ret;
    
    uint32_t epoch_time = (uint32_t) time(NULL);
    printf("Current UTC time (epoch) set to %d\n", epoch_time);
    if ((ret = imubuf_SetUTC(&imu, epoch_time)) < 0) return ret;

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
                                REG_X_ACCL_LOW, REG_X_ACCL_OUT, REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, REG_Z_ACCL_LOW, REG_Z_ACCL_OUT, \
                                REG_DATA_CNT, REG_CRC_LWR, REG_CRC_UPR};
        uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
        if ((ret = imubuf_SetPatternAuto(&imu, bufPatternLen, bufPattern)) < 0) return ret;
    }

    uint16_t getPatternLen = 0;
    uint16_t getPatternRegs[100] = {0};
    if ((ret = imubuf_GetPattern(&imu, &getPatternLen, getPatternRegs)) < 0) return ret;
    printf("\n[pattern]: ");
    for (int i=0; i<getPatternLen*2; i++)
        printf("0x%x ", getPatternRegs[i]);
    printf("\n");

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* set IMU to page 0 before starting capture */
    if ((ret = adi_imu_Write(&imu, 0x0000, 0x0000)) < 0) return ret;

    if (write_to_file) fp=fopen("mydata_15M.bin","ab");

    if (asyncio_init() < 0) return -1;

    if (asyncio_start("adimu_fileio", post_proc) < 0) return -1;

    /* start capture */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StartCapture(&imu, IMUBUF_FALSE, &curBufCnt)) < 0) return ret;
    imu.spiDelay = 20; // kernel latency is large enough for stall time

    uint16_t buf_len = 0;
    uint16_t curDataCnt = 0;
    uint64_t totalDataCnt = 0;
    
    uint16_t burstRaw[MAX_BUF_LEN_BYTES * 10];
    // initial burst read should be discard as it doesn't contain any good data (lets discard initial 5 to be safe)
    if ((ret = imubuf_ReadBurstN(&imu, 5, (uint16_t *)burstRaw, &buf_len)) <0) return ret;

    for(int j=0; j<15000000; j++)
    {
        if ((ret = imubuf_ReadBurstN(&imu, 5, (uint16_t *)burstRaw, &buf_len)) <0) return ret;
        for (int n=0; n<5; n++){
            AsyncIOBufElement_e element;
            element.size = (size_t)buf_len * 2;
            element.buf = (uint8_t *) malloc(element.size);
            // element.cur_cnt = (uint64_t)j;
            memcpy(element.buf, (uint8_t*)(burstRaw + buf_len * n), element.size);
            asyncio_to_queue(element);
            // printf("Element queued %d %d\n", n, j);
        }
    }

    asyncio_stop();

    delay_MicroSeconds(100 * 1000); // 100ms delay

    if (write_to_file) fclose(fp);
    
    printf("\n\n Total data count %ld\n", totalDataCnt);
    imu.spiDelay = 100; // stall time (us); to be safe

    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, &curBufCnt)) < 0) return ret;
    return 0;
}