/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		imu_test.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Example usage of IMU driver.
 **/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <time.h>
#include "adi_imu_driver.h"
#include "imu_spi_buffer.h"

static unsigned g_en_pps = 0;
static unsigned g_en_burst_mode_imu = 1;
static unsigned g_en_burst_mode_buf = 1;

void usage()
{
    printf("\nUsage:\n\n");
    printf("imu_test [options]\n\n");
    printf("Options:\n");
    printf("-b          Enable buffer board\n");
    printf("-t          Enable PPS input\n");
    printf("-u          Uses UART instead of SPI device\n");
    printf("-s <str>    SPI/UART Device name [Default: /dev/spidev1.0(spi), /dev/ttyACM0 (uart)]\n");
    printf("-p <int>    Product ID of IMU (Ex: 16470) [Default: 16495]\n");
    printf("-f <int>    SPI freq in Hz/UART Baud rate [Default: 9000000(spi),921600(uart)]\n");
    printf("-d <int>    SPI delay (before each transaction) in microseconds\n");
    printf("-r <int>    Run count [Default: 100] (Due to polling, actual count might differ)\n");
    printf("-v <int>    Enable verbose. \n\t\t0: no effect, \n\t\t1: Prints valid IMU burst data. \n\t\t2: prints all IMU burst data\n");
    printf("-o <int>    IMU output data format. \n\t\t0: 16-bit, \n\t\t1: 32-bit\n");
    exit(0);
}

int init(adi_imu_Device_t* imu)
{
    /* initialize spi device */
    int ret = hw_Init(imu);
    if (ret < 0) return ret;

    /* detect if buffer board is present */
    ret = imubuf_Detect(imu);
    if(imu->enable_buffer == IMU_TRUE && ret < 0)
    {
        printf("Buffer board not present, but buffer mode enabled (en_buffer==True). Error: %d\n", ret);
        return ret;
    }
    else if(imu->enable_buffer == IMU_FALSE && ret >= 0)
    {
        printf("Buffer board present, but buffer mode disabled (en_buffer==False).\n");
        return -1;
    }

    if (imu->enable_buffer)
    {
        /* Initialize IMU BUF first to stop any activity*/
        ret = imubuf_init(imu);
        if (ret != Err_imu_Success_e) return ret;
    }

    /* Initialize IMU */
    ret = adi_imu_Init(imu);
    if (ret != Err_imu_Success_e) return ret;

    /* Set DATA ready pin */
    if(imu->prodId==16470 || imu->prodId==16500)
    {
        if ((ret = adi_imu_ConfigDataReady(imu, IMU_NULL, IMU_POS_POLARITY)) < 0) return ret;
    }
    else // Default ADIS16495
    {
        if ((ret = adi_imu_ConfigDataReady(imu, IMU_DIO1, IMU_POS_POLARITY)) < 0) return ret;
        if ((ret = adi_imu_SetDataReady(imu, IMU_ENABLE)) < 0) return ret;
    }

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(imu, 1000)) < 0) return ret;

    if (imu->enable_buffer)
    {
        /* set DIO pin config (both input and output) for iSensor SPI buffer */
        imubuf_ImuDioConfig_t dioConfig;
        dioConfig.dataReadyPin = IMUBUF_DIO1;
        dioConfig.dataReadyPolarity = IMU_RISING_EDGE;
        dioConfig.ppsPin = (g_en_pps) ? IMUBUF_DIO2 : 0x00;
        dioConfig.ppsPolarity = IMU_FALLING_EDGE;
        dioConfig.passThruPin = 0x00;
        dioConfig.watermarkIrqPin = 0x00;
        dioConfig.overflowIrqPin = 0x00;
        dioConfig.errorIrqPin = 0x00;
        if ((ret = imubuf_SetDioConfig(imu, &dioConfig)) < 0) return ret;

        if (g_en_pps)
        {
            if ((ret = imubuf_EnablePPSSync(imu)) < 0) return ret;
        
            uint32_t epoch_time = (uint32_t) time(NULL);
            if ((ret = imubuf_SetUTC(imu, epoch_time)) < 0) return ret;
            uint32_t epoch_readback = 0;
            if ((ret = imubuf_GetUTC(imu, &epoch_readback)) < 0) return ret;
            printf("UTC time (epoch) set to %d\n", epoch_readback);
            if (epoch_readback < epoch_time)
            {
                printf("UTC time not properly set.\n");
                return -1;
            }
        }
	    else
        {
            if ((ret = imubuf_DisablePPSSync(imu)) < 0) return ret;
        }

        /* enable burst mode */
        imubuf_BufConfig_t bufconfig;
        bufconfig.overflowAction = 0;
        bufconfig.imuBurstEn = g_en_burst_mode_imu;
        bufconfig.bufBurstEn = g_en_burst_mode_buf;
        if(imu->prodId == 16470 || imu->prodId == 16500)
        {
          bufconfig.imuPageAddr = 0;
        }
        else // Default ADIS16495
        {
          bufconfig.imuPageAddr = 1;
        }

        if ((ret = imubuf_SetBufConfig(imu, &bufconfig)) < 0) return ret;

        if(imu->prodId==16500)
        {
            /* Set IMU Burst Data Format */
            if ((ret = adi_imu_ConfigBurstDataFormat(imu, imu->dataFormat)) < 0) return ret;
        }

        if (bufconfig.imuBurstEn)
        {
            printf("IMU Burst Enabled \n");
            if ((ret = imubuf_SetPatternImuBurst(imu)) < 0) return ret;
            printf("IMU Data Format: %d -bit\n",(imu->dataFormat==IMU_DATA_32BIT) ? 32 : 16);
        }
        else
        {
            printf("IMU Burst Disabled \n");
            if(imu->prodId==16470)
            {
                /**
                   Note: Using IMU non-Burst was tested and usually results in Errors.
                   TODO: Assess if IMU non-Burst will be supported and if yes, fix issues
                **/
                if(imu->dataFormat==IMU_DATA_32BIT)
                {
                    // set register pattern to read/write IMU registers after every data ready interrupt
                    uint16_t bufPattern[] = {
                        IMUBUF_PATTERN_READ_REG(REG_DIAG_STAT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_TEMP_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_LOW_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_LOW_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_LOW_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_LOW_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_LOW_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_LOW_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_DATA_CNTR_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_DATA_CNTR_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_DATA_CNTR_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_DATA_CNTR_47x), \
                    }; //Temporary fix: last reg=crc will always be non-zero
                    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
                    if ((ret = imubuf_SetPatternRaw(imu, bufPatternLen, bufPattern)) < 0) return ret;
                }
                else if(imu->dataFormat==IMU_DATA_16BIT)
                {
                    // set register pattern to read/write IMU registers after every data ready interrupt
                    uint16_t bufPattern[] = {
                        IMUBUF_PATTERN_READ_REG(REG_DIAG_STAT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_TEMP_OUT_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_DATA_CNTR_47x), \
                        IMUBUF_PATTERN_READ_REG(REG_TIME_STAMP_47x), \
                    }; //Temporary fix: last reg=crc will always be non-zero
                    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
                    if ((ret = imubuf_SetPatternRaw(imu, bufPatternLen, bufPattern)) < 0) return ret;
                }
            }
            else if(imu->prodId==16500)
            {
                /**
                   Note: Using IMU non-Burst was tested and usually results in Errors.
                   TODO: Assess if IMU non-Burst will be supported and if yes, fix issues
                **/
            }
            else // Default ADIS16495
            {
                if(imu->dataFormat==IMU_DATA_32BIT)
                {
                    // set register pattern to read/write IMU registers after every data ready interrupt
                    uint16_t bufPattern[] = {
                        IMUBUF_PATTERN_READ_REG(REG_SYS_E_FLAG_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_TEMP_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_LOW_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_LOW_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_LOW_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_LOW_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_LOW_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_LOW_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_DATA_CNT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_CRC_LWR_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_CRC_UPR_49x), 0x0000, \
                    };
                    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
                    if ((ret = imubuf_SetPatternRaw(imu, bufPatternLen, bufPattern)) < 0) return ret;
                }
                else if(imu->dataFormat==IMU_DATA_16BIT)
                {
                    // set register pattern to read/write IMU registers after every data ready interrupt
                    uint16_t bufPattern[] = {
                        IMUBUF_PATTERN_READ_REG(REG_SYS_E_FLAG_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_TEMP_OUT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_DATA_CNT_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_CRC_LWR_49x), \
                        IMUBUF_PATTERN_READ_REG(REG_CRC_UPR_49x), 0x0000, \
                    };
                    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
                    if ((ret = imubuf_SetPatternRaw(imu, bufPatternLen, bufPattern)) < 0) return ret;
                }
            }
        }
    }

    /* enable these to store the current settings on flash to retain after POR */
    // if ((ret = imubuf_ClearFault(&imu)) < 0) return ret;
    // if ((ret = imubuf_FlashUpdate(&imu)) < 0) return ret;
    // if ((ret = imubuf_FactoryReset(&imu)) < 0) return ret;
    // if ((ret = imubuf_SoftwareReset(&imu)) < 0) return ret;

    /* Read and print IMU device info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(imu, &imuInfo)) < 0) return ret;

    /* Test Configure and Trigger Bias Correction Update */
    //adi_imu_ConfigBiasCorrectionTime(imu,11);
    //adi_imu_SelectBiasConfigAxes(imu,IMU_ENABLE,IMU_ENABLE,IMU_ENABLE,IMU_ENABLE,IMU_ENABLE,IMU_ENABLE);
    //adi_imu_UpdateBiasCorrection(imu);
    //if ((ret = adi_imu_GetDevInfo(imu, &imuInfo)) < 0) return ret;
    //if ((ret = adi_imu_PrintDevInfo(imu, &imuInfo)) < 0) return ret;

    /* Start Test Setting Accl and Gyro Scale and/or Bias */
    // adi_imu_AcclScale_t acclScale;
    // adi_imu_GyroScale_t gyroScale;
    //
    // acclScale.x = 4096;
    // acclScale.y = 4;
    // acclScale.z = 100;
    // if((ret=adi_imu_SetAcclScale(imu, acclScale)) < 0) return ret;
    //
    // gyroScale.x = 4;
    // gyroScale.y = 100;
    // gyroScale.z = 4096;
    // if((ret=adi_imu_SetGyroScale(imu, acclScale)) < 0) return ret;
    //
    // if((ret=adi_imu_GetAcclScale(imu, &acclScale)) < 0) return ret;
    // if((ret=adi_imu_GetGyroScale(imu, &gyroScale)) < 0) return ret;
    // printf("acclScale x: %d, y: %d, z: %d\n",acclScale.x, acclScale.y, acclScale.z);
    // printf("gyroScale x: %d, y: %d, z: %d\n",gyroScale.x, gyroScale.y, gyroScale.z);
    //
    // adi_imu_AcclBiasRaw32_t acclBias;
    // adi_imu_GyroBiasRaw32_t gyroBias;
    //
    // acclBias.x = 0x1000000;
    // acclBias.y = 0x100000;
    // acclBias.z = 0x100;
    // if((ret=adi_imu_SetAcclBias(imu, acclBias)) < 0) return ret;
    //
    // gyroBias.x = 0x100;
    // gyroBias.y = 0x1000000;
    // gyroBias.z = 0x100000;
    // if((ret=adi_imu_SetGyroBias(imu, gyroBias)) < 0) return ret;
    //
    // if((ret=adi_imu_GetAcclBias(imu, &acclBias)) < 0) return ret;
    // if((ret=adi_imu_GetGyroBias(imu, &gyroBias)) < 0) return ret;
    // printf("acclBias x: %d, y: %d, z: %d\n",acclBias.x, acclBias.y, acclBias.z);
    // printf("gyroBias x: %d, y: %d, z: %d\n",gyroBias.x, gyroBias.y, gyroBias.z);
    /* End Test Setting Accl and Gyro Scale and/or Bias */

    if (imu->enable_buffer)
    {
        imubuf_DevInfo_t imuBufInfo;
        if ((ret = imubuf_GetInfo(imu, &imuBufInfo)) < 0) return ret;
        if ((ret = imubuf_PrintInfo(imu, &imuBufInfo)) < 0) return ret;
    }
    
    /* Check Lib Build Info */
    adi_imu_BuildInfo_t binfo = adi_imu_GetBuildInfo(imu);
    printf("IMU_LIB_VERSION= %s\n", binfo.version_full);
    printf("IMU_LIB_BUILD_TIME= %s\n", binfo.build_time);
    printf("IMU_LIB_BUILD_TYPE= %s\n", binfo.build_type);

    return 0;
}

int main(int argc, char** argv)
{
    adi_imu_Device_t imu;
    imu.prodId = 16495;
    imu.g = 9.81;
    imu.devType = IMU_HW_SPI;
    imu.uartDev.dev = "/dev/ttyACM0";
    imu.uartDev.baud = 921600;
    imu.spiDev.dev = "/dev/spidev1.0";
    imu.spiDev.speed = 9000000;
    imu.spiDev.mode = 3;
    imu.spiDev.bitsPerWord = 16;
    imu.spiDev.delay = 0;
    imu.enable_buffer = IMU_FALSE;
    imu.dataFormat = IMU_DATA_32BIT;

    int run_count = 100;

    int verbose_level = 0;
    int dataFormat = 0;
    int c;
    opterr = 0;
    while ((c = getopt (argc, argv, "hbtus:p:f:d:r:v:o:")) != -1)
    {
        switch (c)
        {
            case 'h':
                usage();
                break;
            case 'b':
                imu.enable_buffer = IMU_TRUE;
                printf("Buffer board enabled\n");
                break;
            case 't':
                g_en_pps = 1;
                printf("PPS enable\n");
                break;
            case 's':
                if(imu.devType==IMU_HW_SPI)
                {
                    imu.spiDev.dev = optarg;
                    printf("IMU SPI device set to %s\n", imu.spiDev.dev);
                }
                else if (imu.devType==IMU_HW_UART)
                {
                    imu.uartDev.dev = optarg;
                    printf("IMU UART device set to %s\n", imu.uartDev.dev);
                }
                break;
            case 'p':
                imu.prodId = atoi(optarg);
                printf("IMU Product ID set to %d\n", imu.prodId);
                break;
            case 'f':
                if(imu.devType==IMU_HW_SPI)
                {
                    imu.spiDev.speed = atoi(optarg);
                    printf("SPI Clock frequency set to %d Hz\n", imu.spiDev.speed);
                }
                else if (imu.devType==IMU_HW_UART)
                {
                    imu.uartDev.baud = atoi(optarg);
                    printf("IMU UART Baud Rate set to %d\n", imu.uartDev.baud);
                }
                break;
            case 'd':
                imu.spiDev.delay = atoi(optarg);
                printf("SPI delay set to %d microseconds\n", imu.spiDev.delay);
                break;
            case 'r':
                run_count = atoi(optarg);
                printf("Run count set to %d\n", run_count);
                break;
            case 'v':
                verbose_level = atoi(optarg);
                break;
            case 'u':
                imu.devType = IMU_HW_UART;
                break;
            case 'o':
                dataFormat = atoi(optarg);
                if(dataFormat == IMU_DATA_16BIT)
                    imu.dataFormat = IMU_DATA_16BIT;
                else if(dataFormat == IMU_DATA_32BIT)
                    imu.dataFormat = IMU_DATA_32BIT;
                printf("IMU Data Format: %d -bit\n",(imu.dataFormat==IMU_DATA_32BIT)? 32:16);
                break;
            case '?':
                if (optopt == 's' || optopt == 'p' || optopt == 'f' || optopt == 'd' || optopt == 'r' || optopt == 'v' || optopt == 'o')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr,
                            "Unknown option character `\\x%x'.\n",
                            optopt);
                return 1;
            default:
                usage();
        }
    }

    /* Initialize spi, imu/imu_buffer */
    int ret = init(&imu);
    if (ret < 0) return ret;

    uint16_t buf_len = 0;

    /* data counters to track IMU data count value and data drop count */
    uint32_t prevDataCnt = 0;
    uint64_t imuDataCount = 0;
    uint64_t startDataCount = 0;
    uint64_t driverDataCount = 0;
    uint32_t rolloverCnt = 0; 
    uint32_t dropCount = 0;
    uint16_t curBufCnt = 0;
    uint16_t burstRaw[MAX_BUF_LEN_BYTES * 10];
    adi_imu_BurstOutput_t burstOut = {0};
    imubuf_BurstOutput_t bufBurstOut = {0};
    uint32_t utc_time, utc_time_us;
    imubuf_SysStatus_t bufStatus = {0};
    unsigned buf_utc_valid = 0;

    if (imu.enable_buffer)
    {
        /* set page to 255 to start capture */
        if ((ret = imubuf_StartCapture(&imu, IMU_FALSE, &curBufCnt)) < 0) return ret;

        /* initial burst read should be discarded as it receives previous outputs */
        if ((ret = imubuf_ReadBurstN(&imu, 5, (uint16_t *)burstRaw, &buf_len)) <0) return ret;
    }

    unsigned warn_suppress = 0;
    printf("IMU capture started\n");
    uint32_t burstcnt = 10;
    uint16_t remainingCnt = 0;
    bool break_capture = false;
    for (int i=0; i<run_count; i++)
    {
        if (imu.enable_buffer)
        {
            /* update burst count to remaining elements in the buffer, maxed at 10 */
            burstcnt = (remainingCnt > 10) ? 10 : (remainingCnt < 3) ? 2 : remainingCnt;
            /* lets read 10 bursts at a time to avoid calling 10 times which might be costly */
            ret = imubuf_ReadBurstN(&imu, burstcnt, (uint16_t *)burstRaw, &buf_len);
        }
        else
        {
            /* for IMU only mode, since we have only one burstOut struct, lets set burstcnt = 1 */
            burstcnt = 1;
            ret = adi_imu_ReadBurst(&imu, (uint8_t *)burstRaw, burstcnt, &burstOut);
        }

        if (ret >= 0)
        {
            for (int n=0; n<burstcnt; n++)
            {
                if (imu.enable_buffer)
                {
                    if ((ret = imubuf_ScaleBurstOut(&imu, (imubuf_BurstOutputRaw_t*) (burstRaw + buf_len * n), &bufBurstOut)) < 0) 
                    {
                        break_capture = true; 
                        break;
                    }
                    memset((uint8_t*)&burstOut, 0, sizeof(burstOut));
                    adi_imu_Boolean_e en_byte_swap = (imu.devType == IMU_HW_UART && imu.uartDev.status >= IMUBUF_UART_READY) ? IMU_FALSE : IMU_TRUE;
                    if(g_en_burst_mode_imu)
                    {
                        if(imu.prodId==16470 || imu.prodId==16500)
                            ret = adi_imu_ScaleBurstOut(&imu, bufBurstOut.data, IMU_FALSE, en_byte_swap, &burstOut);
                        else // Default ADIS16495
                            ret = adi_imu_ScaleBurstOut(&imu, bufBurstOut.data, IMU_TRUE, en_byte_swap, &burstOut);

                    }
                    else
                    {
                        ret = adi_imu_ScaleBurstOut(&imu, bufBurstOut.data, IMU_FALSE, en_byte_swap, &burstOut);
                    }
                    if (ret == Err_imu_BurstFrameInvalid_e) continue;
                    
                    /* get remaining data count in buffer */
                    remainingCnt = bufBurstOut.bufCount;

                    /* parse UTC timestamps */
                    utc_time = bufBurstOut.bufUtcTime; 
                    utc_time_us = bufBurstOut.bufTimestamp;

                    if(verbose_level > 2)
                    {
                        printf("[BURST_RAW]: ");
                        for (int i=0; i<buf_len; i++)
                            printf("0x%x ", burstRaw[i]);
                        printf("\n");
                    }

                    if(verbose_level > 1) printf("[BURST_RAW]: [UTC: %d.%d] datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc =%x\n", utc_time, utc_time_us, burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc);
                
                    /* verify timesync between system and buffer board. This will stop data capture and requires re-enabling data capture */
                    // if(g_en_pps)
                    // {
                    //     if ((ret = imubuf_CheckSysStatus(&imu, &bufStatus)) < 0) return ret;
                    //     buf_utc_valid = (bufStatus.ppsUnlock) ? 0 : 1;
                    //     if ((ret = imubuf_StartCapture(&imu, IMU_FALSE, &curBufCnt)) < 0) return ret;
                    // }
                }
                else 
                {
                    utc_time = 0;
                    utc_time_us = 0;
                }

                if(burstOut.crc == 0)
                {
                    if(imu.prodId==16470 || imu.prodId==16500)
                    {
                      burstOut.crc = 1; //Temp fix
                    }
                }

                /* process only valid burst data */
                if (burstOut.crc != 0)
                {
                    /* update data counters for the first time */
                    if (driverDataCount == 0) {
                        prevDataCnt = (burstOut.dataCntOrTimeStamp > 0) ? burstOut.dataCntOrTimeStamp - 1 : 0;
                        driverDataCount = burstOut.dataCntOrTimeStamp;
                        startDataCount = driverDataCount;
                    }
                    else driverDataCount++;

                    /* update rollover count on every overflow (i.e. 65535 to 0 transition) */
                    if ( (prevDataCnt > 0) && (burstOut.dataCntOrTimeStamp < prevDataCnt)) rolloverCnt++;
                    prevDataCnt = burstOut.dataCntOrTimeStamp;
                    imuDataCount = burstOut.dataCntOrTimeStamp + rolloverCnt * 65536;
                    uint32_t driverCntPlusDropCnt = driverDataCount + dropCount;

                    if (imuDataCount > driverCntPlusDropCnt)
                    {
                        uint32_t dropCountCurrent = imuDataCount - driverCntPlusDropCnt;
                        dropCount += dropCountCurrent;
                    }
                    else if (imuDataCount < driverCntPlusDropCnt)
                        printf("[IMU driver]: Data count invalid. IMU count: %ld Driver count: %ld Drop Count: %d\n", imuDataCount, driverDataCount, dropCount);

                    if ((driverDataCount%1000) == 0) printf("imu data cnt= %ld driver data cnt = %ld\n", imuDataCount, driverDataCount);
                    if(verbose_level > 0) printf("[UTC: %d: %d.%d] datacnt=%d, status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf crc =%x\n", buf_utc_valid, utc_time, utc_time_us, burstOut.dataCntOrTimeStamp, burstOut.sysEFlag, burstOut.tempOut, burstOut.accl.x, burstOut.accl.y, burstOut.accl.z, burstOut.gyro.x, burstOut.gyro.y, burstOut.gyro.z, burstOut.crc);
                }
            }
        }
        // too much delay here will make driver fall back to the rate at which buffer board is accumulating data
    	delay_MicroSeconds(1000);
        if (break_capture) break;
    }
    if (imu.enable_buffer)
        imubuf_StopCapture(&imu, &curBufCnt);
    
    printf("=================================\n");
    printf("Total samples received = %ld\n", driverDataCount - startDataCount);
    printf("Total samples dropped = %d\n", dropCount);
    printf("=================================\n");

    /* Perform self test and display results*/
    if ((ret = adi_imu_PerformSelfTest(&imu)) < 0) return ret;

    return 0;
}
