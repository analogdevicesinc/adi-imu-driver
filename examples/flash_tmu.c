/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		flash_tmu.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		flash application for IMU used for TMU demo.
 **/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <time.h> // time()
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include "imu_spi_buffer.h"

int main(int argc, char** argv)
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.g = 9.8;
    imu.spiDev = "/dev/spidev1.0";
    imu.spiSpeed = 4000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 100; // stall time (us); to be safe

    /* initialize spi device */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* detect is buffer board is present */
    ret = imubuf_Detect(&imu);
    if (ret < 0) return ret;

    imubuf_DevInfo_t imuBufInfo;
    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    printf("Initializing buffer board..\n");
    /* Initialize IMU BUF first to stop any activity*/
    ret = imubuf_init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    printf("Initializing IMU..\n");
    /* Initialize IMU */
    ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    printf("Performing factory reset for buffer board..\n");
    if ((ret = imubuf_ClearFault(&imu)) < 0) return ret;
    if ((ret = imubuf_FactoryReset(&imu)) < 0) return ret;
    if ((ret = imubuf_SoftwareReset(&imu)) < 0) return ret;
    
    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    printf("Setting IO pin configuration: date ready: DIO1 (Rising edge), PPS: DIO2 (Rising edge) to 1000Hz..\n");
    /* set DIO pin config (both input and output) for iSensor SPI buffer */
    imubuf_ImuDioConfig_t dioConfig;
    dioConfig.dataReadyPin = IMUBUF_DIO1;
    dioConfig.dataReadyPolarity = RISING_EDGE;
    dioConfig.ppsPin = IMUBUF_DIO2;
    dioConfig.ppsPolarity = RISING_EDGE;
    dioConfig.ppsFreq = IMUBUF_PPS_FREQ_1HZ;
    dioConfig.passThruPin = 0x00;
    dioConfig.watermarkIrqPin = 0x00;
    dioConfig.overflowIrqPin = 0x00;
    dioConfig.errorIrqPin = 0x00;
    if ((ret = imubuf_ConfigDio(&imu, dioConfig)) < 0) return ret;

    printf("Performing flash update..");
    if ((ret = imubuf_ClearFault(&imu)) < 0) return ret;
    if ((ret = imubuf_FlashUpdate(&imu)) < 0) return ret;
    
    printf("Performing reset..");
    if ((ret = imubuf_FactoryReset(&imu)) < 0) return ret;
    if ((ret = imubuf_SoftwareReset(&imu)) < 0) return ret;

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    printf("Verifying flash update..");

    assert(IMU_BUF_FIRM_REV_MAJOR(imuBufInfo.fwRev) == 0x1);
    assert(IMU_BUF_FIRM_REV_MINOR(imuBufInfo.fwRev) == 0x13);
    assert(IMU_BUF_FIRM_REV_DEBUG(imuBufInfo.fwRev) == 0x0);

    uint16_t dio_in_config = TO_REG(dioConfig.dataReadyPin, BITP_ISENSOR_DIO_IN_CFG_DR_SEL, BITM_ISENSOR_DIO_IN_CFG_DR_SEL) \
                        | TO_REG(dioConfig.dataReadyPolarity, BITP_ISENSOR_DIO_IN_CFG_DR_POL, BITM_ISENSOR_DIO_IN_CFG_DR_POL) \
                        | TO_REG(dioConfig.ppsPolarity, BITP_ISENSOR_DIO_IN_CFG_PPS_POL, BITM_ISENSOR_DIO_IN_CFG_PPS_POL) \
                        | TO_REG(dioConfig.ppsPin, BITP_ISENSOR_DIO_IN_CFG_PPS_SEL, BITM_ISENSOR_DIO_IN_CFG_PPS_SEL) \
                        | TO_REG(dioConfig.ppsFreq, BITP_ISENSOR_DIO_IN_CFG_PPS_FREQ, BITM_ISENSOR_DIO_IN_CFG_PPS_FREQ);

    /* Set Output pins (between Spi buffer <-> Host)  */
    uint16_t dio_out_config = TO_REG(dioConfig.passThruPin, BITP_ISENSOR_DIO_OUT_CFG_PIN_PASS, BITM_ISENSOR_DIO_OUT_CFG_PIN_PASS) \
                            | TO_REG(dioConfig.watermarkIrqPin, BITP_ISENSOR_DIO_OUT_CFG_WTRMRK, BITM_ISENSOR_DIO_OUT_CFG_WTRMRK) \
                            | TO_REG(dioConfig.overflowIrqPin, BITP_ISENSOR_DIO_OUT_CFG_OVRFLW, BITM_ISENSOR_DIO_OUT_CFG_OVRFLW) \
                            | TO_REG(dioConfig.errorIrqPin, BITP_ISENSOR_DIO_OUT_CFG_ERROR, BITM_ISENSOR_DIO_OUT_CFG_ERROR);

    assert(imuBufInfo.dioInputConfig == dio_in_config);
    assert(imuBufInfo.dioOutputConfig == dio_out_config);
    printf("PASSED\n\n");

    /* Read and print IMU device info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;
    return 0;
}
