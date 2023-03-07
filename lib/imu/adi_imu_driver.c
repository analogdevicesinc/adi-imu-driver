/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		adi_imu_driver.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for ADIS16xxx IMU.
 **/

#include "adi_imu_driver.h"

/* Burst read transmit buf */
/* For 16495 */
static const uint8_t BURST_REQ_49x[MAX_BRF32_LEN_BYTES_49x] = {REG_BURST_CMD_49x, 0x00};
/* For 16470 */
static const uint8_t BURST_REQ_47x[MAX_BRF16_LEN_BYTES_47x] = {REG_BURST_CMD_47x, 0x00};
/* For 16500 */
static const uint8_t BURST32_REQ_50x[MAX_BRF32_LEN_BYTES_50x] = {REG_BURST_CMD_50x, 0x00};
static const uint8_t BURST16_REQ_50x[MAX_BRF16_LEN_BYTES_50x] = {REG_BURST_CMD_50x, 0x00};

int adi_imu_Init (adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    
    /* check SPI clock frequency */
    if (pDevice->devType == IMU_HW_SPI && pDevice->spiDev.speed > IMU_MAX_SPI_CLK && pDevice->enable_buffer == IMU_FALSE) 
    {
        DEBUG_PRINT("Warning: SPI clock out of range (%d Hz) (Setting to max speed = %d Hz)\n", pDevice->spiDev.speed, IMU_MAX_SPI_CLK);
        pDevice->spiDev.speed = IMU_MAX_SPI_CLK;
        if ((ret = hw_Init(pDevice)) < 0) return ret;
    }

    /* check stall time */
    if (pDevice->devType == IMU_HW_SPI && pDevice->spiDev.delay < IMU_MIN_STALL_US && pDevice->enable_buffer == IMU_FALSE) 
    {
        DEBUG_PRINT("Warning: SPI STALL time is low (%d us) (Setting to min stall time = %d us)\n", pDevice->spiDev.delay, IMU_MIN_STALL_US);
        pDevice->spiDev.delay = IMU_MIN_STALL_US;
    }

    if (pDevice->devType != IMU_HW_SPI && pDevice->enable_buffer == IMU_FALSE) 
    {
        DEBUG_PRINT("Error: Only SPI is supported.\n");
        return Err_imu_UnsupportedProtocol_e;
    }

    /* read current page ID */
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

    /* Set IMU Product based on Product ID */
    if((pDevice->prodId % ADIS1646x) < 10)
        pDevice->imuProd = ADIS1646x;
    else if((pDevice->prodId % ADIS1647x) < 10)
        pDevice->imuProd = ADIS1647x;
    else if((pDevice->prodId % ADIS1649x) < 10)
        pDevice->imuProd = ADIS1649x;
    else if((pDevice->prodId % ADIS1650x) < 10)
        pDevice->imuProd = ADIS1650x;
    else if((pDevice->prodId % ADIS1654x) < 10)
        pDevice->imuProd = ADIS1654x;

    /* read and verify product id */
    uint16_t prodId = 0x0000;
    if ((ret = hw_ReadReg(pDevice, REG_PROD_ID(pDevice->imuProd), &prodId)) < 0) return ret;
    if (prodId != pDevice->prodId) {
        DEBUG_PRINT("Error: IMU product-id verification failed: Expected: %d, Read: %d.\n", pDevice->prodId, prodId);
        return Err_imu_ProdIdVerifyFailed_e;
    }
    else DEBUG_PRINT("\nIMU product ADIS%d found.\n\n", prodId);

    /* read range model for future gyro scale calc */
    if(pDevice->prodId==16470)
    {
        pDevice->rangeModel=0xF; // Since ADIS16470 has no Reg for Range Model
    }
    else
    {
        if ((ret = hw_ReadReg(pDevice, REG_RANG_MDL(pDevice->imuProd), &(pDevice->rangeModel))) < 0) return ret;
    }
    return ret;
}

static double getAccl16bitRes(adi_imu_Device_t *pDevice)
{
    switch(pDevice->prodId){
        case 16465:
            return IMU_RES_ACCL16_465;
        case 16467:
            return IMU_RES_ACCL16_467;
        case 16470:
            return IMU_RES_ACCL16_470;
        case 16475:
            return IMU_RES_ACCL16_475;
        case 16477:
            return IMU_RES_ACCL16_477;
        case 16495:
            return IMU_RES_ACCL16_495;
        case 16497:
            return IMU_RES_ACCL16_497;
        case 16500:
            return IMU_RES_ACCL16_500;
        case 16505:
            return IMU_RES_ACCL16_505;
        case 16507:
            return IMU_RES_ACCL16_507;
        case 16545:
            return IMU_RES_ACCL16_545;
        case 16547:
            return IMU_RES_ACCL16_547;
        default:
            return 1.0;
    }
}

static double getGyro16bitRes(adi_imu_Device_t *pDevice)
{
    double defScale = 1.0;
    uint16_t model = pDevice->rangeModel;

    if(pDevice->prodId == 16465 || pDevice->prodId == 16467)
        defScale = (model == 0x3) ? IMU_RES_GYRO16_46x1 : (model == 0x7) ? IMU_RES_GYRO16_46x2 : (model == 0xF) ? IMU_RES_GYRO16_46x3 : defScale;
    else if(pDevice->prodId == 16470 || pDevice->prodId == 16475 || pDevice->prodId == 16477)
        defScale = (model == 0x3) ? IMU_RES_GYRO16_47x1 : (model == 0x7) ? IMU_RES_GYRO16_47x2 : (model == 0xF) ? IMU_RES_GYRO16_47x3 : defScale;
    else if(pDevice->prodId == 16495 || pDevice->prodId == 16497)
        defScale = (model == 0x3) ? IMU_RES_GYRO16_49x1 : (model == 0x7) ? IMU_RES_GYRO16_49x2 : (model == 0xF) ? IMU_RES_GYRO16_49x3 : defScale;
    else if(pDevice->prodId == 16500 || pDevice->prodId == 16505 || pDevice->prodId == 16507)
        defScale = (model == 0x3) ? IMU_RES_GYRO16_50x1 : (model == 0x7) ? IMU_RES_GYRO16_50x2 : (model == 0xF) ? IMU_RES_GYRO16_50x3 : defScale;
    else if(pDevice->prodId == 16545 || pDevice->prodId == 16547)
        defScale = (model == 0x3) ? IMU_RES_GYRO16_54x1 : (model == 0x7) ? IMU_RES_GYRO16_54x2 : (model == 0xF) ? IMU_RES_GYRO16_54x3 : defScale;
    else
        defScale = 1.0;
    return defScale;
}

static double getAccl32bitRes(adi_imu_Device_t *pDevice)
{
    switch(pDevice->prodId){
        case 16465:
            return IMU_RES_ACCL32_465;
        case 16467:
            return IMU_RES_ACCL32_467;
        case 16470:
            return IMU_RES_ACCL32_470;
        case 16475:
            return IMU_RES_ACCL32_475;
        case 16477:
            return IMU_RES_ACCL32_477;
        case 16495:
            return IMU_RES_ACCL32_495;
        case 16497:
            return IMU_RES_ACCL32_497;
        case 16500:
            return IMU_RES_ACCL32_500;
        case 16505:
            return IMU_RES_ACCL32_505;
        case 16507:
            return IMU_RES_ACCL32_507;
        case 16545:
            return IMU_RES_ACCL32_545;
        case 16547:
            return IMU_RES_ACCL32_547;
        default:
            return 1.0;
    }
}

static double getGyro32bitRes(adi_imu_Device_t *pDevice)
{
    double defScale = 1.0;
    uint16_t model = pDevice->rangeModel;
    
    if(pDevice->prodId == 16465 || pDevice->prodId == 16467)
        defScale = (model == 0x3) ? IMU_RES_GYRO32_46x1 : (model == 0x7) ? IMU_RES_GYRO32_46x2 : (model == 0xF) ? IMU_RES_GYRO32_46x3 : defScale;
    else if(pDevice->prodId == 16470 || pDevice->prodId == 16475 || pDevice->prodId == 16477)
        defScale = (model == 0x3) ? IMU_RES_GYRO32_47x1 : (model == 0x7) ? IMU_RES_GYRO32_47x2 : (model == 0xF) ? IMU_RES_GYRO32_47x3 : defScale;
    else if(pDevice->prodId == 16495 || pDevice->prodId == 16497)
        defScale = (model == 0x3) ? IMU_RES_GYRO32_49x1 : (model == 0x7) ? IMU_RES_GYRO32_49x2 : (model == 0xF) ? IMU_RES_GYRO32_49x3 : defScale;
    else if(pDevice->prodId == 16500 || pDevice->prodId == 16505 || pDevice->prodId == 16507)
        defScale = (model == 0x3) ? IMU_RES_GYRO32_50x1 : (model == 0x7) ? IMU_RES_GYRO32_50x2 : (model == 0xF) ? IMU_RES_GYRO32_50x3 : defScale;
    else if(pDevice->prodId == 16545 || pDevice->prodId == 16547)
        defScale = (model == 0x3) ? IMU_RES_GYRO32_54x1 : (model == 0x7) ? IMU_RES_GYRO32_54x2 : (model == 0xF) ? IMU_RES_GYRO32_54x3 : defScale;
    else
        defScale = 1.0;
    return defScale;
}

static float getTempRes(adi_imu_Device_t *pDevice)
{
    switch(pDevice->prodId){
        case 16465:
            return IMU_RES_TEMP_46x;
        case 16467:
            return IMU_RES_TEMP_46x;
        case 16470:
            return IMU_RES_TEMP_47x;
        case 16475:
            return IMU_RES_TEMP_47x;
        case 16477:
            return IMU_RES_TEMP_47x;
        case 16495:
            return IMU_RES_TEMP_49x;
        case 16497:
            return IMU_RES_TEMP_49x;
        case 16500:
            return IMU_RES_TEMP_50x;
        case 16505:
            return IMU_RES_TEMP_50x;
        case 16507:
            return IMU_RES_TEMP_50x;
        case 16545:
            return IMU_RES_TEMP_54x;
        case 16547:
            return IMU_RES_TEMP_54x;
        default:
            return 1.0;
    }
}

static float getTempOffset(adi_imu_Device_t *pDevice)
{
    switch(pDevice->prodId){
        case 16465:
            return IMU_OFFSET_TEMP_46x;
        case 16467:
            return IMU_OFFSET_TEMP_46x;
        case 16470:
            return IMU_OFFSET_TEMP_47x;
        case 16475:
            return IMU_OFFSET_TEMP_47x;
        case 16477:
            return IMU_OFFSET_TEMP_47x;
        case 16495:
            return IMU_OFFSET_TEMP_49x;
        case 16497:
            return IMU_OFFSET_TEMP_49x;
        case 16500:
            return IMU_OFFSET_TEMP_50x;
        case 16505:
            return IMU_OFFSET_TEMP_50x;
        case 16507:
            return IMU_OFFSET_TEMP_50x;
        case 16545:
            return IMU_OFFSET_TEMP_54x;
        case 16547:
            return IMU_OFFSET_TEMP_54x;
        default:
            return 0.0;
    }
}

int adi_imu_SetOutputDataRate (adi_imu_Device_t *pDevice, uint16_t outputRate)
{
    int ret = Err_imu_Success_e;

    uint16_t maxOutputRate = 4250;
    if (pDevice->prodId == 16545 || pDevice->prodId == 16547)
        maxOutputRate = 4000;
    if (pDevice->prodId == 16470 || pDevice->prodId == 16500)
        maxOutputRate = 2000;

    uint16_t decRate = (uint16_t)(maxOutputRate / outputRate) - 1;
    /* Set decimation rate */
    uint32_t spidelay = pDevice->spiDev.delay;
    uint16_t imuStallUsDecRate = 0x0000;
    if(pDevice->imuProd==ADIS1647x)
        imuStallUsDecRate=IMU_STALL_US_DEC_RATE_47x;
    else if(pDevice->imuProd==ADIS1650x)
        imuStallUsDecRate = IMU_STALL_US_DEC_RATE_50x;
    else // Default ADIS1649x
        imuStallUsDecRate = IMU_STALL_US_DEC_RATE_49x;

    pDevice->spiDev.delay = imuStallUsDecRate;
    if ((ret = hw_WriteReg(pDevice, REG_DEC_RATE(pDevice->imuProd), decRate)) < 0) return ret;
    pDevice->spiDev.delay = spidelay;
    return ret;
}

int adi_imu_GetDevInfo (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    int ret = Err_imu_Success_e;
    uint32_t spidelay = pDevice->spiDev.delay;

    if(pDevice->imuProd==ADIS1647x)
    {
        /* read measurement range model identifier */
        /* Since ADIS16470 has no Range Model, set to the ff: */
        pInfo->gyroModelId = 0xF;

        /* read Miscellaneous Control Register */
        if ((ret = hw_ReadReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), &(pInfo->mscCtrl))) < 0) return ret;

        /* read null cfg: Control, automatic bias correction configuration */
        pDevice->spiDev.delay = IMU_STALL_US_NULLCFG_47x;
        if ((ret = hw_ReadReg(pDevice, REG_NULL_CNFG(pDevice->imuProd), &(pInfo->nullConfig))) < 0) return ret;
    }
    else if(pDevice->imuProd==ADIS1650x)
    {
        /* read measurement range model identifier */
        pDevice->spiDev.delay = 100;
        if ((ret = hw_ReadReg(pDevice, REG_RANG_MDL(pDevice->imuProd), &(pInfo->gyroModelId))) < 0) return ret;

        /* read Miscellaneous Control Register */
        if ((ret = hw_ReadReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), &(pInfo->mscCtrl))) < 0) return ret;
    }
    else // Default ADIS1649x
    {
        /* read function control IO: Control, I/O pins, functional definitions */
        pDevice->spiDev.delay = IMU_STALL_US_FNCTIO_49x;
        if ((ret = hw_ReadReg(pDevice, REG_FNCTIO_CTRL_49x, &(pInfo->fnctioCtrl))) < 0) return ret;

        /* read gpio ctrl io: Control, I/O pins, general-purpose */
        pDevice->spiDev.delay = IMU_STALL_US_GPIO_CTRL_49x;
        if ((ret = hw_ReadReg(pDevice, REG_GPIO_CTRL_49x, &(pInfo->gpioCtrl))) < 0) return ret;

        /* read clk cfg: Control, clock, and miscellaneous correction */
        pDevice->spiDev.delay = IMU_STALL_US_CONFIG_49x;
        if ((ret = hw_ReadReg(pDevice, REG_CONFIG_49x, &(pInfo->clkConfig))) < 0) return ret;

        /* read sync scale: Control, input clock scaling (PPS mode) */
        pDevice->spiDev.delay = IMU_STALL_US_SYNC_SCALE_49x;
        if ((ret = hw_ReadReg(pDevice, REG_SYNC_SCALE_49x, &(pInfo->syncScale))) < 0) return ret;

        /* read Filter bank 0 selection: Filter selection  */
        pDevice->spiDev.delay = IMU_STALL_US_FILTBNK0_49x;
        if ((ret = hw_ReadReg(pDevice, REG_FILTR_BNK_0_49x, &(pInfo->ftrBank0))) < 0) return ret;

        /* read Filter bank 1 selection: Filter selection  */
        pDevice->spiDev.delay = IMU_STALL_US_FILTBNK1_49x;
        if ((ret = hw_ReadReg(pDevice, REG_FILTR_BNK_1_49x, &(pInfo->ftrBank1))) < 0) return ret;

        /* read boot loader version */
        if ((ret = hw_ReadReg(pDevice, REG_BOOT_REV_49x, &(pInfo->bootLoadVer))) < 0) return ret;

        /* read measurement range model identifier */
        pDevice->spiDev.delay = 100;
        if ((ret = hw_ReadReg(pDevice, REG_RANG_MDL(pDevice->imuProd), &(pInfo->gyroModelId))) < 0) return ret;

        /* read null cfg: Control, automatic bias correction configuration */
        pDevice->spiDev.delay = IMU_STALL_US_NULLCFG_49x;
        if ((ret = hw_ReadReg(pDevice, REG_NULL_CNFG(pDevice->imuProd), &(pInfo->nullConfig))) < 0) return ret;
    }

    /* read current decimation rate: Control, output sample rate decimation */
    pDevice->spiDev.delay = IMU_STALL_US_DEC_RATE(pDevice->imuProd);
    if ((ret = hw_ReadReg(pDevice, REG_DEC_RATE(pDevice->imuProd), &(pInfo->decimationRate))) < 0) return ret;

    /* read firmware revision */
    pDevice->spiDev.delay = 100;
    if ((ret = hw_ReadReg(pDevice, REG_FIRM_REV(pDevice->imuProd), &(pInfo->fwRev))) < 0) return ret;

    /* read firmware day/month */
    if ((ret = hw_ReadReg(pDevice, REG_FIRM_DM(pDevice->imuProd), &(pInfo->fwDayMonth))) < 0) return ret;

    /* read firmware year */
    if ((ret = hw_ReadReg(pDevice, REG_FIRM_Y(pDevice->imuProd), &(pInfo->fwYear))) < 0) return ret;

    /* read current page id */
    if ((ret = hw_ReadReg(pDevice, REG_PAGE_ID, &(pInfo->pageId))) < 0) return ret;

    /* read product id */
    if ((ret = hw_ReadReg(pDevice, REG_PROD_ID(pDevice->imuProd), &(pInfo->prodId))) < 0) return ret;

    /* read serial number */
    if ((ret = hw_ReadReg(pDevice, REG_SERIAL_NUM(pDevice->imuProd), &(pInfo->serialNumber))) < 0) return ret;
    
    pDevice->spiDev.delay = spidelay;
    return ret;
}

int adi_imu_PrintDevInfo(adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    DEBUG_PRINT("\n================================\n");
    DEBUG_PRINT("IMU Product ID: ADIS%d\n", pInfo->prodId);
    DEBUG_PRINT("IMU FW rev: %02x.%02x\n", IMU_FIRM_REV_UPPER(pInfo->fwRev), IMU_FIRM_REV_LOWER(pInfo->fwRev));
    DEBUG_PRINT("IMU FW date (MM-DD-YYYY): %02x-%02x-%x\n", IMU_FIRM_MONTH(pInfo->fwDayMonth), IMU_FIRM_DAY(pInfo->fwDayMonth), pInfo->fwYear);
    DEBUG_PRINT("IMU Serial no: 0x%x\n", pInfo->serialNumber);
    DEBUG_PRINT("IMU Gyro Model: 0x%x [%s]\n", pInfo->gyroModelId, IMU_RANG_MDL(pInfo->gyroModelId));
    DEBUG_PRINT("IMU Page Id: 0x%x \n", pInfo->pageId);
    DEBUG_PRINT("IMU Decimation rate: %d \n", pInfo->decimationRate);
    if(pDevice->imuProd==ADIS1647x)
    {
        DEBUG_PRINT("IMU Null config: 0x%x \n", pInfo->nullConfig);
        DEBUG_PRINT("IMU Msc ctrl: 0x%x \n", pInfo->mscCtrl);
    }
    else if(pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("IMU Msc ctrl: 0x%x \n", pInfo->mscCtrl);
    }
    else // Default ADIS1649x
    {
        DEBUG_PRINT("IMU Null config: 0x%x \n", pInfo->nullConfig);
        DEBUG_PRINT("IMU Sync scale: 0x%x \n", pInfo->syncScale);
        DEBUG_PRINT("IMU Bootloader ver: %d.%d\n", IMU_BOOT_REV_MAJOR(pInfo->bootLoadVer), IMU_BOOT_REV_MINOR(pInfo->bootLoadVer));
        DEBUG_PRINT("IMU Config: 0x%x \n", pInfo->clkConfig);
        DEBUG_PRINT("IMU FNCTIO control: 0x%x \n", pInfo->fnctioCtrl);
        DEBUG_PRINT("IMU GPIO control: 0x%x \n", pInfo->gpioCtrl);
        DEBUG_PRINT("IMU Filter bank 0: 0x%x \n", pInfo->ftrBank0);
        DEBUG_PRINT("IMU Filter bank 1: 0x%x \n", pInfo->ftrBank1);
    }        

    DEBUG_PRINT("=================================\n\n");
    return Err_imu_Success_e;
}

int adi_imu_FindBurstPayloadIdx(const uint8_t *pBuf, unsigned bufLength, unsigned* pPayloadOffset)
{
    unsigned offset = 0; // offset cannot be zero
    const uint16_t* pShortBuf = (const uint16_t*)pBuf;
    const int max_index = 4;

    while(offset++ < max_index)
    {
        if (pShortBuf[offset] == 0xA5A5 && pShortBuf[offset+1] != 0xA5A5)
        {
            *pPayloadOffset = (offset+1)*2; // converting back to byte index
            return Err_imu_Success_e;
        }
    }
    // DEBUG_PRINT("Error: burst frame invalid. Could not find BURST_ID pattern (0xA5A5-0x0000).\n");
    return Err_imu_BurstFrameInvalid_e;
}

int adi_imu_ReadBurstRaw(adi_imu_Device_t *pDevice, uint8_t *pBuf, uint32_t numBursts) //TODO: Test
{
    uint8_t pageId = (REG_BURST_CMD(pDevice->imuProd) >> 8) & 0xFF;

    int ret = Err_imu_Success_e;
    /* ensure we are in right page */
    if ((ret = hw_SetPage(pDevice, pageId, 0)) < 0) return ret;

    /* send burst request and read response */
    /* as per ADIS16495 datasheet pg 7, its sufficient to send single 16-bit read access to read whole burst unlike regular read */
    if(pDevice->imuProd==ADIS1647x)
    {
        for (int i=0; i<numBursts; i++)
            if ((ret = hw_ReadWriteRaw(pDevice, BURST_REQ_47x, pDevice->maxBrfLenBytes, pBuf+pDevice->maxBrfLenBytes*i, pDevice->maxBrfLenBytes)) < 0) return ret;
    }
    else if(pDevice->imuProd==ADIS1650x)
    {
        if(pDevice->dataFormat==IMU_DATA_32BIT)
        {
            for (int i=0; i<numBursts; i++)
                if ((ret = hw_ReadWriteRaw(pDevice, BURST32_REQ_50x, pDevice->maxBrfLenBytes, pBuf+pDevice->maxBrfLenBytes*i, pDevice->maxBrfLenBytes)) < 0) return ret;
        }
        else if(pDevice->dataFormat==IMU_DATA_16BIT)
        {
            for (int i=0; i<numBursts; i++)
                if ((ret = hw_ReadWriteRaw(pDevice, BURST16_REQ_50x, pDevice->maxBrfLenBytes, pBuf+pDevice->maxBrfLenBytes*i, pDevice->maxBrfLenBytes)) < 0) return ret;
        }
    }
    else // Default ADIS1649x
    {
        for (int i=0; i<numBursts; i++)
            if ((ret = hw_ReadWriteRaw(pDevice, BURST_REQ_49x, pDevice->maxBrfLenBytes, pBuf+pDevice->maxBrfLenBytes*i, pDevice->maxBrfLenBytes)) < 0) return ret;
    }
    return ret;
}

int adi_imu_ReadBurst(adi_imu_Device_t *pDevice, uint8_t *pBuf, uint32_t numBursts, adi_imu_BurstOutput_t *pData)
{
    int ret = Err_imu_Success_e;
    unsigned pPayloadOffset = 0;
    if ((ret = adi_imu_ReadBurstRaw(pDevice, pBuf, numBursts)) < 0) return ret;
    // Scale and copy data to output
    for (int i=0; i<numBursts; ++i)
        adi_imu_ScaleBurstOut(pDevice, pBuf + pDevice->maxBrfLenBytes * i, IMU_TRUE, IMU_TRUE, &pData[i]); //TODO: Test
    return ret;
}

int adi_imu_CheckDiagStatus(adi_imu_Device_t *pDevice, adi_imu_DiagStatus_t *pStatus)
{
    int ret = Err_imu_Success_e;
    uint16_t status;

    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        uint16_t status;

        /* read diagnostic status */
        if ((ret = hw_ReadReg(pDevice, REG_DIAG_STAT(pDevice->imuProd), &status)) < 0) return ret;
        pStatus->data = status;

        if(pDevice->imuProd==ADIS1650x)
        {
            if (status & BITM_DIAG_STAT_ACCL_FAILURE_50x){
                ret = Err_imu_SystemError_e;
                DEBUG_PRINT("\nError: Accelerometer failure."
                            "A 1 indicates failure of the accelerometer at the conclusion of the self test (Register GLOB_CMD,Bit 2, see Table 112)."
                            "If this error occurs, repeat the same test. If this error persists, replace the ADIS16500."
                            "Motion during this test may cause a false failure.\n");
            }

            if (status & BITM_DIAG_STAT_GYRO2_FAILURE_50x){
                ret = Err_imu_SystemError_e;
                DEBUG_PRINT("\nError: Gyroscope 2 failure."
                            "A 1 indicates failure of Gyroscope 2 at the conclusion of the self test (Register GLOB_CMD, Bit 2, see Table 112)."
                            "If this error occurs, repeat the same test. If this error persists, replace the ADIS16500."
                            "Motion during this test may cause a false failure. \n");
            }

            if (status & BITM_DIAG_STAT_GYRO1_FAILURE_50x){
                ret = Err_imu_SystemError_e;
                DEBUG_PRINT("\nError: Gyroscope 1 failure."
                            "A 1 indicates failure of Gyroscope 1 at the conclusion of the self test (Register GLOB_CMD, Bit 2, see Table 112)."
                            "If this error occurs, repeat the same test. If this error persists, replace the ADIS16500."
                            "Motion during this test may cause a false failure. \n");
            }
        }

        uint16_t bitmDiagStatClkErr = (pDevice->imuProd==ADIS1647x) ? BITM_DIAG_STAT_CLK_ERR_47x : (pDevice->imuProd==ADIS1650x)  ? BITM_DIAG_STAT_CLK_ERR_50x : 0x0000;
        if (status & bitmDiagStatClkErr){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Clock Error."
                        "Internal data sampling clock does not synchronize with external clock."
                        "Adjust frequency of clock signal on SYNC pin. \n");
        }

        uint16_t bitmDiagStatMemFail = (pDevice->imuProd==ADIS1647x) ? BITM_DIAG_STAT_MEM_FAIL_47x : (pDevice->imuProd==ADIS1650x) ? BITM_DIAG_STAT_MEM_FAIL_50x : 0x0000;
        if (status & bitmDiagStatMemFail){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Memory Failure."
                        "Flash memoery test failure. Repeat Test."
                        "Replace the ADIS%d if this error occurs. \n",pDevice->prodId);
        }

        uint16_t bitmDiagStatSensorFail = (pDevice->imuProd==ADIS1647x) ? BITM_DIAG_STAT_SENSOR_FAIL_47x : (pDevice->imuProd==ADIS1650x) ? BITM_DIAG_STAT_SENSOR_FAIL_50x : 0x0000;
        if (status & bitmDiagStatSensorFail){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Sensor Failure."
                        "At least one sensor failed during self test. Motion during test can cause false failure."
                        "Replace the ADIS%d if this error occurs. \n",pDevice->prodId);
        }

        uint16_t bitmDiagStatStandbyMode = (pDevice->imuProd==ADIS1647x) ? BITM_DIAG_STAT_STANDBY_MODE_47x : (pDevice->imuProd==ADIS1650x) ? BITM_DIAG_STAT_STANDBY_MODE_50x : 0x0000;
        if (status & bitmDiagStatStandbyMode){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Standby mode."
                        "Voltage across VDD and GND is <2.8V"
                        "When VDD is >= 2.8V for 250ms, ADIS%d reinitializes itself and produce data again. \n", pDevice->prodId);
        }

        uint16_t bitmDiagStatSpiComm = (pDevice->imuProd==ADIS1647x) ? BITM_DIAG_STAT_SPI_COMM_47x : (pDevice->imuProd==ADIS1650x) ? BITM_DIAG_STAT_SPI_COMM_50x : 0x0000;
        if (status & bitmDiagStatSpiComm){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: SPI communication error."
                        "The total number of SCLK cycles is not equal to an integer multiple of 16."
                        "Repeat the previous communication sequence to recover."
                        "Persistence in this error can indicate a weakness in the SPI service from the master processor. \n");
        }

        uint16_t bitmDiagStatFlshMemUpd = (pDevice->imuProd==ADIS1647x) ? BITM_DIAG_STAT_FLSH_MEM_UPD_47x : (pDevice->imuProd==ADIS1650x) ? BITM_DIAG_STAT_FLSH_MEM_UPD_50x : 0x0000;
        if (status & bitmDiagStatFlshMemUpd){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Flash Memory Update Failure."
                        "The most recemt flash memory update failed. (GLOB_CMD, Bit 3)"
                        "Ensure VDD >= 3 V and repeat the update attempt."
                        "Replace the ADIS%d if the error persists.  \n", pDevice->prodId);
        }

        uint16_t bitmDiagStatDataPathOverrun = (pDevice->imuProd==ADIS1647x) ? BITM_DIAG_STAT_DATA_PATH_OVERRUN_47x : (pDevice->imuProd==ADIS1650x) ? BITM_DIAG_STAT_DATA_PATH_OVERRUN_50x : 0x0000;
        if (status & bitmDiagStatDataPathOverrun){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Data path overrun."
                        "Initiate a reset to recover using RST pin or Register GLOB_CMD, Bit 7. \n");
        }
    }
    else // Default ADIS1649x
    {
        /* read diagnostic status */
        if ((ret = hw_ReadReg(pDevice, REG_DIAG_STS_49x, &status)) < 0) return ret;
        pStatus->data = status;

        if (status & BITM_DIAG_STS_X_GYRO_49x){
            ret = Err_imu_SelfTestFailed_e;
            DEBUG_PRINT("\nError: Self test FAILED for x-axis gyroscope.\n");
        }

        if (status & BITM_DIAG_STS_Y_GYRO_49x){
            ret = Err_imu_SelfTestFailed_e;
            DEBUG_PRINT("\nError: Self test FAILED for y-axis gyroscope.\n");
        }

        if (status & BITM_DIAG_STS_Z_GYRO_49x){
            ret = Err_imu_SelfTestFailed_e;
            DEBUG_PRINT("\nError: Self test FAILED for z-axis gyroscope.\n");
        }

        if (status & BITM_DIAG_STS_X_ACCL_49x){
            ret = Err_imu_SelfTestFailed_e;
            DEBUG_PRINT("\nError: Self test FAILED for x-axis accelerometer.\n");
        }

        if (status & BITM_DIAG_STS_Y_ACCL_49x){
            ret = Err_imu_SelfTestFailed_e;
            DEBUG_PRINT("\nError: Self test FAILED for y-axis accelerometer.\n");
        }

        if (status & BITM_DIAG_STS_Z_ACCL_49x){
            ret = Err_imu_SelfTestFailed_e;
            DEBUG_PRINT("\nError: Self test FAILED for z-axis accelerometer.\n");
        }
    }
    return ret;
}

int adi_imu_CheckSysStatus(adi_imu_Device_t *pDevice, adi_imu_SysStatus_t *pStatus)
{
    int ret = Err_imu_Success_e;
    uint16_t status;

    /* read diagnostic status */
    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo SYS E FLAG for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        if ((ret = hw_ReadReg(pDevice, REG_SYS_E_FLAG_49x, &status)) < 0) return ret;
        pStatus->data = status;

        if (status & BITM_SYS_E_FLAG_BOOT_MEM_49x){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Boot memory failed."
                        "The device booted up using code from the backup memory bank."
                        "Replace the ADIS16495 if this error occurs. \n");
        }

        if (status & BITM_SYS_E_FLAG_SRAM_CRC_49x){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: SRAM error condition."
                        "CRC failure between the SRAM and flash memory. Initiate a reset to recover."
                        "Replace the ADIS16495 if this error occurs. \n");
        }

        if (status & BITM_SYS_E_FLAG_SPI_COMM_49x){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: SPI communication error."
                        "The total number of SCLK cycles is not equal to an integer multiple of 16."
                        "Repeat the previous communication sequence to recover."
                        "Persistence in this error can indicate a weakness in the SPI service from the master processor. \n");
        }

        if (status & BITM_SYS_E_FLAG_SENSOR_TEST_49x){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Sensor failure."
                        "Failure in at least one of the inertial sensors."
                        "Replace the ADIS16495 if the error persists, when it is operating in static inertial conditions. \n");
            /* check diag status for more details on failed sensor */
            adi_imu_DiagStatus_t diag;
            adi_imu_CheckDiagStatus(pDevice, &diag);
        }

        if (status & BITM_SYS_E_FLAG_FLSH_MEM_UPD_49x){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Flash memory update failure."
                        "The most recent flash memory update failed (GLOB_CMD, Bit 3, see Table 142)."
                        "Repeat the test and replace the ADIS16495 if this error persists. \n");
        }

        if (status & BITM_SYS_E_FLAG_PROC_OVERRUN_49x){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Processing overrun."
                        "Initiate a reset to recover."
                        "Replace the ADIS16495 if this error persists. \n");
        }

        if (status & BITM_SYS_E_FLAG_SYNC_ERR_49x){
            ret = Err_imu_SystemError_e;
            DEBUG_PRINT("\nError: Sync error."
                        "The sample timing is not scaling correctly, when operating in PPS mode (FNCTIO_CTRL Bit 8)."
                        "Verify that the input sync frequency is correct and that SYNC_SCALE (see Table 154) has the correct value \n");
        }
    }
    return ret;
}

int hw_ReadRegAccl(adi_imu_Device_t *pDevice, adi_imu_AcclOutputRaw32_t *pData)
{
    int ret = Err_imu_Success_e;
    uint16_t low, high;
    double res = getAccl32bitRes(pDevice);

    /* read x-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_X_ACCL_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_X_ACCL_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->x = (int32_t) (((high << 16) & 0xFFFF0000) | low) * res;

    /* read y-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_ACCL_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_ACCL_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->y = (int32_t) (((high << 16) & 0xFFFF0000) | low) * res;

    /* read z-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_ACCL_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_ACCL_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->z = (int32_t) (((high << 16) & 0xFFFF0000) | low) * res;
    return ret;
}

int hw_ReadRegGyro(adi_imu_Device_t *pDevice, adi_imu_GyroOutputRaw32_t *pData)
{
    int ret = Err_imu_Success_e;
    uint16_t low, high;

    double res = getGyro32bitRes(pDevice);

    /* read x-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_X_GYRO_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_X_GYRO_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->x = (((high << 16) & 0xFFFF0000) | low) * res;

    /* read y-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_GYRO_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_GYRO_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->y = (((high << 16) & 0xFFFF0000) | low) * res;

    /* read z-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_GYRO_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_GYRO_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->z = (((high << 16) & 0xFFFF0000) | low) * res;
    return ret;
}

int hw_ReadRegDelAng(adi_imu_Device_t *pDevice, adi_imu_DelAngOutputRaw32_t *pData)
{
    int ret = Err_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_X_DELTANG_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_X_DELTANG_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_DELTANG_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_DELTANG_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_DELTANG_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_DELTANG_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int hw_ReadRegDelVel(adi_imu_Device_t *pDevice, adi_imu_DelVelOutputRaw32_t *pData)
{
    int ret = Err_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_X_DELTVEL_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_X_DELTVEL_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_DELTVEL_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Y_DELTVEL_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_DELTVEL_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_Z_DELTVEL_OUT(pDevice->imuProd), &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}


int adi_imu_GetAcclScale(adi_imu_Device_t *pDevice, adi_imu_AcclScale_t *pData)
{
    int ret = Err_imu_Success_e;

    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo ACCL SCALE for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        /* read x-axis scale output */
        if ((ret = hw_ReadReg(pDevice, REG_X_ACCL_SCALE_49x, (uint16_t*)&(pData->x))) < 0) return ret;

        /* read y-axis scale output */
        if ((ret = hw_ReadReg(pDevice, REG_Y_ACCL_SCALE_49x, (uint16_t*)&(pData->y))) < 0) return ret;

        /* read z-axis scale output */
        if ((ret = hw_ReadReg(pDevice, REG_Z_ACCL_SCALE_49x, (uint16_t*)&(pData->z))) < 0) return ret;
    }
    return ret;
}

int adi_imu_GetGyroScale(adi_imu_Device_t *pDevice, adi_imu_GyroScale_t *pData)
{
    int ret = Err_imu_Success_e;
    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo GYRO SCALE for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        /* read x-axis scale output */
        if ((ret = hw_ReadReg(pDevice, REG_X_GYRO_SCALE_49x, (uint16_t*)&(pData->x))) < 0) return ret;

        /* read y-axis scale output */
        if ((ret = hw_ReadReg(pDevice, REG_Y_GYRO_SCALE_49x, (uint16_t*)&(pData->y))) < 0) return ret;

        /* read z-axis scale output */
        if ((ret = hw_ReadReg(pDevice, REG_Z_GYRO_SCALE_49x, (uint16_t*)&(pData->z))) < 0) return ret;
    }
    return ret;
}

int adi_imu_SetAcclScale(adi_imu_Device_t *pDevice, adi_imu_AcclScale_t data)
{
    int ret = Err_imu_Success_e;
        if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo ACCL SCALE for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        /* Set x-axis accl scale */
        if ((ret = hw_WriteReg(pDevice, REG_X_ACCL_SCALE_49x, (uint16_t)(data.x))) < 0) return ret;

        /* Set y-axis accl scale */
        if ((ret = hw_WriteReg(pDevice, REG_Y_ACCL_SCALE_49x, (uint16_t)(data.y))) < 0) return ret;

        /* Set z-axis accl scale */
        if ((ret = hw_WriteReg(pDevice, REG_Z_ACCL_SCALE_49x, (uint16_t)(data.z))) < 0) return ret;
    }
    return ret;
}

int adi_imu_SetGyroScale(adi_imu_Device_t *pDevice, adi_imu_GyroScale_t data)
{
    int ret = Err_imu_Success_e;
    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo GYRO SCALE for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        /* Set x-axis gyro scale */
        if ((ret = hw_WriteReg(pDevice, REG_X_GYRO_SCALE_49x, (uint16_t)(data.x))) < 0) return ret;

        /* Set y-axis gyro scale */
        if ((ret = hw_WriteReg(pDevice, REG_Y_GYRO_SCALE_49x, (uint16_t)(data.y))) < 0) return ret;

        /* Set z-axis gyro scale */
        if ((ret = hw_WriteReg(pDevice, REG_Z_GYRO_SCALE_49x, (uint16_t)(data.z))) < 0) return ret;
    }
    return ret;
}

int adi_imu_GetAcclBias(adi_imu_Device_t *pDevice, adi_imu_AcclBiasRaw32_t *pData)
{
    int ret = Err_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_XA_BIAS_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_XA_BIAS_HIGH(pDevice->imuProd), &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_YA_BIAS_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_YA_BIAS_HIGH(pDevice->imuProd), &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_ZA_BIAS_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_ZA_BIAS_HIGH(pDevice->imuProd), &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_GetGyroBias(adi_imu_Device_t *pDevice, adi_imu_GyroBiasRaw32_t *pData)
{
    int ret = Err_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_XG_BIAS_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_XG_BIAS_HIGH(pDevice->imuProd), &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_YG_BIAS_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_YG_BIAS_HIGH(pDevice->imuProd), &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = hw_ReadReg(pDevice, REG_ZG_BIAS_LOW(pDevice->imuProd), &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_ReadReg(pDevice, REG_ZG_BIAS_HIGH(pDevice->imuProd), &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_SetAcclBias(adi_imu_Device_t *pDevice, adi_imu_AcclBiasRaw32_t data)
{
    int ret = Err_imu_Success_e;

    /* set x-axis LOW output */
    if ((ret = hw_WriteReg(pDevice, REG_XA_BIAS_LOW(pDevice->imuProd), (uint16_t)(data.x))) < 0) return ret;
    /* set x-axis HIGH output */
    if ((ret = hw_WriteReg(pDevice, REG_XA_BIAS_HIGH(pDevice->imuProd), (uint16_t)(data.x>>16))) < 0) return ret;

    /* set y-axis LOW output */
    if ((ret = hw_WriteReg(pDevice, REG_YA_BIAS_LOW(pDevice->imuProd), (uint16_t)(data.y))) < 0) return ret;
    /* set y-axis HIGH output */
    if ((ret = hw_WriteReg(pDevice, REG_YA_BIAS_HIGH(pDevice->imuProd), (uint16_t)(data.y>>16))) < 0) return ret;

    /* read z-axis LOW output */
    if ((ret = hw_WriteReg(pDevice, REG_ZA_BIAS_LOW(pDevice->imuProd), (uint16_t)(data.z))) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_WriteReg(pDevice, REG_ZA_BIAS_HIGH(pDevice->imuProd), (uint16_t)(data.z>>16))) < 0) return ret;
    return ret;
}

int adi_imu_SetGyroBias(adi_imu_Device_t *pDevice, adi_imu_GyroBiasRaw32_t data)
{
    int ret = Err_imu_Success_e;

    /* set x-axis LOW output */
    if ((ret = hw_WriteReg(pDevice, REG_XG_BIAS_LOW(pDevice->imuProd), (uint16_t)(data.x))) < 0) return ret;
    /* set x-axis HIGH output */
    if ((ret = hw_WriteReg(pDevice, REG_XG_BIAS_HIGH(pDevice->imuProd), (uint16_t)(data.x>>16))) < 0) return ret;

    /* set y-axis LOW output */
    if ((ret = hw_WriteReg(pDevice, REG_YG_BIAS_LOW(pDevice->imuProd), (uint16_t)(data.y))) < 0) return ret;
    /* set y-axis HIGH output */
    if ((ret = hw_WriteReg(pDevice, REG_YG_BIAS_HIGH(pDevice->imuProd), (uint16_t)(data.y>>16))) < 0) return ret;

    /* read z-axis LOW output */
    if ((ret = hw_WriteReg(pDevice, REG_ZG_BIAS_LOW(pDevice->imuProd), (uint16_t)(data.z))) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = hw_WriteReg(pDevice, REG_ZG_BIAS_HIGH(pDevice->imuProd), (uint16_t)(data.z>>16))) < 0) return ret;
    return ret;
}

int adi_imu_ConfigGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, adi_imu_Direction_e direction)
{
    int ret = Err_imu_Success_e;
    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo GPIO for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        DEBUG_PRINT("Configuring GPIO %d as %s...", id, (direction == IMU_INPUT) ? "INPUT" : "OUTPUT");
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_GPIO_CTRL_49x;
        if ((ret = hw_WriteReg(pDevice, REG_GPIO_CTRL_49x, ((direction) << id))) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
        DEBUG_PRINT("done.\n");
    }
    return ret;
}

int adi_imu_SetGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id)
{
    int ret = Err_imu_Success_e;
    uint16_t data = 0x00;
    uint32_t spidelay = pDevice->spiDev.delay;
    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo GPIO for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        pDevice->spiDev.delay = IMU_STALL_US_GPIO_CTRL_49x;
        if ((ret = hw_ReadReg(pDevice, REG_GPIO_CTRL_49x, &data)) < 0) return ret;
        if ((ret = hw_WriteReg(pDevice, REG_GPIO_CTRL_49x, data | (BITM_GPIO_CTRL_DIO1_DATA_49x << id))) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }
    return ret;
}

int adi_imu_ClearGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id)
{
    int ret = Err_imu_Success_e;
    uint16_t data = 0x00;
    uint32_t spidelay = pDevice->spiDev.delay;
    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo GPIO for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        pDevice->spiDev.delay = IMU_STALL_US_GPIO_CTRL_49x;
        if ((ret = hw_ReadReg(pDevice, REG_GPIO_CTRL_49x, &data)) < 0) return ret;
        if ((ret = hw_WriteReg(pDevice, REG_GPIO_CTRL_49x, data & ~(BITM_GPIO_CTRL_DIO1_DATA_49x << id))) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }
    return ret;
}

int adi_imu_GetGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, uint8_t* val)
{
    int ret = Err_imu_Success_e;
    uint16_t data = 0x00;
    uint32_t spidelay = pDevice->spiDev.delay;
    if(pDevice->imuProd==ADIS1647x || pDevice->imuProd==ADIS1650x)
    {
        DEBUG_PRINT("\nNo GPIO for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        pDevice->spiDev.delay = IMU_STALL_US_GPIO_CTRL_49x;
        if ((ret = hw_ReadReg(pDevice, REG_GPIO_CTRL_49x, &data)) < 0) return ret;
        *val = (uint8_t) (data & (BITM_GPIO_CTRL_DIO1_DATA_49x << id));
        pDevice->spiDev.delay = spidelay;
    }
    return ret;
}

int adi_imu_ConfigDataReady(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, adi_imu_Polarity_e polarity)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Configuring data ready...");

    if(pDevice->imuProd == ADIS1647x || pDevice->imuProd == ADIS1650x)
    {
        uint16_t msc_ctrl=0x00;
        if ((ret = hw_ReadReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), &msc_ctrl)) < 0) return ret;
        if(pDevice->imuProd == ADIS1647x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_DR_POL_47x);
            msc_ctrl |= ((polarity) << BITP_MSC_CTRL_DR_POL_47x);
        }
        else if(pDevice->imuProd == ADIS1650x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_DR_POL_50x);
            msc_ctrl |= ((polarity) << BITP_MSC_CTRL_DR_POL_50x);
        }
        if ((ret = hw_WriteReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), msc_ctrl)) < 0) return ret;
    }
    else // Default ADIS1649x
    {
        uint16_t fnctio = 0x00;
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_FNCTIO_49x;
        if ((ret = hw_ReadReg(pDevice, REG_FNCTIO_CTRL_49x, &fnctio)) < 0) return ret;

        fnctio &= ~(BITM_FNCTIO_CTRL_DATA_RDY_POL_49x | BITM_FNCTIO_CTRL_DATA_RDY_DIO_49x);
        fnctio |= ((id) << BITP_FNCTIO_CTRL_DATA_RDY_DIO_49x) | ((polarity) << BITP_FNCTIO_CTRL_DATA_RDY_POL_49x);

        if ((ret = hw_WriteReg(pDevice, REG_FNCTIO_CTRL_49x, fnctio)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }

    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_ConfigSyncClkMode(adi_imu_Device_t *pDevice, adi_imu_ClockMode_e mode, adi_imu_EnDis_e clkEn, \
                              adi_imu_EdgeType_e polarity, adi_imu_GPIO_e inputGpio)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Configuring sync clock mode...");

    if(pDevice->imuProd == ADIS1647x || pDevice->imuProd == ADIS1650x)
    {
        uint16_t msc_ctrl = 0x00;
        uint32_t spidelay = pDevice->spiDev.delay;
        if ((ret = hw_ReadReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), &msc_ctrl)) < 0) return ret;
        if(pDevice->imuProd == ADIS1647x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_SYNC_FUNC_47x| BITM_MSC_CTRL_SYNC_POL_47x);
            msc_ctrl |= ((mode) << BITP_MSC_CTRL_SYNC_FUNC_47x) | ((polarity) << BITP_MSC_CTRL_SYNC_POL_47x);
        }
        else if(pDevice->imuProd == ADIS1650x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_SYNC_FUNC_50x| BITM_MSC_CTRL_SYNC_POL_50x);
            msc_ctrl |= ((mode) << BITP_MSC_CTRL_SYNC_FUNC_50x) | ((polarity) << BITP_MSC_CTRL_SYNC_POL_50x);
        }
        if ((ret = hw_WriteReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), msc_ctrl)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }
    else // Default ADIS1649x
    {
        uint16_t fnctio = 0x00;
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_FNCTIO_49x;
        if ((ret = hw_ReadReg(pDevice, REG_FNCTIO_CTRL_49x, &fnctio)) < 0) return ret;

        fnctio &= ~(BITM_FNCTIO_CTRL_SYNC_CLK_MODE_49x | BITM_FNCTIO_CTRL_SYNC_CLK_EN_49x | BITM_FNCTIO_CTRL_SYNC_CLK_POL_49x | BITM_FNCTIO_CTRL_SYNC_CLK_DIO_49x);
        fnctio |= ((mode) << BITP_FNCTIO_CTRL_SYNC_CLK_MODE_49x) | ((clkEn) << BITP_FNCTIO_CTRL_SYNC_CLK_EN_49x) | \
                          ((polarity) << BITP_FNCTIO_CTRL_SYNC_CLK_POL_49x) | ((inputGpio) << BITP_FNCTIO_CTRL_SYNC_CLK_DIO_49x);

        if ((ret = hw_WriteReg(pDevice, REG_FNCTIO_CTRL_49x, fnctio)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }

    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SetDataReady(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("%s data ready...", (val == IMU_ENABLE) ? "Enabling" : "Disabling");

    if(pDevice->imuProd == ADIS1647x || pDevice->imuProd == ADIS1650x)
    {
        DEBUG_PRINT("\nData Ready always enabled for %dx",pDevice->imuProd);
    }
    else // Default ADIS1649x
    {
        uint16_t fnctio = 0x00;
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_FNCTIO_49x;
        if ((ret = hw_ReadReg(pDevice, REG_FNCTIO_CTRL_49x, &fnctio)) < 0) return ret;

        if ((ret = hw_WriteReg(pDevice, REG_FNCTIO_CTRL_49x, (val == IMU_ENABLE) ? fnctio | BITM_FNCTIO_CTRL_DATA_RDY_EN_49x : fnctio)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }

    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SetLineargComp(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("%s linear g compensation...", (val == IMU_ENABLE) ? "Enabling" : "Disabling");

    if(pDevice->imuProd == ADIS1647x || pDevice->imuProd == ADIS1650x)
    {
        uint16_t msc_ctrl = 0x00;
        if ((ret = hw_ReadReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), &msc_ctrl)) < 0) return ret;
        if(pDevice->imuProd == ADIS1647x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_LIN_G_COMP_47x);
            msc_ctrl |= ((val) << BITP_MSC_CTRL_LIN_G_COMP_47x);
        }
        else if(pDevice->imuProd == ADIS1650x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_LIN_G_COMP_50x);
            msc_ctrl |= ((val) << BITP_MSC_CTRL_LIN_G_COMP_50x);
        }
        if ((ret = hw_WriteReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), msc_ctrl) < 0)) return ret;
    }
    else // Default ADIS1649x
    {
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_CONFIG_49x;
        if ((ret = hw_WriteReg(pDevice, REG_CONFIG_49x, (val == IMU_ENABLE) ? BITM_CONFIG_LIN_G_COMP_49x : 0x00)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SetPPercAlignment(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("%s point of percussion alignment...", (val == IMU_ENABLE) ? "Enabling" : "Disabling");

    if(pDevice->imuProd == ADIS1647x || pDevice->imuProd == ADIS1650x)
    {
        uint16_t msc_ctrl = 0x00;
        if ((ret = hw_ReadReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), &msc_ctrl)) < 0) return ret;
        if(pDevice->imuProd == ADIS1647x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_PNT_PERC_ALIGN_47x);
            msc_ctrl |= ((val) << BITP_MSC_CTRL_PNT_PERC_ALIGN_47x);
        }
        else if(pDevice->imuProd == ADIS1650x)
        {
            msc_ctrl &= ~(BITM_MSC_CTRL_PNT_PERC_ALIGN_50x);
            msc_ctrl |= ((val) << BITP_MSC_CTRL_PNT_PERC_ALIGN_50x);
        }
        if ((ret = hw_WriteReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), msc_ctrl)) < 0) return ret;
    }
    else // Default ADIS1649x
    {
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_CONFIG_49x;
        if ((ret = hw_WriteReg(pDevice, REG_CONFIG_49x, (val == IMU_ENABLE) ? BITM_CONFIG_PNT_PERC_ALIGN_49x : 0x00)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
    }
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SoftwareReset(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Performing software reset...");

    uint16_t bitmGlobCmdSoftRst = 0x0000;
    if(pDevice->imuProd == ADIS1647x)
        bitmGlobCmdSoftRst = BITM_GLOB_CMD_SOFT_RST_47x;
    else if(pDevice->imuProd == ADIS1650x)
        bitmGlobCmdSoftRst = BITM_GLOB_CMD_SOFT_RST_50x;
    else // Default ADIS1649x
        bitmGlobCmdSoftRst = BITM_GLOB_CMD_SOFT_RST_49x;

    if ((ret = hw_WriteReg(pDevice, REG_GLOB_CMD(pDevice->imuProd), bitmGlobCmdSoftRst)) < 0) return ret;
    delay_MicroSeconds(350000); //350ms

    // Read page ID
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

    DEBUG_PRINT("Finished!\n");
    return ret;
}

int adi_imu_ClearUserCalibration(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Clearing all User calibration data...");

    if(pDevice->imuProd == ADIS1647x || pDevice->imuProd == ADIS1650x)
    {
        uint16_t bitmGlobCmdCalibRestore = 0x0000;
        if(pDevice->imuProd == ADIS1647x)
            bitmGlobCmdCalibRestore = BITM_GLOB_CMD_CALIB_RESTORE_47x;
        if(pDevice->imuProd == ADIS1650x)
            bitmGlobCmdCalibRestore = BITM_GLOB_CMD_CALIB_RESTORE_50x;

        if ((ret = hw_WriteReg(pDevice, REG_GLOB_CMD(pDevice->imuProd), bitmGlobCmdCalibRestore)) < 0) return ret;
    }
    else // Default ADIS1649x
    {
        if ((ret = hw_WriteReg(pDevice, REG_GLOB_CMD(pDevice->imuProd), BITM_GLOB_CMD_CLR_USR_CALIB_49x)) < 0) return ret;
    }
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_UpdateFlashMemory(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Writing IMU settings to flash...");
    uint16_t bitmGlobCmdFlashMemUpd = 0x0000;
    if(pDevice->imuProd == ADIS1647x)
        bitmGlobCmdFlashMemUpd = BITM_GLOB_CMD_FLASH_MEM_UPD_47x;
    else if(pDevice->imuProd == ADIS1650x)
        bitmGlobCmdFlashMemUpd = BITM_GLOB_CMD_FLASH_MEM_UPD_50x;
    else // Default ADIS1649x
        bitmGlobCmdFlashMemUpd = BITM_GLOB_CMD_FLASH_MEM_UPD_49x;

    if ((ret = hw_WriteReg(pDevice, REG_GLOB_CMD(pDevice->prodId), bitmGlobCmdFlashMemUpd)) < 0) return ret;
    delay_MicroSeconds(600000); //600ms

    // Read page ID
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

    DEBUG_PRINT("Finished!\n");
    return ret;
}

int adi_imu_PerformSelfTest(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Performing Self test...");
    adi_imu_DiagStatus_t diagStatus;

    if(pDevice->imuProd == ADIS1647x || pDevice->imuProd == ADIS1650x)
    {
        uint16_t bitmGlobCmdSensorSelfTest= 0x0000;
        if(pDevice->imuProd == ADIS1647x)
            bitmGlobCmdSensorSelfTest = BITM_GLOB_CMD_SENSOR_SELF_TEST_47x;
        if(pDevice->imuProd == ADIS1650x)
            bitmGlobCmdSensorSelfTest = BITM_GLOB_CMD_SENSOR_SELF_TEST_50x;

        if ((ret = hw_WriteReg(pDevice, REG_GLOB_CMD(pDevice->imuProd), bitmGlobCmdSensorSelfTest)) < 0) return ret;
        delay_MicroSeconds(100000); //100ms
        DEBUG_PRINT("Finished!\n");
    }
    else // Default ADIS1649x
    {
        if ((ret = hw_WriteReg(pDevice, REG_GLOB_CMD_49x, BITM_GLOB_CMD_SELF_TEST_49x)) < 0) return ret;
        delay_MicroSeconds(50000); //50ms
        DEBUG_PRINT("Finished!\n");
    }
    DEBUG_PRINT("Checking test results..");
    if((ret = adi_imu_CheckDiagStatus(pDevice,&diagStatus)) < 0) return ret;
    DEBUG_PRINT("%s\n", (diagStatus.data) ? "FAILED": "SUCCESS");
    return ret;
}

/* Configure the bias correction accumulation time */
int adi_imu_ConfigBiasCorrectionTime(adi_imu_Device_t *pDevice, uint8_t time)
{
    int ret = Err_imu_Success_e;
    int maxtime = 13;
    if(pDevice->imuProd == ADIS1650x)
    {
        DEBUG_PRINT("Bias Correction Time not Supported in ADIS1650x\n");
    }
    else
    {
        DEBUG_PRINT("Updating bias correction time...");
        
        if(pDevice->prodId==16470)
            maxtime=12;
        else // Default ADIS16495
            maxtime=13;

        if (time > maxtime)
        {
            DEBUG_PRINT("Bias error correction setting invalid! Forced to %d", maxtime);
            time = maxtime;
        }

        uint16_t dat = (time & 0xFF);
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_NULLCFG_49x;
        if ((ret = hw_WriteReg(pDevice, REG_NULL_CNFG(pDevice->imuProd), dat)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
        DEBUG_PRINT("Finished!\n");
    }

    return ret;
}

/* Trigger a bias correction update based on the NULL_CNFG register settings */
int adi_imu_UpdateBiasCorrection(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Triggering bias correction update...");
    if(pDevice->imuProd == ADIS1650x)
    {
        DEBUG_PRINT("Bias Correction Time not Supported in ADIS1650x\n");
    }
    else
    {
        uint16_t bitmGlobCmdBiasCorrUpd = 0x0000;
        if(pDevice->imuProd == ADIS1647x)
            bitmGlobCmdBiasCorrUpd = BITM_GLOB_CMD_BIAS_CORR_UPD_47x;
        else // Default ADIS1649x
            bitmGlobCmdBiasCorrUpd = BITM_GLOB_CMD_BIAS_CORR_UPD_49x;

        if ((ret = hw_WriteReg(pDevice, REG_GLOB_CMD(pDevice->imuProd), bitmGlobCmdBiasCorrUpd)) < 0) return ret;
        DEBUG_PRINT("Finished!\n");
    }

    return ret;
}

/* Select the sensors to be nulled */
int adi_imu_SelectBiasConfigAxes(adi_imu_Device_t *pDevice, adi_imu_EnDis_e XG, adi_imu_EnDis_e YG, \
                                 adi_imu_EnDis_e ZG, adi_imu_EnDis_e XA, adi_imu_EnDis_e YA, \
                                 adi_imu_EnDis_e ZA)
{
    int ret = Err_imu_Success_e;
    if(pDevice->imuProd == ADIS1650x)
    {
        DEBUG_PRINT("Bias Config Axes not Supported in ADIS1650x\n");
    }
    else
    {
        DEBUG_PRINT("Configuring bias config axes...");
        uint16_t nullcnfg = 0x00;
        uint32_t spidelay = pDevice->spiDev.delay;
        pDevice->spiDev.delay = IMU_STALL_US_NULLCFG_49x;
        if ((ret = hw_ReadReg(pDevice, REG_NULL_CNFG(pDevice->imuProd), &nullcnfg)) < 0) return ret;
        if(pDevice->imuProd == ADIS1647x)
        {
            nullcnfg &= ~(BITM_NULL_CNFG_EN_XG_47x | BITM_NULL_CNFG_EN_YG_47x | BITM_NULL_CNFG_EN_ZG_47x | \
                        BITM_NULL_CNFG_EN_XA_47x | BITM_NULL_CNFG_EN_YA_47x | BITM_NULL_CNFG_EN_ZA_47x);
            nullcnfg |= ((XG) << BITP_NULL_CNFG_EN_XG_47x) | ((YG) << BITP_NULL_CNFG_EN_YG_47x) | \
                        ((ZG) << BITP_NULL_CNFG_EN_ZG_47x) | ((XA) << BITP_NULL_CNFG_EN_XA_47x) | \
                        ((YA) << BITP_NULL_CNFG_EN_YA_47x) | ((ZA) << BITP_NULL_CNFG_EN_ZA_47x);
        }
        else // Default ADIS1649x
        {
            nullcnfg &= ~(BITM_NULL_CNFG_EN_XG_49x | BITM_NULL_CNFG_EN_YG_49x | BITM_NULL_CNFG_EN_ZG_49x | \
                        BITM_NULL_CNFG_EN_XA_49x | BITM_NULL_CNFG_EN_YA_49x | BITM_NULL_CNFG_EN_ZA_49x);
            nullcnfg |= ((XG) << BITP_NULL_CNFG_EN_XG_49x) | ((YG) << BITP_NULL_CNFG_EN_YG_49x) | \
                        ((ZG) << BITP_NULL_CNFG_EN_ZG_49x) | ((XA) << BITP_NULL_CNFG_EN_XA_49x) | \
                        ((YA) << BITP_NULL_CNFG_EN_YA_49x) | ((ZA) << BITP_NULL_CNFG_EN_ZA_49x);
        }
        if ((ret = hw_WriteReg(pDevice, REG_NULL_CNFG(pDevice->imuProd), nullcnfg)) < 0) return ret;
        pDevice->spiDev.delay = spidelay;
        DEBUG_PRINT("Finished!\n");
    }

    return ret;
}

int adi_imu_ParseBurst32Out(adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutputRaw32_t *pRawData)
{
    unsigned payloadOffset = 0;

    unsigned sysEFlagOffset = 2; //2 bytes
    unsigned tempOffset = 2;
    unsigned dataOffset = 4; //4 bytes for Gyro X,Y,X and Accl X,Y,Z
    unsigned dataCntOrTimeStampOffset = 2; //2 bytes

    unsigned totalOffset = 0;

    /** BURST READ FORMAT
      * For 16495, BRF for (fsclk < 3MHz):
      *   0x0000, 0xA5A5 (BURST_ID), SYS_E_FLAG, TEMP_OUT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, Y_GYRO_OUT,
      *   Z_GYRO_LOW, Z_GYRO_OUT, X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, Z_ACCL_LOW, Z_ACCL_OUT,
      *   DATA_CNT, CRC_LWR, CRC_UPR
      *
      * For 16500, BRF for:
      *   0x0000, DIAG_STAT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, Y_GYRO_OUT, Z_GYRO_LOW, Z_GYRO_OUT,
      *   X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, Z_ACCL_LOW, Z_ACCL_OUT, TEMP_OUT, DATA_CNT, CRC
      **/

    if (checkBurstID) {
      if(16470==pDevice->prodId || 16500==pDevice->prodId)
      {
         payloadOffset = 0;
      }
      else // Default ADIS16495
      {
          int ret = adi_imu_FindBurstPayloadIdx(pBuf, pDevice->maxBrfLenBytes, &payloadOffset);
          if (ret < 0) return ret;
      }
    }else{
      payloadOffset=2;
    }

    if(enByteSwap == IMU_TRUE)
    {
        pRawData->sysEFlag= IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset));
        totalOffset = totalOffset + sysEFlagOffset;

        if(16500==pDevice->prodId)
        {
            // Proceed to next register
        }
        else // Default ADIS16495
        {
            pRawData->tempOut = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
            totalOffset = totalOffset + tempOffset;
        }

        pRawData->gyro.x = (int32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.y = (int32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.z = (int32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.x = (int32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.y = (int32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.z = (int32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        if(16500==pDevice->prodId)
        {
            pRawData->tempOut = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
            totalOffset = totalOffset + tempOffset;
        }
        else // Default ADIS16495
        {
            // Proceed to next register
        }

        pRawData->dataCntOrTimeStamp = (unsigned) IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataCntOrTimeStampOffset;

        if(16500==pDevice->prodId)
        {
          pRawData->crc = (uint16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));  // 16-bit CRC
        }
        else // Default ADIS16495
        {
          pRawData->crc = (uint32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset))); // 32-bit CRC
        }
    }
    else
    {
        pRawData->sysEFlag= *(uint16_t*)(pBuf + payloadOffset);
        totalOffset = totalOffset + sysEFlagOffset;

        if(16500==pDevice->prodId)
        {
            // Proceed to next register
        }
        else // Default ADIS16495
        {
            pRawData->tempOut = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
            totalOffset = totalOffset + tempOffset;
        }

        pRawData->gyro.x = (int32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.y = (int32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.z = (int32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.x = (int32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.y = (int32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.z = (int32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        if(16500==pDevice->prodId)
        {
            pRawData->tempOut = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
            totalOffset = totalOffset + tempOffset;
        }
        else // Default ADIS16495
        {
            // Proceed to next register
        }

        pRawData->dataCntOrTimeStamp = (unsigned) *(uint16_t*)(pBuf + payloadOffset + totalOffset);
        totalOffset = totalOffset + dataCntOrTimeStampOffset;

        if(16500==pDevice->prodId)
        {
            pRawData->crc = (uint16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        }
        else // Default ADIS16495
        {
            pRawData->crc = (uint32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
        }
    }
    return Err_imu_Success_e;
}

int adi_imu_ParseBurst16Out(adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutputRaw16_t *pRawData)
{
    unsigned payloadOffset = 0;

    unsigned sysEFlagOffset = 2; //2 bytes
    unsigned tempOffset = 2;
    unsigned dataOffset = 2; //2 bytes for Gyro X,Y,X and Accl X,Y,Z
    unsigned dataCntOrTimeStampOffset = 2; //2 bytes

    unsigned totalOffset = 0;

    /** BURST READ FORMAT
      * For 16500 or 16470, BRF for:
      *   0x0000, DIAG_STAT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, Y_GYRO_OUT, Z_GYRO_LOW, Z_GYRO_OUT,
      *   X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, Z_ACCL_LOW, Z_ACCL_OUT, TEMP_OUT, DATA_CNT, CRC
      **/

    if (checkBurstID) {
      if(16470==pDevice->prodId || 16500==pDevice->prodId)
      {
         payloadOffset = 0;
      }
      else // Default ADIS16495
      {
          int ret = adi_imu_FindBurstPayloadIdx(pBuf, pDevice->maxBrfLenBytes, &payloadOffset);
          if (ret < 0) return ret;
      }
    }else{
      payloadOffset=2;
    }

    if(enByteSwap == IMU_TRUE)
    {
        pRawData->sysEFlag= IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset));
        totalOffset = totalOffset + sysEFlagOffset;

        pRawData->gyro.x = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.y = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.z = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.x = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.y = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.z = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + dataOffset;

        pRawData->tempOut = (int16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        totalOffset = totalOffset + tempOffset;

        pRawData->dataCntOrTimeStamp = (unsigned) IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataCntOrTimeStampOffset;

        if(pDevice->prodId==16470 || pDevice->prodId == 16500)
            pRawData->crc = (uint16_t) (IMU_BSWAP_16(*(uint16_t*)(pBuf + payloadOffset + totalOffset)));
        else // Default ADIS16495 //CRC is always 32-bit in 16495
            pRawData->crc = (uint32_t) (IMU_BSWAP_32(*(uint32_t*)(pBuf + payloadOffset + totalOffset)));
    }
    else
    {
        pRawData->sysEFlag= *(uint16_t*)(pBuf + payloadOffset);
        totalOffset = totalOffset + sysEFlagOffset;

        pRawData->gyro.x = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.y = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->gyro.z = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.x = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.y = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->accl.z = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + dataOffset;

        pRawData->tempOut = (int16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        totalOffset = totalOffset + tempOffset;

        pRawData->dataCntOrTimeStamp = (unsigned) *(uint16_t*)(pBuf + payloadOffset + totalOffset);
        totalOffset = totalOffset + dataCntOrTimeStampOffset;

        if(pDevice->prodId==16470 || pDevice->prodId == 16500)
            pRawData->crc = (uint16_t) (*(uint16_t*)(pBuf + payloadOffset + totalOffset));
        else // Default ADIS16495 //CRC is always 32-bit in 16495
            pRawData->crc = (uint32_t) (*(uint32_t*)(pBuf + payloadOffset + totalOffset));
    }
    return Err_imu_Success_e;
}

int adi_imu_ScaleBurstOut(adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutput_t *pData)
{
    int ret =0;
    if(pDevice->dataFormat==IMU_DATA_32BIT)
    {
        ret = adi_imu_ScaleBurst32Out(pDevice, pBuf, checkBurstID, enByteSwap, pData);
    }
    else if(pDevice->dataFormat==IMU_DATA_16BIT)
    {
        ret = adi_imu_ScaleBurst16Out(pDevice, pBuf, checkBurstID, enByteSwap, pData);
    }
    return ret;
}

int adi_imu_ScaleBurst32Out(adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutput_t *pData)
{
    adi_imu_BurstOutputRaw32_t pRawData = {0};
    memset((uint8_t*)&pRawData, 0, sizeof(pRawData));
    int ret = adi_imu_ParseBurst32Out(pDevice,pBuf,checkBurstID,enByteSwap,&pRawData);

    pData->sysEFlag= (unsigned) pRawData.sysEFlag;
    adi_imu_ScaleTemp32Out(pDevice, pRawData.tempOut, &(pData->tempOut));
    adi_imu_ScaleGyro32Out(pDevice, &(pRawData.gyro), &(pData->gyro));
    adi_imu_ScaleAccl32Out(pDevice, &(pRawData.accl), &(pData->accl));
    pData->dataCntOrTimeStamp = (unsigned) pRawData.dataCntOrTimeStamp;
    pData->crc = pRawData.crc;

    return ret;
}

int adi_imu_ScaleBurst16Out(adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutput_t *pData)
{

    adi_imu_BurstOutputRaw16_t pRawData = {0};
    memset((uint8_t*)&pRawData, 0, sizeof(pRawData));
    int ret = adi_imu_ParseBurst16Out(pDevice,pBuf,checkBurstID,enByteSwap,&pRawData);

    pData->sysEFlag= (unsigned) pRawData.sysEFlag;
    adi_imu_ScaleTemp16Out(pDevice, pRawData.tempOut, &(pData->tempOut));
    adi_imu_ScaleGyro16Out(pDevice, &(pRawData.gyro), &(pData->gyro));
    adi_imu_ScaleAccl16Out(pDevice, &(pRawData.accl), &(pData->accl));
    pData->dataCntOrTimeStamp = (unsigned) pRawData.dataCntOrTimeStamp;
    pData->crc = pRawData.crc;

    return ret;
}

void adi_imu_ScaleTemp32Out(adi_imu_Device_t *pDevice, uint16_t rawData, float *pData)
{
    *pData = getTempOffset(pDevice) + ((int32_t) rawData) * getTempRes(pDevice);
}

void adi_imu_ScaleTemp16Out(adi_imu_Device_t *pDevice, uint16_t rawData, float *pData)
{
    *pData = getTempOffset(pDevice) + ((int16_t) rawData) * getTempRes(pDevice);
}

void adi_imu_ScaleAccl32Out(adi_imu_Device_t *pDevice, const adi_imu_AcclOutputRaw32_t *rawData, adi_imu_AcclOutput_t *pOut)
{
    double scale = getAccl32bitRes(pDevice) * pDevice->g;
    pOut->x = (int32_t) (rawData->x) * scale;
    pOut->y = (int32_t) (rawData->y) * scale;
    pOut->z = (int32_t) (rawData->z) * scale;
}

void adi_imu_ScaleGyro32Out(adi_imu_Device_t *pDevice, const adi_imu_GyroOutputRaw32_t *rawData, adi_imu_GyroOutput_t *pOut)
{
    double scale = getGyro32bitRes(pDevice);
    pOut->x = (int32_t) (rawData->x) * scale;
    pOut->y = (int32_t) (rawData->y) * scale;
    pOut->z = (int32_t) (rawData->z) * scale;
}

void adi_imu_ScaleAccl16Out(adi_imu_Device_t *pDevice, const adi_imu_AcclOutputRaw16_t *rawData, adi_imu_AcclOutput_t *pOut)
{
    double scale = getAccl16bitRes(pDevice) * pDevice->g;
    pOut->x = (int16_t) (rawData->x) * scale;
    pOut->y = (int16_t) (rawData->y) * scale;
    pOut->z = (int16_t) (rawData->z) * scale;
}

void adi_imu_ScaleGyro16Out(adi_imu_Device_t *pDevice, const adi_imu_GyroOutputRaw16_t *rawData, adi_imu_GyroOutput_t *pOut)
{
    double scale = getGyro16bitRes(pDevice);
    pOut->x = (int16_t) (rawData->x) * scale;
    pOut->y = (int16_t) (rawData->y) * scale;
    pOut->z = (int16_t) (rawData->z) * scale;
}

int adi_imu_ConfigBurstDataFormat(adi_imu_Device_t *pDevice, adi_imu_DataFormat_e dataFormat)
{
    int ret = Err_imu_Success_e;

    if(pDevice->imuProd == ADIS1650x)
    {
        uint16_t msc_ctrl=0x00;
        if(dataFormat == IMU_DATA_16BIT)
        {
            /* Check decimation rate */
            uint16_t dec_rate=0x00;
            if ((ret = hw_ReadReg(pDevice, REG_DEC_RATE(pDevice->imuProd), &dec_rate)) < 0) return ret;

            uint16_t maxOutputRate = 2000; // For ADIS16500

            uint16_t outputRate = (uint16_t)(maxOutputRate / (dec_rate+1));
            if(outputRate < maxOutputRate)
            {
                DEBUG_PRINT("Forcing IMU Rate to 2000 Hz as recommended for 16-bit Burst Read Mode for ADIS16500.\n");
                /* Set decimation rate */
                uint16_t decRate = 0; // No decimation
                uint32_t spidelay = pDevice->spiDev.delay;
                pDevice->spiDev.delay = IMU_STALL_US_DEC_RATE_50x;
                if ((ret = hw_WriteReg(pDevice, REG_DEC_RATE(pDevice->imuProd), decRate)) < 0) return ret;
                pDevice->spiDev.delay = spidelay;
            }
        }

        if ((ret = hw_ReadReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), &msc_ctrl)) < 0) return ret;
        msc_ctrl &= ~(BITM_MSC_CTRL_BURST32_50x);
        msc_ctrl |= ((dataFormat) << BITP_MSC_CTRL_BURST32_50x);

        DEBUG_PRINT("Setting IMU Burst Read Data Format to %s...", (dataFormat == IMU_DATA_32BIT) ? "32-bit" : "16-bit");
        if ((ret = hw_WriteReg(pDevice, REG_MSC_CTRL(pDevice->imuProd), msc_ctrl) < 0)) return ret;
      }
      else // ADIS1649x or ADIS1647x
      {
          DEBUG_PRINT("Config Burst Data Format not supported in %d\n", pDevice->prodId);
      }
    return ret;
}

adi_imu_BuildInfo_t adi_imu_GetBuildInfo (adi_imu_Device_t *pDevice)
{
    adi_imu_BuildInfo_t info;
    info.version = IMU_LIB_VERSION;
    snprintf(info.version_full, sizeof(info.version_full), IMU_LIB_VERSION_FULL);
    snprintf(info.build_time, sizeof(info.build_time), IMU_LIB_BUILD_DATETIME);
    snprintf(info.build_type, sizeof(info.build_type), IMU_LIB_BUILD_TYPE);
    return info;
}
