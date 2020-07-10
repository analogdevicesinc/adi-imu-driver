/*******************************************************************************
 *   @file   adi_imu_driver.c
 *   @brief  Driver interface for ADIS16xxx IMU
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#include "adi_imu_driver.h"

/* external spi driver API (provided by user) */
extern int adi_imu_SpiInit (adi_imu_Device_t *pDevice);
extern int adi_imu_SpiReadWrite (adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);
extern void adi_imu_DelayMicroSeconds (uint32_t microseconds);


int adi_imu_Init (adi_imu_Device_t *pDevice)
{
    int ret = adi_imu_Success_e;
    pDevice->status = 0;

    /* initialize spi device */
    int spi_status = adi_imu_SpiInit(pDevice);
    if (spi_status < 0) return spi_status;

    /* set device handler after successful initialization */
    pDevice->status = 1;

    /* set current page to 0 at the start for reference, although every read/write checks if in proper page */
    if ((ret = adi_imu_SetPage(pDevice, 0x00)) < 0) return ret;
    pDevice->curPage = 0;

    /* read and verify product id */
    uint16_t prodId = 0x0000;
    if ((ret = adi_imu_Read(pDevice, REG_PROD_ID, &prodId)) < 0) return ret;
    if (prodId != pDevice->prodId) {
        DEBUG_PRINT("Error: IMU product-id verification failed: Expected: %d, Read: %d.\n", pDevice->prodId, prodId);
        return adi_imu_ProdIdVerifyFailed_e;
    }
    else DEBUG_PRINT("\nIMU product ADIS%d found.\n\n", prodId);

    /* set default output rate = 10Hz; (4250 SPS / 10 Hz) - 1 = 424 */
    ret = adi_imu_SetDecimationRate(pDevice, 424);

    return ret;
}

int adi_imu_Read(adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint16_t *val)
{
    if (pDevice->status)
    {
        uint8_t pageId = (pageIdRegAddr >> 8) & 0xFF;
        uint8_t regAddr = pageIdRegAddr & 0xFF;

        int ret = adi_imu_Success_e;
        /* ensure we are in right page */
        if ((ret = adi_imu_SetPage(pDevice, pageId)) < 0) return ret;

        uint8_t buf[2] = { regAddr, 0x00 };
        /* send read request */
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        
        /* recv response */
        buf[0] = 0x00; buf[1] = 0x00;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        
        *val = ((uint16_t)buf[0]) << 8 | buf[1];

        return adi_imu_Success_e;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_ReadBurstRaw(adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint8_t *buf, unsigned length)
{
    if (pDevice->status)
    {
        uint8_t pageId = (pageIdRegAddr >> 8) & 0xFF;
        uint8_t regAddr = pageIdRegAddr & 0xFF;

        int ret = adi_imu_Success_e;
        /* ensure we are in right page */
        if ((ret = adi_imu_SetPage(pDevice, pageId)) < 0) return ret;

        /* send burst request and read response */
        /* as per ADIS16495 datasheet pg 7, its sufficient to send single 16-bit read access to read whole burst unlike regular read */
        buf[0] = REG_BURST_CMD; buf[1] = 0x00;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, length) < 0) return adi_imu_SpiRwFailed_e;
        return ret;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_Write(adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint16_t val)
{
    if (pDevice->status)
    {
        uint8_t pageId = (pageIdRegAddr >> 8) & 0xFF;
        uint8_t regAddr = pageIdRegAddr & 0xFF;

        int ret = adi_imu_Success_e;
        /* ensure we are in right page */
        if ((ret = adi_imu_SetPage(pDevice, pageId)) < 0) return ret;

        uint8_t buf[2];
        /* send write request */
        buf[0] = 0x80 | regAddr; buf[1] = val & 0xFF;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        
        buf[0] = 0x80 | (regAddr + 1); buf[1] = ((val >> 8) & 0xFF);
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;

        return adi_imu_Success_e;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_SetPage(adi_imu_Device_t *pDevice, uint8_t pageId)
{
    if (pDevice->curPage != pageId)
    {
        // DEBUG_PRINT("Changing page from %d to %d...", pDevice->curPage, pageId);
        uint8_t buf[2];
        /* send write request */
        buf[0] = 0x80 | REG_PAGE_ID; buf[1] = pageId;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        // DEBUG_PRINT("done.\n");

        pDevice->curPage = pageId;
    }

    return adi_imu_Success_e;
}

int adi_imu_SetDecimationRate (adi_imu_Device_t *pDevice, uint16_t rate)
{
    int ret = adi_imu_Success_e;
    /* Set decimation rate */
    if ((ret = adi_imu_Write(pDevice, REG_DEC_RATE, rate)) < 0) return ret; 
    DEBUG_PRINT("Decimation rate set to %d, output rate %d Samples per second\n", rate, (uint16_t)4250 / (rate + 1));
    return ret;
}

int adi_imu_GetDevInfo (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    int ret = adi_imu_Success_e;

    /* read function control IO: Control, I/O pins, functional definitions */
    if ((ret = adi_imu_Read(pDevice, REG_FNCTIO_CTRL, &(pInfo->fnctioCtrl))) < 0) return ret;

    /* read gpio ctrl io: Control, I/O pins, general-purpose */
    if ((ret = adi_imu_Read(pDevice, REG_GPIO_CTRL, &(pInfo->gpioCtrl))) < 0) return ret;

    /* read clk cfg: Control, clock, and miscellaneous correction */
    if ((ret = adi_imu_Read(pDevice, REG_CONFIG, &(pInfo->clkConfig))) < 0) return ret;

    /* read current decimation rate: Control, output sample rate decimation */
    if ((ret = adi_imu_Read(pDevice, REG_DEC_RATE, &(pInfo->decimationRate))) < 0) return ret;
    
    /* read null cfg: Control, automatic bias correction configuration */
    if ((ret = adi_imu_Read(pDevice, REG_NULL_CNFG, &(pInfo->nullConfig))) < 0) return ret;

    /* read sync scale: Control, input clock scaling (PPS mode) */
    if ((ret = adi_imu_Read(pDevice, REG_SYNC_SCALE, &(pInfo->syncScale))) < 0) return ret;

    /* read measurement range model identifier */
    if ((ret = adi_imu_Read(pDevice, REG_RANG_MDL, &(pInfo->gyroModelId))) < 0) return ret;

    /* read Filter bank 0 selection: Filter selection  */
    if ((ret = adi_imu_Read(pDevice, REG_FILTR_BNK_0, &(pInfo->ftrBank0))) < 0) return ret;

    /* read Filter bank 1 selection: Filter selection  */
    if ((ret = adi_imu_Read(pDevice, REG_FILTR_BNK_1, &(pInfo->ftrBank1))) < 0) return ret;

    /* read firmware revision */
    if ((ret = adi_imu_Read(pDevice, REG_FIRM_REV, &(pInfo->fwRev))) < 0) return ret;
    
    /* read firmware day/month */
    if ((ret = adi_imu_Read(pDevice, REG_FIRM_DM, &(pInfo->fwDayMonth))) < 0) return ret;

    /* read firmware year */
    if ((ret = adi_imu_Read(pDevice, REG_FIRM_Y, &(pInfo->fwYear))) < 0) return ret;

    /* read boot loader version */
    if ((ret = adi_imu_Read(pDevice, REG_BOOT_REV, &(pInfo->bootLoadVer))) < 0) return ret;

    /* read current page id */
    if ((ret = adi_imu_Read(pDevice, REG_PAGE_ID, &(pInfo->pageId))) < 0) return ret;

    /* read product id */
    if ((ret = adi_imu_Read(pDevice, REG_PROD_ID, &(pInfo->prodId))) < 0) return ret;

    /* read serial number */
    if ((ret = adi_imu_Read(pDevice, REG_SERIAL_NUM, &(pInfo->serialNumber))) < 0) return ret;

    return ret;
}

int adi_imu_PrintDevInfo(adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    DEBUG_PRINT("\n================================\n");
    DEBUG_PRINT("IMU Product ID: ADIS%d\n", pInfo->prodId);
    DEBUG_PRINT("IMU FW rev: %02x.%02x\n", IMU_FIRM_REV_UPPER(pInfo->fwRev), IMU_FIRM_REV_LOWER(pInfo->fwRev));
    DEBUG_PRINT("IMU FW date (MM-DD-YYYY): %02x-%02x-%x\n", IMU_FIRM_MONTH(pInfo->fwDayMonth), IMU_FIRM_DAY(pInfo->fwDayMonth), pInfo->fwYear);
    DEBUG_PRINT("IMU Bootloader ver: %d.%d\n", IMU_BOOT_REV_MAJOR(pInfo->bootLoadVer), IMU_BOOT_REV_MINOR(pInfo->bootLoadVer));
    DEBUG_PRINT("IMU Serial no: 0x%x\n", pInfo->serialNumber);
    DEBUG_PRINT("IMU Gyro Model: 0x%x [%s]\n", pInfo->gyroModelId, IMU_RANG_MDL(pInfo->gyroModelId));
    DEBUG_PRINT("IMU Page Id: 0x%x \n", pInfo->pageId);
    DEBUG_PRINT("IMU Decimation rate: %d \n", pInfo->decimationRate);
    DEBUG_PRINT("IMU Sync scale: 0x%x \n", pInfo->syncScale);
    DEBUG_PRINT("IMU Null config: 0x%x \n", pInfo->nullConfig);
    DEBUG_PRINT("IMU Config: 0x%x \n", pInfo->clkConfig);
    DEBUG_PRINT("IMU FNCTIO control: 0x%x \n", pInfo->fnctioCtrl);
    DEBUG_PRINT("IMU GPIO control: 0x%x \n", pInfo->gpioCtrl);
    DEBUG_PRINT("IMU Filter bank 0: 0x%x \n", pInfo->ftrBank0);
    DEBUG_PRINT("IMU Filter bank 1: 0x%x \n", pInfo->ftrBank1);
    DEBUG_PRINT("=================================\n\n");
    return adi_imu_Success_e;
}

int adi_imu_ReadBurst(adi_imu_Device_t *pDevice, adi_imu_BurstOutput_t *pData)
{
    int ret = adi_imu_Success_e;

    if (pDevice->status)
    {
        uint8_t buf[50] = {0};
        unsigned burst_length_expected = BRF_LENGTH + 4;
        if ((ret = adi_imu_ReadBurstRaw(pDevice, REG_BURST_CMD, buf, burst_length_expected)) < 0) return ret;
        // for(int i=0; i<burst_length_expected; i++) printf("0x%x ", buf[i]);
        // printf("\n");

        unsigned startIdx = 6;
        unsigned foundStartFrame = 1;
        unsigned burstIdIdx = 4;
        /* find the 0xA5A5 to 0x0000 transition that marks the start of burst frame */
        if (IMU_TO_WORD(buf, burstIdIdx) != 0xA5A5) foundStartFrame = 0;
        else{
            if (IMU_TO_WORD(buf, burstIdIdx+2) == 0x0000) foundStartFrame = 1;
            else if (IMU_TO_WORD(buf, burstIdIdx+2) != 0xA5A5) foundStartFrame = 0;
            else if (IMU_TO_WORD(buf, burstIdIdx+4) != 0x0000) foundStartFrame = 0;
            else foundStartFrame = 1;
        }

        if (foundStartFrame == 0){
            /* error */
            DEBUG_PRINT("Error: IMU read burst frame is invalid.\n");
            return adi_imu_BurstFrameInvalid_e;
        }

        pData->sysEFlag= IMU_TO_WORD( buf, startIdx);
        pData->tempOut = IMU_TO_WORD( buf, startIdx + 2);

        pData->gyro.x = IMU_TO_DWORD( buf, startIdx + 4 ); 
        pData->gyro.y = IMU_TO_DWORD( buf, startIdx + 8 );
        pData->gyro.z = IMU_TO_DWORD( buf, startIdx + 12 );

        pData->accl.x = IMU_TO_DWORD( buf, startIdx + 16 );
        pData->accl.y = IMU_TO_DWORD( buf, startIdx + 20 );
        pData->accl.z = IMU_TO_DWORD( buf, startIdx + 24 );
        
        pData->dataCntOrTimeStamp = IMU_TO_WORD( buf, startIdx + 28 );
        pData->crc = IMU_TO_DWORD( buf, startIdx + 30 );
        return ret;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_CheckDiagStatus(adi_imu_Device_t *pDevice, adi_imu_DiagStatus_t *pStatus)
{
    int ret = adi_imu_Success_e;
    uint16_t status;

    /* read diagnostic status */
    if ((ret = adi_imu_Read(pDevice, REG_DIAG_STS, &status)) < 0) return ret;
    pStatus->data = status;

    if (status & BITM_DIAG_STS_X_GYRO){
        ret = adi_imu_SelfTestFailed_e;
        DEBUG_PRINT("\nError: Self test FAILED for x-axis gyroscope.\n");
    }

    if (status & BITM_DIAG_STS_Y_GYRO){
        ret = adi_imu_SelfTestFailed_e;
        DEBUG_PRINT("\nError: Self test FAILED for y-axis gyroscope.\n");
    }

    if (status & BITM_DIAG_STS_Z_GYRO){
        ret = adi_imu_SelfTestFailed_e;
        DEBUG_PRINT("\nError: Self test FAILED for z-axis gyroscope.\n");
    }

    if (status & BITM_DIAG_STS_X_ACCL){
        ret = adi_imu_SelfTestFailed_e;
        DEBUG_PRINT("\nError: Self test FAILED for x-axis accelerometer.\n");
    }

    if (status & BITM_DIAG_STS_Y_ACCL){
        ret = adi_imu_SelfTestFailed_e;
        DEBUG_PRINT("\nError: Self test FAILED for y-axis accelerometer.\n");
    }

    if (status & BITM_DIAG_STS_Z_ACCL){
        ret = adi_imu_SelfTestFailed_e;
        DEBUG_PRINT("\nError: Self test FAILED for z-axis accelerometer.\n");
    }
    return ret;
}

int adi_imu_CheckSysStatus(adi_imu_Device_t *pDevice, adi_imu_SysStatus_t *pStatus)
{
    int ret = adi_imu_Success_e;
    uint16_t status;

    /* read diagnostic status */
    if ((ret = adi_imu_Read(pDevice, REG_SYS_E_FLAG, &status)) < 0) return ret;
    pStatus->data = status;

    if (status & BITM_SYS_E_FLAG_BOOT_MEM){
        ret = adi_imu_SystemError_e;
        DEBUG_PRINT("\nError: Boot memory failed."
                    "The device booted up using code from the backup memory bank."
                    "Replace the ADIS16495 if this error occurs. \n");
    }

    if (status & BITM_SYS_E_FLAG_SRAM_CRC){
        ret = adi_imu_SystemError_e;
        DEBUG_PRINT("\nError: SRAM error condition."
                    "CRC failure between the SRAM and flash memory. Initiate a reset to recover."
                    "Replace the ADIS16495 if this error occurs. \n");
    }

    if (status & BITM_SYS_E_FLAG_SPI_COMM){
        ret = adi_imu_SystemError_e;
        DEBUG_PRINT("\nError: SPI communication error."
                    "The total number of SCLK cycles is not equal to an integer multiple of 16."
                    "Repeat the previous communication sequence to recover."
                    "Persistence in this error can indicate a weakness in the SPI service from the master processor. \n");
    }

    if (status & BITM_SYS_E_FLAG_SENSOR_TEST){
        ret = adi_imu_SystemError_e;
        DEBUG_PRINT("\nError: Sensor failure."
                    "Failure in at least one of the inertial sensors."
                    "Replace the ADIS16495 if the error persists, when it is operating in static inertial conditions. \n");
        /* check diag status for more details on failed sensor */
        adi_imu_DiagStatus_t diag;
        adi_imu_CheckDiagStatus(pDevice, &diag);
    }

    if (status & BITM_SYS_E_FLAG_FLSH_MEM_UPD){
        ret = adi_imu_SystemError_e;
        DEBUG_PRINT("\nError: Flash memory update failure."
                    "The most recent flash memory update failed (GLOB_CMD, Bit 3, see Table 142)."
                    "Repeat the test and replace the ADIS16495 if this error persists. \n");
    }

    if (status & BITM_SYS_E_FLAG_PROC_OVERRUN){
        ret = adi_imu_SystemError_e;
        DEBUG_PRINT("\nError: Processing overrun."
                    "Initiate a reset to recover."
                    "Replace the ADIS16495 if this error persists. \n");
    }

    if (status & BITM_SYS_E_FLAG_SYNC_ERR){
        ret = adi_imu_SystemError_e;
        DEBUG_PRINT("\nError: Sync error."
                    "The sample timing is not scaling correctly, when operating in PPS mode (FNCTIO_CTRL Bit 8)."
                    "Verify that the input sync frequency is correct and that SYNC_SCALE (see Table 154) has the correct value \n");
    }
    return ret;
}

int adi_imu_ReadAccl(adi_imu_Device_t *pDevice, adi_imu_AcclOutput_t *pData)
{
    int ret = adi_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_X_ACCL_LOW, &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_X_ACCL_OUT, &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_ACCL_LOW, &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_ACCL_OUT, &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_ACCL_LOW, &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_ACCL_OUT, &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_ReadGyro(adi_imu_Device_t *pDevice, adi_imu_GyroOutput_t *pData)
{
    int ret = adi_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_X_GYRO_LOW, &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_X_GYRO_OUT, &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_GYRO_LOW, &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_GYRO_OUT, &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_GYRO_LOW, &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_GYRO_OUT, &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_ReadDelAng(adi_imu_Device_t *pDevice, adi_imu_DelAngOutput_t *pData)
{
    int ret = adi_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_X_DELTANG_LOW, &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_X_DELTANG_OUT, &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_DELTANG_LOW, &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_DELTANG_OUT, &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_DELTANG_LOW, &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_DELTANG_OUT, &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_ReadDelVel(adi_imu_Device_t *pDevice, adi_imu_DelVelOutput_t *pData)
{
    int ret = adi_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_X_DELTVEL_LOW, &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_X_DELTVEL_OUT, &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_DELTVEL_LOW, &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_DELTVEL_OUT, &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_DELTVEL_LOW, &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_DELTVEL_OUT, &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_GetAcclScale(adi_imu_Device_t *pDevice, adi_imu_AcclScale_t *pData)
{
    int ret = adi_imu_Success_e;
    /* read x-axis scale output */
    if ((ret = adi_imu_Read(pDevice, REG_X_ACCL_SCALE, (uint16_t*)&(pData->x))) < 0) return ret;

    /* read y-axis scale output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_ACCL_SCALE, (uint16_t*)&(pData->y))) < 0) return ret;

    /* read z-axis scale output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_ACCL_SCALE, (uint16_t*)&(pData->z))) < 0) return ret;
    return ret;
}

int adi_imu_GetGyroScale(adi_imu_Device_t *pDevice, adi_imu_GyroScale_t *pData)
{
    int ret = adi_imu_Success_e;
    /* read x-axis scale output */
    if ((ret = adi_imu_Read(pDevice, REG_X_GYRO_SCALE, (uint16_t*)&(pData->x))) < 0) return ret;

    /* read y-axis scale output */
    if ((ret = adi_imu_Read(pDevice, REG_Y_GYRO_SCALE, (uint16_t*)&(pData->y))) < 0) return ret;

    /* read z-axis scale output */
    if ((ret = adi_imu_Read(pDevice, REG_Z_GYRO_SCALE, (uint16_t*)&(pData->z))) < 0) return ret;
    return ret;
}

int adi_imu_GetAcclBias(adi_imu_Device_t *pDevice, adi_imu_AcclBias_t *pData)
{
    int ret = adi_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_XA_BIAS_LOW, &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_XA_BIAS_HIGH, &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_YA_BIAS_LOW, &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_YA_BIAS_HIGH, &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_ZA_BIAS_LOW, &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_ZA_BIAS_HIGH, &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_GetGyroBias(adi_imu_Device_t *pDevice, adi_imu_GyroBias_t *pData)
{
    int ret = adi_imu_Success_e;
    uint16_t low, high;
    /* read x-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_XG_BIAS_LOW, &low)) < 0) return ret;
    /* read x-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_XG_BIAS_HIGH, &high)) < 0) return ret;
    pData->x = ((high << 16) & 0xFFFF0000) | low;

    /* read y-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_YG_BIAS_LOW, &low)) < 0) return ret;
    /* read y-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_YG_BIAS_HIGH, &high)) < 0) return ret;
    pData->y = ((high << 16) & 0xFFFF0000) | low;

    /* read z-axis LOW output */
    if ((ret = adi_imu_Read(pDevice, REG_ZG_BIAS_LOW, &low)) < 0) return ret;
    /* read z-axis HIGH output */
    if ((ret = adi_imu_Read(pDevice, REG_ZG_BIAS_HIGH, &high)) < 0) return ret;
    pData->z = ((high << 16) & 0xFFFF0000) | low;
    return ret;
}

int adi_imu_ConfigGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, adi_imu_Direction_e direction)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("Configuring GPIO %d as %s...", id, (direction == INPUT) ? "INPUT" : "OUTPUT");
    if ((ret = adi_imu_Write(pDevice, REG_GPIO_CTRL, ((direction) << id))) < 0) return ret; 
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SetGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id)
{
    int ret = adi_imu_Success_e;
    uint16_t data = 0x00;
    if ((ret = adi_imu_Read(pDevice, REG_GPIO_CTRL, &data)) < 0) return ret; 
    if ((ret = adi_imu_Write(pDevice, REG_GPIO_CTRL, data | (BITM_GPIO_CTRL_DIO1_DATA << id))) < 0) return ret; 
    return ret;
}

int adi_imu_ClearGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id)
{
    int ret = adi_imu_Success_e;
    uint16_t data = 0x00;
    if ((ret = adi_imu_Read(pDevice, REG_GPIO_CTRL, &data)) < 0) return ret; 
    if ((ret = adi_imu_Write(pDevice, REG_GPIO_CTRL, data & ~(BITM_GPIO_CTRL_DIO1_DATA << id))) < 0) return ret; 
    return ret;
}

int adi_imu_GetGpio(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, uint8_t* val)
{
    int ret = adi_imu_Success_e;
    uint16_t data = 0x00;
    if ((ret = adi_imu_Read(pDevice, REG_GPIO_CTRL, &data)) < 0) return ret; 
    *val = (uint8_t) (data & (BITM_GPIO_CTRL_DIO1_DATA << id));
    return ret;
}

int adi_imu_ConfigDataReady(adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, adi_imu_Polarity_e polarity)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("Configuring data ready...");
    uint16_t fnctio = 0x00;
    if ((ret = adi_imu_Read(pDevice, REG_FNCTIO_CTRL, &fnctio)) < 0) return ret; 

    fnctio |= ((id) << BITP_FNCTIO_CTRL_DATA_RDY_DIO) | ((polarity) << BITP_FNCTIO_CTRL_DATA_RDY_POL);
    if ((ret = adi_imu_Write(pDevice, REG_FNCTIO_CTRL, fnctio)) < 0) return ret; 
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_ConfigSyncClkMode(adi_imu_Device_t *pDevice, adi_imu_ClockMode_e mode, adi_imu_EnDis_e clkEn, \
                              adi_imu_EdgeType_e polarity, adi_imu_GPIO_e inputGpio)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("Configuring sync clock mode...");
    uint16_t fnctio = 0x00;
    if ((ret = adi_imu_Read(pDevice, REG_FNCTIO_CTRL, &fnctio)) < 0) return ret; 

    fnctio |= ((mode) << BITP_FNCTIO_CTRL_SYNC_CLK_MODE) | ((clkEn) << BITP_FNCTIO_CTRL_SYNC_CLK_EN) | \
                      ((polarity) << BITP_FNCTIO_CTRL_SYNC_CLK_POL) | ((inputGpio) << BITP_FNCTIO_CTRL_SYNC_CLK_DIO);
    if ((ret = adi_imu_Write(pDevice, REG_FNCTIO_CTRL, fnctio)) < 0) return ret; 
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SetDataReady(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("%s data ready...", (val == ENABLE) ? "Enabling" : "Disabling");
    uint16_t fnctio = 0x00;
    if ((ret = adi_imu_Read(pDevice, REG_FNCTIO_CTRL, &fnctio)) < 0) return ret; 

    if ((ret = adi_imu_Write(pDevice, REG_FNCTIO_CTRL, (val == ENABLE) ? fnctio | BITM_FNCTIO_CTRL_DATA_RDY_EN : fnctio)) < 0) return ret; 
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SetLineargComp(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("%s linear g compensation...", (val == ENABLE) ? "Enabling" : "Disabling");
    if ((ret = adi_imu_Write(pDevice, REG_CONFIG, (val == ENABLE) ? BITM_CONFIG_LIN_G_COMP : 0x00)) < 0) return ret; 
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SetPPercAlignment(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("%s point of percussion alignment...", (val == ENABLE) ? "Enabling" : "Disabling");
    if ((ret = adi_imu_Write(pDevice, REG_CONFIG, (val == ENABLE) ? BITM_CONFIG_PNT_PERC_ALIGN : 0x00)) < 0) return ret; 
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_SoftwareReset(adi_imu_Device_t *pDevice)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("Performing software reset...");
    if ((ret = adi_imu_Write(pDevice, REG_GLOB_CMD, BITM_GLOB_CMD_SOFT_RST)) < 0) return ret; 
    /* stall time for software reset = 210 ms */
    adi_imu_DelayMicroSeconds(210000);
    DEBUG_PRINT("done.\n");
    return ret;
}

int adi_imu_ClearUserCalibration(adi_imu_Device_t *pDevice)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("Clearing all User calibration data...");
    if ((ret = adi_imu_Write(pDevice, REG_GLOB_CMD, BITM_GLOB_CMD_CLR_USR_CALIB)) < 0) return ret; 
    DEBUG_PRINT("done.\n");
    return ret;
}

// int adi_imu_UpdateFlashMemory       (adi_imu_Device_t *pDevice); // TODO: implement

int adi_imu_PerformSelfTest(adi_imu_Device_t *pDevice)
{
    int ret = adi_imu_Success_e;
    DEBUG_PRINT("Performing Self test...");
    if ((ret = adi_imu_Write(pDevice, REG_GLOB_CMD, BITM_GLOB_CMD_SELF_TEST)) < 0) return ret;
    DEBUG_PRINT("done.\n");

    /* stall time for On demand self test = 20 ms */
    adi_imu_DelayMicroSeconds(50000);

    DEBUG_PRINT("Reading test results..");
    adi_imu_SysStatus_t sysStatus;
    if ((ret = adi_imu_CheckSysStatus(pDevice, &sysStatus)) < 0) return ret;
    DEBUG_PRINT("\n\nSelf Test result: %s\n", (sysStatus.data) ? "FAILED": "SUCCESS");
    return ret;
}

// int adi_imu_UpdateBiasCorrection    (adi_imu_Device_t *pDevice); // TODO: implement
