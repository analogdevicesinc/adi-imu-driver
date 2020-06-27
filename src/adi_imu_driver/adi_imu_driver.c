/*
    File: adi_imu_driver.c

    Description:
    Driver interface for ADIS16xxx IMU

    Author: Sundar Palani <sundar.palani@analog.com>
*/

#include "adi_imu_driver.h"

static uint8_t gCurPage = 0x00;

/* external spi driver API (provided by user) */
extern int adi_imu_SpiInit (adi_imu_Device_t *pDevice);
extern int adi_imu_SpiReadWrite (adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);


int adi_imu_Init (adi_imu_Device_t *pDevice)
{
    int ret = adi_imu_Success_e;
    pDevice->status = 0;

    /* initialize spi device */
    int spi_status = adi_imu_SpiInit(pDevice);
    if (spi_status < 0) return spi_status;

    /* set device handler after successful initialization */
    pDevice->status = 1;

    /* read and verify product id */
    uint16_t prodId;
    if ((ret = adi_imu_Read(pDevice, REG_PROD_ID, &prodId)) < 0) return ret;
    if (prodId != pDevice->prodId) {
        IMU_DEBUG_PRINT("Error: IMU product-id verification failed: Expected: %d, Read: %d\n", pDevice->prodId, prodId);
        return adi_imu_ProdIdVerifyFailed_e;
    }

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

        uint8_t buf[2];
        /* send read request */
        buf[0] = regAddr; buf[1] = 0x00;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        
        /* recv response */
        buf[0] = 0x00; buf[1] = 0x00;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        
        *val = ((uint16_t)buf[0]) << 8 | buf[1];

        return adi_imu_Success_e;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_ReadBurstRaw(adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint8_t *val, unsigned length)
{
    if (pDevice->status)
    {
        uint8_t pageId = (pageIdRegAddr >> 8) & 0xFF;
        uint8_t regAddr = pageIdRegAddr & 0xFF;

        int ret = adi_imu_Success_e;
        /* ensure we are in right page */
        if ((ret = adi_imu_SetPage(pDevice, pageId)) < 0) return ret;

        /* send burst request and read response */
        val[0] = REG_BURST_CMD; val[1] = 0x00;
        if (adi_imu_SpiReadWrite(pDevice, val, val, length) < 0) return adi_imu_SpiRwFailed_e;
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
        
        buf[0] = 0x80 | (regAddr + 2); buf[1] = ((val >> 8) & 0xFF);
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;

        return adi_imu_Success_e;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_SetPage(adi_imu_Device_t *pDevice, uint8_t pageNo)
{
    if (gCurPage != pageNo)
    {
        uint8_t buf[2];
        /* send write request */
        buf[0] = 0x80 | REG_PAGE_ID; buf[1] = pageNo & 0xFF;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        
        buf[0] = 0x80 | (REG_PAGE_ID + 2); buf[1] = ((pageNo >> 8) & 0xFF);
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;

        gCurPage = pageNo;
    }

    return adi_imu_Success_e;
}

int adi_imu_SetDecimationRate (adi_imu_Device_t *pDevice, uint16_t rate)
{
    int ret = adi_imu_Success_e;
    /* Set decimation rate */
    ret = adi_imu_Write(pDevice, REG_DEC_RATE, rate);
    return ret;
}

int adi_imu_GetDevInfo (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    int ret = adi_imu_Success_e;

    /* read measurement range model identifier */
    if ((ret = adi_imu_Read(pDevice, REG_RANG_MDL, &(pInfo->gyroModelId))) < 0) return ret;

    /* read firmware revision */
    if ((ret = adi_imu_Read(pDevice, REG_FIRM_REV, &(pInfo->fwRev))) < 0) return ret;
    
    /* read firmware day/month */
    if ((ret = adi_imu_Read(pDevice, REG_FIRM_DM, &(pInfo->fwDayMonth))) < 0) return ret;

    /* read firmware year */
    if ((ret = adi_imu_Read(pDevice, REG_FIRM_Y, &(pInfo->fwYear))) < 0) return ret;

    /* read boot loader version */
    if ((ret = adi_imu_Read(pDevice, REG_BOOT_REV, &(pInfo->bootLoadVer))) < 0) return ret;

    /* read serial number */
    if ((ret = adi_imu_Read(pDevice, REG_SERIAL_NUM, &(pInfo->serialNumber))) < 0) return ret;

    /* read product id */
    if ((ret = adi_imu_Read(pDevice, REG_PROD_ID, &(pInfo->prodId))) < 0) return ret;
    
    return ret;
}

int adi_imu_PrintDevInfo(adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    IMU_DEBUG_PRINT("IMU Product ID: ADIS%d\n", pInfo->prodId);
    IMU_DEBUG_PRINT("IMU FW rev: %02x.%02x\n", IMU_FIRM_REV_UPPER(pInfo->fwRev), IMU_FIRM_REV_LOWER(pInfo->fwRev));
    IMU_DEBUG_PRINT("IMU FW date (MM-DD-YYYY): %02x-%02x-%x\n", IMU_FIRM_MONTH(pInfo->fwDayMonth), IMU_FIRM_DAY(pInfo->fwDayMonth), pInfo->fwYear);
    IMU_DEBUG_PRINT("IMU Bootloader ver: %d.%d\n", IMU_BOOT_REV_MAJOR(pInfo->bootLoadVer), IMU_BOOT_REV_MINOR(pInfo->bootLoadVer));
    IMU_DEBUG_PRINT("IMU Serial no: 0x%x\n", pInfo->serialNumber);
    IMU_DEBUG_PRINT("IMU Gyro Model: 0x%x [%s]\n", pInfo->gyroModelId, IMU_RANG_MDL(pInfo->gyroModelId));
    return adi_imu_Success_e;
}

int adi_imu_ReadBurst(adi_imu_Device_t *pDevice, adi_imu_BurstOutput_t *pData)
{
    int ret = adi_imu_Success_e;

    if (pDevice->status)
    {
        uint8_t buf[50] = {0};
        if ((ret = adi_imu_ReadBurstRaw(pDevice, REG_BURST_CMD, buf, 40)) < 0) return ret;

        unsigned startIdx;
        // if (IMU_TO_HALFWORD(gBuffer, 2) != 0x0000) return -3;
        // if (IMU_TO_HALFWORD(gBuffer, 4) != 0xA5A5) return -2;
        if (IMU_TO_HALFWORD(buf, 6) == 0x0000) { // BRF 20 segment
            startIdx = 6;
        }
        else if (IMU_TO_HALFWORD(buf, 6) == 0xA5A5) { // BRF 19 segment
            startIdx = 8;
        }
        else {
            /* error */
            IMU_DEBUG_PRINT("Error: IMU read burst frame is invalid\n");
            return adi_imu_BurstFrameInvalid_e;
        }

        (*pData).sysEFlag= IMU_TO_HALFWORD( buf, startIdx);
        (*pData).tempOut = IMU_TO_HALFWORD( buf, startIdx + 2);

        (*pData).gyro.x = IMU_TO_WORD( buf, startIdx + 4 ); 
        (*pData).gyro.y = IMU_TO_WORD( buf, startIdx + 8 );
        (*pData).gyro.z = IMU_TO_WORD( buf, startIdx + 12 );

        (*pData).accl.x = IMU_TO_WORD( buf, startIdx + 16 );
        (*pData).accl.y = IMU_TO_WORD( buf, startIdx + 20 );
        (*pData).accl.z = IMU_TO_WORD( buf, startIdx + 24 );

        (*pData).crc = IMU_TO_WORD( buf, startIdx + 28 );
        return ret;
    }
    else return adi_imu_BadDevice_e;
}

