/*
    File: adi_imu_driver.c

    Description:
    Driver interface for ADIS16xxx IMU

    Author: Sundar Palani <sundar.palani@analog.com>
*/

#include "adi_imu_driver.h"

static uint8_t gBuffer[10] = { 0 };

/* external spi driver API (provided by user) */
extern int adi_imu_SpiInit (adi_imu_Device_t *pDevice);
extern int adi_imu_SpiReadWrite (adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);


int adi_imu_Init (adi_imu_Device_t *pDevice)
{
    pDevice->status = 0;

    int spi_status = adi_imu_SpiInit(pDevice);
    if (spi_status < 0) return spi_status;

    /* set device handler after successful initialization */
    pDevice->status = 1;

    int ret = adi_imu_Success_e;

    /* Go to Page 0 */
    gBuffer[0] = 0x80; gBuffer[1] = 0x00;
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;

    /* read product id */
    gBuffer[0] = 0x7E; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    uint16_t prodId = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];

    if (prodId != pDevice->prodId) {
        IMU_DEBUG_PRINT("Error: IMU product ID verification failed: Expected: %d, Read: %d\n", pDevice->prodId, prodId);
        return adi_imu_ProdIdVerifyFailed_e;
    }

    /* set default output rate = 10Hz; (4250 SPS / 10 Hz) - 1 = 424 */
    ret = adi_imu_SetDecimationRate(pDevice, 424);

    return ret;
}

int adi_imu_read(adi_imu_Device_t *pDevice, uint8_t *buf)
{
    if (pDevice->status) {
        /* send request */
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        
        /* recv response */
        buf[0] = 0x00; buf[1] = 0x00;
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        return adi_imu_Success_e;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_read_burst(adi_imu_Device_t *pDevice, uint8_t *buf, unsigned length)
{
    if (pDevice->status) {
        /* send burst request and read response */
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, length) < 0) return adi_imu_SpiRwFailed_e;
        return adi_imu_Success_e;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_write(adi_imu_Device_t *pDevice, uint8_t *buf)
{
    if (pDevice->status) {
        if (adi_imu_SpiReadWrite(pDevice, buf, buf, 2) < 0) return adi_imu_SpiRwFailed_e;
        return adi_imu_Success_e;
    }
    else return adi_imu_BadDevice_e;
}

int adi_imu_SetDecimationRate (adi_imu_Device_t *pDevice, uint16_t rate)
{
    int ret = adi_imu_Success_e;

    /* Go to Page 3 */
    gBuffer[0] = 0x80; gBuffer[1] = 0x03;
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;

    /* Set decimation rate */
    gBuffer[0] = 0x8C; gBuffer[1] = rate & 0xFF;
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;
    gBuffer[0] = 0x8D; gBuffer[1] = ((rate >> 8) & 0xFF);
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;

    /* Go back to Page 0 */
    gBuffer[0] = 0x80; gBuffer[1] = 0x00;
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;

    return ret;
}

int adi_imu_GetDevInfo (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    int ret = adi_imu_Success_e;
    /* Go to Page 3 */
    gBuffer[0] = 0x80; gBuffer[1] = 0x03;
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;

    /* read measurement range model identifier */
    gBuffer[0] = 0x12; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    (*pInfo).gyroModelId = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];

    /* read firmware revision, day/month, year,  bootloader version*/
    gBuffer[0] = 0x78; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    (*pInfo).fwRev = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];
    
    gBuffer[0] = 0x7A; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    (*pInfo).fwDayMonth = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];

    gBuffer[0] = 0x7C; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    (*pInfo).fwYear = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];

    gBuffer[0] = 0x7E; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    (*pInfo).bootLoadVer = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];

    /* Go to Page 4 */
    gBuffer[0] = 0x80; gBuffer[1] = 0x04;
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;

    /* read serial number */
    gBuffer[0] = 0x20; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    (*pInfo).serialNumber = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];

    /* Go to Page 0 */
    gBuffer[0] = 0x80; gBuffer[1] = 0x00;
    if ((ret = adi_imu_write(pDevice, gBuffer)) < 0) return ret;

    /* read product id */
    gBuffer[0] = 0x7E; gBuffer[1] = 0x00;
    if ((ret = adi_imu_read(pDevice, gBuffer)) < 0) return ret;
    (*pInfo).prodId = ((uint16_t)gBuffer[0]) << 8 | gBuffer[1];
    
    return ret;
}

int adi_imu_PrintDevInfo(adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo)
{
    IMU_DEBUG_PRINT("IMU Product ID: ADIS%d\n", pInfo->prodId);
    IMU_DEBUG_PRINT("IMU FW rev: %02x.%02x\n", IMU_FW_REV_UPPER(pInfo->fwRev), IMU_FW_REV_LOWER(pInfo->fwRev));
    IMU_DEBUG_PRINT("IMU FW date (MM-DD-YYYY): %02x-%02x-%x\n", IMU_FW_MONTH(pInfo->fwDayMonth), IMU_FW_DAY(pInfo->fwDayMonth), pInfo->fwYear);
    IMU_DEBUG_PRINT("IMU Bootloader ver: %d.%d\n", IMU_BOOT_REV_MAJOR(pInfo->bootLoadVer), IMU_BOOT_REV_MINOR(pInfo->bootLoadVer));
    IMU_DEBUG_PRINT("IMU Serial no: 0x%x\n", pInfo->serialNumber);
    IMU_DEBUG_PRINT("IMU Gyro Model: 0x%x [%s]\n", pInfo->gyroModelId, IMU_RANGE_MODEL(pInfo->gyroModelId));
    return adi_imu_Success_e;
}

int adi_imu_ReadBurst(adi_imu_Device_t *pDevice, adi_imu_BurstOutput_t *pData)
{
    int ret = adi_imu_Success_e;
    uint8_t buf[50] = {0};
    buf[0] = 0x7C; buf[1] = 0x00;
    if ((ret = adi_imu_read_burst(pDevice, buf, 40)) < 0) return ret;

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

