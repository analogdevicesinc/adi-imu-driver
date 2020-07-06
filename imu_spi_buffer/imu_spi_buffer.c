/*******************************************************************************
 *   @file   imu_spi_buffer.c
 *   @brief  Driver interface for iSensor-Spi-buffer + ADIS16xxx IMU
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#include "imu_spi_buffer.h"

int imubuf_init (adi_imu_Device_t *pDevice)
{
    return adi_imu_Init(pDevice);
}

int imubuf_ConfigBuf (adi_imu_Device_t *pDevice, imubuf_BufConfig_t config)
{
    int ret = adi_imu_Success_e;

    /* Set BUF_CONFIG */
    uint16_t buf_config = TO_REG(config.overflowAction, BITP_ISENSOR_BUF_CFG_OVERFLOW, BITM_ISENSOR_BUF_CFG_OVERFLOW) |
                            TO_REG(config.spiWordSize, BITP_ISENSOR_BUF_CFG_SPIWORDSIZE, BITM_ISENSOR_BUF_CFG_SPIWORDSIZE);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_CONFIG, buf_config)) < 0) return ret; 

    /* Set BUF_LEN */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_CONFIG, config.bufLen)) < 0) return ret; 

    /* Set BUF_MAX_CNT */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_CONFIG, config.bufMaxCnt)) < 0) return ret; 

    return ret;
}

int imubuf_ConfigDio(adi_imu_Device_t *pDevice, imubuf_ImuDioConfig_t config)
{
    int ret = adi_imu_Success_e;
    /* Set Input pins (between IMU <-> Spi buffer) */
    uint16_t dio_in_config = TO_REG(config.dataReadyPin, BITP_ISENSOR_DIO_IN_CFG_DR_SEL, BITM_ISENSOR_DIO_IN_CFG_DR_SEL) \
                            | TO_REG(config.dataReadyPolarity, BITP_ISENSOR_DIO_IN_CFG_DR_POL, BITM_ISENSOR_DIO_IN_CFG_DR_POL) \
                            | TO_REG(config.ppsPin, BITP_ISENSOR_DIO_IN_CFG_PPS_SEL, BITM_ISENSOR_DIO_IN_CFG_PPS_SEL) \
                            | TO_REG(config.ppsPolarity, BITP_ISENSOR_DIO_IN_CFG_PPS_POL, BITM_ISENSOR_DIO_IN_CFG_PPS_POL);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_CONFIG, dio_in_config)) < 0) return ret; 

    /* Set Output pins (between Spi buffer <-> Host)  */
    uint16_t dio_out_config = TO_REG(config.passThruPin, BITP_ISENSOR_DIO_OUT_CFG_PIN_PASS, BITM_ISENSOR_DIO_OUT_CFG_PIN_PASS) \
                            | TO_REG(config.watermarkIrqPin, BITP_ISENSOR_DIO_OUT_CFG_WTRMRK, BITM_ISENSOR_DIO_OUT_CFG_WTRMRK) \
                            | TO_REG(config.overflowIrqPin, BITP_ISENSOR_DIO_OUT_CFG_OVRFLW, BITM_ISENSOR_DIO_OUT_CFG_OVRFLW) \
                            | TO_REG(config.errorIrqPin, BITP_ISENSOR_DIO_OUT_CFG_ERROR, BITM_ISENSOR_DIO_OUT_CFG_ERROR);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_CONFIG, dio_out_config)) < 0) return ret; 

    return ret;
}

int imubuf_ConfigUserSpi(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = adi_imu_Success_e;
    /* Set User SPI config */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_USER_SPI_CONFIG, val)) < 0) return ret; 
    return ret;
}

int imubuf_ConfigImuSpi(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = adi_imu_Success_e;
    /* Set IMU SPI config */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_IMU_SPI_CONFIG, val)) < 0) return ret; 
    return ret;
}

int imubuf_GetUTC(adi_imu_Device_t *pDevice, uint32_t* pTime)
{
    int ret = adi_imu_Success_e;
    uint16_t utc_lwr, utc_upr;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_UTC_TIMESTAMP_LWR, &utc_lwr)) < 0) return ret; 
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_UTC_TIMESTAMP_UPR, &utc_upr)) < 0) return ret; 
    *pTime = (((uint32_t)(utc_upr) << 16) | utc_lwr);
    return ret;
}

int imubuf_GetUpTime(adi_imu_Device_t *pDevice, uint32_t* pTime)
{
    int ret = adi_imu_Success_e;
    uint16_t uptime_lwr, uptime_upr;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_TIMESTAMP_LWR, &uptime_lwr)) < 0) return ret; 
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_TIMESTAMP_UPR, &uptime_upr)) < 0) return ret; 
    *pTime = (((uint32_t)(uptime_upr) << 16) | uptime_lwr);
    return ret;
}

int imubuf_GetInfo(adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo)
{
    int ret = adi_imu_Success_e;
    /* read system status */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_STATUS, &(pInfo->sysStatus))) < 0) return ret; 
    /* read firmware day/month */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_FW_DAY_MONTH, &(pInfo->fwDayMonth))) < 0) return ret; 
    /* read firmware year */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_FW_YEAR, &(pInfo->fwYear))) < 0) return ret; 
    /* read firmware revision */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_FW_REV, &(pInfo->fwRev))) < 0) return ret; 
    return ret;
}

int imubuf_PrintInfo(adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo)
{
    DEBUG_PRINT("\n================================\n");
    DEBUG_PRINT("IMU BUF FW rev: %02x.%02x\n", IMU_BUF_FIRM_REV_UPPER(pInfo->fwRev), IMU_BUF_FIRM_REV_LOWER(pInfo->fwRev));
    DEBUG_PRINT("IMU BUF FW date (MM-DD-YYYY): %02x-%02x-%x\n", IMU_BUF_FIRM_MONTH(pInfo->fwDayMonth), IMU_BUF_FIRM_DAY(pInfo->fwDayMonth), pInfo->fwYear);
    DEBUG_PRINT("IMU BUF System Status: 0x%x\n", pInfo->sysStatus);
    DEBUG_PRINT("=================================\n\n");
    return adi_imu_Success_e;
}

int imubuf_SetUserCmd(adi_imu_Device_t *pDevice, uint16_t val)
{
    return adi_imu_Write(pDevice, REG_ISENSOR_USER_COMMAND, val);
}

int imubuf_CheckSysStatus(adi_imu_Device_t *pDevice, imubuf_SysStatus_t* pStatus)
{
    int ret = adi_imu_Success_e;
    uint16_t val = 0x00;
    /* read system status */   
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_STATUS, &val)) < 0) return ret; 

    pStatus->bufWaterMark = FROM_REG(val, BITP_ISENSOR_STATUS_BUF_WTRMRK, BITM_ISENSOR_STATUS_BUF_WTRMRK);
    pStatus->bufFull = FROM_REG(val, BITP_ISENSOR_STATUS_BUF_FULL, BITM_ISENSOR_STATUS_BUF_FULL);
    pStatus->spiError = FROM_REG(val, BITP_ISENSOR_STATUS_SPI_ERROR, BITM_ISENSOR_STATUS_SPI_ERROR);
    pStatus->spiOverflow = FROM_REG(val, BITP_ISENSOR_STATUS_SPI_OVRFLW, BITM_ISENSOR_STATUS_SPI_OVRFLW);
    pStatus->overrun = FROM_REG(val, BITP_ISENSOR_STATUS_OVERRUN, BITM_ISENSOR_STATUS_OVERRUN);
    pStatus->dmaError = FROM_REG(val, BITP_ISENSOR_STATUS_DMA_ERROR, BITM_ISENSOR_STATUS_DMA_ERROR);
    pStatus->ppsUnlock = FROM_REG(val, BITP_ISENSOR_STATUS_PPS_UNLOCK, BITM_ISENSOR_STATUS_PPS_UNLOCK);
    pStatus->flashError = FROM_REG(val, BITP_ISENSOR_STATUS_FLASH_ERROR, BITM_ISENSOR_STATUS_FLASH_ERROR);
    pStatus->flashUpdateError = FROM_REG(val, BITP_ISENSOR_STATUS_FLASH_UPD_ERROR, BITM_ISENSOR_STATUS_FLASH_UPD_ERROR);
    pStatus->fault = FROM_REG(val, BITP_ISENSOR_STATUS_FAULT, BITM_ISENSOR_STATUS_FAULT);
    pStatus->watchdog = FROM_REG(val, BITP_ISENSOR_STATUS_WATCHDOG, BITM_ISENSOR_STATUS_WATCHDOG);
    return ret;
}