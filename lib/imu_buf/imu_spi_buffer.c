/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		imu_spi_buffer.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for iSensor-Spi-buffer + ADIS16xxx IMU.
 **/


#include "imu_spi_buffer.h"

/* 
NOTE: 

1. BUF_WRITE_N registers contains spi commands to read IMU regs from buffer board to IMU,
    requires extra dummy seq if last transaction is read.

2. BUF_DATA_N registers contains spi responses from IMU to buffer board.

3. `g_bufBurstReadSeq` contains spi commands to read all BUF_DATA_N regs from host to buffer board,
    requires extra dummy seq if last transaction is read.

4. `g_bufBurstReadSeqAuto` contains spi commands to read BUF_DATA_N regs from host to buffer board, 
    but only to read subset of BUF_DATA_N regs that contains valid reg read responses from IMU.
*/

static uint16_t g_maxBufCnt = 0;
static uint16_t g_bufLengthBytes = 0;
static uint8_t g_captureStarted = 0;

/* store read sequences of only those BUF_DATA_N registers that contain valid reg read responses, lets ignore reg write responses */
static uint8_t g_bufBurstReadSeqAuto[MAX_BUF_LEN_BYTES+4] = {0xFF & REG_ISENSOR_BUF_RETRIEVE, 0x00};
static uint32_t g_bufBurstReadSeqAutoCnt = 2;

static uint8_t g_bufBurstReadSeq[MAX_BUF_LEN_BYTES+4] = { 0xFF & REG_ISENSOR_BUF_RETRIEVE, 0x00 ,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_0, 0x00,\
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
                                                        0xFF & REG_ISENSOR_BUF_DATA_16, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_17, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_18, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_19, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_20, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_21, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_22, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_23, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_24, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_25, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_26, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_27, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_28, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_29, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_30, 0x00,\
                                                        0xFF & REG_ISENSOR_BUF_DATA_31, 0x00,\
                                                        0x00, 0x00,\
                                                        };

// extra 2 for last dummy 16bit overhead for read
static uint8_t g_BurstTxBuf[MAX_BUF_LEN_BYTES + IMU_BUF_BURST_HEADER_LEN_BYTES + 2] = {0xFF & REG_ISENSOR_BUF_RETRIEVE, 0x00};

int imubuf_init (adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;

    /* check SPI clock frequency */
    if (pDevice->spiSpeed > IMU_BUF_MAX_SPI_CLK) 
    {
        DEBUG_PRINT("Warning: SPI clock out of range (%d Hz) (Setting to max speed = %d Hz)\n", pDevice->spiSpeed, IMU_BUF_MAX_SPI_CLK);
        pDevice->spiSpeed = IMU_BUF_MAX_SPI_CLK;
        if ((ret = spi_Init(pDevice)) < 0) return ret;
    }

    /* check stall time */
    if (pDevice->spiDelay < IMU_BUF_MIN_STALL_US) 
    {
        DEBUG_PRINT("Warning: SPI STALL time is low (%d us) (Setting to min stall time = %d us)\n", pDevice->spiDelay, IMU_BUF_MIN_STALL_US);
        pDevice->spiDelay = IMU_BUF_MIN_STALL_US;
    }

    /* software reset */
    if ((ret = imubuf_SoftwareReset(pDevice)) < 0) return ret;
    
    imubuf_DevInfo_t pInfo = {0};
    /* read firmware revision */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_FW_REV, &(pInfo.fwRev))) < 0) return ret; 
    float fw_rev = (float)IMU_BUF_FIRM_REV_MAJOR(pInfo.fwRev) + (float)(IMU_BUF_FIRM_REV_MINOR(pInfo.fwRev)) / 100.0;
    if (fw_rev < IMU_BUF_MIN_FW_REV_REQUIRED)
    {
        DEBUG_PRINT("IMU BUF FW Rev (v%.2f) not supported. (Min required: v%.2f)\n", fw_rev, IMU_BUF_MIN_FW_REV_REQUIRED);
        return Err_Imubuf_FwRevNotSupported_e;
    }

    /* setting IMU spi stall time to 16us (from default: 7us)*/
    if ((ret = imubuf_ConfigImuSpi(pDevice, 0x080A)) < 0) return ret;

    /* stop capture and delete any old buffered data */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StopCapture(pDevice, &curBufCnt)) < 0) return ret;
    
    /* read max buffer cnt (READ ONLY)*/
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_MAX_CNT, &g_maxBufCnt)) < 0) return ret; 
    DEBUG_PRINT("IMU BUF MAX Count: %d buffers\n", g_maxBufCnt);

    /* read buffer length currently set */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_LEN, &g_bufLengthBytes)) < 0) return ret; 
    DEBUG_PRINT("IMU BUF length: %d bytes\n", g_bufLengthBytes);
    return ret;
}

int imubuf_Detect(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0x00;
    if ((ret = adi_imu_SetPage(pDevice, 0xFD)) < 0) return ret;
    if ((ret = adi_imu_Read(pDevice, 0xFD00, &val)) < 0) return ret;

    if (val != 0xFD) {
        DEBUG_PRINT("IMU BUFFER Board not detected\n");
        return -1;
    }
    DEBUG_PRINT("IMU BUFFER Board detected\n");
    return 0;
}

int imubuf_ConfigBuf (adi_imu_Device_t *pDevice, imubuf_BufConfig_t config)
{
    int ret = Err_imu_Success_e;

    /* Set BUF_CONFIG */
    uint16_t buf_config = TO_REG(config.overflowAction, BITP_ISENSOR_BUF_CFG_OVERFLOW, BITM_ISENSOR_BUF_CFG_OVERFLOW) |
                            TO_REG(config.imuBurstEn, BITP_ISENSOR_BUF_CFG_IMU_BURST_EN, BITM_ISENSOR_BUF_CFG_IMU_BURST_EN) |
                            TO_REG(config.bufBurstEn, BITP_ISENSOR_BUF_CFG_BUF_BURST_EN, BITM_ISENSOR_BUF_CFG_BUF_BURST_EN);
    printf("Burst config : 0x%x\n", buf_config);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_CONFIG, buf_config)) < 0) return ret; 

    return ret;
}

int imubuf_ConfigDio(adi_imu_Device_t *pDevice, imubuf_ImuDioConfig_t config)
{
    int ret = Err_imu_Success_e;
    /* Set Input pins (between IMU <-> Spi buffer) */
    uint16_t dio_in_config = TO_REG(config.dataReadyPin, BITP_ISENSOR_DIO_IN_CFG_DR_SEL, BITM_ISENSOR_DIO_IN_CFG_DR_SEL) \
                            | TO_REG(config.dataReadyPolarity, BITP_ISENSOR_DIO_IN_CFG_DR_POL, BITM_ISENSOR_DIO_IN_CFG_DR_POL) \
                            | TO_REG(config.ppsPolarity, BITP_ISENSOR_DIO_IN_CFG_PPS_POL, BITM_ISENSOR_DIO_IN_CFG_PPS_POL) \
                            | TO_REG(config.ppsPin, BITP_ISENSOR_DIO_IN_CFG_PPS_SEL, BITM_ISENSOR_DIO_IN_CFG_PPS_SEL) \
                            | TO_REG(config.ppsFreq, BITP_ISENSOR_DIO_IN_CFG_PPS_FREQ, BITM_ISENSOR_DIO_IN_CFG_PPS_FREQ);
    DEBUG_PRINT("DIO INPUT config = 0x%04X\n", dio_in_config);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_DIO_INPUT_CONFIG, dio_in_config)) < 0) return ret; 

    /* Set Output pins (between Spi buffer <-> Host)  */
    uint16_t dio_out_config = TO_REG(config.passThruPin, BITP_ISENSOR_DIO_OUT_CFG_PIN_PASS, BITM_ISENSOR_DIO_OUT_CFG_PIN_PASS) \
                            | TO_REG(config.watermarkIrqPin, BITP_ISENSOR_DIO_OUT_CFG_WTRMRK, BITM_ISENSOR_DIO_OUT_CFG_WTRMRK) \
                            | TO_REG(config.overflowIrqPin, BITP_ISENSOR_DIO_OUT_CFG_OVRFLW, BITM_ISENSOR_DIO_OUT_CFG_OVRFLW) \
                            | TO_REG(config.errorIrqPin, BITP_ISENSOR_DIO_OUT_CFG_ERROR, BITM_ISENSOR_DIO_OUT_CFG_ERROR);
    DEBUG_PRINT("DIO OUTPUT config = 0x%04X\n", dio_out_config);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_DIO_OUTPUT_CONFIG, dio_out_config)) < 0) return ret; 

    return ret;
}

int imubuf_ConfigUserSpi(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = Err_imu_Success_e;
    /* Set User SPI config */
    val |= TO_REG(IMU_BUF_USR_SPI_CONFIG_KEY, BITP_ISENSOR_USR_SPI_KEY, BITM_ISENSOR_USR_SPI_KEY);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_USER_SPI_CONFIG, val)) < 0) return ret; 
    return ret;
}

int imubuf_ConfigImuSpi(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = Err_imu_Success_e;
    /* Set IMU SPI config */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_IMU_SPI_CONFIG, val)) < 0) return ret; 
    return ret;
}

int imubuf_ConfigCli(adi_imu_Device_t *pDevice, imubuf_CliConfig_t config)
{
    int ret = Err_imu_Success_e;
    /* Set Input pins (between IMU <-> Spi buffer) */
    uint16_t cli_config = TO_REG(config.usbStream, BITP_ISENSOR_CLI_CFG_USB_STREAM, BITM_ISENSOR_CLI_CFG_USB_STREAM) \
                            | TO_REG(config.sdStream, BITP_ISENSOR_CLI_CFG_SD_STREAM, BITM_ISENSOR_CLI_CFG_SD_STREAM) \
                            | TO_REG(config.usbEchoDisable, BITP_ISENSOR_CLI_CFG_USB_ECHO_DISABLE, BITM_ISENSOR_CLI_CFG_USB_ECHO_DISABLE) \
                            | TO_REG(config.scriptAutorun, BITP_ISENSOR_CLI_CFG_SCRIPT_AUTORUN, BITM_ISENSOR_CLI_CFG_SCRIPT_AUTORUN) \
                            | TO_REG(config.delimiterAscii, BITP_ISENSOR_CLI_CFG_DELIM, BITM_ISENSOR_CLI_CFG_DELIM);
    DEBUG_PRINT("CLI config = 0x%04X\n", cli_config);
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_DIO_INPUT_CONFIG, cli_config)) < 0) return ret; 
    return ret;
}

int imubuf_SetUTC(adi_imu_Device_t *pDevice, uint32_t utcTime)
{
    int ret = Err_imu_Success_e;
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_UTC_TIMESTAMP_LWR, utcTime & 0xFFFF)) < 0) return ret; 
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_UTC_TIMESTAMP_UPR, (utcTime >> 16) & 0xFFFF)) < 0) return ret; 
    return ret;
}

int imubuf_GetUTC(adi_imu_Device_t *pDevice, uint32_t* pTime)
{
    int ret = Err_imu_Success_e;
    uint16_t utc_lwr, utc_upr;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_UTC_TIMESTAMP_LWR, &utc_lwr)) < 0) return ret; 
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_UTC_TIMESTAMP_UPR, &utc_upr)) < 0) return ret; 
    *pTime = (((uint32_t)(utc_upr) << 16) | utc_lwr);
    return ret;
}

int imubuf_GetUpTime(adi_imu_Device_t *pDevice, uint32_t* pTime)
{
    int ret = Err_imu_Success_e;
    uint16_t uptime_lwr, uptime_upr;    
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_TIMESTAMP_LWR, &uptime_lwr)) < 0) return ret; 
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_TIMESTAMP_UPR, &uptime_upr)) < 0) return ret; 
    *pTime = (((uint32_t)(uptime_upr) << 16) | uptime_lwr);
    return ret;
}

int imubuf_GetInfo(adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo)
{
    int ret = Err_imu_Success_e;
    /* read system status */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_STATUS, &(pInfo->sysStatus))) < 0) return ret; 
    /* read firmware day/month */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_FW_DAY_MONTH, &(pInfo->fwDayMonth))) < 0) return ret; 
    /* read firmware year */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_FW_YEAR, &(pInfo->fwYear))) < 0) return ret; 
    /* read firmware revision */
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_FW_REV, &(pInfo->fwRev))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CONFIG, &(pInfo->bufConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BTN_CONFIG, &(pInfo->buttonConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_LEN, &(pInfo->bufLen))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_MAX_CNT, &(pInfo->bufMaxCnt))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT, &(pInfo->bufCnt))) < 0) return ret;

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_DIO_INPUT_CONFIG, &(pInfo->dioInputConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_DIO_OUTPUT_CONFIG, &(pInfo->dioOutputConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_WATERMARK_INT_CONFIG, &(pInfo->wtrmrkIntConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_ERROR_INT_CONFIG, &(pInfo->errorIntConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_IMU_SPI_CONFIG, &(pInfo->imuSpiConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_USER_SPI_CONFIG, &(pInfo->userSpiConfig))) < 0) return ret; 

    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_CLI_CONFIG, &(pInfo->cliConfig))) < 0) return ret; 

    return ret;
}

int imubuf_PrintInfo(adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo)
{
    DEBUG_PRINT("\n================================\n");
    DEBUG_PRINT("IMU BUF System Status: 0x%x\n", pInfo->sysStatus);
    DEBUG_PRINT("IMU BUF FW rev: %02x.%02x (Debug=%01x)\n", IMU_BUF_FIRM_REV_MAJOR(pInfo->fwRev), IMU_BUF_FIRM_REV_MINOR(pInfo->fwRev), IMU_BUF_FIRM_REV_DEBUG(pInfo->fwRev));
    DEBUG_PRINT("IMU BUF FW date (MM-DD-YYYY): %02x-%02x-%x\n", IMU_BUF_FIRM_MONTH(pInfo->fwDayMonth), IMU_BUF_FIRM_DAY(pInfo->fwDayMonth), pInfo->fwYear);
    DEBUG_PRINT("IMU BUF Buf Config: 0x%x\n", pInfo->bufConfig);
    DEBUG_PRINT("IMU BUF Buf Length: %d\n", pInfo->bufLen);
    DEBUG_PRINT("IMU BUF Buf Max Cnt: %d\n", pInfo->bufMaxCnt);
    DEBUG_PRINT("IMU BUF Cur Buf Cnt: %d\n", pInfo->bufCnt);
    DEBUG_PRINT("IMU BUF DIO In Cfg: 0x%x\n", pInfo->dioInputConfig);
    DEBUG_PRINT("IMU BUF DIO Out Cfg: 0x%x\n", pInfo->dioOutputConfig);
    DEBUG_PRINT("IMU BUF Watermrk int Cfg: 0x%x\n", pInfo->wtrmrkIntConfig);
    DEBUG_PRINT("IMU BUF Error int Cfg: 0x%x\n", pInfo->errorIntConfig);
    DEBUG_PRINT("IMU BUF IMU spi Config: 0x%x\n", pInfo->imuSpiConfig);
    DEBUG_PRINT("IMU BUF User spi Config: 0x%x\n", pInfo->userSpiConfig);
    DEBUG_PRINT("IMU BUF BTN Config: 0x%x\n", pInfo->buttonConfig);
    DEBUG_PRINT("IMU BUF CLI Config: 0x%x\n", pInfo->cliConfig);
    DEBUG_PRINT("=================================\n\n");
    return Err_imu_Success_e;
}

int imubuf_SetUserCmd(adi_imu_Device_t *pDevice, uint16_t val)
{
    return adi_imu_Write(pDevice, REG_ISENSOR_USER_COMMAND, val);
}

int imubuf_CheckSysStatus(adi_imu_Device_t *pDevice, imubuf_SysStatus_t* pStatus)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0x00;
    /* read system status */   
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_STATUS, &val)) < 0) return ret; 

    pStatus->bufWaterMark = FROM_REG(val, BITP_ISENSOR_STATUS_BUF_WTRMRK, BITM_ISENSOR_STATUS_BUF_WTRMRK);
    if (pStatus->bufWaterMark)
        DEBUG_PRINT("IMU BUF: INFO: Buffer watermark is set.\n");
    pStatus->bufFull = FROM_REG(val, BITP_ISENSOR_STATUS_BUF_FULL, BITM_ISENSOR_STATUS_BUF_FULL);
    if (pStatus->bufFull)
        DEBUG_PRINT("IMU BUF: INFO: Buffer is FULL.\n");
    pStatus->spiError = FROM_REG(val, BITP_ISENSOR_STATUS_SPI_ERROR, BITM_ISENSOR_STATUS_SPI_ERROR);
    if (pStatus->spiError)
        DEBUG_PRINT("IMU BUF: ERROR: SPI error.\n");
    pStatus->spiOverflow = FROM_REG(val, BITP_ISENSOR_STATUS_SPI_OVRFLW, BITM_ISENSOR_STATUS_SPI_OVRFLW);
    if (pStatus->spiOverflow)
        DEBUG_PRINT("IMU BUF: ERROR: SPI overflow error. (Possible violation of stall time).\n");
    pStatus->overrun = FROM_REG(val, BITP_ISENSOR_STATUS_OVERRUN, BITM_ISENSOR_STATUS_OVERRUN);
    if (pStatus->overrun)
        DEBUG_PRINT("IMU BUF: ERROR: Data capture overrun error. (Previous data capture was still in progess).\n");
    pStatus->dmaError = FROM_REG(val, BITP_ISENSOR_STATUS_DMA_ERROR, BITM_ISENSOR_STATUS_DMA_ERROR);
    if (pStatus->dmaError)
        DEBUG_PRINT("IMU BUF: ERROR: DMA error.\n");
    pStatus->ppsUnlock = FROM_REG(val, BITP_ISENSOR_STATUS_PPS_UNLOCK, BITM_ISENSOR_STATUS_PPS_UNLOCK);
    if (pStatus->ppsUnlock)
        DEBUG_PRINT("IMU BUF: INFO: PPS is UNLOCKED.\n");
    pStatus->tempWarning = FROM_REG(val, BITP_ISENSOR_STATUS_TEMP_WARNING, BITM_ISENSOR_STATUS_TEMP_WARNING);
    if (pStatus->tempWarning)
        DEBUG_PRINT("IMU BUF: WARNING: Temperature outside safe range [-40C to 85C].\n");
    pStatus->scriptError = FROM_REG(val, BITP_ISENSOR_STATUS_SCRIPT_ERROR, BITM_ISENSOR_STATUS_SCRIPT_ERROR);
    if (pStatus->scriptError)
        DEBUG_PRINT("IMU BUF: ERROR: Script launch error.\n");
    pStatus->scriptActive = FROM_REG(val, BITP_ISENSOR_STATUS_SCRIPT_ACTIVE, BITM_ISENSOR_STATUS_SCRIPT_ACTIVE);
    if (pStatus->scriptActive)
        DEBUG_PRINT("IMU BUF: INFO: Script is ACTIVE.\n");
    pStatus->flashError = FROM_REG(val, BITP_ISENSOR_STATUS_FLASH_ERROR, BITM_ISENSOR_STATUS_FLASH_ERROR);
    if (pStatus->flashError)
        DEBUG_PRINT("IMU BUF: ERROR: FLASH verify failed.\n");
    pStatus->flashUpdateError = FROM_REG(val, BITP_ISENSOR_STATUS_FLASH_UPD_ERROR, BITM_ISENSOR_STATUS_FLASH_UPD_ERROR);
    if (pStatus->flashUpdateError)
        DEBUG_PRINT("IMU BUF: ERROR: FLASH update failed.\n");
    pStatus->fault = FROM_REG(val, BITP_ISENSOR_STATUS_FAULT, BITM_ISENSOR_STATUS_FAULT);
    if (pStatus->fault)
        DEBUG_PRINT("IMU BUF: ERROR: Processor fault occured.\n");
    pStatus->watchdog = FROM_REG(val, BITP_ISENSOR_STATUS_WATCHDOG, BITM_ISENSOR_STATUS_WATCHDOG);
    if (pStatus->watchdog)
        DEBUG_PRINT("IMU BUF: INFO: Processor reset due to watchdog.\n");
    return ret;
}

int imubuf_StartCapture(adi_imu_Device_t *pDevice, unsigned clear_buffer, uint16_t* curBufLength)
{
    int ret = Err_imu_Success_e;
    /* goto page 255 (not required since below adi_imu_Write() puts page to 255 anyways) */
    if ((ret = adi_imu_SetPage(pDevice, 0xFF)) < 0) return ret;

    /* clear buffer cnt */
    if (clear_buffer){
        if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_CNT_1, 0x0000)) < 0) return ret;
        DEBUG_PRINT("Start capture: cleared buffer\n");
        delay_MicroSeconds(20000); // 20ms delay
    }
    *curBufLength = 0;
    // if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT_1, curBufLength)) < 0) return ret;
    g_captureStarted = 1;
    // DEBUG_PRINT("Started capture: %d sample(s) remaining in buffer\n", *curBufLength);

    return ret;
}

int imubuf_StopCapture(adi_imu_Device_t *pDevice, uint16_t* curBufLength)
{
    /* leave pg 255 */
    int ret = Err_imu_Success_e;

    /* leave pg 255 to stop capture, lets goto page 253 and read buf cnt to verify it is cleared*/
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT, curBufLength)) < 0) return ret; 
    g_captureStarted = 0;
    DEBUG_PRINT("Stopped capture: %d sample(s) remaining in buffer\n", *curBufLength);
    
    return ret;
}

/**
  * @brief Programs SPI transaction pattern to BUF_WRITE registers
  *
  * @param pDevice Device handle for IMU
  *
  * @param length Length of cmds array
  *
  * @param cmds Array of 16 bit spi commands
  *
  * @return enum adi_imu_Error_e
  *
  * This function programs list of SPI commands that is sent to IMU on receiving data ready interrupt 
 **/
int imubuf_SetPatternRaw(adi_imu_Device_t *pDevice, uint16_t length, uint16_t* cmds)
{
    /* write capture registers */
    int ret = Err_imu_Success_e;

    if ((length*2) > (g_maxBufCnt * g_bufLengthBytes)) return Err_Imubuf_BufLenOverflow_e;

    for (int i=0; i< length; i++)
        if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_WRITE_0 + i*2, cmds[i])) < 0) return ret;

    // extra register to account for extra 16bit transaction for reads (although this is necessary only if last transaction is read)
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_WRITE_0 + length*2, 0x0000)) < 0) return ret;

    /* set buffer length to (num_registers + 1) *2 (since it requires extra transaction for read) */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_LEN, (length+1) * 2)) < 0) return ret; 
    g_bufLengthBytes = (length+1) * 2;
    
    return ret;
}


/**
  * @brief Programs SPI transactions pattern to BUF_WRITE registers by injecting page id transfers automatically
  *
  * @param pDevice Device handle for IMU
  *
  * @param length Length of cmds array
  *
  * @param regs Array of 16 bit register addresses in the format [15:8] page-id, [7:0] reg addr
  *
  * @return enum adi_imu_Error_e
  *
  * This function programs list of SPI commands that is sent to IMU on receiving data ready interrupt 
  * It parses page id from the regs array element and injects transactions to change pages automatically
  * It also keeps track of offsets for each read transaction for reading output from the buffer later
 **/
int imubuf_SetPatternAuto(adi_imu_Device_t *pDevice, uint16_t length, uint16_t* regs)
{
    /* write capture registers */
    int ret = Err_imu_Success_e;

    g_bufBurstReadSeqAutoCnt = 2;
    uint16_t bufLen = 0;
    /* initialize with non-supported page id on IMU to be safe*/
    uint16_t curPageId = 0xFFFF; 
    for (int i=0; i< length; i++)
    {
        /* ensure page id is correctly set */
        uint16_t pageId = (regs[i] & 0xFF00) >> 8; /* shifting right as endianness is reversed in this case */
        if ((i==0) || (pageId != curPageId)) {
            
            if ((bufLen) > (g_maxBufCnt * g_bufLengthBytes)) return Err_Imubuf_BufLenOverflow_e;

            if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_WRITE_0 + bufLen, pageId | 0x8000)) < 0) return ret;
            bufLen += 2;
        }
        curPageId = pageId;

        if ((bufLen) > (g_maxBufCnt * g_bufLengthBytes)) return Err_Imubuf_BufLenOverflow_e;

        uint16_t regAddr = (regs[i] & 0x00FF) << 8; /* shifting left as endianness is reversed in this case */
        if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_WRITE_0 + bufLen, regAddr)) < 0) return ret;

        /* store reg offset of expected register read outputs so that we may know which transactions are reg values, and which are not in BUF DATA_N */
        /* There might be some transactions that are page write(s) or other reg write transactions whose outputs have to be discarded */ 
        if ((regAddr & 0x8000) == 0) /* if read request */
        {
            g_bufBurstReadSeqAuto[g_bufBurstReadSeqAutoCnt++] = (0xFF & REG_ISENSOR_BUF_DATA_0) + (bufLen + 2); // reg read output is received in the next transaction so "bufLen+2"
            g_bufBurstReadSeqAuto[g_bufBurstReadSeqAutoCnt++] = 0x00;
        }
        bufLen += 2;
    }

    // extra dummy read for last reg read response, (for reading IMU registers)
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_WRITE_0 + bufLen, 0x0000)) < 0) return ret;
        bufLen += 2;

    // extra dummy read for last reg read response, (for reading BUF_DATA_N registers)
    g_bufBurstReadSeqAuto[g_bufBurstReadSeqAutoCnt++] = 0x00;
    g_bufBurstReadSeqAuto[g_bufBurstReadSeqAutoCnt++] = 0x00;

    /* set buffer length to (num_registers + 1) *2 (since it requires extra transaction for read) */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_LEN, bufLen)) < 0) return ret; 
    g_bufLengthBytes = bufLen; 
    
    return ret;
}

int imubuf_SetPatternImuBurst(adi_imu_Device_t *pDevice)
{
    /* write capture registers */
    int ret = Err_imu_Success_e;

    g_bufBurstReadSeqAutoCnt = 2;
    uint16_t bufLen = 0;

    if ((bufLen) > (g_maxBufCnt * g_bufLengthBytes)) return Err_Imubuf_BufLenOverflow_e;

    uint16_t regAddr = (REG_BURST_CMD & 0x00FF) << 8; /* shifting left as endianness is reversed in this case */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_WRITE_0 + bufLen, regAddr)) < 0) return ret;
    bufLen += 2;

    /* store reg offset of expected register read outputs so that we may know which transactions are reg values, and which are not in BUF DATA_N */
    /* There might be some transactions that are page write(s) or other reg write transactions whose outputs have to be discarded */ 
    for (int j=0; j<MAX_BRF_LEN_BYTES; j=j+2)
    {
        g_bufBurstReadSeqAuto[g_bufBurstReadSeqAutoCnt++] = (0xFF & REG_ISENSOR_BUF_DATA_0) + j; // REG_BURST_CMD in first 16 bits so "bufLen + 2"
        g_bufBurstReadSeqAuto[g_bufBurstReadSeqAutoCnt++] = 0x00;
    }
    bufLen += MAX_BRF_LEN_BYTES;

    /* set buffer length to (num_registers + 1) *2 (since it requires extra transaction for read) */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BUF_LEN, bufLen)) < 0) return ret; 
    g_bufLengthBytes = bufLen; 

    return ret;
}

/**
  * @brief Reads programmed SPI transactions pattern from BUF_WRITE registers
  *
  * @param pDevice Device handle for IMU
  *
  * @return length Length of output array
  *
  * @return regs Output array containing list of 16 bit raw SPI commands
  *
  * @return enum adi_imu_Error_e
  *
  * This function reads list of SPI commands that is sent to IMU on receiving data ready interrupt 
 **/
int imubuf_GetPattern(adi_imu_Device_t *pDevice, uint16_t* length, uint16_t* regs)
{
    /* write capture registers */
    int ret = Err_imu_Success_e;

    for (int i=0; i<(g_bufLengthBytes/2); i++)
        if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_WRITE_0 + i*2, &regs[i])) < 0) return ret;
    *length = g_bufLengthBytes/2;

    return ret;
}


/**
  * @brief Fetches 'readBufCnt' number of buffer samples (full)
  *
  * @param pDevice Device handle for IMU
  *
  * @param readBufCnt Number of buffered samples actually dequeued and read (cutoff at MAX_BUF_CNT)
  *
  * @return pBuf Output array storing array of buffer samples 
  *
  * @return bufLen Length of output array(pBuf) ( = length of each buffer * num of buffered samples read)
  *
  * @return enum adi_imu_Error_e
  *
  * This function deques buffer element from internal memory to BUF_DATA registers in the same order of 
  * programmed pattern in BUF_WRITE registers. Each buffered sample can contain multiple register values.
  * The sample is then read and stored to output array.
  * If requested count of buffer elements is greater than the max allowed, it is cutoff to max allowed value.
  * This functions assumes pBuf contains readBufCnt * BUFLEN bytes of memory allocated. 
  * Reads 'readBufCnt' number of buffered samples; even if not available (read as zeros)
 **/
int imubuf_ReadBufferN(adi_imu_Device_t *pDevice, int32_t readBufCnt, uint16_t* pBuf, uint16_t* bufLen)
{
    int ret = Err_imu_Success_e;
    
    if (readBufCnt > g_maxBufCnt) readBufCnt = g_maxBufCnt;
    /* read buffer data registers*/
    if (spi_ReadWrite(pDevice, g_bufBurstReadSeq, (uint8_t*)pBuf, 2, g_bufLengthBytes/2, readBufCnt, IMU_TRUE) < 0) return Err_spi_RwFailed_e;
    *bufLen = g_bufLengthBytes/2;
    return ret;
}


/**
  * @brief Same as imubuf_ReadBufferN, but read upto max available buffered samples 
  *
  * @param pDevice Device handle for IMU
  *
  * @param maxReadCnt Number of buffered samples to deque and read (cutoff at MAX_BUF_CNT)
  *
  * @return readBufCnt Number of buffered samples actually dequeued and read (cutoff at min(MAX_BUF_CNT, maxReadCnt))
  *
  * @return pBuf Output array storing array of buffer samples 
  *
  * @return bufLen Length of output array(pBuf) ( = length of each buffer * num of buffered samples read)
  *
  * @return enum adi_imu_Error_e
  *
  * This function deques buffer sample from internal memory to BUF_DATA registers in the same order of 
  * programmed pattern in BUF_WRITE registers. Each buffered sample can contain multiple register values.
  * The sample is then read and stored to output array.
  * If requested count of buffered samples is greater than the max allowed, it is cutoff to max allowed value.
  * This functions assumes pBuf contains maxReadCnt * BUFLEN bytes of memory allocated. 
  * Reads upto max available elements in the buffer.
 **/
int imubuf_ReadBufferMax(adi_imu_Device_t *pDevice, int32_t maxReadCnt, int32_t* readBufCnt, uint16_t* pBuf, uint16_t* bufLen)
{
    int ret = Err_imu_Success_e;
    
    /* read current buf count */
    uint16_t bufCnt = 0;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT_1, &bufCnt)) < 0) return ret;
    *readBufCnt = (int32_t) bufCnt;
    if (*readBufCnt > maxReadCnt) *readBufCnt = maxReadCnt;

    return imubuf_ReadBufferN(pDevice, *readBufCnt, pBuf, bufLen);
}

/**
  * @brief Same as imubuf_ReadBufferN, but reads only outputs from read transactions.
  *
  * @param pDevice Device handle for IMU
  *
  * @param readBufCnt Number of buffer samples to deque and read (cutoff at MAX_BUF_CNT)
  *
  * @return pBuf Output array storing array of buffer samples 
  *
  * @return bufLen Length of output array(pBuf) ( = length of each buffer * num of buffered samples read)
  *
  * @return enum adi_imu_Error_e
  *
  * This function deques buffer element from internal memory to BUF_DATA registers in the same order of 
  * programmed pattern in BUF_WRITE registers. Each buffered sample can contain multiple register values.
  * The sample is then read and stored to output array.
  * If requested count of buffer elements is greater than the max allowed, it is cutoff to max allowed value.
  * This functions assumes pBuf contains readBufCnt * BUFLEN bytes of memory allocated. 
  * Reads 'readBufCnt' number of buffered samples; even if not available (read as zeros)
  * Reads only outputs from read transactions, discards output automatically from write or invalid transactions
 **/
int imubuf_ReadBufferAutoN(adi_imu_Device_t *pDevice, int32_t readBufCnt, uint16_t* pBuf, uint16_t* bufLen)
{
    int ret = Err_imu_Success_e;
    
    if (readBufCnt > g_maxBufCnt) readBufCnt = g_maxBufCnt;

    /* read buffer data registers*/
    if (spi_ReadWrite(pDevice, g_bufBurstReadSeqAuto, (uint8_t*)pBuf, 2, g_bufBurstReadSeqAutoCnt/2, readBufCnt, IMU_TRUE) < 0) return Err_spi_RwFailed_e;
    *bufLen = g_bufBurstReadSeqAutoCnt/2;
    return ret;
}


/**
  * @brief Same as imubuf_ReadBufferN, but read upto max available buffered samples 
  *
  * @param pDevice Device handle for IMU
  *
  * @param maxReadCnt Number of buffer samples to deque and read (cutoff at MAX_BUF_CNT)
  *
  * @return readBufCnt Number of buffered samples actually dequeued and read (cutoff at MAX_BUF_CNT)
  *
  * @return pBuf Output array storing array of buffer samples 
  *
  * @return bufLen Length of output array(pBuf) ( = length of each buffer * num of buffered samples read)
  *
  * @return enum adi_imu_Error_e
  *
  * This function deques buffer element from internal memory to BUF_DATA registers in the same order of 
  * programmed pattern in BUF_WRITE registers. Each buffered sample can contain multiple register values.
  * The sample is then read and stored to output array.
  * If requested count of buffer elements is greater than the max allowed, it is cutoff to max allowed value.
  * This functions assumes pBuf contains maxReadCnt * BUFLEN bytes of memory allocated. 
  * Reads upto max available elements in the buffer.
  * Reads only outputs from read transactions, discards output automatically from write or invalid transactions
 **/
int imubuf_ReadBufferAutoMax(adi_imu_Device_t *pDevice, int32_t maxReadCnt, int32_t* readBufCnt, uint16_t* pBuf, uint16_t* bufLen)
{
    int ret = Err_imu_Success_e;
    
    /* read current buf count */
    uint16_t bufCnt = 0;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT_1, &bufCnt)) < 0) return ret;
    *readBufCnt = (int32_t) bufCnt;
    if (*readBufCnt > maxReadCnt) *readBufCnt = maxReadCnt;
    // printf("cur buf cnt: %d\n", bufCnt);
    
    return imubuf_ReadBufferAutoN(pDevice, *readBufCnt, pBuf, bufLen);
}


/**
  * @brief Fetches 'readBufCnt' number of buffer samples (full) in BURST mode
  *
  * @param pDevice Device handle for IMU
  *
  * @param readBufCnt Number of buffered samples actually dequeued and read (cutoff at MAX_BUF_CNT)
  *
  * @return pBuf Output array storing array of buffer samples 
  *
  * @return bufLen Length of output array(pBuf) ( = length of each buffer * num of buffered samples read)
  *
  * @return enum adi_imu_Error_e
  *
  * This function deques buffer element from internal memory to BUF_DATA registers in the same order of 
  * programmed pattern in BUF_WRITE registers. Each buffered sample can contain multiple register values.
  * The sample is then read in BURST mode and stored to output array.
  * This functions assumes pBuf contains readBufCnt * (BUFLEN + IMU_BUF_BURST_HEADER_LEN_BYTES) bytes of memory allocated. 
  * Reads 'readBufCnt' number of buffered samples; even if not available (read as zeros)
 **/
int imubuf_ReadBurstN(adi_imu_Device_t *pDevice, int32_t readBufCnt, uint16_t* pBuf, uint16_t* bufLen)
{
    int ret = Err_imu_Success_e;
    
    /* read current buf count */
    // uint16_t bufCnt = 0;
    // if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT_1, &bufCnt)) < 0) return ret;
    // *readBufCnt = (int32_t) bufCnt;
    // if (*readBufCnt > maxReadCnt) *readBufCnt = maxReadCnt;

    /* read buffer data registers*/
    if (spi_ReadWrite(pDevice, g_BurstTxBuf, (uint8_t*)pBuf, g_bufLengthBytes + IMU_BUF_BURST_HEADER_LEN_BYTES, 1, readBufCnt, IMU_TRUE) < 0) return Err_spi_RwFailed_e;
    *bufLen = (g_bufLengthBytes + IMU_BUF_BURST_HEADER_LEN_BYTES)/2;
    return ret;
}

int imubuf_ScaleBurstOut(adi_imu_Device_t *pDevice, imubuf_BurstOutputRaw_t *pRawData, imubuf_BurstOutput_t *pData)
{
    if(pData != NULL)
    {
        pData->bufCount = pRawData->bufCount;
        pData->bufUtcTime = IMU_BSWAP_16(pRawData->bufUtcTimeLwr) | (IMU_BSWAP_16(pRawData->bufUtcTimeUpr) << 16);
        pData->bufTimestamp = IMU_BSWAP_16(pRawData->bufTimestampLwr) | (IMU_BSWAP_16(pRawData->bufTimestampUpr) << 16);
        pData->bufSig = pRawData->bufSig;
        pData->data = pRawData->data;
        return Err_imu_Success_e;
    }
    else return Err_Imubuf_ScalingOutputFailed_e;
}

int imubuf_GetBufCount(adi_imu_Device_t *pDevice, uint16_t* countBytes)
{
    int ret = Err_imu_Success_e;

    if (g_captureStarted){
        if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT_1, countBytes)) < 0) return ret; 
    }
    else{
        if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_BUF_CNT, countBytes)) < 0) return ret; 
    }
    
    return ret;
}

int imubuf_GetBufLength(adi_imu_Device_t *pDevice, uint16_t* lengthBytes)
{
    *lengthBytes = g_bufLengthBytes;
    return Err_imu_Success_e;
}

int imubuf_GetTemperature(adi_imu_Device_t *pDevice, float* temp)
{
    int ret = Err_imu_Success_e;
    uint16_t val=0;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_TEMP_OUT, &val)) < 0) return ret; 
    *temp = val * IMU_BUF_TEMP_SCALE;
    return ret;
}

int imubuf_GetVDD(adi_imu_Device_t *pDevice, float* vdd)
{
    int ret = Err_imu_Success_e;
    uint16_t val=0;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_VDD_OUT, &val)) < 0) return ret; 
    *vdd = val * IMU_BUF_VDD_SCALE;
    return ret;
}

int imubuf_GetScriptLine(adi_imu_Device_t *pDevice, unsigned* line)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_SCRIPT_LINE, &val)) < 0) return ret;
    *line = val;
    return ret;
}

int imubuf_GetScriptError(adi_imu_Device_t *pDevice, imubuf_ScriptError_t* error)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0;
    if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_SCRIPT_ERROR, &val)) < 0) return ret;
    error->noSDCard = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_NO_SD, BITM_ISENSOR_SCRIPT_ERROR_NO_SD);
    error->mountError = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_MOUNT_ERROR, BITM_ISENSOR_SCRIPT_ERROR_MOUNT_ERROR);
    error->scriptOpenError = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_SCRIPT_OPEN_ERROR, BITM_ISENSOR_SCRIPT_ERROR_SCRIPT_OPEN_ERROR);
    error->resultOpenError = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_RESULT_OPEN_ERROR, BITM_ISENSOR_SCRIPT_ERROR_RESULT_OPEN_ERROR);
    error->parseInvalidCmd = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_PARSE_INVALID_CMD, BITM_ISENSOR_SCRIPT_ERROR_PARSE_INVALID_CMD);
    error->parseInvalidArgs = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_PARSE_INVALID_ARGS, BITM_ISENSOR_SCRIPT_ERROR_PARSE_INVALID_ARGS);
    error->parseInvalidLoop = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_PARSE_INVALID_LOOP, BITM_ISENSOR_SCRIPT_ERROR_PARSE_INVALID_LOOP);
    error->writeFail = FROM_REG(val, BITP_ISENSOR_SCRIPT_ERROR_WRITE_FAIL, BITM_ISENSOR_SCRIPT_ERROR_WRITE_FAIL);

    return ret;
}

int imubuf_SoftwareReset(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_SOFT_RST)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(300000);
    return ret;
}

int imubuf_FactoryReset(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_FACTORY_RST)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(500000);
    return ret;
}

int imubuf_FlashUpdate(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_FLASH_UPDATE)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(900000);
    return ret;
}

int imubuf_ClearFault(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_CLR_FLT)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(100000);
    return ret;
}

int imubuf_EnablePPSSync(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_PPS_EN)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(10);
    return ret;
}

int imubuf_DisablePPSSync(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_PPS_DIS)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(10);
    return ret;
}

int imubuf_WaitForPPSLock(adi_imu_Device_t *pDevice, uint32_t min_lock_duration_ms, uint32_t timeout_ms)
{
    int ret = Err_imu_Success_e;

    uint32_t _timeout_ms = timeout_ms;
    uint32_t _min_lock_duration_ms = min_lock_duration_ms;

    if(_timeout_ms > IMU_BUF_MAX_PPS_LOCK_TIMEOUT_MS)
    {
        DEBUG_PRINT("Warning: PPS lock timeout = %d ms is too high. Capping timeout at %d ms)\n", _timeout_ms, IMU_BUF_MAX_PPS_LOCK_TIMEOUT_MS);
        _timeout_ms = IMU_BUF_MAX_PPS_LOCK_TIMEOUT_MS;
    }

    if(_min_lock_duration_ms > IMU_BUF_MIN_PPS_LOCK_DURATION_MS)
    {
        DEBUG_PRINT("Warning: Min PPS lock duration  = %d ms is too high. Capping timeout at %d ms)\n", _min_lock_duration_ms, IMU_BUF_MIN_PPS_LOCK_DURATION_MS);
        _min_lock_duration_ms = IMU_BUF_MIN_PPS_LOCK_DURATION_MS;
    }

    uint16_t unlockStatus = 0x01;
    while(_timeout_ms--)
    {
        /* PPS unlock Status */
        if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_STATUS, &unlockStatus)) < 0) return ret; 
        unlockStatus = FROM_REG(unlockStatus, BITP_ISENSOR_STATUS_PPS_UNLOCK, BITM_ISENSOR_STATUS_PPS_UNLOCK);
        if (unlockStatus == 0) 
        {
            DEBUG_PRINT("PPS locked (in %d ms)!\n", timeout_ms - _timeout_ms);
            break;
        }
        delay_MicroSeconds(1000);
    }

    if (_timeout_ms == 0) ret = Err_Imubuf_BufPPSLockTimedout_e;
    else
    {
        /* Poll PPS lock Status */
        while(_min_lock_duration_ms--)
        {
            if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_STATUS, &unlockStatus)) < 0) return ret; 
            unlockStatus = FROM_REG(unlockStatus, BITP_ISENSOR_STATUS_PPS_UNLOCK, BITM_ISENSOR_STATUS_PPS_UNLOCK);
            if (unlockStatus == 1)
            {
                DEBUG_PRINT("PPS unlocked (in %d ms)!\n", min_lock_duration_ms - _min_lock_duration_ms);
                break;
            }
            delay_MicroSeconds(1000);
        }
    }

    if (_min_lock_duration_ms == 0) ret = Err_Imubuf_BufPPSLockUnstable_e;

    return ret;
}

// int imubuf_SetBurstMode(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
// {
//     int ret = Err_imu_Success_e;
//     /* Set User SPI config */
//     uint16_t config = 0;
//     if ((ret = adi_imu_Read(pDevice, REG_ISENSOR_USER_SPI_CONFIG, &config)) < 0) return ret; 

//     config |= (1 << 15); // bit 15 
//     if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_USER_SPI_CONFIG, config)) < 0) return ret; 

//     return ret;
// }

int imubuf_SetBtnConfig(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = Err_imu_Success_e;
    /* Set BTN config */
    if ((ret = adi_imu_Write(pDevice, REG_ISENSOR_BTN_CONFIG, val)) < 0) return ret; 
    return ret;
}

int imubuf_PerformWatermarkSet(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_WATERMARK_SET)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(10);
    return ret;
}

int imubuf_PerformSyncGen(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_SYNC_GEN)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(10);
    return ret;
}

int imubuf_PerformDFUReboot(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_DFU_REBOOT)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(500000);
    return ret;
}
