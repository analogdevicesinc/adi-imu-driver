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

*/

static uint16_t g_maxBufCnt = 0;
static uint16_t g_bufLengthBytes = 0;
static uint8_t g_captureStarted = 0;
static uint8_t g_streamStarted = 0;

static uint16_t g_bufOutRegs[MAX_BUF_LEN_BYTES/2] = { REG_ISENSOR_BUF_DATA_0, REG_ISENSOR_BUF_DATA_1, REG_ISENSOR_BUF_DATA_2, REG_ISENSOR_BUF_DATA_3, \
                                                    REG_ISENSOR_BUF_DATA_4, REG_ISENSOR_BUF_DATA_5, REG_ISENSOR_BUF_DATA_6, REG_ISENSOR_BUF_DATA_7, \
                                                    REG_ISENSOR_BUF_DATA_8, REG_ISENSOR_BUF_DATA_9, REG_ISENSOR_BUF_DATA_10, REG_ISENSOR_BUF_DATA_11, \
                                                    REG_ISENSOR_BUF_DATA_12, REG_ISENSOR_BUF_DATA_13, REG_ISENSOR_BUF_DATA_14, REG_ISENSOR_BUF_DATA_15, \
                                                    REG_ISENSOR_BUF_DATA_16, REG_ISENSOR_BUF_DATA_17, REG_ISENSOR_BUF_DATA_18, REG_ISENSOR_BUF_DATA_19, \
                                                    REG_ISENSOR_BUF_DATA_20, REG_ISENSOR_BUF_DATA_21, REG_ISENSOR_BUF_DATA_22, REG_ISENSOR_BUF_DATA_23, \
                                                    REG_ISENSOR_BUF_DATA_24, REG_ISENSOR_BUF_DATA_25, REG_ISENSOR_BUF_DATA_26, REG_ISENSOR_BUF_DATA_27, \
                                                    REG_ISENSOR_BUF_DATA_28, REG_ISENSOR_BUF_DATA_29, REG_ISENSOR_BUF_DATA_30, REG_ISENSOR_BUF_DATA_31};

// extra 2 for last dummy 16bit overhead for read
static const uint8_t g_BurstTxBuf[IMU_BUF_BURST_HEADER_LEN_BYTES + MAX_BUF_LEN_BYTES + 2] = {0xFF & REG_ISENSOR_BUF_RETRIEVE, 0x00};

int _imubuf_UartInit(adi_imu_Device_t* pDevice);
int _imubuf_FlushAll(adi_imu_Device_t* pDevice);

int imubuf_init (adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;

    /* check SPI clock frequency */
    if (pDevice->devType == IMU_HW_SPI && pDevice->spiDev.speed > IMU_BUF_MAX_SPI_CLK) 
    {
        DEBUG_PRINT("Warning: SPI clock out of range (%d Hz) (Setting to max speed = %d Hz)\n", pDevice->spiDev.speed, IMU_BUF_MAX_SPI_CLK);
        pDevice->spiDev.speed = IMU_BUF_MAX_SPI_CLK;
        if ((ret = hw_Init(pDevice)) < 0) return ret;
    }

    /* check stall time */
    if (pDevice->devType == IMU_HW_SPI && pDevice->spiDev.delay < IMU_BUF_MIN_STALL_US) 
    {
        DEBUG_PRINT("Warning: SPI STALL time is low (%d us) (Setting to min stall time = %d us)\n", pDevice->spiDev.delay, IMU_BUF_MIN_STALL_US);
        pDevice->spiDev.delay = IMU_BUF_MIN_STALL_US;
    }
    else if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status == IMUBUF_UART_CONFIGURED && pDevice->uartDev.status != IMUBUF_UART_READY)
    {
        if ((ret = _imubuf_UartInit(pDevice)) < 0) return ret;
        pDevice->uartDev.status = IMUBUF_UART_READY;
    }
    /* read current page ID */
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

    /* software reset */
    // if ((ret = imubuf_SoftwareReset(pDevice)) < 0) return ret;
    
    imubuf_DevInfo_t pInfo = {0};
    /* read firmware revision */
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_FW_REV, &(pInfo.fwRev))) < 0) return ret; 
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
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_MAX_CNT, &g_maxBufCnt)) < 0) return ret; 
    DEBUG_PRINT("IMU BUF MAX Count: %d buffers\n", g_maxBufCnt);

    /* read buffer length currently set */
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_LEN, &g_bufLengthBytes)) < 0) return ret; 
    DEBUG_PRINT("IMU BUF length: %d bytes\n", g_bufLengthBytes);
    return ret;
}

int imubuf_Detect(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0x00;

    // initializing here to switch if communicating uart device
    if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status == IMUBUF_UART_CONFIGURED && pDevice->uartDev.status != IMUBUF_UART_READY)
    {
        if ((ret = _imubuf_UartInit(pDevice)) < 0) return ret;
        pDevice->uartDev.status = IMUBUF_UART_READY;
    }

    if ((ret = hw_SetPage(pDevice, 0xFD, 1)) < 0) return ret;
    if ((ret = hw_ReadReg(pDevice, 0xFD00, &val)) < 0) return ret;

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
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_BUF_CONFIG, buf_config)) < 0) return ret; 

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
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_DIO_INPUT_CONFIG, dio_in_config)) < 0) return ret; 

    /* Set Output pins (between Spi buffer <-> Host)  */
    uint16_t dio_out_config = TO_REG(config.passThruPin, BITP_ISENSOR_DIO_OUT_CFG_PIN_PASS, BITM_ISENSOR_DIO_OUT_CFG_PIN_PASS) \
                            | TO_REG(config.watermarkIrqPin, BITP_ISENSOR_DIO_OUT_CFG_WTRMRK, BITM_ISENSOR_DIO_OUT_CFG_WTRMRK) \
                            | TO_REG(config.overflowIrqPin, BITP_ISENSOR_DIO_OUT_CFG_OVRFLW, BITM_ISENSOR_DIO_OUT_CFG_OVRFLW) \
                            | TO_REG(config.errorIrqPin, BITP_ISENSOR_DIO_OUT_CFG_ERROR, BITM_ISENSOR_DIO_OUT_CFG_ERROR);
    DEBUG_PRINT("DIO OUTPUT config = 0x%04X\n", dio_out_config);
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_DIO_OUTPUT_CONFIG, dio_out_config)) < 0) return ret; 

    return ret;
}

int imubuf_ConfigUserSpi(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = Err_imu_Success_e;
    /* Set User SPI config */
    val |= TO_REG(IMU_BUF_USR_SPI_CONFIG_KEY, BITP_ISENSOR_USR_SPI_KEY, BITM_ISENSOR_USR_SPI_KEY);
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_USER_SPI_CONFIG, val)) < 0) return ret; 
    return ret;
}

int imubuf_ConfigImuSpi(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = Err_imu_Success_e;
    /* Set IMU SPI config */
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_IMU_SPI_CONFIG, val)) < 0) return ret; 
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
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_DIO_INPUT_CONFIG, cli_config)) < 0) return ret; 
    return ret;
}

int imubuf_SetUTC(adi_imu_Device_t *pDevice, uint32_t utcTime)
{
    int ret = Err_imu_Success_e;
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_UTC_TIMESTAMP_LWR, utcTime & 0xFFFF)) < 0) return ret; 
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_UTC_TIMESTAMP_UPR, (utcTime >> 16) & 0xFFFF)) < 0) return ret; 
    return ret;
}

int imubuf_GetUTC(adi_imu_Device_t *pDevice, uint32_t* pTime)
{
    int ret = Err_imu_Success_e;
    uint16_t utc_lwr, utc_upr;
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_UTC_TIMESTAMP_LWR, &utc_lwr)) < 0) return ret; 
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_UTC_TIMESTAMP_UPR, &utc_upr)) < 0) return ret; 
    *pTime = (((uint32_t)(utc_upr) << 16) | utc_lwr);
    return ret;
}

int imubuf_GetUpTime(adi_imu_Device_t *pDevice, uint32_t* pTime)
{
    int ret = Err_imu_Success_e;
    uint16_t uptime_lwr, uptime_upr;    
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_TIMESTAMP_LWR, &uptime_lwr)) < 0) return ret; 
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_TIMESTAMP_UPR, &uptime_upr)) < 0) return ret; 
    *pTime = (((uint32_t)(uptime_upr) << 16) | uptime_lwr);
    return ret;
}

int imubuf_GetInfo(adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo)
{
    int ret = Err_imu_Success_e;
    /* read system status */
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_STATUS, &(pInfo->sysStatus))) < 0) return ret; 
    /* read firmware day/month */
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_FW_DAY_MONTH, &(pInfo->fwDayMonth))) < 0) return ret; 
    /* read firmware year */
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_FW_YEAR, &(pInfo->fwYear))) < 0) return ret; 
    /* read firmware revision */
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_FW_REV, &(pInfo->fwRev))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_CONFIG, &(pInfo->bufConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BTN_CONFIG, &(pInfo->buttonConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_LEN, &(pInfo->bufLen))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_MAX_CNT, &(pInfo->bufMaxCnt))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_CNT, &(pInfo->bufCnt))) < 0) return ret;

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_DIO_INPUT_CONFIG, &(pInfo->dioInputConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_DIO_OUTPUT_CONFIG, &(pInfo->dioOutputConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_WATERMARK_INT_CONFIG, &(pInfo->wtrmrkIntConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_ERROR_INT_CONFIG, &(pInfo->errorIntConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_IMU_SPI_CONFIG, &(pInfo->imuSpiConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_USER_SPI_CONFIG, &(pInfo->userSpiConfig))) < 0) return ret; 

    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_CLI_CONFIG, &(pInfo->cliConfig))) < 0) return ret; 

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
    return hw_WriteReg(pDevice, REG_ISENSOR_USER_COMMAND, val);
}

int imubuf_CheckSysStatus(adi_imu_Device_t *pDevice, imubuf_SysStatus_t* pStatus)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0x00;
    /* read system status */   
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_STATUS, &val)) < 0) return ret; 

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

    // flush all buffers
    if ((ret = _imubuf_FlushAll(pDevice)) < 0) return ret; 
    
    /* set IMU to page 0 before starting capture */
    if ((ret = hw_WriteReg(pDevice, 0x0000, 0x0000)) < 0) return ret;

    if (pDevice->devType == IMU_HW_UART && pDevice->uartDev.status == IMUBUF_UART_READY)
    {
        // run watermark autoset
        if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_WATERMARK_SET)) < 0) return ret;
        delay_MicroSeconds(10000);

        DEBUG_PRINT("Starting uart stream..\n");
        unsigned char txbuf[20] = "stream 1\r\n";
        if ((ret = hw_ReadWriteRaw(pDevice, txbuf, strlen((char*)txbuf), NULL, 0)) < 0) return ret;
        delay_MicroSeconds(10000);
    }

    /* goto page 255 (not required since below hw_WriteReg() puts page to 255 anyways) */
    if ((ret = hw_SetPage(pDevice, 0xFF, 1)) < 0) return ret;

    /* clear buffer cnt */
    if (clear_buffer){
        if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_BUF_CNT_1, 0x0000)) < 0) return ret;
        DEBUG_PRINT("Start capture: cleared buffer\n");
        delay_MicroSeconds(20000); // 20ms delay
    }
    *curBufLength = 0;
    // if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_CNT_1, curBufLength)) < 0) return ret;
    g_captureStarted = 1;
    DEBUG_PRINT("Started capture: %d sample(s) remaining in buffer\n", *curBufLength);

    return ret;
}

int imubuf_StopCapture(adi_imu_Device_t *pDevice, uint16_t* curBufLength)
{
    /* leave pg 255 */
    int ret = Err_imu_Success_e;

    if (pDevice->devType == IMU_HW_UART && pDevice->uartDev.status == IMUBUF_UART_READY)
    {
        DEBUG_PRINT("Stopping uart stream..\n");
        unsigned char txbuf[20] = "stream 0\r\n";
        if ((ret = hw_ReadWriteRaw(pDevice, txbuf, strlen((char*)txbuf), NULL, 0)) < 0) return ret;
        delay_MicroSeconds(10000);
    }

    // flush all buffers
    if ((ret = _imubuf_FlushAll(pDevice)) < 0) return ret; 
    
    /* leave pg 255 to stop capture, lets goto page 253 and read buf cnt to verify it is cleared*/
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_CNT, curBufLength)) < 0) return ret; 
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
    int ret = Err_imu_Success_e;

    uint32_t bufCapacityBytes = g_maxBufCnt * g_bufLengthBytes;
    uint16_t patternBytes = length*2;

    if (patternBytes > bufCapacityBytes) return Err_Imubuf_BufLenOverflow_e;

    // program pattern
    if ((ret = hw_WriteRegs(pDevice, g_bufOutRegs, length, cmds, IMU_TRUE)) < 0) return ret;

    // update buffer length register
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_BUF_LEN, (uint16_t)patternBytes)) < 0) return ret; 
    g_bufLengthBytes = patternBytes;
    return ret;
}

int imubuf_SetPatternImuBurst(adi_imu_Device_t *pDevice)
{
    /* write capture registers */
    int ret = Err_imu_Success_e;

    uint32_t bufCapacityBytes = g_maxBufCnt * g_bufLengthBytes;
    uint16_t bufLen = 0;

    if (bufLen > bufCapacityBytes) return Err_Imubuf_BufLenOverflow_e;

    uint16_t regAddr = (REG_BURST_CMD & 0x00FF) << 8; /* shifting left as endianness is reversed in this case */
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_BUF_WRITE_0 + bufLen, regAddr)) < 0) return ret;
    bufLen += MAX_BRF_LEN_BYTES; // IMU burst length

    /* set buffer length to (num_registers + 1) *2 (since it requires extra transaction for read) */
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_BUF_LEN, bufLen)) < 0) return ret; 
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

    if ((ret = hw_ReadRegs(pDevice, g_bufOutRegs, g_bufLengthBytes/2, regs, IMU_TRUE)) < 0) return ret;
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

    // Ask board to prepare the buffer sample
    uint16_t val;
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_RETRIEVE, &val)) < 0) return ret;

    // Get the sample(s)
    for (int i=0; i<readBufCnt; i++)
        if ((ret = hw_ReadRegs(pDevice, g_bufOutRegs, g_bufLengthBytes/2, pBuf+(g_bufLengthBytes/2)*i, IMU_TRUE)) < 0) return ret;

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
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_CNT_1, &bufCnt)) < 0) return ret;
    *readBufCnt = (int32_t) bufCnt;
    if (*readBufCnt > maxReadCnt) *readBufCnt = maxReadCnt;

    return imubuf_ReadBufferN(pDevice, *readBufCnt, pBuf, bufLen);
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
    
    size_t _bufLenBytes = g_bufLengthBytes + IMU_BUF_BURST_HEADER_LEN_BYTES;
    size_t _bufLen = _bufLenBytes/2;

    for (int i=0; i<readBufCnt; i++)
    {
        if (pDevice->spiDev.status == IMUBUF_SPI_READY)
        {
            if ((ret = hw_ReadWriteRaw(pDevice, g_BurstTxBuf, _bufLenBytes, (uint8_t*)pBuf + _bufLenBytes*i, _bufLenBytes)) < 0) 
                DEBUG_PRINT_RET(ret, "Error: %d\n", ret);
        }
        else if (pDevice->uartDev.status == IMUBUF_UART_READY)
        {
            unsigned char rxbuf[300] = {0};
            ret = hw_ReadWriteRaw(pDevice, NULL, 0, rxbuf, 300);
            if (ret < 0 && ret != Err_uart_ReadEmpty_e) break;
            if (ret > 0) {
                pBuf[0] = 0x00; // hacky way to match SPI output (Buf_len (first field) missing in UART burst output)
                if ((ret = uart_rx_parse16(rxbuf, pBuf + _bufLen*i + 1, _bufLen)) > _bufLen) DEBUG_PRINT("Truncation while parsing UART-Rx buffer\n");
                memset(rxbuf, 0, 250);
            }
            
            // unsigned char txbuf[20] = "readbuf\r\n";
            // unsigned char rxbuf[300] = {0};
            // if ((ret = hw_ReadWriteRaw(pDevice, txbuf, strlen((char*)txbuf), rxbuf, 300)) < 0) return ret;
            // if ((ret = uart_rx_parse16(rxbuf, pBuf + _bufLen*i, _bufLen)) > _bufLen) DEBUG_PRINT("Truncation while parsing UART-Rx buffer\n");
        }
    }
    /* read buffer data registers*/
    *bufLen = _bufLen;
    return ret;
}

int imubuf_ScaleBurstOut(adi_imu_Device_t *pDevice, imubuf_BurstOutputRaw_t *pRawData, imubuf_BurstOutput_t *pData)
{
    if(pData != NULL)
    {
        pData->bufCount = pRawData->bufCount;
        if(pDevice->spiDev.status == IMUBUF_SPI_READY)
        {
            pData->bufUtcTime = IMU_BSWAP_16(pRawData->bufUtcTimeLwr) | (IMU_BSWAP_16(pRawData->bufUtcTimeUpr) << 16);
            pData->bufTimestamp = IMU_BSWAP_16(pRawData->bufTimestampLwr) | (IMU_BSWAP_16(pRawData->bufTimestampUpr) << 16);
        }
        else
        {
            pData->bufUtcTime = pRawData->bufUtcTimeLwr | (pRawData->bufUtcTimeUpr << 16);
            pData->bufTimestamp = pRawData->bufTimestampLwr | (pRawData->bufTimestampUpr << 16);
        }
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
        if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_CNT_1, countBytes)) < 0) return ret; 
    }
    else{
        if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_BUF_CNT, countBytes)) < 0) return ret; 
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
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_TEMP_OUT, &val)) < 0) return ret; 
    *temp = val * IMU_BUF_TEMP_SCALE;
    return ret;
}

int imubuf_GetVDD(adi_imu_Device_t *pDevice, float* vdd)
{
    int ret = Err_imu_Success_e;
    uint16_t val=0;
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_VDD_OUT, &val)) < 0) return ret; 
    *vdd = val * IMU_BUF_VDD_SCALE;
    return ret;
}

int imubuf_GetScriptLine(adi_imu_Device_t *pDevice, unsigned* line)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0;
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_SCRIPT_LINE, &val)) < 0) return ret;
    *line = val;
    return ret;
}

int imubuf_GetScriptError(adi_imu_Device_t *pDevice, imubuf_ScriptError_t* error)
{
    int ret = Err_imu_Success_e;
    uint16_t val = 0;
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_SCRIPT_ERROR, &val)) < 0) return ret;
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

    if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status == IMUBUF_UART_READY)
        DEBUG_PRINT_RET(Err_imu_UnsupportedProtocol_e, "Error: Software reset is not supported over UART\n");

    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_SOFT_RST)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(300000);

    /* read current page ID */
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

    return ret;
}

int imubuf_FactoryReset(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;

    if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status == IMUBUF_UART_READY)
    {
        unsigned char txbuf[20] = "freset\r\n";
        if ((ret = hw_ReadWriteRaw(pDevice, txbuf, strlen((char*)txbuf), NULL, 0)) < 0) return ret;
    }
    else
    {
        if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_FACTORY_RST)) < 0) return ret;
    }
    /* wait for some time */
    delay_MicroSeconds(500000);

    if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status == IMUBUF_UART_READY)
    {
        pDevice->uartDev.status = IMUBUF_UART_CONFIGURED;
        if ((ret = _imubuf_UartInit(pDevice)) < 0) return ret;
        pDevice->uartDev.status = IMUBUF_UART_READY;
    }

    /* read current page ID */
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

    return ret;
}

int imubuf_FlashUpdate(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_FLASH_UPDATE)) < 0) return ret;
    /* wait for some time */
    delay_MicroSeconds(900000);

    /* read current page ID */
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

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

    int _timeout_sec = timeout_ms / 1000;
    int _min_lock_duration_sec = min_lock_duration_ms / 1000;

    if(timeout_ms > IMU_BUF_MAX_PPS_LOCK_TIMEOUT_MS)
    {
        DEBUG_PRINT("Warning: PPS lock timeout = %d ms is too high. Capping timeout at %d ms)\n", timeout_ms, IMU_BUF_MAX_PPS_LOCK_TIMEOUT_MS);
        _timeout_sec = IMU_BUF_MAX_PPS_LOCK_TIMEOUT_MS / 1000;
    }

    if(min_lock_duration_ms > IMU_BUF_MIN_PPS_LOCK_DURATION_MS)
    {
        DEBUG_PRINT("Warning: Min PPS lock duration  = %d ms is too high. Capping timeout at %d ms)\n", min_lock_duration_ms, IMU_BUF_MIN_PPS_LOCK_DURATION_MS);
        _min_lock_duration_sec = IMU_BUF_MIN_PPS_LOCK_DURATION_MS / 1000;
    }

    // lets clear all non-sticky bits
    uint16_t temp;
    if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_STATUS, &temp)) < 0) return ret; 
    delay_MicroSeconds(1.1*1e6); // waiting for atleast 100ms to wait buffer board update PPS status

    DEBUG_PRINT("Waiting for PPS lock.. (timeout=%d seconds)\n", _timeout_sec);
    uint16_t unlockStatus = 0x01;
    while(_timeout_sec--)
    {
        /* PPS unlock Status */
        if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_STATUS, &unlockStatus)) < 0) return ret; 
        unlockStatus = FROM_REG(unlockStatus, BITP_ISENSOR_STATUS_PPS_UNLOCK, BITM_ISENSOR_STATUS_PPS_UNLOCK);
        if (unlockStatus == 0) 
        {
            DEBUG_PRINT("PPS locked (in %d ms)!\n", (timeout_ms/1000) - _timeout_sec);
            break;
        }
        delay_MicroSeconds(1e6);
    }

    if (_timeout_sec <= 0) DEBUG_PRINT_RET(Err_Imubuf_BufPPSLockTimedout_e, "Failed to lock to PPS signal.\n");
    else
    {
        /* Poll PPS lock Status */
        DEBUG_PRINT("Waiting for PPS lock to be stable for %d seconds..\n", _min_lock_duration_sec);
        while(_min_lock_duration_sec--)
        {
            if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_STATUS, &unlockStatus)) < 0) return ret; 
            unlockStatus = FROM_REG(unlockStatus, BITP_ISENSOR_STATUS_PPS_UNLOCK, BITM_ISENSOR_STATUS_PPS_UNLOCK);
            if (unlockStatus == 1)
            {
                DEBUG_PRINT("PPS unlocked (in %d ms)!\n", (min_lock_duration_ms/1000) - _min_lock_duration_sec);
                break;
            }
            delay_MicroSeconds(1e6);
        }
    }

    if (_min_lock_duration_sec <= 0) DEBUG_PRINT_RET(Err_Imubuf_BufPPSLockUnstable_e, "PPS seems to be unstable.\n");

    return ret;
}

// int imubuf_SetBurstMode(adi_imu_Device_t *pDevice, adi_imu_EnDis_e val)
// {
//     int ret = Err_imu_Success_e;
//     /* Set User SPI config */
//     uint16_t config = 0;
//     if ((ret = hw_ReadReg(pDevice, REG_ISENSOR_USER_SPI_CONFIG, &config)) < 0) return ret; 

//     config |= (1 << 15); // bit 15 
//     if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_USER_SPI_CONFIG, config)) < 0) return ret; 

//     return ret;
// }

int imubuf_SetBtnConfig(adi_imu_Device_t *pDevice, uint16_t val)
{
    int ret = Err_imu_Success_e;
    /* Set BTN config */
    if ((ret = hw_WriteReg(pDevice, REG_ISENSOR_BTN_CONFIG, val)) < 0) return ret; 
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

    /* read current page ID */
    if ((ret = hw_GetPage(pDevice)) < 0) return ret;

    return ret;
}

int _imubuf_FlushAll(adi_imu_Device_t* pDevice)
{
    int ret = Err_imu_Success_e;
    delay_MicroSeconds(10000);
    // flush all buffers
    DEBUG_PRINT("Flushing serial port and clearing buffer board's all buffers..\n");
    if ((ret = imubuf_SetUserCmd(pDevice, BITM_ISENSOR_USER_COMMAND_CLR_BUF)) < 0) return ret;
    delay_MicroSeconds(100000);
    if ((ret = hw_FlushInput(pDevice)) < 0) return ret;
    delay_MicroSeconds(10000);
    if ((ret = hw_FlushOutput(pDevice)) < 0) return ret;
    delay_MicroSeconds(100000);
    return ret;
}

int _imubuf_UartInit(adi_imu_Device_t* pDevice)
{
    int ret = Err_imu_Success_e;
    DEBUG_PRINT("Initializing buffer board's uart..\n");
    if ((ret = hw_FlushInput(pDevice)) < 0) return ret;
    if ((ret = hw_FlushOutput(pDevice)) < 0) return ret;
    delay_MicroSeconds(10000);

    // dummy command
    hw_SetPage(pDevice, 0xFD, 1);
    delay_MicroSeconds(10000);

    // Set to page 253
    if ((ret = hw_SetPage(pDevice, 0xFD, 1)) < 0) return ret;
    delay_MicroSeconds(10000);
    if ((ret = hw_FlushInput(pDevice)) < 0) return ret;

    // 0x20: space char in ASCII, 0x04: Disable echo
    uint16_t reg = REG_ISENSOR_CLI_CONFIG;
    uint16_t regval = BITM_ISENSOR_CLI_CFG_USB_ECHO_DISABLE | TO_REG(0x20, BITP_ISENSOR_CLI_CFG_DELIM, BITM_ISENSOR_CLI_CFG_DELIM);
    if ((ret = hw_WriteRegs(pDevice, &reg, 1, &regval, IMU_TRUE)) < 0) return ret;
    delay_MicroSeconds(10000);
    
    if ((ret = hw_FlushInput(pDevice)) < 0) return ret;
    delay_MicroSeconds(10000);

    uint16_t tempval=0;
    if (((ret = hw_ReadRegs(pDevice, &reg, 1, &tempval, IMU_TRUE)) < 0)) return ret; 
    if (regval != tempval)
		DEBUG_PRINT_RET(-1, "IMU BUF init failed to disable echo %d\n", tempval);

    if ((ret = _imubuf_FlushAll(pDevice)) < 0) return ret; 
    
    return ret;
}

adi_imu_BuildInfo_t imubuf_GetBuildInfo (adi_imu_Device_t *pDevice)
{
    adi_imu_BuildInfo_t info;
    snprintf(info.version, sizeof(info.version), IMU_LIB_VERSION);
    snprintf(info.build_time, sizeof(info.build_time), IMU_LIB_BUILD_DATETIME);
    snprintf(info.build_type, sizeof(info.build_type), IMU_LIB_BUILD_TYPE);
    return info;
}
