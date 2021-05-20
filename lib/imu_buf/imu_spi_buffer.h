/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		imu_spi_buffer.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for iSensor-Spi-buffer + ADIS16xxx IMU.
 **/


#ifndef __IMU_SPI_BUFFER_H_
#define __IMU_SPI_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_imu_common.h"

#define IMUBUF_PATTERN_WRITE_REG(addr, val)      ( 0x8000 | ((addr << 8) & 0xFF00) | ((val) & 0xFF) )
#define IMUBUF_PATTERN_READ_REG(addr)            ( (addr << 8) & 0xFF00 )

typedef enum {
    IMUBUF_DIO1 = 0x1,
    IMUBUF_DIO2 = 0x2,
    IMUBUF_DIO3 = 0x4,
    IMUBUF_DIO4 = 0x8
} imubuf_DioPinMap_e;

typedef enum {
    IMUBUF_PPS_FREQ_1HZ = 0,
    IMUBUF_PPS_FREQ_10HZ = 1,
    IMUBUF_PPS_FREQ_100HZ = 2,
    IMUBUF_PPS_FREQ_1000HZ = 3,
} imubuf_PPSFreq_e;

typedef struct {
    uint8_t overflowAction;
    uint8_t imuBurstEn;
    uint8_t bufBurstEn;
} imubuf_BufConfig_t;

typedef struct {
    uint8_t usbStream;
    uint8_t sdStream;
    uint8_t usbEchoDisable;
    uint8_t scriptAutorun;
    uint8_t delimiterAscii;
} imubuf_CliConfig_t;

typedef struct {
    uint16_t sysStatus;
    uint16_t fwDayMonth;
    uint16_t fwYear;
    uint16_t fwRev;
    uint16_t dioInputConfig;
    uint16_t dioOutputConfig;
    uint16_t bufConfig;
    uint16_t buttonConfig;
    uint16_t bufLen;
    uint16_t bufMaxCnt;
    uint16_t bufCnt;
    uint16_t wtrmrkIntConfig;
    uint16_t errorIntConfig;
    uint16_t imuSpiConfig;
    uint16_t userSpiConfig;
    uint16_t cliConfig;
} imubuf_DevInfo_t;

typedef struct {
    /* inputs (between IMU <-> Spi buffer) */
    uint16_t dataReadyPin;
    uint16_t dataReadyPolarity;
    uint16_t ppsPin;
    uint16_t ppsPolarity;
    uint16_t ppsFreq;
    /* outputs (between Spi buffer <-> Host) */
    uint16_t passThruPin;
    uint16_t watermarkIrqPin;
    uint16_t overflowIrqPin;
    uint16_t errorIrqPin;
} imubuf_ImuDioConfig_t;

typedef struct {
    uint8_t bufWaterMark;
    uint8_t bufFull;
    uint8_t spiError;
    uint8_t spiOverflow;
    uint8_t overrun;
    uint8_t dmaError;
    uint8_t ppsUnlock;
    uint8_t tempWarning;
    uint8_t scriptError;
    uint8_t scriptActive;
    uint8_t flashError;
    uint8_t flashUpdateError;
    uint8_t fault;
    uint8_t watchdog;
} imubuf_SysStatus_t;

typedef struct {
    uint8_t noSDCard;
    uint8_t mountError;
    uint8_t scriptOpenError;
    uint8_t resultOpenError;
    uint8_t parseInvalidCmd;
    uint8_t parseInvalidArgs;
    uint8_t parseInvalidLoop;
    uint8_t writeFail;
} imubuf_ScriptError_t;

typedef struct {
    uint16_t bufCount;
    uint16_t bufUtcTimeLwr;
    uint16_t bufUtcTimeUpr;
    uint16_t bufTimestampLwr;
    uint16_t bufTimestampUpr;
    uint16_t bufSig;
    uint8_t data[MAX_BUF_LEN_BYTES];
} __attribute__ ((packed)) imubuf_BurstOutputRaw_t;

typedef struct {
    uint32_t bufCount;
    uint32_t bufUtcTime;
    uint32_t bufTimestamp;
    uint32_t bufSig;
    uint8_t* data;
} imubuf_BurstOutput_t;

int imubuf_init                 (adi_imu_Device_t *pDevice);

int imubuf_Detect               (adi_imu_Device_t *pDevice);

int imubuf_SetBufConfig         (adi_imu_Device_t *pDevice, imubuf_BufConfig_t* config);

int imubuf_GetBufConfig         (adi_imu_Device_t *pDevice, imubuf_BufConfig_t* config);

int imubuf_SetDioConfig         (adi_imu_Device_t *pDevice, imubuf_ImuDioConfig_t* config);

int imubuf_GetDioConfig         (adi_imu_Device_t *pDevice, imubuf_ImuDioConfig_t* config);

int imubuf_SetUserSpiConfig     (adi_imu_Device_t *pDevice, uint16_t val);

int imubuf_GetUserSpiConfig     (adi_imu_Device_t *pDevice, uint16_t* val);

int imubuf_SetImuSpiConfig      (adi_imu_Device_t *pDevice, uint16_t val);

int imubuf_GetImuSpiConfig      (adi_imu_Device_t *pDevice, uint16_t* val);

int imubuf_SetCliConfig         (adi_imu_Device_t *pDevice, imubuf_CliConfig_t* config);

int imubuf_GetCliConfig         (adi_imu_Device_t *pDevice, imubuf_CliConfig_t* config);

int imubuf_SetUTC               (adi_imu_Device_t *pDevice, uint32_t utcTime);

int imubuf_GetUTC               (adi_imu_Device_t *pDevice, uint32_t* pTime);

int imubuf_GetUpTime            (adi_imu_Device_t *pDevice, uint32_t* pTime);

int imubuf_GetInfo              (adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo);

int imubuf_PrintInfo            (adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo);

int imubuf_SetUserCmd           (adi_imu_Device_t *pDevice, uint16_t val);

int imubuf_GetSysStatus         (adi_imu_Device_t *pDevice, imubuf_SysStatus_t* pStatus);

int imubuf_StartCapture         (adi_imu_Device_t *pDevice, unsigned clear_buffer, uint16_t* curBufLength);

int imubuf_StopCapture          (adi_imu_Device_t *pDevice, uint16_t* curBufLength);

int imubuf_SetPatternRaw        (adi_imu_Device_t *pDevice, uint16_t length, uint16_t* regs);

int imubuf_SetPatternImuBurst   (adi_imu_Device_t *pDevice);

int imubuf_GetPattern           (adi_imu_Device_t *pDevice, uint16_t* length, uint16_t* regs);

int imubuf_ReadBufferN          (adi_imu_Device_t *pDevice, int32_t readBufCnt, uint16_t* pBuf, uint16_t* bufLen);

int imubuf_ReadBufferMax        (adi_imu_Device_t *pDevice, int32_t maxReadCnt, int32_t* readBufCnt, uint16_t* pBuf, uint16_t* bufLen);

int imubuf_ReadBurstN           (adi_imu_Device_t *pDevice, int32_t readBufCnt, uint16_t* pBuf, uint16_t* bufLen);

int imubuf_ScaleBurstOut        (adi_imu_Device_t *pDevice, imubuf_BurstOutputRaw_t *pRawData, imubuf_BurstOutput_t *pData);

int imubuf_GetBufCount          (adi_imu_Device_t *pDevice, uint16_t* count);

int imubuf_GetBufLength         (adi_imu_Device_t *pDevice, uint16_t* lengthBytes);

int imubuf_GetTemperature       (adi_imu_Device_t *pDevice, float* temp);

int imubuf_GetVDD               (adi_imu_Device_t *pDevice, float* vdd);

int imubuf_GetScriptLine        (adi_imu_Device_t *pDevice, unsigned* line);

int imubuf_GetScriptError       (adi_imu_Device_t *pDevice, imubuf_ScriptError_t* error);

int imubuf_SoftwareReset        (adi_imu_Device_t *pDevice);

int imubuf_FactoryReset         (adi_imu_Device_t *pDevice);

int imubuf_FlashUpdate          (adi_imu_Device_t *pDevice);

int imubuf_ClearFault           (adi_imu_Device_t *pDevice);

int imubuf_EnablePPSSync        (adi_imu_Device_t *pDevice);

int imubuf_DisablePPSSync       (adi_imu_Device_t *pDevice);

int imubuf_SetBtnConfig         (adi_imu_Device_t *pDevice, uint16_t val);

int imubuf_PerformWatermarkSet  (adi_imu_Device_t *pDevice);

int imubuf_PerformSyncGen       (adi_imu_Device_t *pDevice);

int imubuf_PerformDFUReboot     (adi_imu_Device_t *pDevice);

// RESERVED for future
int imubuf_WaitForPPSLock       (adi_imu_Device_t *pDevice, uint32_t min_lock_duration_ms, uint32_t timeout_ms);

adi_imu_BuildInfo_t imubuf_GetBuildInfo (adi_imu_Device_t *pDevice);

#ifdef __cplusplus
}
#endif
#endif