/*******************************************************************************
 *   @file   imu_spi_buffer.h
 *   @brief  Driver interface for iSensor-Spi-buffer + ADIS16xxx IMU
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#ifndef __IMU_SPI_BUFFER_H_
#define __IMU_SPI_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_imu_driver.h"
#include "imu_spi_buffer_regmap.h"

#define TO_REG(val, pos, mask)              (((val) << pos) & mask)
#define FROM_REG(val, pos, mask)            (((val) & mask) >> pos)

/* USER SPI Config */
#define IMUBUF_SPI_CPHA                     (0x01)
#define IMUBUF_SPI_CPOL                     (0x02)
#define IMUBUF_SPI_MSBFIRST                 (0x04)
#define IMUBUF_SPI_LSBFIRST                 (0x00)
#define IMUBUF_SPI_EN_DMA_BURST             (0x8000)

/* IMU SPI Config */
#define IMUBUF_SPI_IMU_STALL_MASK           (0xFF)
#define IMUBUF_SPI_IMU_SCLK_2x              (0x100)
#define IMUBUF_SPI_IMU_SCLK_4x              (0x200)
#define IMUBUF_SPI_IMU_SCLK_8x              (0x400)
#define IMUBUF_SPI_IMU_SCLK_16x             (0x800)
#define IMUBUF_SPI_IMU_SCLK_32x             (0x1000)
#define IMUBUF_SPI_IMU_SCLK_64x             (0x2000)
#define IMUBUF_SPI_IMU_SCLK_128x            (0x4000)
#define IMUBUF_SPI_IMU_SCLK_256x            (0x8000)

/* USER command */
#define IMUBUF_USR_CMD_CLEAR_BUF            (0x01)
#define IMUBUF_USR_CMD_CLEAR_FAULT          (0x02)
#define IMUBUF_USR_CMD_FACTORY_RST          (0x04)
#define IMUBUF_USR_CMD_FLASH_UPDATE         (0x08)
#define IMUBUF_USR_CMD_PPS_ENABLE           (0x10)
#define IMUBUF_USR_CMD_PPS_DISABLE          (0x20)
#define IMUBUF_USR_CMD_SOFT_RST             (0x8000)

//TODO: USB config

typedef enum {

    imubuf_BufClearFailed_e = -501,
    imubuf_BufLenOverflow_e = -500,
} imubuf_Error_e;

typedef enum {
    IMUBUF_DIO1 = 0x1,
    IMUBUF_DIO2 = 0x2,
    IMUBUF_DIO3 = 0x4,
    IMUBUF_DIO4 = 0x8
} imubuf_DioPinMap_e;

typedef enum {
    IMUBUF_TRUE = 0x1,
    IMUBUF_FALSE = 0x0,
} imubuf_Bool_e;

typedef struct {
    uint8_t overflowAction;
    uint8_t spiWordSize;
    uint16_t bufLen;
} imubuf_BufConfig_t;

typedef struct {
    uint16_t sysStatus;
    uint16_t fwDayMonth;
    uint16_t fwYear;
    uint16_t fwRev;
    uint16_t dioInputConfig;
    uint16_t dioOutputConfig;
    uint16_t bufConfig;
    uint16_t bufLen;
    uint16_t bufMaxCnt;
    uint16_t bufCnt;
    uint16_t wtrmrkIntConfig;
    uint16_t errorIntConfig;
    uint16_t imuSpiConfig;
    uint16_t userSpiConfig;
    uint16_t usbSpiConfig;
} imubuf_DevInfo_t;

typedef struct {
    /* inputs (between IMU <-> Spi buffer) */
    uint16_t dataReadyPin;
    uint16_t dataReadyPolarity;
    uint16_t ppsPin;
    uint16_t ppsPolarity;
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
    uint8_t flashError;
    uint8_t flashUpdateError;
    uint8_t fault;
    uint8_t watchdog;
} imubuf_SysStatus_t;


int imubuf_init                 (adi_imu_Device_t *pDevice);

int imubuf_ConfigBuf            (adi_imu_Device_t *pDevice, imubuf_BufConfig_t config);

int imubuf_ConfigDio            (adi_imu_Device_t *pDevice, imubuf_ImuDioConfig_t config);

int imubuf_ConfigUserSpi        (adi_imu_Device_t *pDevice, uint16_t val);

int imubuf_ConfigImuSpi         (adi_imu_Device_t *pDevice, uint16_t val);

int imubuf_GetUTC               (adi_imu_Device_t *pDevice, uint32_t* pTime);

int imubuf_GetUpTime            (adi_imu_Device_t *pDevice, uint32_t* pTime);

int imubuf_GetInfo              (adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo);

int imubuf_PrintInfo            (adi_imu_Device_t *pDevice, imubuf_DevInfo_t* pInfo);

int imubuf_SetUserCmd           (adi_imu_Device_t *pDevice, uint16_t val);

int imubuf_CheckSysStatus       (adi_imu_Device_t *pDevice, imubuf_SysStatus_t* pStatus);

int imubuf_StartCapture         (adi_imu_Device_t *pDevice, unsigned clear_buffer, uint16_t* curBufLength);

int imubuf_StopCapture          (adi_imu_Device_t *pDevice, uint16_t* curBufLength);

int imubuf_SetPatternRaw        (adi_imu_Device_t *pDevice, uint16_t length, uint16_t* regs);

int imubuf_SetPatternAuto       (adi_imu_Device_t *pDevice, uint16_t length, uint16_t* regs);

int imubuf_GetPattern           (adi_imu_Device_t *pDevice, uint16_t* length, uint16_t* regs);

int imubuf_ReadBufferN          (adi_imu_Device_t *pDevice, int32_t readBufCnt, uint16_t* pBuf, uint16_t* bufLen);

int imubuf_ReadBufferMax        (adi_imu_Device_t *pDevice, int32_t maxReadCnt, int32_t* readBufCnt, uint16_t* pBuf, uint16_t* bufLen);

int imubuf_ReadBufferAutoN      (adi_imu_Device_t *pDevice, int32_t readBufCnt, uint16_t* pBuf, uint16_t* bufLen);

int imubuf_ReadBufferAutoMax    (adi_imu_Device_t *pDevice, int32_t maxReadCnt, int32_t* readBufCnt, uint16_t* pBuf, uint16_t* bufLen);

int imubuf_GetBufCount          (adi_imu_Device_t *pDevice, uint16_t* count);

int imubuf_GetBufLength         (adi_imu_Device_t *pDevice, uint16_t* lengthBytes);

int imubuf_SoftwareReset        (adi_imu_Device_t *pDevice);

#ifdef __cplusplus
}
#endif
#endif