/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		adi_imu_driver.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for ADIS16xxx IMU.
 **/

#ifndef __ADI_IMU_DRIVER_H_
#define __ADI_IMU_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* minimum delay during register access after every 16bit transactions */
#define IMU_MIN_STALL_US 10

#define IMU_MAX_SPI_CLK 6000000

#include "adi_imu_regmap.h"

#define IMU_GET_16BITS(buf, idx)        ( ((buf[idx] << 8) & 0xFF00) | (buf[1+idx] & 0xFF) )

#define IMU_GET_32BITS(buf, idx)        ( (uint32_t)((buf[2+idx] << 24) & 0xFF000000) | (uint32_t)((buf[3+idx] << 16) & 0xFF0000) | (uint32_t)((buf[idx] << 8) & 0xFF00) | (uint32_t)(buf[1+idx] & 0xFF) )

#ifndef BAREMETAL
#include <stdio.h>
#define DEBUG_PRINT(format, ...) printf(format , ##__VA_ARGS__)

#define DEBUG_PRINT_RET(ret, msg, ...) do {\
        DEBUG_PRINT(msg , ##__VA_ARGS__);\
        return ret;\
    } while(0)

#else
#define DEBUG_PRINT(format, ...) do{} while(0)
#define DEBUG_PRINT_RET(ret, msg, ...) do{} while(0)
#endif



typedef enum {
    adi_imu_UnsupportedHardware_e = -8,
    adi_imu_SystemError_e = -7,
    adi_imu_SelfTestFailed_e = -6,
    adi_imu_BadDevice_e = -5,
    adi_imu_BurstFrameInvalid_e = -4,
    adi_imu_ProdIdVerifyFailed_e = -3,
    adi_spi_RwFailed_e = -2,
    adi_spi_InitFailed_e = -1,
    adi_imu_Success_e = 0,
} adi_imu_Error_e;

typedef volatile void* adi_imu_DevHandler_t;
typedef struct {
    /* for verification */
    uint16_t prodId;

    /* gravity constant; if 1.0 accelerometer output is normalized to g */
    double g;

    /* spi config */
    const char* spiDev;
    uint32_t spiSpeed;
    uint8_t spiMode;
    uint8_t spiBitsPerWord;
    uint32_t spiDelay;

    /* pin config */
    uint8_t pinRstn;
    uint8_t pinCsn;
    uint8_t pinSclk;
    uint8_t pinDout;
    uint8_t pinDin;
    uint8_t pinDio1;
    uint8_t pinDio2;
    uint8_t pinDio3;
    uint8_t pinDio4;

    /* device handler */
    adi_imu_DevHandler_t spiHandle;

    /* device status */
    uint8_t status; // 0: bad, greater than 1: good
    uint16_t curPage;
    uint16_t rangeModel;

} adi_imu_Device_t;

typedef struct {
    uint16_t prodId;
    uint16_t fwRev;
    uint16_t fwDayMonth;
    uint16_t fwYear;
    uint16_t bootLoadVer;
    uint16_t serialNumber;
    uint16_t gyroModelId;
    uint16_t pageId;
    uint16_t decimationRate;
    uint16_t syncScale;
    uint16_t nullConfig;
    uint16_t clkConfig;
    uint16_t fnctioCtrl;
    uint16_t gpioCtrl;
    uint16_t ftrBank0;
    uint16_t ftrBank1;
} adi_imu_DevInfo_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} adi_imu_XYZOutputRaw16_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} __attribute__ ((packed)) adi_imu_XYZOutputRaw32_t;

typedef struct {
    double x;
    double y;
    double z;
} adi_imu_XYZOutput_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} adi_imu_XYZScale_t;

typedef struct {
    uint16_t data; 
} adi_imu_DiagStatus_t;

typedef struct {
    uint16_t data; 
} adi_imu_SysStatus_t;

typedef adi_imu_XYZOutputRaw16_t adi_imu_AcclOutputRaw16_t;
typedef adi_imu_XYZOutputRaw32_t adi_imu_AcclOutputRaw32_t;
typedef adi_imu_XYZOutputRaw16_t adi_imu_GyroOutputRaw16_t;
typedef adi_imu_XYZOutputRaw32_t adi_imu_GyroOutputRaw32_t;

typedef adi_imu_XYZOutput_t adi_imu_AcclOutput_t;
typedef adi_imu_XYZOutput_t adi_imu_GyroOutput_t;

typedef adi_imu_XYZOutputRaw32_t adi_imu_DelAngOutputRaw32_t;
typedef adi_imu_XYZOutputRaw32_t adi_imu_DelVelOutputRaw32_t;

typedef adi_imu_XYZOutputRaw32_t adi_imu_GyroBiasRaw32_t;
typedef adi_imu_XYZOutputRaw32_t adi_imu_AcclBiasRaw32_t;

typedef adi_imu_XYZScale_t adi_imu_GyroScale_t;
typedef adi_imu_XYZScale_t adi_imu_AcclScale_t;

typedef struct {
    uint16_t sysEFlag;
    int16_t tempOut;
    adi_imu_GyroOutputRaw32_t gyro;
    adi_imu_AcclOutputRaw32_t accl;
    uint16_t dataCntOrTimeStamp;
    uint32_t crc;
} __attribute__ ((packed)) adi_imu_BurstOutputRaw_t;

typedef struct {
    unsigned sysEFlag;
    float tempOut;
    adi_imu_GyroOutput_t gyro; // units: deg/sec
    adi_imu_AcclOutput_t accl; // units: g (gravity constant, m/s2)
    unsigned dataCntOrTimeStamp;
    uint32_t crc;
} adi_imu_BurstOutput_t;

typedef enum {
    XG = 0,
    YG = 0,
    ZG = 0,
    XA = 0,
    YA = 0,
    ZA = 0
} adi_imu_NullConfig_e;

typedef enum {
    FALSE = 0,
    TRUE = 1
} adi_imu_Boolean_e;

typedef enum {
    NEGATIVE = 0,
    POSITIVE = 1
} adi_imu_Polarity_e;

typedef enum {
    FALLING_EDGE = 0,
    RISING_EDGE = 1
} adi_imu_EdgeType_e;

typedef enum {
    DIO1 = 0,
    DIO2 = 1,
    DIO3 = 2,
    DIO4 = 3
} adi_imu_GPIO_e;

typedef enum {
    INPUT = 0,
    OUTPUT = 1
} adi_imu_Direction_e;

typedef enum {
    DISABLE = 0,
    ENABLE = 1
} adi_imu_EnDis_e;

typedef enum {
    SYNC = 0,
    PPS = 1
} adi_imu_ClockMode_e;


/* external spi driver API (provided by user) */
extern int spi_Init                 (adi_imu_Device_t *pDevice);

extern int spi_ReadWrite            (adi_imu_Device_t *pDevice, const uint8_t *txBuf, uint8_t *rxBuf, uint32_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx);

extern void delay_MicroSeconds      (uint32_t microseconds);

/* Available APIs */
int adi_imu_Init                    (adi_imu_Device_t *pDevice);

void adi_imu_ToggleEndian16         (uint8_t *pBuf, uint32_t lenBytes);

void adi_imu_ToggleEndian32         (uint8_t *pBuf, uint32_t lenBytes);

uint32_t adi_imu_Get32Bits          (uint8_t *buf, int idx);

int adi_imu_SetPage                 (adi_imu_Device_t *pDevice, uint8_t pageId);

int adi_imu_Read                    (adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint16_t *val);

int adi_imu_Write                   (adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint16_t val);

int adi_imu_GetDevInfo              (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo);

int adi_imu_PrintDevInfo            (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo);

int adi_imu_CheckDiagStatus         (adi_imu_Device_t *pDevice, adi_imu_DiagStatus_t *pStatus);

int adi_imu_CheckSysStatus          (adi_imu_Device_t *pDevice, adi_imu_SysStatus_t *pStatus);

int adi_imu_SetOutputDataRate       (adi_imu_Device_t *pDevice, uint16_t outputRate); /* Output data rate = 4250 / (DEC_RATE + 1) */

int adi_imu_ConfigGpio              (adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, adi_imu_Direction_e direction);

int adi_imu_SetGpio                 (adi_imu_Device_t *pDevice, adi_imu_GPIO_e id);

int adi_imu_ClearGpio               (adi_imu_Device_t *pDevice, adi_imu_GPIO_e id);

int adi_imu_GetGpio                 (adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, uint8_t* val);

int adi_imu_ConfigSyncClkMode       (adi_imu_Device_t *pDevice, adi_imu_ClockMode_e mode, adi_imu_EnDis_e clkEn, \
                                    adi_imu_EdgeType_e polarity, adi_imu_GPIO_e inputGpio);

int adi_imu_ConfigDataReady         (adi_imu_Device_t *pDevice, adi_imu_GPIO_e id, adi_imu_Polarity_e polarity);

int adi_imu_SetDataReady            (adi_imu_Device_t *pDevice, adi_imu_EnDis_e val);

int adi_imu_SetLineargComp          (adi_imu_Device_t *pDevice, adi_imu_EnDis_e val);

int adi_imu_SetPPercAlignment       (adi_imu_Device_t *pDevice, adi_imu_EnDis_e val);

int adi_imu_SoftwareReset           (adi_imu_Device_t *pDevice); 

int adi_imu_ClearUserCalibration    (adi_imu_Device_t *pDevice);

int adi_imu_UpdateFlashMemory       (adi_imu_Device_t *pDevice);

int adi_imu_PerformSelfTest         (adi_imu_Device_t *pDevice);

int adi_imu_UpdateBiasCorrection    (adi_imu_Device_t *pDevice);

int adi_imu_ReadAccl                (adi_imu_Device_t *pDevice, adi_imu_AcclOutputRaw32_t *pData);

int adi_imu_ReadGyro                (adi_imu_Device_t *pDevice, adi_imu_GyroOutputRaw32_t *pData);

int adi_imu_ReadDelAng              (adi_imu_Device_t *pDevice, adi_imu_DelAngOutputRaw32_t *pData);

int adi_imu_ReadDelVel              (adi_imu_Device_t *pDevice, adi_imu_DelVelOutputRaw32_t *pData);

int adi_imu_ReadBurstRaw            (adi_imu_Device_t *pDevice, uint8_t *pBuf, uint32_t numBursts);

int adi_imu_ReadBurst               (adi_imu_Device_t *pDevice, uint8_t *pBuf, uint32_t numBursts, adi_imu_BurstOutput_t *pData);

int adi_imu_GetAcclScale            (adi_imu_Device_t *pDevice, adi_imu_AcclScale_t *pData);

int adi_imu_GetGyroScale            (adi_imu_Device_t *pDevice, adi_imu_GyroScale_t *pData);

int adi_imu_GetAcclBias             (adi_imu_Device_t *pDevice, adi_imu_AcclBiasRaw32_t *pData);

int adi_imu_GetGyroBias             (adi_imu_Device_t *pDevice, adi_imu_GyroBiasRaw32_t *pData);

int adi_imu_FindBurstPayloadIdx     (const uint8_t* pBuf, unsigned bufLength, unsigned* pPayloadOffset);

int adi_imu_ParseBurstOut           (adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_BurstOutputRaw_t *pRawData);

int adi_imu_ScaleBurstOut_1         (adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_BurstOutput_t *pData);

void adi_imu_ScaleBurstOut_2        (adi_imu_Device_t *pDevice, const adi_imu_BurstOutputRaw_t *pRawData, adi_imu_BurstOutput_t *pData);

void adi_imu_ScaleTempOut           (adi_imu_Device_t *pDevice, uint16_t rawData, float *pData);

void adi_imu_ScaleAccl32Out         (adi_imu_Device_t *pDevice, const adi_imu_AcclOutputRaw32_t *pRawData, adi_imu_AcclOutput_t *pData);

void adi_imu_ScaleGyro32Out         (adi_imu_Device_t *pDevice, const adi_imu_GyroOutputRaw32_t *pRawData, adi_imu_GyroOutput_t *pData);

void adi_imu_ScaleAccl16Out         (adi_imu_Device_t *pDevice, const adi_imu_AcclOutputRaw16_t *pRawData, adi_imu_AcclOutput_t *pData);

void adi_imu_ScaleGyro16Out         (adi_imu_Device_t *pDevice, const adi_imu_GyroOutputRaw16_t *pRawData, adi_imu_GyroOutput_t *pData);

#ifdef __cplusplus
}
#endif
#endif