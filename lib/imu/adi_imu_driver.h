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

#include "adi_imu_common.h"

typedef struct {
    uint16_t prodId;
    uint16_t fwRev;
    uint16_t fwDayMonth;
    uint16_t fwYear;
    uint16_t serialNumber;
    uint16_t gyroModelId;
    uint16_t pageId;
    uint16_t decimationRate;
    uint16_t nullConfig; // Used in 16470, 16495 only
    uint16_t syncScale; // Used in 16495 only
    uint16_t bootLoadVer; // Used in 16495 only
    uint16_t clkConfig; // Used in 16495 only
    uint16_t fnctioCtrl; // Used in 16495 only
    uint16_t gpioCtrl; // Used in 16495 only
    uint16_t ftrBank0; // Used in 16495 only
    uint16_t ftrBank1; // Used in 16495 only
    uint16_t mscCtrl; // Used in 16470, 16500 only
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
} __attribute__ ((packed)) adi_imu_BurstOutputRaw32_t;

typedef struct {
    uint16_t sysEFlag;
    int16_t tempOut;
    adi_imu_GyroOutputRaw16_t gyro;
    adi_imu_AcclOutputRaw16_t accl;
    uint16_t dataCntOrTimeStamp;
    uint32_t crc; //Note: There are instances when CRC is 16/32-bit
} __attribute__ ((packed)) adi_imu_BurstOutputRaw16_t;

typedef struct {
    unsigned sysEFlag;
    float tempOut;
    adi_imu_GyroOutput_t gyro; // units: deg/sec
    adi_imu_AcclOutput_t accl; // units: g (gravity constant, m/s2)
    unsigned dataCntOrTimeStamp;
    uint32_t crc;
} adi_imu_BurstOutput_t;

typedef enum {
    IMU_NEG_POLARITY = 0,
    IMU_POS_POLARITY = 1
} adi_imu_Polarity_e;

typedef enum {
    IMU_FALLING_EDGE = 0,
    IMU_RISING_EDGE = 1
} adi_imu_EdgeType_e;

typedef enum {
    IMU_DIO1 = 0,
    IMU_DIO2 = 1,
    IMU_DIO3 = 2,
    IMU_DIO4 = 3
} adi_imu_GPIO_e;

typedef enum {
    IMU_INPUT = 0,
    IMU_OUTPUT = 1
} adi_imu_Direction_e;

typedef enum {
    IMU_DISABLE = 0,
    IMU_ENABLE = 1
} adi_imu_EnDis_e;

typedef enum {
    // Used in ADIS16495
    IMU_SYNC = 0,
    IMU_PPS = 1,
    // Used in ADIS16470/16500
    IMU_INTERNAL_CLOCK = 0,
    IMU_DIRECT_SYNC = 1,
    IMU_SCALED_SYNC = 2,
    IMU_OUTPUT_SYNC = 3
} adi_imu_ClockMode_e;

/* Available APIs */
int adi_imu_Init                    (adi_imu_Device_t *pDevice);

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

int adi_imu_ConfigBiasCorrectionTime(adi_imu_Device_t *pDevice, uint8_t time);

int adi_imu_SelectBiasConfigAxes    (adi_imu_Device_t *pDevice, adi_imu_EnDis_e XG, adi_imu_EnDis_e YG, \
                                    adi_imu_EnDis_e ZG, adi_imu_EnDis_e XA, adi_imu_EnDis_e YA, adi_imu_EnDis_e ZA);

int adi_imu_ReadAccl                (adi_imu_Device_t *pDevice, adi_imu_AcclOutputRaw32_t *pData);

int adi_imu_ReadGyro                (adi_imu_Device_t *pDevice, adi_imu_GyroOutputRaw32_t *pData);

int adi_imu_ReadDelAng              (adi_imu_Device_t *pDevice, adi_imu_DelAngOutputRaw32_t *pData);

int adi_imu_ReadDelVel              (adi_imu_Device_t *pDevice, adi_imu_DelVelOutputRaw32_t *pData);

int adi_imu_ReadBurstRaw            (adi_imu_Device_t *pDevice, uint8_t *pBuf, uint32_t numBursts);

int adi_imu_ReadBurst               (adi_imu_Device_t *pDevice, uint8_t *pBuf, uint32_t numBursts, adi_imu_BurstOutput_t *pData);

int adi_imu_GetAcclScale            (adi_imu_Device_t *pDevice, adi_imu_AcclScale_t *pData);

int adi_imu_GetGyroScale            (adi_imu_Device_t *pDevice, adi_imu_GyroScale_t *pData);

int adi_imu_SetAcclScale            (adi_imu_Device_t *pDevice, adi_imu_AcclScale_t data);

int adi_imu_SetGyroScale            (adi_imu_Device_t *pDevice, adi_imu_GyroScale_t data);

int adi_imu_GetAcclBias             (adi_imu_Device_t *pDevice, adi_imu_AcclBiasRaw32_t *pData);

int adi_imu_GetGyroBias             (adi_imu_Device_t *pDevice, adi_imu_GyroBiasRaw32_t *pData);

int adi_imu_SetAcclBias             (adi_imu_Device_t *pDevice, adi_imu_AcclBiasRaw32_t data);

int adi_imu_SetGyroBias             (adi_imu_Device_t *pDevice, adi_imu_GyroBiasRaw32_t data);

int adi_imu_FindBurstPayloadIdx     (const uint8_t* pBuf, unsigned bufLength, unsigned* pPayloadOffset);

int adi_imu_ParseBurst32Out         (adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutputRaw32_t *pRawData);

int adi_imu_ParseBurst16Out         (adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutputRaw16_t *pRawData);

int adi_imu_ScaleBurstOut           (adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutput_t *pData);

int adi_imu_ScaleBurst32Out         (adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutput_t *pData);

int adi_imu_ScaleBurst16Out         (adi_imu_Device_t *pDevice, const uint8_t *pBuf, adi_imu_Boolean_e checkBurstID, adi_imu_Boolean_e enByteSwap, adi_imu_BurstOutput_t *pData);

void adi_imu_ScaleTemp32Out         (adi_imu_Device_t *pDevice, uint16_t rawData, float *pData);

void adi_imu_ScaleTemp16Out         (adi_imu_Device_t *pDevice, uint16_t rawData, float *pData);

void adi_imu_ScaleAccl32Out         (adi_imu_Device_t *pDevice, const adi_imu_AcclOutputRaw32_t *pRawData, adi_imu_AcclOutput_t *pData);

void adi_imu_ScaleGyro32Out         (adi_imu_Device_t *pDevice, const adi_imu_GyroOutputRaw32_t *pRawData, adi_imu_GyroOutput_t *pData);

void adi_imu_ScaleAccl16Out         (adi_imu_Device_t *pDevice, const adi_imu_AcclOutputRaw16_t *pRawData, adi_imu_AcclOutput_t *pData);

void adi_imu_ScaleGyro16Out         (adi_imu_Device_t *pDevice, const adi_imu_GyroOutputRaw16_t *pRawData, adi_imu_GyroOutput_t *pData);

int adi_imu_ConfigBurstDataFormat   (adi_imu_Device_t *pDevice, adi_imu_DataFormat_e dataFormat);

adi_imu_BuildInfo_t adi_imu_GetBuildInfo (adi_imu_Device_t *pDevice);

#ifdef __cplusplus
}
#endif
#endif