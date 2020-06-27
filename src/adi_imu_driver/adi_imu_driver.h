/*
    File: adi_imu_driver.h

    Description:
    Driver interface for ADIS16xxx IMU

    Author: Sundar Palani <sundar.palani@analog.com>
*/

#ifndef __ADI_IMU_DRIVER_H_
#define __ADI_IMU_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

#define IMU_BOOT_REV_MAJOR(val)     ( ((val) >> 8 ) & 0xFF )
#define IMU_BOOT_REV_MINOR(val)     ( (val) & 0xFF )

#define IMU_FW_REV_UPPER(val)       ( ((val) >> 8 ) & 0xFF )
#define IMU_FW_REV_LOWER(val)       ( (val) & 0xFF )

#define IMU_FW_DAY(val)             ( (val) & 0xFF )
#define IMU_FW_MONTH(val)           ( ((val) >> 8 ) & 0xFF )

#define IMU_RANGE_MODEL(id)         ((id == 0x3) ? "±125°/sec" : (id == 0x7) ? "±450°/sec" : (id == 0xF) ? "±2000°/sec" : "UNKNOWN")

#define IMU_TO_HALFWORD(buf, idx)   ( ((buf[idx] << 8) & 0xFF00) | (buf[1+idx] & 0xFF) )

#define IMU_TO_WORD(buf, idx)       ( (uint32_t)((buf[2+idx] << 24) & 0xFF000000) | (uint32_t)((buf[3+idx] << 16) & 0xFF0000) | (uint32_t)((buf[idx] << 8) & 0xFF00) | (uint32_t)(buf[1+idx] & 0xFF) )

#define IMU_TO_WORD1(high, low)     ( (( (high) << 16) & 0xFFFF0000) | ( (low) & 0xFFFF) )

#ifndef BAREMETAL
#include <stdio.h>
#define IMU_DEBUG_PRINT(format, ...) printf(format , ##__VA_ARGS__)
#else
#define IMU_DEBUG_PRINT(format, ...) {}
#endif
    
enum adi_imu_Error_e {

    adi_imu_BadDevice_e = -5,
    adi_imu_BurstFrameInvalid_e = -4,
    adi_imu_ProdIdVerifyFailed_e = -3,
    adi_imu_SpiRwFailed_e = -2,
    adi_imu_SpiInitFailed_e = -1,
    adi_imu_Success_e = 0,
};

typedef volatile void* adi_imu_DevHandler_t;
typedef struct {
    /* for verification */
    uint16_t prodId;

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

} adi_imu_Device_t;

typedef struct {
    uint16_t prodId;
    uint16_t fwRev;
    uint16_t fwDayMonth;
    uint16_t fwYear;
    uint16_t bootLoadVer;
    uint16_t serialNumber;
    uint16_t gyroModelId;
} adi_imu_DevInfo_t;

typedef struct {
    uint32_t accX;
    uint32_t accY;
    uint32_t accZ;
} adi_imu_AcclOutput_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} adi_imu_GyroOutput_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} adi_imu_DelAngOutput_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} adi_imu_DelVelOutput_t;

typedef struct {
    uint16_t data; 
} adi_imu_DiagStatus_t;

typedef struct {
    uint16_t data; 
} adi_imu_SysStatus_t;

typedef struct {
    uint16_t sysEFlag;
    uint16_t tempOut;
    adi_imu_GyroOutput_t gyro;
    adi_imu_GyroOutput_t accl;
    uint32_t crc;
} adi_imu_BurstOutput_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} adi_imu_GyroScale_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} adi_imu_AcclScale_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} adi_imu_GyroBias_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} adi_imu_AcclBias_t;

int adi_imu_Init                    (adi_imu_Device_t *pDevice);

int adi_imu_GetDevInfo              (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo);

int adi_imu_PrintDevInfo            (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo);

int adi_imu_CheckDiagStatus         (adi_imu_Device_t *pDevice, adi_imu_DiagStatus_t *pStatus);

int adi_imu_CheckSysStatus          (adi_imu_Device_t *pDevice, adi_imu_SysStatus_t *pStatus);

int adi_imu_SetDecimationRate       (adi_imu_Device_t *pDevice, uint16_t rate); /* Output data rate = 4250 / (DEC_RATE + 1) */

int adi_imu_ReadAccl                (adi_imu_Device_t *pDevice, adi_imu_AcclOutput_t *pData);

int adi_imu_ReadGyro                (adi_imu_Device_t *pDevice, adi_imu_GyroOutput_t *pData);

int adi_imu_ReadDelAng              (adi_imu_Device_t *pDevice, adi_imu_DelAngOutput_t *pData);

int adi_imu_ReadDelVel              (adi_imu_Device_t *pDevice, adi_imu_DelVelOutput_t *pData);

int adi_imu_ReadBurst               (adi_imu_Device_t *pDevice, adi_imu_BurstOutput_t *pData);

int adi_imu_GetAcclScale            (adi_imu_Device_t *pDevice, adi_imu_AcclScale_t *pScale);

int adi_imu_GetGyroScale            (adi_imu_Device_t *pDevice, adi_imu_GyroScale_t *pScale);

int adi_imu_GetAcclBias             (adi_imu_Device_t *pDevice, adi_imu_AcclBias_t *pBias);

int adi_imu_GetGyroBias             (adi_imu_Device_t *pDevice, adi_imu_GyroBias_t *pBias);

int adi_imu_read                    (adi_imu_Device_t *pDevice, uint8_t *buf);

int adi_imu_read_burst              (adi_imu_Device_t *pDevice, uint8_t *buf, unsigned length);

int adi_imu_write                   (adi_imu_Device_t *pDevice, uint8_t *buf);

#ifdef __cplusplus
}
#endif
#endif